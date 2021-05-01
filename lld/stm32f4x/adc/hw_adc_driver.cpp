/********************************************************************************
 *  File Name:
 *    hw_adc_driver.cpp
 *
 *  Description:
 *    Implements the LLD interface to the ADC hardware.
 *
 *  Limitations:
 *    This driver does not support injected channel conversions.
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* STL Includes */
#include <cstring>
#include <limits>

/* Aurora Includes */
#include <Aurora/utility>

/* Chimera Includes */
#include <Chimera/algorithm>
#include <Chimera/common>
#include <Chimera/utility>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/inc/adc>
#include <Thor/lld/interface/inc/interrupt>
#include <Thor/lld/interface/inc/rcc>
#include <Thor/lld/interface/inc/timer>


#if defined( THOR_LLD_ADC ) && defined( TARGET_STM32F4 )

namespace Thor::LLD::ADC
{
  /*-------------------------------------------------------------------------------
  Macros
  -------------------------------------------------------------------------------*/
  /**
   *  A conversion is in progress if:
   *    ADSTART is set    OR
   *    JADSTART is set   OR
   *    EOC flag is set
   */
#define CNVRT_IN_PROGRESS( periph_ptr ) ( STRT::get( periph_ptr ) || JSTRT::get( periph_ptr ) || EOC::get( periph_ptr ) )

  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  /**
   *  Internal voltage reference, address of parameter VREFINT_CAL: VrefInt ADC raw
   *  data acquired at temperature 30 DegC (tolerance: +-5 DegC), Vref+ = 3.0 V
   *  (tolerance: +-10 mV)
   */
  static uint16_t *VREFINT_CAL_ADDR = ( ( uint16_t * )( 0x1FFF75AAUL ) );

  /**
   *  Analog voltage reference (Vref+) value with which temperature sensor has been
   *  calibrated in production (tolerance: +-10 mV) (unit: mV).
   */
  static constexpr float VREFINT_CAL_VREF = 3000.0f;


  /*-------------------------------------------------------------------------------
  Low Level Driver Implementation
  -------------------------------------------------------------------------------*/
  Driver::Driver() : mPeriph( nullptr ), mResourceIndex( INVALID_RESOURCE_INDEX )
  {
  }

  Driver::~Driver()
  {
  }


  /*-------------------------------------------------
  Driver Public Methods
  -------------------------------------------------*/
  Chimera::Status_t Driver::attach( RegisterMap *const peripheral )
  {
    using namespace Chimera::ADC;

    /*------------------------------------------------
    Get peripheral descriptor settings
    ------------------------------------------------*/
    mPeriph        = peripheral;
    mCommon        = ADC_COMMON;
    mResourceIndex = getResourceIndex( reinterpret_cast<std::uintptr_t>( peripheral ) );

    /*------------------------------------------------
    Handle the ISR configuration
    ------------------------------------------------*/
    Thor::LLD::INT::disableIRQ( Resource::IRQSignals[ mResourceIndex ] );
    Thor::LLD::INT::clearPendingIRQ( Resource::IRQSignals[ mResourceIndex ] );
    Thor::LLD::INT::setPriority( Resource::IRQSignals[ mResourceIndex ], Thor::LLD::INT::ADC_IT_PREEMPT_PRIORITY, 0u );

    /*-------------------------------------------------
    Reset the channel sample times to defaults
    -------------------------------------------------*/
    for ( size_t idx = 0; idx < NUM_ADC_CHANNELS_PER_PERIPH; idx++ )
    {
      setSampleTime( static_cast<Channel>( idx ), SampleTime::SMP_28 );
    }

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::initialize( const Chimera::ADC::DriverConfig &cfg )
  {
    using namespace Thor::LLD::RCC;

    /*-------------------------------------------------
    Reset the device back to default register values
    -------------------------------------------------*/
    this->clockEnable();
    this->reset();

    /*-------------------------------------------------
    Select the clock prescaler. On the F4, the ADC is
    driven directly from the APB2 clock. (RM Fig.13)
    -------------------------------------------------*/
    switch ( cfg.clockPrescale )
    {
      case Chimera::ADC::Prescaler::DIV_2:
        ADCPRE::set( ADC_COMMON, 0 << CCR_ADCPRE_Pos );
        break;

      case Chimera::ADC::Prescaler::DIV_4:
        ADCPRE::set( ADC_COMMON, 1 << CCR_ADCPRE_Pos );
        break;

      case Chimera::ADC::Prescaler::DIV_6:
        ADCPRE::set( ADC_COMMON, 2 << CCR_ADCPRE_Pos );
        break;

      case Chimera::ADC::Prescaler::DIV_8:
      default:
        ADCPRE::set( ADC_COMMON, 3 << CCR_ADCPRE_Pos );
        break;
    };

    /*-------------------------------------------------
    Enable the temperature sensor and internal VRef
    -------------------------------------------------*/
    TSVREFE::set( ADC_COMMON, CCR_TSVREFE );

    /*-------------------------------------------------
    Assign the ADC resolution
    -------------------------------------------------*/
    switch ( cfg.resolution )
    {
      case Chimera::ADC::Resolution::BIT_12:
        RES::set( mPeriph, 0x00 << CR1_RES_Pos );
        break;

      case Chimera::ADC::Resolution::BIT_10:
        RES::set( mPeriph, 0x01 << CR1_RES_Pos );
        break;

      case Chimera::ADC::Resolution::BIT_8:
        RES::set( mPeriph, 0x02 << CR1_RES_Pos );
        break;

      case Chimera::ADC::Resolution::BIT_6:
      default:
        RES::set( mPeriph, 0x03 << CR1_RES_Pos );
        break;
    };

    /*-------------------------------------------------
    Select the EOC interrupt to occur at the end of
    each regular conversion.
    -------------------------------------------------*/
    EOCS::set( mPeriph, CR2_EOCS );

    /*-------------------------------------------------
    Initially disable DMA transfers
    -------------------------------------------------*/
    DDS::clear( mPeriph, CR2_DDS );
    DMA::clear( mPeriph, CR2_DMA );

    /*-------------------------------------------------
    Enable ISRs
    -------------------------------------------------*/
    EOCIE::set( mPeriph, CR1_EOCIE ); /* End of conversion interrupt */

    /*-------------------------------------------------
    Bring the ADC out of power down mode
    -------------------------------------------------*/
    ADON::set( mPeriph, CR2_ADON );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::setSampleTime( const Chimera::ADC::Channel ch, const SampleTime time )
  {
    using namespace Chimera::ADC;

    /*-------------------------------------------------
    Input Protection
    -------------------------------------------------*/
    if ( !( EnumValue( ch ) < NUM_ADC_CHANNELS_PER_PERIPH) )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------
    Apply sample times to the registers
    -------------------------------------------------*/
    size_t chNum = static_cast<size_t>( ch );

    if ( ch < Channel::ADC_CH_10 )
    {
      auto chPos     = static_cast<size_t>( chNum ) * SMPRx_BIT_Wid;
      Reg32_t regVal = static_cast<size_t>( time ) << chPos;
      Reg32_t curVal = SMPR1_ALL::get( mPeriph );

      curVal &= ~( SMPRx_BIT_Msk << chPos );
      curVal |= regVal;

      SMPR1_ALL::set( mPeriph, curVal );
    }
    else
    {
      auto chOffset  = static_cast<size_t>( Channel::ADC_CH_10 );
      auto chPos     = static_cast<size_t>( chNum ) - chOffset;
      Reg32_t regVal = static_cast<size_t>( time ) << chPos;
      Reg32_t curVal = SMPR2_ALL::get( mPeriph );

      curVal &= ~( SMPRx_BIT_Msk << chPos );
      curVal |= regVal;

      SMPR2_ALL::set( mPeriph, curVal );
    }

    return Chimera::Status::OK;
  }


  Chimera::Status_t setupSequence( const Chimera::ADC::SequenceInit& sequence )
  {


    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::ADC::Sample Driver::sampleChannel( const Chimera::ADC::Channel channel )
  {
    using namespace Chimera::ADC;
    return Chimera::ADC::Sample();

    /*-------------------------------------------------
    Wait for the hardware to indicate it's free for a
    new transfer.

    This may have issues if a continuous transfer is
    running in the background. Access to these bits are
    not atomic, so stopping an existing conversion may
    be necessary in the future.
    -------------------------------------------------*/
    while ( CNVRT_IN_PROGRESS( mPeriph ) )
    {
      continue;
    }

    /*-------------------------------------------------
    Set single conversion mode.
    Set EOC flag to be set at end of single conversion
    -------------------------------------------------*/
    CONT::clear( mPeriph, CR2_CONT );
    EOCS::set( mPeriph, CR2_EOCS );

    /*-------------------------------------------------
    Configure the channel to be measured
    -------------------------------------------------*/
    L::set( mPeriph, 1u << SQR1_L_Pos );
    SQ1::set( mPeriph, EnumValue( channel ) << SQR3_SQ1_Pos );

    /*-------------------------------------------------
    Prevent peripheral interrupts as this conversion is
    fast enough that it doesn't make sense to use ISRs.
    -------------------------------------------------*/
    disableInterrupts();

    /*-------------------------------------------------
    Perform the conversion
    -------------------------------------------------*/
    startSequence();
    while ( !EOC::get( mPeriph ) )
    {
      continue;
    }
    EOC::set( mPeriph, SR_EOC );

    Sample measurement;
    measurement.counts = DATA::get( mPeriph );
    measurement.us     = Chimera::micros();

    /*-------------------------------------------------
    Re-enable ISRs and return the valid measurement
    -------------------------------------------------*/
    Thor::LLD::INT::clearPendingIRQ( Resource::IRQSignals[ mResourceIndex ] );
    enableInterrupts();

    return measurement;
  }


  float Driver::sampleToVoltage( const Chimera::ADC::Sample sample )
  {
    return 0.0f;
  }


  void Driver::startSequence()
  {
    /*-------------------------------------------------
    Set the start bit, then wait for HW to clear it
    -------------------------------------------------*/
    SWSTART::set( mPeriph, CR2_SWSTART );
    while( SWSTART::get( mPeriph ) )
    {
      /* If stuck here, the ADC probably isn't enabled */
      continue;
    }
  }


  void Driver::stopSequence()
  {
    /*-------------------------------------------------
    You can't stop a single in-progress conversion, but
    the ADC can be prevented from continuous conversion.
    -------------------------------------------------*/
    CONT::clear( mPeriph, CR2_CONT );
  }


  void Driver::IRQHandler()
  {
    // Handle ISR event
    // Push data to the queue if needed
    // Alert the HLD user ISR task with Chimera::ADC::Interrupt signal
  }
}    // namespace Thor::LLD::ADC

#endif /* TARGET_STM32L4 && THOR_DRIVER_ADC */
