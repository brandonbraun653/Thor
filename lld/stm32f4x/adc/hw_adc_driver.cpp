/********************************************************************************
 *  File Name:
 *    hw_adc_driver.cpp
 *
 *  Description:
 *    Implements the LLD interface to the ADC hardware.
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* STL Includes */
#include <cstring>
#include <limits>

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
#define CNVRT_IN_PROGRESS( periph_ptr ) ( ADSTART::get( periph_ptr ) || JADSTART::get( periph_ptr ) || EOC::get( periph_ptr ) )

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
  Driver::Driver() : mPeriph( nullptr ), mResourceIndex( INVALID_RESOURCE_INDEX ), mConversionInProgress( false )
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
    for ( size_t idx = 0; idx < ARRAY_COUNT( mChannelSampleTime ); idx++ )
    {
      mChannelSampleTime[ idx ] = SampleTime::SMP_24P5;
    }

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::initialize( const Chimera::ADC::DriverConfig &cfg )
  {
    using namespace Thor::LLD::RCC;

    /*-------------------------------------------------
    Configure the clock
    -------------------------------------------------*/
    if ( cfg.clockSource != Chimera::Clock::Bus::PCLK2 )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    // // Select the clock source
    // ADCSEL::set( RCC1_PERIPH, Config::CCIPR::ADCSEL_SYS_CLOCK );

    // // Turn on the core peripheral clock
    // this->clockEnable();
    // this->reset();

    // // Set the clock mode to be asynchronous
    // CKMODE::set( ADC_COMMON, 0 );

    // // Set the prescaler
    // switch ( cfg.clockPrescale )
    // {
    //   case Chimera::ADC::Prescaler::DIV_1:
    //     PRESC::set( ADC_COMMON, 0 << CCR_PRESC_Pos );
    //     break;

    //   case Chimera::ADC::Prescaler::DIV_2:
    //     PRESC::set( ADC_COMMON, 1 << CCR_PRESC_Pos );
    //     break;

    //   case Chimera::ADC::Prescaler::DIV_4:
    //     PRESC::set( ADC_COMMON, 2 << CCR_PRESC_Pos );
    //     break;

    //   case Chimera::ADC::Prescaler::DIV_6:
    //     PRESC::set( ADC_COMMON, 3 << CCR_PRESC_Pos );
    //     break;

    //   case Chimera::ADC::Prescaler::DIV_8:
    //     PRESC::set( ADC_COMMON, 4 << CCR_PRESC_Pos );
    //     break;

    //   case Chimera::ADC::Prescaler::DIV_10:
    //     PRESC::set( ADC_COMMON, 5 << CCR_PRESC_Pos );
    //     break;

    //   case Chimera::ADC::Prescaler::DIV_12:
    //     PRESC::set( ADC_COMMON, 6 << CCR_PRESC_Pos );
    //     break;

    //   case Chimera::ADC::Prescaler::DIV_16:
    //     PRESC::set( ADC_COMMON, 7 << CCR_PRESC_Pos );
    //     break;

    //   case Chimera::ADC::Prescaler::DIV_32:
    //     PRESC::set( ADC_COMMON, 8 << CCR_PRESC_Pos );
    //     break;

    //   case Chimera::ADC::Prescaler::DIV_64:
    //     PRESC::set( ADC_COMMON, 9 << CCR_PRESC_Pos );
    //     break;

    //   case Chimera::ADC::Prescaler::DIV_128:
    //     PRESC::set( ADC_COMMON, 10 << CCR_PRESC_Pos );
    //     break;

    //   case Chimera::ADC::Prescaler::DIV_256:
    //     PRESC::set( ADC_COMMON, 11 << CCR_PRESC_Pos );
    //     break;

    //   default:
    //     // Prescaler not supported
    //     break;
    // };

    // /*-------------------------------------------------
    // Enable the sensor monitors
    // -------------------------------------------------*/
    // VBATEN::set( ADC_COMMON, CCR_VBATEN );    // External battery sensor
    // TSEN::set( ADC_COMMON, CCR_TSEN );        // Internal temperature sensor
    // VREFEN::set( ADC_COMMON, CCR_VREFEN );    // Internal bandgap vref sensor

    // /*-------------------------------------------------
    // Bring the ADC out of deep power down
    // -------------------------------------------------*/
    // DEEPPWD::clear( mPeriph, CR_DEEPPWD );    // Disable deep power down
    // ADVREGEN::set( mPeriph, CR_ADVREGEN );    // Enable the voltage regulator

    // // Allow the analog voltage regulator time to boot
    // Chimera::delayMilliseconds( 250 );

    // /*-------------------------------------------------
    // Assign the ADC resolution
    // -------------------------------------------------*/
    // switch ( cfg.resolution )
    // {
    //   case Chimera::ADC::Resolution::BIT_12:
    //     RES::set( mPeriph, 0 << CFGR_RES_Pos );
    //     break;

    //   case Chimera::ADC::Resolution::BIT_10:
    //     RES::set( mPeriph, 1 << CFGR_RES_Pos );
    //     break;

    //   case Chimera::ADC::Resolution::BIT_8:
    //     RES::set( mPeriph, 2 << CFGR_RES_Pos );
    //     break;

    //   case Chimera::ADC::Resolution::BIT_6:
    //   default:
    //     RES::set( mPeriph, 3 << CFGR_RES_Pos );
    //     break;
    // };

    // /*-------------------------------------------------
    // Calibrate the ADC
    // -------------------------------------------------*/
    // // Default to single ended inputs for all channels
    // DIFSEL::clear( mPeriph, DIFSEL_DIFSEL );

    // // Start calibration for single ended inputs
    // ADCALDIF::clear( mPeriph, CR_ADCALDIF );
    // ADCAL::set( mPeriph, CR_ADCAL );

    // // Calibration complete when bit is cleared
    // while ( ADCAL::get( mPeriph ) )
    // {
    //   continue;
    // }

    // /*-------------------------------------------------
    // Enable ISRs
    // -------------------------------------------------*/
    // EOCIE::set( mPeriph, IER_EOCIE );
    // JEOCIE::set( mPeriph, IER_JEOCIE );

    // /*-------------------------------------------------
    // Enable the ADC
    // -------------------------------------------------*/
    // ADEN::set( mPeriph, CR_ADEN );

    // // Wait for ADC to signal it's ready, then ACK it via write 1.
    // while ( !ADRDY::get( mPeriph ) )
    // {
    //   continue;
    // }
    // ADRDY::set( mPeriph, ISR_ADRDY );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::setSampleTime( const Chimera::ADC::Channel ch, const SampleTime time )
  {
    /*-------------------------------------------------
    Input Protection
    -------------------------------------------------*/
    if ( !( static_cast<size_t>( ch ) < ARRAY_COUNT( mChannelSampleTime ) ) )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------
    Write access protection is provided by the HLD
    -------------------------------------------------*/
    mChannelSampleTime[ static_cast<size_t>( ch ) ] = time;
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

    // /*-------------------------------------------------
    // Wait for the hardware to indicate it's free for a
    // new transfer.

    // This may have issues if a continuous transfer is
    // running in the background. Access to these bits are
    // not atomic, so stopping an existing conversion may
    // be necessary in the future.
    // -------------------------------------------------*/
    // while ( !mConversionInProgress && CNVRT_IN_PROGRESS( mPeriph ) )
    // {
    //   continue;
    // }

    // // Internally lock out other measurements
    // mConversionInProgress = true;

    // /*-------------------------------------------------
    // Apply the currently configured sample time for this channel
    // -------------------------------------------------*/
    // size_t chNum = static_cast<size_t>( channel );

    // if ( channel < Channel::ADC_CH_10 )
    // {
    //   auto chPos     = static_cast<size_t>( chNum ) * SMPRx_BIT_Wid;
    //   Reg32_t regVal = static_cast<size_t>( mChannelSampleTime[ chNum ] ) << chPos;

    //   SMPR1_ALL::set( mPeriph, regVal );
    // }
    // else
    // {
    //   auto chOffset  = static_cast<size_t>( Channel::ADC_CH_10 );
    //   auto chPos     = static_cast<size_t>( chNum ) - chOffset;
    //   Reg32_t regVal = static_cast<size_t>( mChannelSampleTime[ chNum ] ) << chPos;

    //   SMPR2_ALL::set( mPeriph, regVal );
    // }

    // /*-------------------------------------------------
    // Set single conversion mode
    // -------------------------------------------------*/
    // CONT::clear( mPeriph, CFGR_CONT );

    // /*-------------------------------------------------
    // Configure the channel to be measured
    // -------------------------------------------------*/
    // L::set( mPeriph, 1 );
    // SQ1::set( mPeriph, chNum << SQR1_SQ1_Pos );

    // /*-------------------------------------------------
    // Clear the EOC flag (by set)
    // -------------------------------------------------*/
    // EOC::set( mPeriph, ISR_EOC );

    // /*-------------------------------------------------
    // Cache the ISR flags set before conversion. This
    // allows clearing any new interrupts generated later.
    // -------------------------------------------------*/
    // Reg32_t isrFlags = ISR_ALL::get( mPeriph );

    // /*-------------------------------------------------
    // Prevent peripheral interrupts as this conversion is
    // fast enough that it doesn't make sense to use ISRs.
    // -------------------------------------------------*/
    // disableInterrupts();

    // /*-------------------------------------------------
    // Perform the conversion, applying the fix for
    // Errata 2.5.2.
    // -------------------------------------------------*/
    // // Start conversion, then acknowledge completion
    // ADSTART::set( mPeriph, CR_ADSTART );
    // while ( !EOC::get( mPeriph ) )
    // {
    //   continue;
    // }
    // EOC::set( mPeriph, ISR_EOC );

    // // Convert one more time. This is the Errata fix.
    // ADSTART::set( mPeriph, CR_ADSTART );
    // while ( !EOC::get( mPeriph ) )
    // {
    //   continue;
    // }
    // EOC::set( mPeriph, ISR_EOC );

    // // Read out the result of the conversion
    // Sample measurement = DATA::get( mPeriph );

    // // Clear any other new ISR flags before interrupts are re-enabled
    // Reg32_t newFlags = ISR_ALL::get( mPeriph ) & ~isrFlags;
    // ISR_ALL::set( mPeriph, newFlags );

    // // Let this object know transfers are done
    // mConversionInProgress = false;

    // /*-------------------------------------------------
    // Re-enable ISRs and return the valid measurement
    // -------------------------------------------------*/
    // enableInterrupts();
    // return measurement;
  }


  float Driver::sampleToVoltage( const Chimera::ADC::Sample sample )
  {
    return 0.0f;
  }


  void Driver::startSequence()
  {

  }


  void Driver::stopSequence()
  {

  }


  void Driver::IRQHandler()
  {
  }
}    // namespace Thor::LLD::ADC

#endif /* TARGET_STM32L4 && THOR_DRIVER_ADC */
