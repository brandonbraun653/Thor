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
#include <Chimera/adc>
#include <Chimera/algorithm>
#include <Chimera/common>
#include <Chimera/thread>
#include <Chimera/utility>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/inc/adc>
#include <Thor/lld/interface/inc/interrupt>
#include <Thor/lld/interface/inc/rcc>
#include <Thor/lld/interface/inc/timer>

#include "SEGGER_SYSVIEW.h"


#if defined( THOR_LLD_ADC ) && defined( TARGET_STM32F4 )

/*-------------------------------------------------
This driver expects some project side constants to
be defined for helping with a few calculations.

#define PRJ_ADC_VREF  (x.yzf)
-------------------------------------------------*/
#include "thor_adc_prj_config.hpp"

namespace Thor::LLD::ADC
{
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
  Driver::Driver() : mPeriph( nullptr ), mResourceIndex( INVALID_RESOURCE_INDEX ), mSequenceIdx( 0 )
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


  Chimera::Status_t Driver::setupSequence( const Chimera::ADC::SequenceInit& sequence )
  {
    using namespace Chimera::ADC;

    /*-------------------------------------------------
    Input protections
    -------------------------------------------------*/
    if ( !sequence.channels || !sequence.numChannels )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------
    Wait for the hardware to indicate it's free for a
    new transfer.
    -------------------------------------------------*/
    stopSequence();
    while ( STRT::get( mPeriph ) || EOC::get( mPeriph ) )
    {
      continue;
    }

    disableInterrupts();
    {
      /*-------------------------------------------------
      Continuous or polling conversion?
      -------------------------------------------------*/
      switch( sequence.mode )
      {
        case SamplingMode::ONE_SHOT:
          CONT::clear( mPeriph, CR2_CONT );
          break;

        case SamplingMode::CONTINUOUS:
          CONT::set( mPeriph, CR2_CONT );
          break;

        case SamplingMode::TRIGGER:
        default:
          return Chimera::Status::NOT_SUPPORTED;
          break;
      }

      /*-------------------------------------------------
      Data will be transferred in the interrupts
      -------------------------------------------------*/
      EOCIE::set( mPeriph, CR1_EOCIE );
      EOCS::set( mPeriph, CR2_EOCS );
      Thor::LLD::INT::clearPendingIRQ( Resource::IRQSignals[ mResourceIndex ] );

      /*-------------------------------------------------
      Setup the sequence registers
      -------------------------------------------------*/
      Reg32_t totalSize = 0;

      for( size_t idx = 0; idx < sequence.numChannels; idx++ )
      {
        Channel next = ( *sequence.channels )[ idx ];

        /* Max of 16 channels can be queued */
        if( idx >= 16 || !( next < Channel::NUM_OPTIONS ) )
        {
          break;
        }
        else
        {
          totalSize++;
        }

        /* Write to the appropriate register */
        Reg32_t chNum = EnumValue( next );
        if ( idx <= 5 )
        {
          auto chOffset  = 0;
          auto chPos     = ( idx - chOffset ) * SQR1_BIT_Wid;
          Reg32_t regVal = ( chNum & SQR3_BIT_Msk ) << chPos;
          Reg32_t curVal = SQR3_ALL::get( mPeriph );

          curVal &= ~( SQR3_BIT_Msk << chPos );
          curVal |= regVal;

          SQR3_ALL::set( mPeriph, curVal );
        }
        else if( idx <= 11 )
        {
          auto chOffset  = 5;
          auto chPos     = ( idx - chOffset ) * SQR1_BIT_Wid;
          Reg32_t regVal = ( chNum & SQR2_BIT_Msk ) << chPos;
          Reg32_t curVal = SQR2_ALL::get( mPeriph );

          curVal &= ~( SQR2_BIT_Msk << chPos );
          curVal |= regVal;

          SQR2_ALL::set( mPeriph, curVal );
        }
        else if( idx <= 15 )
        {
          auto chOffset  = 11;
          auto chPos     = ( idx - chOffset ) * SQR1_BIT_Wid;
          Reg32_t regVal = ( chNum & SQR1_BIT_Msk ) << chPos;
          Reg32_t curVal = SQR1_ALL::get( mPeriph );

          curVal &= ~( SQR1_BIT_Msk << chPos );
          curVal |= regVal;

          SQR1_ALL::set( mPeriph, curVal );
        }
      }

      /*-------------------------------------------------
      Let the ADC know how many channels to sequence
      -------------------------------------------------*/
      L::set( mPeriph, totalSize << SQR1_L_Pos );
    }
    enableInterrupts();

    return Chimera::Status::OK;
  }


  Chimera::ADC::Sample Driver::sampleChannel( const Chimera::ADC::Channel channel )
  {
    using namespace Chimera::ADC;

    /*-------------------------------------------------
    Wait for the hardware to indicate it's free for a
    new transfer.

    This may have issues if a continuous transfer is
    running in the background. Access to these bits are
    not atomic, so stopping an existing conversion may
    be necessary in the future.
    -------------------------------------------------*/
    while ( STRT::get( mPeriph ) || EOC::get( mPeriph ) )
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
    Perform the conversion. Debuggers beware! Stepping
    through this section may cause your debugger to
    read the DATA register behind the scenes, clearing
    the EOC bit and causing this while loop to become
    infinite!
    -------------------------------------------------*/
    Sample measurement;

    disableInterrupts();
    {
      startSequence();
      while ( !EOC::get( mPeriph ) )
      {
        /* This bit is later cleared by the DATA register read */
        continue;
      }
      stopSequence();

      /*-------------------------------------------------
      Consume the measurement
      -------------------------------------------------*/
      measurement.counts = DATA::get( mPeriph );
      measurement.us     = Chimera::micros();

      /*-------------------------------------------------
      Clear out any ISRs that may have fired
      -------------------------------------------------*/
      Thor::LLD::INT::clearPendingIRQ( Resource::IRQSignals[ mResourceIndex ] );
    }
    enableInterrupts();

    return measurement;
  }


  float Driver::toVoltage( const Chimera::ADC::Sample sample )
  {
    /*-------------------------------------------------
    Get the current configured resolution
    -------------------------------------------------*/
    Reg32_t resolution = RES::get( mPeriph ) >> CR1_RES_Pos;
    float fRes = 0.0;

    switch( resolution )
    {
      case 0: // 12-bit
        fRes = 4096;
        break;

      case 1: // 10-bit
        fRes = 1024;
        break;

      case 2: // 8-bit
        fRes = 256;
        break;

      case 3: // 6-bit
        fRes = 64;
        break;

      default:
        return 0.0f;
        break;
    }

    /*-------------------------------------------------
    Calculate the output voltage
    -------------------------------------------------*/
    float vSense = ( static_cast<float>( PRJ_ADC_VREF ) / fRes ) * static_cast<float>( sample.counts );
    return vSense;
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
    STRT::clear( mPeriph, SR_STRT );
  }


  void Driver::IRQHandler()
  {
    using namespace Chimera::ADC;
    using namespace Chimera::Interrupt;
    using namespace Chimera::Peripheral;
    using namespace Chimera::Thread;

    bool wakeUserTask = false;
    TaskMsg tskMsg = ITCMsg::TSK_MSG_NOP;

    SEGGER_SYSVIEW_RecordEnterISR();

    /*-------------------------------------------------
    Data Available?
    -------------------------------------------------*/
    if( EOC::get( mPeriph ) )
    {
      /* Pull out the data */
      Sample measurement;
      measurement.counts = DATA::get( mPeriph ); // Also clears EOC
      measurement.us = Chimera::micros();

      /* Get the channel currently being measured */
      uint32_t channel = 0;
      uint32_t offset = 0;
      uint32_t maxIdx = L::get( mPeriph ) >> SQR1_L_Pos;

      if( mSequenceIdx < 6 )
      {
        offset = ( mSequenceIdx - 0 ) * SQR1_BIT_Wid;
        channel = ( SQR3_ALL::get( mPeriph ) >> offset ) & SQR1_BIT_Msk;
      }
      else if( mSequenceIdx < 12 )
      {
        offset = ( mSequenceIdx - 6 ) * SQR1_BIT_Wid;
        channel = ( SQR2_ALL::get( mPeriph ) >> offset ) & SQR1_BIT_Msk;
      }
      else if( mSequenceIdx < 16 )
      {
        offset = ( mSequenceIdx - 12 ) * SQR1_BIT_Wid;
        channel = ( SQR1_ALL::get( mPeriph ) >> offset ) & SQR1_BIT_Msk;
      }

      /* Push to the queue */
      if( ( channel < ADC1_Queue.size() ) && ADC1_Queue[ channel ]->available_from_unlocked() )
      {
        ADC1_Queue[ channel ]->push_from_unlocked( measurement );
      }

      /* Update the sequence counter */
      mSequenceIdx++;
      if( mSequenceIdx >= maxIdx )
      {
        mSequenceIdx = 0;
      }

      wakeUserTask = true;
      tskMsg |= ITCMsg::TSK_MSG_ISR_DATA_READY;
    }

    /*-------------------------------------------------
    Overrun error?
    -------------------------------------------------*/
    if( OVR::get( mPeriph ) )
    {
      OVR::clear( mPeriph, SR_OVR );

      /*-------------------------------------------------
      If not using DMA transfers, the conversion can run
      faster than software can process. Simply re-trigger
      the ADC to continue where it left off. (RM 13.8.2)
      -------------------------------------------------*/
      if( !DMA::get( mPeriph ) && EOCS::get( mPeriph ) )
      {
        SWSTART::set( mPeriph, CR2_SWSTART );
      }
      else
      {
        wakeUserTask = true;
        tskMsg |= ITCMsg::TSK_MSG_ISR_ERROR;
      }
    }

    /*-------------------------------------------------
    Wake up the user thread
    -------------------------------------------------*/
    if( wakeUserTask )
    {
      sendTaskMsg( INT::getUserTaskId( Type::PERIPH_ADC ), tskMsg, TIMEOUT_DONT_WAIT );
    }

    SEGGER_SYSVIEW_RecordExitISR();
  }
}    // namespace Thor::LLD::ADC

#endif /* TARGET_STM32L4 && THOR_DRIVER_ADC */
