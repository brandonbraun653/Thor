/******************************************************************************
 *  File Name:
 *    hw_adc_driver.cpp
 *
 *  Description:
 *    Implements the LLD interface to the ADC hardware
 *
 *  2021-2023 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/utility>
#include <Chimera/adc>
#include <Chimera/algorithm>
#include <Chimera/common>
#include <Chimera/thread>
#include <Chimera/utility>
#include <Thor/cfg>
#include <Thor/dma>
#include <Thor/lld/interface/inc/adc>
#include <Thor/lld/interface/inc/dma>
#include <Thor/lld/interface/inc/interrupt>
#include <Thor/lld/interface/inc/rcc>
#include <Thor/lld/interface/inc/timer>
#include <cstring>
#include <limits>

#if defined( THOR_ADC ) && defined( TARGET_STM32F4 )
/*-----------------------------------------------------------------------------
This driver expects some project side constants to be defined for helping with
a few calculations.

#define PRJ_ADC_VREF  (x.yzf)
-----------------------------------------------------------------------------*/
#include "thor_adc_prj_config.hpp"

namespace Thor::LLD::ADC
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  /**
   * @brief Internal voltage reference, address of parameter VREFINT_CAL
   *
   * VrefInt ADC raw data acquired at temperature 30 DegC (tolerance: +-5 DegC)
   * Vref+ = 3.0 V (tolerance: +-10 mV).
   */
  static const uint16_t *const VREFINT_CAL_ADDR = reinterpret_cast<uint16_t *>( 0x1FFF7A2AU );

  /**
   * @brief Internal analog voltage reference (Vref+)
   *
   * Value with which temperature sensor has been calibrated in production
   * (tolerance: +-10 mV) (unit: mV).
   */
  static constexpr float VREFINT_CAL_VREF_MV = 3300.0f;

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static float s_adc_vdda = 0.0f; /**< */

  /*---------------------------------------------------------------------------
  Driver Implementation
  ---------------------------------------------------------------------------*/
  Driver::Driver() :
      mPeriph( nullptr ), mCommon( nullptr ), mResourceIndex( INVALID_RESOURCE_INDEX ), mCfg( {} ),
      mSeqCfg( {} ), mDMAPipeID( 0 )
  {
  }


  Driver::~Driver()
  {
  }


  Chimera::Status_t Driver::attach( RegisterMap *const peripheral )
  {
    /*-------------------------------------------------------------------------
    Get peripheral descriptor settings
    -------------------------------------------------------------------------*/
    mPeriph        = peripheral;
    mCommon        = ADC_COMMON;
    mResourceIndex = getResourceIndex( reinterpret_cast<std::uintptr_t>( peripheral ) );

    /*-------------------------------------------------------------------------
    Handle the ISR configuration
    -------------------------------------------------------------------------*/
    Thor::LLD::INT::disableIRQ( Resource::IRQSignals[ mResourceIndex ] );
    Thor::LLD::INT::clearPendingIRQ( Resource::IRQSignals[ mResourceIndex ] );
    Thor::LLD::INT::setPriority( Resource::IRQSignals[ mResourceIndex ], Thor::LLD::INT::ADC_IT_PREEMPT_PRIORITY, 0u );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::initialize( const Chimera::ADC::DriverConfig &cfg )
  {
    using namespace Chimera::DMA;
    using namespace Chimera::Peripheral;
    using namespace Thor::LLD::RCC;

    /*-------------------------------------------------------------------------
    Only enable the clock because the reset signals will totally destroy other
    peripheral settings.
    -------------------------------------------------------------------------*/
    this->clockEnable();

    /*-------------------------------------------------------------------------
    Select the clock prescaler. On the F4, the ADC is
    driven directly from the APB2 clock. (RM Fig.13)
    -------------------------------------------------------------------------*/
    switch ( cfg.clockPrescale )
    {
      case Chimera::ADC::PreScaler::DIV_2:
        ADCPRE::set( ADC_COMMON, 0 << CCR_ADCPRE_Pos );
        break;

      case Chimera::ADC::PreScaler::DIV_4:
        ADCPRE::set( ADC_COMMON, 1 << CCR_ADCPRE_Pos );
        break;

      case Chimera::ADC::PreScaler::DIV_6:
        ADCPRE::set( ADC_COMMON, 2 << CCR_ADCPRE_Pos );
        break;

      case Chimera::ADC::PreScaler::DIV_8:
      default:
        ADCPRE::set( ADC_COMMON, 3 << CCR_ADCPRE_Pos );
        break;
    };

    /*-------------------------------------------------------------------------
    Enable the temperature sensor and internal VRef
    -------------------------------------------------------------------------*/
    TSVREFE::set( ADC_COMMON, CCR_TSVREFE );

    /*-------------------------------------------------------------------------
    Assign the ADC resolution
    -------------------------------------------------------------------------*/
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

    /*-------------------------------------------------------------------------
    Reset the channel sample times to defaults
    -------------------------------------------------------------------------*/
    for ( size_t idx = 0; idx < NUM_ADC_CHANNELS_PER_PERIPH; idx++ )
    {
      setSampleTime( static_cast<Chimera::ADC::Channel>( idx ), SampleTime::SMP_480 );
    }

    /*-------------------------------------------------------------------------
    Configure oversampling if needed (not supported yet!)
    -------------------------------------------------------------------------*/
    RT_HARD_ASSERT( cfg.overSampleRate == Chimera::ADC::OverSampler::OS_NONE );

    /*-------------------------------------------------------------------------
    Configure injected conversion registers
    -------------------------------------------------------------------------*/
    JDISCEN::set( mPeriph, CR1_JDISCEN );    // Allow injected conversions
    JAUTO::clear( mPeriph, CR1_JAUTO );      // Disable automatic injected conversions

    /*-------------------------------------------------------------------------
    Configure DMA transfers
    -------------------------------------------------------------------------*/
    DDS::clear( mPeriph, CR2_DDS );
    DMA::clear( mPeriph, CR2_DMA );

    /*-------------------------------------------------------------------------
    Configure the ADC DMA pipe
    -------------------------------------------------------------------------*/
    PipeConfig dmaCfg;
    dmaCfg.clear();
    dmaCfg.dstAlignment       = Alignment::HALF_WORD;
    dmaCfg.srcAlignment       = Alignment::WORD;
    dmaCfg.direction          = Direction::PERIPH_TO_MEMORY;
    dmaCfg.mode               = Mode::CIRCULAR;
    dmaCfg.periphAddr         = reinterpret_cast<std::uintptr_t>( &mPeriph->DR );
    dmaCfg.priority           = Priority::VERY_HIGH;
    dmaCfg.resourceIndex      = Thor::LLD::DMA::getResourceIndex( Resource::DMASignals[ mResourceIndex ] );
    dmaCfg.channel            = static_cast<size_t>( Thor::LLD::DMA::getChannel( Resource::DMASignals[ mResourceIndex ] ) );
    dmaCfg.threshold          = FifoThreshold::NONE;
    dmaCfg.persistent         = true;
    dmaCfg.wakeUserOnComplete = false;

    mDMAPipeID = Chimera::DMA::constructPipe( dmaCfg );
    RT_HARD_ASSERT( mDMAPipeID != Chimera::DMA::INVALID_REQUEST );

    /*-------------------------------------------------------------------------
    Bring the ADC out of power down mode
    -------------------------------------------------------------------------*/
    ADON::set( mPeriph, CR2_ADON );
    Chimera::blockDelayMilliseconds( 5 );

    /*-------------------------------------------------------------------------
    Sample the internal voltage reference to calculate the real VDDA+ present
    on the MCU pin. This will always be present on Channel 0.
    -------------------------------------------------------------------------*/
    if( mPeriph == ADC1_PERIPH )
    {
      size_t startTime  = Chimera::millis();
      float  numSamples = 0.0f;
      s_adc_vdda        = 0.0f;

      while ( Chimera::millis() - startTime < Chimera::Thread::TIMEOUT_100MS )
      {
        Chimera::ADC::Sample vref_sample  = sampleChannel( Chimera::ADC::Channel::ADC_CH_17 );
        const float          vrefint_data = static_cast<float>( vref_sample.counts );
        const float          vrefint_cal  = static_cast<float>( *VREFINT_CAL_ADDR );

        s_adc_vdda += ( VREFINT_CAL_VREF_MV * ( vrefint_cal / vrefint_data ) ) / 1000.0f;
        numSamples++;
      }

      s_adc_vdda /= numSamples;
    }

    return Chimera::Status::OK;
  }


  Chimera::ADC::Sample Driver::sampleChannel( const Chimera::ADC::Channel channel )
  {
    using namespace Chimera::ADC;

    /*-------------------------------------------------------------------------
    Wait for the hardware to indicate it's free for a new transfer. Use the
    injected channels to allow interruption of free-running conversions.
    -------------------------------------------------------------------------*/
    while ( JSTRT::get( mPeriph ) || JEOC::get( mPeriph ) )
    {
      continue;
    }

    /*-------------------------------------------------------------------------
    Set up the conversion mode
    -------------------------------------------------------------------------*/
    JDISCEN::clear( mPeriph, CR1_JDISCEN );    // Force sequentially ordered sampling

    /*-------------------------------------------------------------------------
    Configure the channel to be measured
    -------------------------------------------------------------------------*/
    JSQR_ALL::set( mPeriph, 0u );    // Sequence length of 1, no external triggers.
    SQ1::set( mPeriph, EnumValue( channel ) << SQR3_SQ1_Pos );

    /*-------------------------------------------------------------------------
    Perform the conversion. Debuggers beware! Stepping through this section may
    cause your debugger to read the DATA register behind the scenes, clearing
    the EOC bit and causing this while loop to become infinite!
    -------------------------------------------------------------------------*/
    Sample measurement;

    JSWSTART::set( mPeriph, CR2_JSWSTART );
    while ( !JEOC::get( mPeriph ) )
    {
      continue;
    }

    /*-------------------------------------------------------------------------
    Read the data register and store the result
    -------------------------------------------------------------------------*/
    measurement.counts = JDATA1::get( mPeriph );
    measurement.us     = Chimera::micros();

    /*-------------------------------------------------------------------------
    Clear the EOC flags
    -------------------------------------------------------------------------*/
    JEOC::clear( mPeriph, SR_JEOC );
    JSTRT::clear( mPeriph, SR_JSTRT );

    return measurement;
  }


  float Driver::resolution() const
  {
    Reg32_t resolution = RES::get( mPeriph ) >> CR1_RES_Pos;
    switch ( resolution )
    {
      case 0:    // 12-bit
        return 4096.0f;

      case 1:    // 10-bit
        return 1024.0f;

      case 2:    // 8-bit
        return 256.0f;

      case 3:    // 6-bit
        return 64.0f;

      default:
        return 0.0f;
    }
  }


  float Driver::analogReference() const
  {
    return s_adc_vdda;
  }


  float Driver::toVoltage( const Chimera::ADC::Sample sample )
  {
    /*-----------------------------------------------------------------------
    Use the current resolution to interpret the ADC sample
    -----------------------------------------------------------------------*/
    float fRes = resolution();
    RT_DBG_ASSERT( fRes != 0.0f );

    return ( s_adc_vdda * static_cast<float>( sample.counts ) ) / fRes;
  }


  Chimera::Status_t Driver::monitorChannel( const Chimera::ADC::WatchdogConfig &cfg )
  {
    using namespace Chimera::ADC;

    /*---------------------------------------------------------------------------
    Input Protections
    ---------------------------------------------------------------------------*/
    if ( cfg.wdgChannel == Chimera::ADC::Watchdog::ANALOG_0 )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }

    /*-------------------------------------------------------------------------
    Register the ISR handler
    -------------------------------------------------------------------------*/
    onInterrupt( Interrupt::ANALOG_WD, cfg.callback );

    /*-------------------------------------------------------------------------
    Enable the desired channel for monitoring
    -------------------------------------------------------------------------*/
    uint32_t hi_lo_data = ( ( cfg.highThreshold & 0xFF ) << 16u ) | ( cfg.lowThreshold & 0xFF );

    // TODO: Finish the watchdog implementation
    // if ( cfg.wdgChannel == Chimera::ADC::Watchdog::ANALOG_1 )
    // {
    //   AWD2CH::setbit( mPeriph, ( 1u << EnumValue( cfg.adcChannel ) ) );
    //   TR2_ALL::set( mPeriph, hi_lo_data );
    //   AWD2IE::setbit( mPeriph, IER_AWD2IE );
    // }
    // else if ( cfg.wdgChannel == Chimera::ADC::Watchdog::ANALOG_2 )
    // {
    //   AWD3CH::setbit( mPeriph, ( 1u << EnumValue( cfg.adcChannel ) ) );
    //   TR3_ALL::set( mPeriph, hi_lo_data );
    //   AWD3IE::setbit( mPeriph, IER_AWD3IE );
    // }

    return Chimera::Status::FAIL;
  }


  Chimera::Status_t Driver::setSampleTime( const Chimera::ADC::Channel ch, const SampleTime time )
  {
    using namespace Chimera::ADC;

    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if ( !( EnumValue( ch ) < NUM_ADC_CHANNELS_PER_PERIPH ) )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------------------------------
    Apply sample times to the registers
    -------------------------------------------------------------------------*/
    size_t chNum = static_cast<size_t>( ch );

    if ( ch < Channel::ADC_CH_10 )
    {
      auto    chPos  = static_cast<size_t>( chNum ) * SMPRx_BIT_Wid;
      Reg32_t regVal = static_cast<size_t>( time ) << chPos;
      Reg32_t curVal = SMPR1_ALL::get( mPeriph );

      curVal &= ~( SMPRx_BIT_Msk << chPos );
      curVal |= regVal;

      SMPR1_ALL::set( mPeriph, curVal );
    }
    else
    {
      auto    chOffset = static_cast<size_t>( Channel::ADC_CH_10 );
      auto    chPos    = static_cast<size_t>( chNum ) - chOffset;
      Reg32_t regVal   = static_cast<size_t>( time ) << chPos;
      Reg32_t curVal   = SMPR2_ALL::get( mPeriph );

      curVal &= ~( SMPRx_BIT_Msk << chPos );
      curVal |= regVal;

      SMPR2_ALL::set( mPeriph, curVal );
    }

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::setupSequence( const Chimera::ADC::SequenceInit &sequence )
  {
    using namespace Chimera::ADC;

    /*-------------------------------------------------------------------------
    Input protections
    -------------------------------------------------------------------------*/
    if ( !sequence.channels || !sequence.numChannels )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------------------------------
    Wait for the hardware to indicate it's free for a new transfer.
    -------------------------------------------------------------------------*/
    stopSequence();
    disableInterrupts();
    {
      /*-----------------------------------------------------------------------
      Select the conversion mode
      -----------------------------------------------------------------------*/
      switch ( sequence.seqMode )
      {
        case SamplingMode::ONE_SHOT:
        case SamplingMode::TRIGGER:
          CONT::clear( mPeriph, CR2_CONT );
          break;

        case SamplingMode::CONTINUOUS:
          CONT::set( mPeriph, CR2_CONT );
          break;

        default:
          return Chimera::Status::NOT_SUPPORTED;
          break;
      }
      mSeqCfg = sequence;

      /*-----------------------------------------------------------------------
      Set up the hardware sequence sampling registers
      -----------------------------------------------------------------------*/
      Reg32_t totalSize = 0;
      mDMASampleBuffer.channelSequence.fill( Chimera::ADC::Channel::UNKNOWN );

      for ( size_t idx = 0; idx < sequence.numChannels; idx++ )
      {
        Channel next = ( *sequence.channels )[ idx ];

        /*---------------------------------------------------------------------
        Maximum of 16 channels may be sampled in sequence
        ---------------------------------------------------------------------*/
        if ( idx >= 16 || !( next < Channel::NUM_OPTIONS ) )
        {
          break;
        }
        else
        {
          totalSize++;
          mDMASampleBuffer.channelSequence[ idx ] = next;
        }

        /*---------------------------------------------------------------------
        Configure the appropriate register
        ---------------------------------------------------------------------*/
        Reg32_t chNum  = EnumValue( next );
        size_t  chPos  = 0;
        Reg32_t regVal = 0;
        Reg32_t curVal = 0;
        Reg32_t chOffset = 0;

        if ( idx <= 5 )
        {
          chOffset = 0;
          chPos    = ( idx - chOffset ) * SQR1_BIT_Wid;
          regVal   = ( chNum & SQR3_BIT_Msk ) << chPos;
          curVal   = SQR3_ALL::get( mPeriph );

          curVal &= ~( SQR3_BIT_Msk << chPos );
          curVal |= regVal;

          SQR3_ALL::set( mPeriph, curVal );
        }
        else if ( idx <= 11 )
        {
          chOffset = 5;
          chPos    = ( idx - chOffset ) * SQR1_BIT_Wid;
          regVal   = ( chNum & SQR2_BIT_Msk ) << chPos;
          curVal   = SQR2_ALL::get( mPeriph );

          curVal &= ~( SQR2_BIT_Msk << chPos );
          curVal |= regVal;

          SQR2_ALL::set( mPeriph, curVal );
        }
        else if ( idx <= 15 )
        {
          chOffset = 11;
          chPos    = ( idx - chOffset ) * SQR1_BIT_Wid;
          regVal   = ( chNum & SQR1_BIT_Msk ) << chPos;
          curVal   = SQR1_ALL::get( mPeriph );

          curVal &= ~( SQR1_BIT_Msk << chPos );
          curVal |= regVal;

          SQR1_ALL::set( mPeriph, curVal );
        }
      }

      /*-----------------------------------------------------------------------
      Set the number of conversions to perform
      -----------------------------------------------------------------------*/
      L::set( mPeriph, totalSize << SQR1_L_Pos );
      SCAN::set( mPeriph, CR1_SCAN );
    }
    enableInterrupts();

    return Chimera::Status::OK;
  }


  void Driver::startSequence()
  {
    using namespace Chimera::ADC;
    using namespace Chimera::DMA;

    /*-------------------------------------------------------------------------
    Wait for the hardware to not be busy
    -------------------------------------------------------------------------*/
    while ( SWSTART::get( mPeriph ) )
    {
      continue;
    }

    /*-------------------------------------------------------------------------
    Prime the DMA pipe to accept new transfer requests
    -------------------------------------------------------------------------*/
    PipeTransfer cfg;
    cfg.isrCallback = TransferCallback::create<Driver, &Driver::dma_isr_transfer_complete_callback>( *this );
    cfg.pipe        = mDMAPipeID;
    cfg.size        = ( L::get( mPeriph ) >> SQR1_L_Pos ) + 1u;
    cfg.addr        = reinterpret_cast<std::uintptr_t>( mDMASampleBuffer.rawSamples.data() );

    /* Won't start the transfer, but does prime the DMA hw for the ADC */
    Chimera::DMA::transfer( cfg );

    /*-------------------------------------------------------------------------
    Enable the DMA functionality on the ADC peripheral
    -------------------------------------------------------------------------*/
    DDS::set( mPeriph, CR2_DDS );    // Send new DMA requests at end of conversion
    DMA::set( mPeriph, CR2_DMA );    // Enable DMA mode

    /*-------------------------------------------------------------------------
    Select which trigger to use to start a new ADC sample
    -------------------------------------------------------------------------*/
    if ( mSeqCfg.seqMode == SamplingMode::TRIGGER )
    {
      switch ( mSeqCfg.trigMode )
      {
        case TriggerMode::RISING_EDGE:
          EXTEN::set( mPeriph, ( 0x1 << CR2_EXTEN_Pos ) );
          break;

        case TriggerMode::FALLING_EDGE:
          EXTEN::set( mPeriph, ( 0x2 << CR2_EXTEN_Pos ) );
          break;

        case TriggerMode::BOTH_EDGE:
          EXTEN::set( mPeriph, ( 0x3 << CR2_EXTEN_Pos ) );
          break;

        default:
          RT_HARD_ASSERT( false );
          break;
      };

      RT_HARD_ASSERT( mSeqCfg.trigChannel <= NUM_ADC_EXT_TRIG_CHANNELS );
      EXTSEL::set( mPeriph, ( mSeqCfg.trigChannel << CR2_EXTSEL_Pos ) );
    }
    else /* Software Trigger or Continuous */
    {
      EXTEN::clear( mPeriph, CR2_EXTEN );
    }

    /*-------------------------------------------------------------------------
    Set the start bit, then wait for HW to clear it. If stuck here, the ADC
    probably isn't enabled.
    -------------------------------------------------------------------------*/
    SWSTART::set( mPeriph, CR2_SWSTART );
    while ( SWSTART::get( mPeriph ) )
    {
      continue;
    }
  }


  void Driver::stopSequence()
  {
    using namespace Chimera::ADC;

    /*-------------------------------------------------------------------------
    You can't stop a single in-progress conversion, but it can be prevented
    from continuous conversion.
    -------------------------------------------------------------------------*/
    CONT::clear( mPeriph, CR2_CONT );
    STRT::clear( mPeriph, SR_STRT );
    JSWSTART::clear( mPeriph, CR2_JSWSTART );

    while ( STRT::get( mPeriph ) || EOC::get( mPeriph ) || JSWSTART::get( mPeriph ) || JEOC::get( mPeriph ) )
    {
      continue;
    }

    /*-------------------------------------------------------------------------
    Stop generating DMA requests on ADC side (but leave DMA configured)
    -------------------------------------------------------------------------*/
    DMA::clear( mPeriph, CR2_DMA );
    OVR::clear( mPeriph, SR_OVR );

    /*-------------------------------------------------------------------------
    Stop listening to hardware trigger signals
    -------------------------------------------------------------------------*/
    if ( mSeqCfg.seqMode == SamplingMode::TRIGGER )
    {
      EXTEN::clear( mPeriph, CR2_EXTEN );
    }
  }


  void Driver::syncSequence()
  {
    /*-------------------------------------------------------------------------
    Select the queue associated with this instance
    -------------------------------------------------------------------------*/
    Thor::LLD::ADC::PeriphQueue *queue = nullptr;
    switch ( getChannel( reinterpret_cast<std::uintptr_t>( mPeriph ) ) )
    {
      case Chimera::ADC::Peripheral::ADC_0:
        queue = &Thor::LLD::ADC::ADC1_Queue;
        break;

      case Chimera::ADC::Peripheral::ADC_1:
        queue = &Thor::LLD::ADC::ADC2_Queue;
        break;

      case Chimera::ADC::Peripheral::ADC_2:
        queue = &Thor::LLD::ADC::ADC3_Queue;
        break;

      default:
        RT_HARD_ASSERT( false );
        break;
    }

    /*-------------------------------------------------------------------------
    Fill the queue with all the sample data
    -------------------------------------------------------------------------*/
    for ( size_t idx = 0; idx < mDMASampleBuffer.channelSequence.size(); idx++ )
    {
      /*---------------------------------------------------------------------
      Is the current sequence channel valid?
      ---------------------------------------------------------------------*/
      auto channel = mDMASampleBuffer.channelSequence[ idx ];
      if ( channel == Chimera::ADC::Channel::UNKNOWN )
      {
        break;
      }

      /*---------------------------------------------------------------------
      Move data into the queue for this channel
      ---------------------------------------------------------------------*/
      if ( !( ( *queue )[ EnumValue( channel ) ]->full() ) )
      {
        Chimera::ADC::Sample sample;
        sample.us     = Chimera::micros();
        sample.counts = mDMASampleBuffer.rawSamples[ idx ];

        ( *queue )[ EnumValue( channel ) ]->push( sample );
      }
    }
  }


  void Driver::ISRHandler()
  {
    /*-------------------------------------------------------------------------
    Did any of the watchdog monitors trip?
    -------------------------------------------------------------------------*/
    if ( ( AWD::get( mPeriph ) ) && mCallbacks[ EnumValue( Chimera::ADC::Interrupt::ANALOG_WD ) ] )
    {
      /* ACK the event, only writing the specific bits to clear */
      AWD::clear( mPeriph, SR_AWD );

      /* Populate the data for the ADC handler */
      Chimera::ADC::InterruptDetail isrData;
      isrData.clear();
      isrData.isr = Chimera::ADC::Interrupt::ANALOG_WD;

      /* Invoke the user handler */
      mCallbacks[ EnumValue( Chimera::ADC::Interrupt::ANALOG_WD ) ]( isrData );
    }
  }


  void Driver::dma_isr_transfer_complete_callback( const Chimera::DMA::TransferStats &stats )
  {
    if ( !stats.error && mCallbacks[ EnumValue( Chimera::ADC::Interrupt::EOC_SEQUENCE ) ] )
    {
      /* Populate the data for the ADC handler */
      Chimera::ADC::InterruptDetail isrData;
      isrData.clear();

      isrData.isr         = Chimera::ADC::Interrupt::EOC_SEQUENCE;
      isrData.samples     = mDMASampleBuffer.rawSamples.data();
      isrData.num_samples = static_cast<uint16_t>( stats.size );
      isrData.resolution  = this->resolution();
      isrData.vref        = this->analogReference();

      /* Invoke the user handler */
      mCallbacks[ EnumValue( Chimera::ADC::Interrupt::EOC_SEQUENCE ) ]( isrData );
    }
  }
}    // namespace Thor::LLD::ADC

#endif /* THOR_ADC && TARGET_STM32F4 */
