/********************************************************************************
 *  File Name:
 *    hw_adc_driver.cpp
 *
 *  Description:
 *    Implements the LLD interface to the ADC hardware.
 *
 *  2020-2022 | Brandon Braun | brandonbraun653@gmail.com
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
#include <Thor/dma>
#include <Thor/lld/interface/inc/adc>
#include <Thor/lld/interface/inc/dma>
#include <Thor/lld/interface/inc/interrupt>
#include <Thor/lld/interface/inc/rcc>
#include <Thor/lld/interface/inc/timer>


#if defined( THOR_LLD_ADC ) && defined( TARGET_STM32L4 )
/*-----------------------------------------------------------------------------
This driver expects some project side constants to be defined for helping with
a few calculations.

#define PRJ_ADC_VREF  (x.yzf)
-----------------------------------------------------------------------------*/
#include "thor_adc_prj_config.hpp"


namespace Thor::LLD::ADC
{
  /*---------------------------------------------------------------------------
  Static Variables
  ---------------------------------------------------------------------------*/
  static Driver s_adc_drivers[ NUM_ADC_PERIPHS ];

  /*---------------------------------------------------------------------------
  Driver Implementation
  ---------------------------------------------------------------------------*/
  Driver::Driver() :
      mPeriph( nullptr ), mCommon( nullptr ), mConversionInProgress( false ), mResourceIndex( INVALID_RESOURCE_INDEX ),
      mSequenceIdx( 0 ), mCfg( {} ), mDMAPipeID( 0 ), mDMAPaused( false )
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
    Handle the ISR configuration. Not actually used, but it puts the system in
    a known and controlled state.
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
    Configure the clock (16.4.3)
    -------------------------------------------------------------------------*/
    if ( cfg.clockSource != Chimera::Clock::Bus::SYSCLK )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    // Select the clock source
    ADCSEL::set( RCC1_PERIPH, Config::CCIPR::ADCSEL_SYS_CLOCK );

    // Turn on the core peripheral clock
    this->clockEnable();
    this->reset();

    // Set the clock mode to be asynchronous
    CKMODE::set( ADC_COMMON, 0 );

    // Set the prescaler. Chimera enums line up with the bit shift for now, but may not later.
    // Don't try to simplify this to a smaller statement.
    switch ( cfg.clockPrescale )
    {
      case Chimera::ADC::Prescaler::DIV_1:
        PRESC::set( ADC_COMMON, 0 << CCR_PRESC_Pos );
        break;

      case Chimera::ADC::Prescaler::DIV_2:
        PRESC::set( ADC_COMMON, 1 << CCR_PRESC_Pos );
        break;

      case Chimera::ADC::Prescaler::DIV_4:
        PRESC::set( ADC_COMMON, 2 << CCR_PRESC_Pos );
        break;

      case Chimera::ADC::Prescaler::DIV_6:
        PRESC::set( ADC_COMMON, 3 << CCR_PRESC_Pos );
        break;

      case Chimera::ADC::Prescaler::DIV_8:
        PRESC::set( ADC_COMMON, 4 << CCR_PRESC_Pos );
        break;

      case Chimera::ADC::Prescaler::DIV_10:
        PRESC::set( ADC_COMMON, 5 << CCR_PRESC_Pos );
        break;

      case Chimera::ADC::Prescaler::DIV_12:
        PRESC::set( ADC_COMMON, 6 << CCR_PRESC_Pos );
        break;

      case Chimera::ADC::Prescaler::DIV_16:
        PRESC::set( ADC_COMMON, 7 << CCR_PRESC_Pos );
        break;

      case Chimera::ADC::Prescaler::DIV_32:
        PRESC::set( ADC_COMMON, 8 << CCR_PRESC_Pos );
        break;

      case Chimera::ADC::Prescaler::DIV_64:
        PRESC::set( ADC_COMMON, 9 << CCR_PRESC_Pos );
        break;

      case Chimera::ADC::Prescaler::DIV_128:
        PRESC::set( ADC_COMMON, 10 << CCR_PRESC_Pos );
        break;

      case Chimera::ADC::Prescaler::DIV_256:
        PRESC::set( ADC_COMMON, 11 << CCR_PRESC_Pos );
        break;

      default:
        // Prescaler not supported
        break;
    };

    /*-------------------------------------------------------------------------
    Enable the sensor monitors
    -------------------------------------------------------------------------*/
    VBATEN::set( ADC_COMMON, CCR_VBATEN );    // External battery sensor
    TSEN::set( ADC_COMMON, CCR_TSEN );        // Internal temperature sensor
    VREFEN::set( ADC_COMMON, CCR_VREFEN );    // Internal bandgap vref sensor

    /*-------------------------------------------------------------------------
    Bring the ADC out of deep power down
    -------------------------------------------------------------------------*/
    DEEPPWD::clear( mPeriph, CR_DEEPPWD );    // Disable deep power down
    ADVREGEN::set( mPeriph, CR_ADVREGEN );    // Enable the voltage regulator

    // Allow the analog voltage regulator time to boot
    Chimera::delayMilliseconds( 250 );

    /*-------------------------------------------------------------------------
    Assign the ADC resolution
    -------------------------------------------------------------------------*/
    switch ( cfg.resolution )
    {
      case Chimera::ADC::Resolution::BIT_12:
        RES::set( mPeriph, 0 << CFGR_RES_Pos );
        break;

      case Chimera::ADC::Resolution::BIT_10:
        RES::set( mPeriph, 1 << CFGR_RES_Pos );
        break;

      case Chimera::ADC::Resolution::BIT_8:
        RES::set( mPeriph, 2 << CFGR_RES_Pos );
        break;

      case Chimera::ADC::Resolution::BIT_6:
      default:
        RES::set( mPeriph, 3 << CFGR_RES_Pos );
        break;
    };

    /*-------------------------------------------------------------------------
    Reset the channel sample times to defaults
    -------------------------------------------------------------------------*/
    for ( size_t idx = 0; idx < NUM_ADC_CHANNELS_PER_PERIPH; idx++ )
    {
      setSampleTime( static_cast<Chimera::ADC::Channel>( idx ), SampleTime::SMP_24P5 );
    }

    /*-------------------------------------------------------------------------
    Calibrate the ADC
    -------------------------------------------------------------------------*/
    // Default to single ended inputs for all channels
    DIFSEL::clear( mPeriph, DIFSEL_DIFSEL );

    // Start calibration for single ended inputs
    ADCALDIF::clear( mPeriph, CR_ADCALDIF );
    ADCAL::set( mPeriph, CR_ADCAL );

    // Calibration complete when bit is cleared
    while ( ADCAL::get( mPeriph ) )
    {
      continue;
    }

    /*-------------------------------------------------------------------------
    Configure injected conversion registers that depend on idle hardware
    -------------------------------------------------------------------------*/
    JQDIS::set( mPeriph, CFGR_JQDIS );      // Allow injected transactions to start via SW
    JAUTO::clear( mPeriph, CFGR_JAUTO );    // Auto injected group conversion OFF

    /*-------------------------------------------------------------------------
    Configure the DMA transfers from the ADC's perspective
    -------------------------------------------------------------------------*/
    DMACFG::set( mPeriph, CFGR_DMACFG );    // Circular mode
    DMAEN::clear( mPeriph, CFGR_DMAEN );    // Disable DMA (for now)

    /*-------------------------------------------------------------------------
    Configure the DMA pipe
    -------------------------------------------------------------------------*/
    PipeConfig dmaCfg;
    dmaCfg.clear();
    dmaCfg.alignment          = Alignment::HALF_WORD;
    dmaCfg.direction          = Direction::PERIPH_TO_MEMORY;
    dmaCfg.mode               = Mode::CIRCULAR;
    dmaCfg.periphAddr         = reinterpret_cast<std::uintptr_t>( &mPeriph->DR );
    dmaCfg.priority           = Priority::MEDIUM;
    dmaCfg.resourceIndex      = Thor::LLD::DMA::getResourceIndex( Resource::DMASignals[ mResourceIndex ] );
    dmaCfg.channel            = static_cast<size_t>( Thor::LLD::DMA::getChannel( Resource::DMASignals[ mResourceIndex ] ) );
    dmaCfg.threshold          = FifoThreshold::NONE;
    dmaCfg.persistent         = true;
    dmaCfg.wakeUserOnComplete = false;

    mDMAPipeID = Thor::DMA::constructPipe( dmaCfg );
    mDMAPaused = false;

    /*-------------------------------------------------------------------------
    Enable the ADC
    -------------------------------------------------------------------------*/
    ADEN::set( mPeriph, CR_ADEN );

    // Wait for ADC to signal it's ready, then ACK it via write 1.
    while ( !ADRDY::get( mPeriph ) )
    {
      continue;
    }
    ADRDY::set( mPeriph, ISR_ADRDY );

    return Chimera::Status::OK;
  }


  Chimera::ADC::Sample Driver::sampleChannel( const Chimera::ADC::Channel channel )
  {
    using namespace Chimera::ADC;

    /*-------------------------------------------------------------------------
    Wait for the hardware to indicate it's free for a new transfer. Use the
    injected channels to allow interruption of free-running conversions.
    -------------------------------------------------------------------------*/
    JADSTP::set( mPeriph, CR_JADSTP );
    while ( JADSTART::get( mPeriph ) || JADSTP::get( mPeriph ) )
    {
      continue;
    }

    /*-------------------------------------------------------------------------
    Set up the conversion modes
    -------------------------------------------------------------------------*/
    JDISCEN::clear( mPeriph, CFGR_JDISCEN );    // Force sequentially ordered sampling
    JOVSE::clear( mPeriph, CFGR2_JOVSE );       // Disable injected group oversampling

    /*-------------------------------------------------------------------------
    Configure the channel being measured
    -------------------------------------------------------------------------*/
    JSQR_ALL::set( mPeriph, 0 );    // Sequence length of 1, no external triggers.
    JSQ1::set( mPeriph, EnumValue( channel ) << JSQR_JSQ1_Pos );

    /*-------------------------------------------------------------------------
    Disable ISRs as they really aren't needed
    -------------------------------------------------------------------------*/
    IER_ALL::clear( mPeriph, ( IER_JQOVFIE | IER_JEOSIE | IER_JEOCIE ) );

    /*-------------------------------------------------------------------------
    Perform the conversion. Debuggers beware! Stepping through this section may
    cause your debugger to read the DATA register behind the scenes, clearing
    the EOC bit and causing this while loop to become infinite!
    -------------------------------------------------------------------------*/
    Sample measurement;

    /*-----------------------------------------------------------------------
    Kick off the transfer and wait for it to complete
    -----------------------------------------------------------------------*/
    JADSTART::set( mPeriph, CR_JADSTART );
    while ( !JEOC::get( mPeriph ) )
    {
      /* This bit is later cleared by the DATA register read */
      continue;
    }

    /*-----------------------------------------------------------------------
    Consume the measurement
    -----------------------------------------------------------------------*/
    measurement.counts = JDATA1::get( mPeriph );
    measurement.us     = Chimera::micros();

    return measurement;
  }


  float Driver::toVoltage( const Chimera::ADC::Sample sample )
  {
    /*-------------------------------------------------
    Get the current configured resolution
    -------------------------------------------------*/
    Reg32_t resolution = RES::get( mPeriph ) >> CFGR_RES_Pos;
    float fRes         = 0.0;

    switch ( resolution )
    {
      case 0:    // 12-bit
        fRes = 4096.0f;
        break;

      case 1:    // 10-bit
        fRes = 1024.0f;
        break;

      case 2:    // 8-bit
        fRes = 256.0f;
        break;

      case 3:    // 6-bit
        fRes = 64.0f;
        break;

      default:
        return 0.0f;
        break;
    }

    /*-------------------------------------------------
    Calculate the output voltage
    -------------------------------------------------*/
    float vSense = ( static_cast<float>( PRJ_ADC_VREF ) * static_cast<float>( sample.counts ) ) / fRes;
    return vSense;
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


  Chimera::Status_t Driver::setupSequence( const Chimera::ADC::SequenceInit &sequence )
  {
    using namespace Chimera::ADC;

    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if ( !sequence.channels || !sequence.numChannels )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------------------------------
    Wait for hardware to indicate it's free for a new transfer
    -------------------------------------------------------------------------*/
    stopSequence();

    disableInterrupts();
    {
      /*-----------------------------------------------------------------------
      Select continuous or polling conversion
      -----------------------------------------------------------------------*/
      switch ( sequence.mode )
      {
        case SamplingMode::ONE_SHOT:
          CONT::clear( mPeriph, CFGR_CONT );
          break;

        case SamplingMode::CONTINUOUS:
          CONT::set( mPeriph, CFGR_CONT );
          break;

        case SamplingMode::TRIGGER:
        default:
          return Chimera::Status::NOT_SUPPORTED;
          break;
      }

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
        size_t chPos   = 0;
        Reg32_t regVal = 0;
        Reg32_t curVal = 0;

        if ( idx <= 3 )
        {
          chPos  = ( idx * SQR1_BIT_Wid ) + SQR1_SQ1_Pos + idx;
          regVal = ( chNum & SQR1_BIT_Msk ) << chPos;
          curVal = SQR1_ALL::get( mPeriph );

          curVal &= ~( SQR1_BIT_Msk << chPos );
          curVal |= regVal;

          SQR1_ALL::set( mPeriph, curVal );
        }
        else if ( idx <= 8 )
        {
          auto chOffset = 3;
          chPos         = ( idx - chOffset ) * SQR2_BIT_Wid;
          regVal        = ( chNum & SQR2_BIT_Msk ) << chPos;
          curVal        = SQR2_ALL::get( mPeriph );

          curVal &= ~( SQR2_BIT_Msk << chPos );
          curVal |= regVal;

          SQR2_ALL::set( mPeriph, curVal );
        }
        else if ( idx <= 13 )
        {
          auto chOffset = 8;
          chPos         = ( idx - chOffset ) * SQR3_BIT_Wid;
          regVal        = ( chNum & SQR3_BIT_Msk ) << chPos;
          curVal        = SQR3_ALL::get( mPeriph );

          curVal &= ~( SQR3_BIT_Msk << chPos );
          curVal |= regVal;

          SQR3_ALL::set( mPeriph, curVal );
        }
        else if ( idx <= 15 )
        {
          auto chOffset = 13;
          chPos         = ( idx - chOffset ) * SQR4_BIT_Wid;
          regVal        = ( chNum & SQR4_BIT_Msk ) << chPos;
          curVal        = SQR4_ALL::get( mPeriph );

          curVal &= ~( SQR4_BIT_Msk << chPos );
          curVal |= regVal;

          SQR4_ALL::set( mPeriph, curVal );
        }

        // Double check the setting. All register bit masks are the same.
        RT_HARD_ASSERT( ( ( curVal >> chPos ) & SQR1_BIT_Msk ) == EnumValue( next ) );
      }


      /*-----------------------------------------------------------------------
      Let the ADC know how many channels to sequence
      -----------------------------------------------------------------------*/
      L::set( mPeriph, totalSize << SQR1_L_Pos );
    }
    enableInterrupts();

    return Chimera::Status::OK;
  }


  void Driver::startSequence()
  {
    using namespace Chimera::DMA;

    /*-----------------------------------------------------------------------
    Disable interrupts. DMA is the primary data mover.
    -----------------------------------------------------------------------*/
    IER_ALL::clear( mPeriph, ( IER_OVRIE | IER_EOSIE | IER_EOCIE | IER_EOSMPIE | IER_ADRDYIE ) );
    Thor::LLD::INT::clearPendingIRQ( Resource::IRQSignals[ mResourceIndex ] );

    /*-------------------------------------------------------------------------
    Configure the DMA transfer. Assumes each transfer is sized to a half-word.
    -------------------------------------------------------------------------*/
    PipeTransfer cfg;
    cfg.isrCallback = TransferCallback::create<Driver, &Driver::onDMAComplete>( *this );
    cfg.pipe        = mDMAPipeID;
    cfg.size        = L::get( mPeriph ) >> SQR1_L_Pos;
    cfg.addr        = reinterpret_cast<std::uintptr_t>( mDMASampleBuffer.rawSamples.data() );

    /* Won't start the transfer b/c ADC is the controller, but does prime the DMA hw for the ADC */
    Thor::DMA::transfer( cfg );
    DMAEN::set( mPeriph, CFGR_DMAEN );

    /*-------------------------------------------------------------------------
    Start the transfer
    -------------------------------------------------------------------------*/
    mDMASampleBuffer.rawSamples.fill( 0 );
    ADSTART::set( mPeriph, CR_ADSTART );
  }


  void Driver::stopSequence()
  {
    /*-------------------------------------------------------------------------
    Command the stop and wait for it to take effect
    -------------------------------------------------------------------------*/
    ADSTP::set( mPeriph, CR_ADSTP );
    while ( ADSTART::get( mPeriph ) )
    {
      continue;
    }

    /*-------------------------------------------------------------------------
    Prevent any repeated conversions from happening
    -------------------------------------------------------------------------*/
    DMAEN::clear( mPeriph, CFGR_DMAEN );
    OVR::clear( mPeriph, ISR_OVR );
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

      default:
        RT_HARD_ASSERT( false );
        break;
    }

    /*-------------------------------------------------------------------------
    Fill the queue with all the sample data
    -------------------------------------------------------------------------*/
    for ( size_t idx = 0; idx < mDMASampleBuffer.channelSequence.size(); idx++ )
    {
      /*-----------------------------------------------------------------------
      Is the current sequence channel valid?
      -----------------------------------------------------------------------*/
      auto channel = mDMASampleBuffer.channelSequence[ idx ];
      if ( channel == Chimera::ADC::Channel::UNKNOWN )
      {
        break;
      }

      /*-----------------------------------------------------------------------
      Move data into the queue for this channel
      -----------------------------------------------------------------------*/
      if ( !( ( *queue )[ EnumValue( channel ) ]->full() ) )
      {
        Chimera::ADC::Sample sample;
        sample.us     = Chimera::micros();
        sample.counts = mDMASampleBuffer.rawSamples[ idx ];

        ( *queue )[ EnumValue( channel ) ]->push( sample );
      }
    }
  }


  void Driver::onDMAComplete( const Chimera::DMA::TransferStats &stats )
  {
    using namespace Chimera::Thread;
    using namespace Chimera::Peripheral;

    /*-------------------------------------------------------------------------
    Determine some information about the execution context
    -------------------------------------------------------------------------*/
    const bool isContinuous = CONT::get( mPeriph );
    const bool isOverrun    = OVR::get( mPeriph );

    /*-------------------------------------------------------------------------
    Running in continuous conversion mode? Don't bother the main ADC thread
    (except on error) as this would generate an enormous number of wakeups.
    -------------------------------------------------------------------------*/
    if ( isContinuous )
    {
      if ( stats.error || isOverrun )
      {
        stopSequence();
        sendTaskMsg( INT::getUserTaskId( Type::PERIPH_ADC ), ITCMsg::TSK_MSG_ISR_ERROR, TIMEOUT_DONT_WAIT );
      }

      return;
    }
    // else running in single shot mode

    /*-------------------------------------------------------------------------
    Disable the DMA on the ADC to force the user to set up a new transfer.
    -------------------------------------------------------------------------*/
    stopSequence();

    /*-------------------------------------------------------------------------
    Overrun error? RM 13.8.1 states to re-initialize the DMA after clearing
    this bit. This will happen automatically at the start of next transfer.
    -------------------------------------------------------------------------*/
    if ( isOverrun )
    {
      OVR::clear( mPeriph, ISR_OVR );
    }

    /*-------------------------------------------------------------------------
    Wake up the HLD with either an error or new data
    -------------------------------------------------------------------------*/
    if ( stats.error )
    {
      sendTaskMsg( INT::getUserTaskId( Type::PERIPH_ADC ), ITCMsg::TSK_MSG_ISR_ERROR, TIMEOUT_DONT_WAIT );
    }
    else
    {
      syncSequence();
      sendTaskMsg( INT::getUserTaskId( Type::PERIPH_ADC ), ITCMsg::TSK_MSG_ISR_DATA_READY, TIMEOUT_DONT_WAIT );
    }
  }


  void Driver::IRQHandler()
  {
    // Not actually used
  }
}    // namespace Thor::LLD::ADC

#endif /* TARGET_STM32L4 && THOR_DRIVER_ADC */
