/******************************************************************************
 *  File Name:
 *    hw_adc_driver.cpp
 *
 *  Description:
 *    Implements the LLD interface to the ADC hardware.
 *
 *  2020-2022 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/algorithm>
#include <Chimera/common>
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


#if defined( THOR_ADC ) && defined( TARGET_STM32L4 )
/*-----------------------------------------------------------------------------
This driver expects some project side constants to be defined for helping with
a few calculations.

#define PRJ_ADC_VREF  (x.yzf)
-----------------------------------------------------------------------------*/
#include "thor_adc_prj_config.hpp"


namespace Thor::LLD::ADC
{
  /*---------------------------------------------------------------------------
  Static Constants
  ---------------------------------------------------------------------------*/
  /**
   * @brief Internal voltage reference, address of parameter VREFINT_CAL
   *
   * VrefInt ADC raw data acquired at temperature 30 DegC (tolerance: +-5 DegC)
   * Vref+ = 3.0 V (tolerance: +-10 mV).
   */
  static const uint16_t *const VREFINT_CAL_ADDR = reinterpret_cast<uint16_t *>( 0x1FFF75AAUL );

  /**
   * @brief Internal analog voltage reference (Vref+)
   *
   * Value with which temperature sensor has been calibrated in production
   * (tolerance: +-10 mV) (unit: mV).
   */
  static constexpr float VREFINT_CAL_VREF_MV = 3000.0f;


  /*---------------------------------------------------------------------------
  Static Variables
  ---------------------------------------------------------------------------*/
  static Driver s_adc_drivers[ NUM_ADC_PERIPHS ];

  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  static void configure_oversampling( Thor::LLD::ADC::RegisterMap *const periph, const Chimera::ADC::OverSampler rate,
                                      const Chimera::ADC::OverSampleShift shift )
  {
    using namespace Chimera::ADC;

    /*-------------------------------------------------------------------------
    Input Protections
    -------------------------------------------------------------------------*/
    if ( !periph || ( rate < OverSampler::OS_2X ) || ( rate >= OverSampler::NUM_OPTIONS ) )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Decide field for OVSR register
    -------------------------------------------------------------------------*/
    uint32_t ovsr_field = 0;
    switch ( rate )
    {
      case OverSampler::OS_2X:
      default:
        ovsr_field = 0;
        break;

      case OverSampler::OS_4X:
        ovsr_field = CFGR2_OVSR_0;
        break;

      case OverSampler::OS_8X:
        ovsr_field = CFGR2_OVSR_1;
        break;

      case OverSampler::OS_16X:
        ovsr_field = CFGR2_OVSR_1 | CFGR2_OVSR_0;
        break;

      case OverSampler::OS_32X:
        ovsr_field = CFGR2_OVSR_2;
        break;

      case OverSampler::OS_64X:
        ovsr_field = CFGR2_OVSR_2 | CFGR2_OVSR_0;
        break;

      case OverSampler::OS_128X:
        ovsr_field = CFGR2_OVSR_2 | CFGR2_OVSR_1;
        break;

      case OverSampler::OS_256X:
        ovsr_field = CFGR2_OVSR_2 | CFGR2_OVSR_1 | CFGR2_OVSR_0;
        break;
    }

    /*-------------------------------------------------------------------------
    Decide field for OVSS register
    -------------------------------------------------------------------------*/
    uint32_t ovss_field = 0;
    switch ( shift )
    {
      case OverSampleShift::OS_NONE:
      default:
        ovss_field = 0;
        break;
      case OverSampleShift::OS_1_BIT:
        ovss_field = CFGR2_OVSS_0;
        break;
      case OverSampleShift::OS_2_BIT:
        ovss_field = CFGR2_OVSS_1;
        break;
      case OverSampleShift::OS_3_BIT:
        ovss_field = CFGR2_OVSS_1 | CFGR2_OVSS_0;
        break;
      case OverSampleShift::OS_4_BIT:
        ovss_field = CFGR2_OVSS_2;
        break;
      case OverSampleShift::OS_5_BIT:
        ovss_field = CFGR2_OVSS_2 | CFGR2_OVSS_0;
        break;
      case OverSampleShift::OS_6_BIT:
        ovss_field = CFGR2_OVSS_2 | CFGR2_OVSS_1;
        break;
      case OverSampleShift::OS_7_BIT:
        ovss_field = CFGR2_OVSS_2 | CFGR2_OVSS_1 | CFGR2_OVSS_0;
        break;
      case OverSampleShift::OS_8_BIT:
        ovss_field = CFGR2_OVSS_3;
        break;
    }

    /*-------------------------------------------------------------------------
    Apply the fields and enable oversampling for injected and regular channels
    -------------------------------------------------------------------------*/
    OVSR::set( periph, ovsr_field );
    OVSS::set( periph, ovss_field );
    JOVSE::set( periph, CFGR2_JOVSE );    // Injected oversampling enable
    ROVSE::set( periph, CFGR2_ROVSE );    // Regular oversampling enable
  }

  /*---------------------------------------------------------------------------
  Driver Implementation
  ---------------------------------------------------------------------------*/
  Driver::Driver() :
      mPeriph( nullptr ), mCommon( nullptr ), mResourceIndex( INVALID_RESOURCE_INDEX ), mCalcVdda( 0.0f ), mCfg( {} ),
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
    Set up the ISR
    -------------------------------------------------------------------------*/
    Thor::LLD::INT::disableIRQ( Resource::IRQSignals[ mResourceIndex ] );
    Thor::LLD::INT::clearPendingIRQ( Resource::IRQSignals[ mResourceIndex ] );
    Thor::LLD::INT::setPriority( Resource::IRQSignals[ mResourceIndex ], Thor::LLD::INT::ADC_IT_PREEMPT_PRIORITY, 0u );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::initialize( const Chimera::ADC::DriverConfig &cfg )
  {
    using namespace Chimera::ADC;
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

    // Select the clock source to use the system clock
    ADCSEL::set( RCC1_PERIPH, Config::CCIPR::ADCSEL_SYS_CLOCK );

    // Turn on the core peripheral clock
    this->clockEnable();
    this->reset();

    // Select the highest available synchronous clock rate (16.7.2, CKMODE[1:0])
    if ( HPRE::get( RCC1_PERIPH ) == 0 )
    {
      CKMODE::set( ADC_COMMON, ( 0x1 << CCR_CKMODE_Pos ) );
    }
    else
    {
      CKMODE::set( ADC_COMMON, ( 0x2 << CCR_CKMODE_Pos ) );
    }

    /*-------------------------------------------------------------------------
    Bring the ADC out of deep power down
    -------------------------------------------------------------------------*/
    DEEPPWD::clear( mPeriph, CR_DEEPPWD );    // Disable deep power down
    ADVREGEN::set( mPeriph, CR_ADVREGEN );    // Enable the voltage regulator

    // Allow the analog voltage regulator time to boot
    Chimera::delayMilliseconds( 250 );

    /*-------------------------------------------------------------------------
    Enable the sensor monitors
    -------------------------------------------------------------------------*/
    VBATEN::set( ADC_COMMON, CCR_VBATEN );    // External battery sensor
    TSEN::set( ADC_COMMON, CCR_TSEN );        // Internal temperature sensor
    VREFEN::set( ADC_COMMON, CCR_VREFEN );    // Internal bandgap vref sensor

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
      setSampleTime( static_cast<Chimera::ADC::Channel>( idx ), SampleTime::SMP_640P5 );
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
    Configure oversampling if needed
    -------------------------------------------------------------------------*/
    configure_oversampling( mPeriph, cfg.overSampleRate, cfg.overSampleShift );

    /*-------------------------------------------------------------------------
    Configure the DMA transfers from the ADC's perspective
    -------------------------------------------------------------------------*/
    DMACFG::clear( mPeriph, CFGR_DMACFG );    // Disable circular mode
    DMAEN::clear( mPeriph, CFGR_DMAEN );      // Disable DMA (for now)

    /*-------------------------------------------------------------------------
    Configure the DMA pipe to run in the background without event interrupts
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

    /*-------------------------------------------------------------------------
    Sample the internal voltage reference to calculate the real VDDA+ present
    on the MCU pin. This will always be present on Channel 0.
    -------------------------------------------------------------------------*/
    size_t startTime  = Chimera::millis();
    float  numSamples = 0.0f;
    mCalcVdda = 0.0f;

    while ( Chimera::millis() - startTime < Chimera::Thread::TIMEOUT_100MS )
    {
      Chimera::ADC::Sample vref_sample  = sampleChannel( Chimera::ADC::Channel::ADC_CH_0 );
      const float          vrefint_data = static_cast<float>( vref_sample.counts );
      const float          vrefint_cal  = static_cast<float>( *VREFINT_CAL_ADDR );

      mCalcVdda += ( VREFINT_CAL_VREF_MV * ( vrefint_cal / vrefint_data ) ) / 1000.0f;
      numSamples++;
    }

    mCalcVdda /= numSamples;

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


  float Driver::resolution() const
  {
    Reg32_t resolution = RES::get( mPeriph ) >> CFGR_RES_Pos;
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
    return mCalcVdda;
  }


  float Driver::toVoltage( const Chimera::ADC::Sample sample )
  {
    /*-----------------------------------------------------------------------
    Use the current resolution to interpret the ADC sample
    -----------------------------------------------------------------------*/
    float fRes = resolution();
    RT_DBG_ASSERT( fRes != 0.0f );

    return ( mCalcVdda * static_cast<float>( sample.counts ) ) / fRes;
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

    if ( cfg.wdgChannel == Chimera::ADC::Watchdog::ANALOG_1 )
    {
      AWD2CH::setbit( mPeriph, ( 1u << EnumValue( cfg.adcChannel ) ) );
      TR2_ALL::set( mPeriph, hi_lo_data );
      AWD2IE::setbit( mPeriph, IER_AWD2IE );
    }
    else if ( cfg.wdgChannel == Chimera::ADC::Watchdog::ANALOG_2 )
    {
      AWD3CH::setbit( mPeriph, ( 1u << EnumValue( cfg.adcChannel ) ) );
      TR3_ALL::set( mPeriph, hi_lo_data );
      AWD3IE::setbit( mPeriph, IER_AWD3IE );
    }

    return Chimera::Status::OK;
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
      switch ( sequence.seqMode )
      {
        case SamplingMode::ONE_SHOT:    // Software initiated
        case SamplingMode::TRIGGER:     // Hardware initiated
          CONT::clear( mPeriph, CFGR_CONT );
          break;

        case SamplingMode::CONTINUOUS:
          CONT::set( mPeriph, CFGR_CONT );
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
      L::set( mPeriph, ( totalSize - 1u ) << SQR1_L_Pos );
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
    while ( ADSTART::get( mPeriph ) || JADSTP::get( mPeriph ) )
    {
      continue;
    }

    /*-------------------------------------------------------------------------
    Prime the DMA pipe to accept new transfer requests
    -------------------------------------------------------------------------*/
    auto callback = Chimera::DMA::TransferCallback::create<Driver, &Driver::dma_isr_transfer_complete_callback>( *this );

    PipeTransfer cfg;
    cfg.pipe        = mDMAPipeID;
    cfg.size        = ( L::get( mPeriph ) >> SQR1_L_Pos ) + 1u;
    cfg.addr        = reinterpret_cast<std::uintptr_t>( mDMASampleBuffer.rawSamples.data() );
    cfg.isrCallback = callback;

    /* Won't start the transfer b/c ADC is the controller, but does prime the DMA hw for the ADC */
    Chimera::DMA::transfer( cfg );

    /*-------------------------------------------------------------------------
    Turn on the ADC peripheral DMA request hardware
    -------------------------------------------------------------------------*/
    DMACFG::set( mPeriph, CFGR_DMACFG ); /* Send new DMA txfr request on every conversion */
    DMAEN::set( mPeriph, CFGR_DMAEN );   /* Turn on ADC DMA. Does NOT start the txfr yet. */

    /*-------------------------------------------------------------------------
    Select which trigger to use to start a new ADC sample
    -------------------------------------------------------------------------*/
    if ( mSeqCfg.seqMode == SamplingMode::TRIGGER )
    {
      switch ( mSeqCfg.trigMode )
      {
        case TriggerMode::RISING_EDGE:
          EXTEN::set( mPeriph, ( 0x1 << CFGR_EXTEN_Pos ) );
          break;

        case TriggerMode::FALLING_EDGE:
          EXTEN::set( mPeriph, ( 0x2 << CFGR_EXTEN_Pos ) );
          break;

        case TriggerMode::BOTH_EDGE:
          EXTEN::set( mPeriph, ( 0x3 << CFGR_EXTEN_Pos ) );
          break;

        default:
          RT_HARD_ASSERT( false );    // Bad configuration
          break;
      };

      RT_HARD_ASSERT( mSeqCfg.trigChannel <= NUM_ADC_EXT_TRIG_CHANNELS );
      EXTSEL::set( mPeriph, ( mSeqCfg.trigChannel << CFGR_EXTSEL_Pos ) );
      ADSTART::set( mPeriph, CR_ADSTART );
    }
    else /* Software Trigger or Continuous */
    {
      EXTEN::clear( mPeriph, CFGR_EXTEN );
    }

    /*-------------------------------------------------------------------------
    Finally, start the transfer
    -------------------------------------------------------------------------*/
    ADSTART::set( mPeriph, CR_ADSTART );
  }


  void Driver::stopSequence()
  {
    using namespace Chimera::ADC;

    /*-------------------------------------------------------------------------
    Command the stop and wait for it to take effect
    -------------------------------------------------------------------------*/
    ADSTP::set( mPeriph, CR_ADSTP );
    JADSTP::set( mPeriph, CR_JADSTP );
    while ( ADSTART::get( mPeriph ) || JADSTP::get( mPeriph ) )
    {
      continue;
    }

    /*-------------------------------------------------------------------------
    Stop generating DMA requests on ADC side (but leave DMA configured)
    -------------------------------------------------------------------------*/
    DMAEN::clear( mPeriph, CFGR_DMAEN );
    OVR::clear( mPeriph, ISR_OVR );

    /*-------------------------------------------------------------------------
    Stop listening to hardware trigger signals
    -------------------------------------------------------------------------*/
    if ( mSeqCfg.seqMode == SamplingMode::TRIGGER )
    {
      EXTEN::clear( mPeriph, CFGR_EXTEN );
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
    Local Constants
    -------------------------------------------------------------------------*/
    constexpr uint32_t ANY_WDG_TRIP = 0x380;

    /*-------------------------------------------------------------------------
    Get the status/enablement registers at time of ISR firing
    -------------------------------------------------------------------------*/
    const uint32_t ISR = ISR_ALL::get( mPeriph );

    /*-------------------------------------------------------------------------
    Did any of the watchdog monitors trip?
    -------------------------------------------------------------------------*/
    if ( ( ISR & ANY_WDG_TRIP ) && mCallbacks[ EnumValue( Chimera::ADC::Interrupt::ANALOG_WD ) ] )
    {
      /* ACK the event, only writing the specific bits to clear */
      ISR_ALL::set( mPeriph, ANY_WDG_TRIP );

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

#endif /* TARGET_STM32L4 && THOR_DRIVER_ADC */
