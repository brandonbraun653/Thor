/******************************************************************************
 *  File Name:
 *    hld_timer_tri_phase_pwm.cpp
 *
 *  Description:
 *    Thor implementation of the 3-phase PWM driver for BLDC motors using SVM.
 *
 *  2022-2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <memory>
#include <Chimera/common>
#include <Chimera/peripheral>
#include <Chimera/timer>
#include <Thor/lld/interface/inc/timer>
#include <Thor/lld/interface/inc/gpio>

/*-----------------------------------------------------------------------------
Constants
-----------------------------------------------------------------------------*/
static constexpr size_t REQ_HW_TIMER_TYPES = Thor::LLD::TIMER::HardwareType::TIMER_HW_ADVANCED;


namespace Chimera::Timer::Inverter
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr uint32_t s_all_output_channel_bf =
      /* clang-format off */
    ( 1u << EnumValue( Chimera::Timer::Output::OUTPUT_1P ) ) |
    ( 1u << EnumValue( Chimera::Timer::Output::OUTPUT_1N ) ) |
    ( 1u << EnumValue( Chimera::Timer::Output::OUTPUT_2P ) ) |
    ( 1u << EnumValue( Chimera::Timer::Output::OUTPUT_2N ) ) |
    ( 1u << EnumValue( Chimera::Timer::Output::OUTPUT_3P ) ) |
    ( 1u << EnumValue( Chimera::Timer::Output::OUTPUT_3N ) ) |
    ( 1u << EnumValue( Chimera::Timer::Output::OUTPUT_4P ) );
  /* clang-format on */

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct ControlBlock
  {
    Thor::LLD::TIMER::Handle_rPtr timer;                  /**< Handle to the timer */
    Chimera::Timer::Output        pinMap[ NUM_SWITCHES ]; /**< Which channel each pin maps to */
    uint32_t                      maxCCRef;               /**< Max capture compare reference to meet min PWM requirements */
    float                         T_half;                 /**< Half of the PWM period */
    float                         T_freq;                 /**< Core timer frequency */
  };

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static Chimera::DeviceManager<ControlBlock, Chimera::Timer::Instance, EnumValue( Chimera::Timer::Instance::NUM_OPTIONS )>
      s_timer_data;

  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  static float fast_sin( float angle )
  {
    constexpr float M_PI_F = static_cast<float>( M_PI );

    /*-------------------------------------------------------------------------
    Wrap the angle from -PI to PI
    -------------------------------------------------------------------------*/
    while ( angle < -M_PI_F )
    {
      angle += 2.0f * M_PI_F;
    }

    while ( angle > M_PI_F )
    {
      angle -= 2.0f * M_PI_F;
    }

    /*-------------------------------------------------------------------------
    Compute Sine
    -------------------------------------------------------------------------*/
    if ( angle < 0.0f )
    {
      return 1.27323954f * angle + 0.405284735f * angle * angle;
    }
    else
    {
      return 1.27323954f * angle - 0.405284735f * angle * angle;
    }
  }

  /*---------------------------------------------------------------------------
  Class Implementation
  ---------------------------------------------------------------------------*/
  Driver::Driver() : mTimerImpl( nullptr )
  {
  }

  Driver::~Driver()
  {
  }


  Chimera::Status_t Driver::init( const DriverConfig &cfg )
  {
    using namespace Thor::LLD::TIMER;

    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if ( !( getHardwareType( cfg.coreCfg.instance ) & REQ_HW_TIMER_TYPES ) )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }

    /*-------------------------------------------------------------------------
    Grab the driver for this instance and register it with the class
    -------------------------------------------------------------------------*/
    ControlBlock *cb = s_timer_data.getOrCreate( cfg.coreCfg.instance );
    RT_HARD_ASSERT( cb );

    if ( !mTimerImpl )
    {
      mTimerImpl = reinterpret_cast<void *>( cb );
    }

    /*-------------------------------------------------------------------------
    Initialize the control block data
    -------------------------------------------------------------------------*/
    RT_HARD_ASSERT( Chimera::Status::OK == allocate( cfg.coreCfg.instance ) );

    cb->timer = Thor::LLD::TIMER::getHandle( cfg.coreCfg.instance );
    memcpy( cb->pinMap, cfg.pinMap, sizeof( cb->pinMap ) );

    /*-------------------------------------------------------------------------
    Basic timer configuration
    -------------------------------------------------------------------------*/
    auto result = Chimera::Status::OK;

    /* Power on the timer core */
    result |= Thor::LLD::TIMER::Master::initCore( cb->timer, cfg.coreCfg );
    cb->T_freq = 1.0f / ( getBaseTickPeriod( cb->timer ) / 1e9f );

    /* Center-aligned up/down counting with output compare flags set on both count directions */
    setAlignment( cb->timer, AlignMode::CENTER_ALIGNED_3 );

    /* Set the pwm output frequency that drives the IO pins. Multiply by two b/c of the
       center aligned counting mode, whose period is defined by BOTH up/down cycles. */
    result |= setCarrierFrequency( cfg.pwmFrequency * 2.0f );
    cb->T_half = 0.5f / cfg.pwmFrequency;

    /*-------------------------------------------------------------------------
    Break signal(s) configuration to control emergency shutdowns
    -------------------------------------------------------------------------*/
    setBreakPolarity( cb->timer, BREAK_SOURCE_INTERNAL, BREAK_INPUT_1, BREAK_ACTIVE_HIGH );
    setBreakPolarity( cb->timer, BREAK_SOURCE_INTERNAL, BREAK_INPUT_2, BREAK_ACTIVE_HIGH );
    breakEnable( cb->timer, BREAK_SOURCE_INTERNAL, BREAK_INPUT_1 );
    breakEnable( cb->timer, BREAK_SOURCE_INTERNAL, BREAK_INPUT_2 );

    /*-------------------------------------------------------------------------
    3-Phase configuration
    -------------------------------------------------------------------------*/
    /*-------------------------------------------------------------------
    Buffer the phase PWM set-point updates for seamless transitions
    -------------------------------------------------------------------*/
    useOCPreload( cb->timer, Chimera::Timer::Channel::CHANNEL_1, true );
    useOCPreload( cb->timer, Chimera::Timer::Channel::CHANNEL_2, true );
    useOCPreload( cb->timer, Chimera::Timer::Channel::CHANNEL_3, true );

    /*-------------------------------------------------------------------
    Set the PWM mode for each output channel. Use pwm mode 2 to enforce
    the view that specification of PWM widths are done in terms of the
    low-side on-time. This is to make it easier for aligning ADC samples
    with the power stage switching, which measure low side current.
    -------------------------------------------------------------------*/
    setOCMode( cb->timer, Chimera::Timer::Channel::CHANNEL_1, OCMode::OC_MODE_PWM_MODE_2 );
    setOCMode( cb->timer, Chimera::Timer::Channel::CHANNEL_2, OCMode::OC_MODE_PWM_MODE_2 );
    setOCMode( cb->timer, Chimera::Timer::Channel::CHANNEL_3, OCMode::OC_MODE_PWM_MODE_2 );

    /*-------------------------------------------------------------------
    Set the output idle (safe) states. Assumes positive IO logic.
    -------------------------------------------------------------------*/
    setRunModeOffState( cb->timer, OffStateMode::TIMER_CONTROL );
    setIdleModeOffState( cb->timer, OffStateMode::TIMER_CONTROL );
    setOutputIdleStateBulk( cb->timer, s_all_output_channel_bf, Chimera::GPIO::State::LOW );

    /* Assign dead-time during complementary output transitions */
    RT_HARD_ASSERT( setDeadTime( cb->timer, cfg.deadTimeNs ) );

    /*-------------------------------------------------------------------
    Set capture compare behavior
    -------------------------------------------------------------------*/
    /* Set capture/compare mode */
    setCCMode( cb->timer, Chimera::Timer::Channel::CHANNEL_1, CCMode::CCM_OUTPUT );
    setCCMode( cb->timer, Chimera::Timer::Channel::CHANNEL_2, CCMode::CCM_OUTPUT );
    setCCMode( cb->timer, Chimera::Timer::Channel::CHANNEL_3, CCMode::CCM_OUTPUT );

    /* Set output polarity */
    setCCOutputPolarityBulk( cb->timer, s_all_output_channel_bf, CCPolarity::CCP_OUT_ACTIVE_HIGH );

    /* Enable the outputs */
    enableCCOutputBulk( cb->timer, s_all_output_channel_bf );

    /*-------------------------------------------------------------------------
    Output trigger configuration (TRGO). Can be used to drive ADC sampling or
    other peripherals.
    -------------------------------------------------------------------------*/
    /* Use PWM mode 2 here to set the rising edge once CNT > CCR4 */
    setOCMode( cb->timer, Chimera::Timer::Channel::CHANNEL_4, OCMode::OC_MODE_PWM_MODE_2 );

    /* Buffer trigger timing updates so they synchronize correctly */
    useOCPreload( cb->timer, Chimera::Timer::Channel::CHANNEL_4, true );

    /* Configure the TRGO signal to track the output compare channel */
    setMasterMode( cb->timer, MasterMode::COMPARE_OC4REF );

    /* Naive initial OC ref. Use updateTriggerTiming() for exact setting. */
    cb->maxCCRef = getAutoReload( cb->timer );
    setOCReference( cb->timer, Chimera::Timer::Channel::CHANNEL_4, cb->maxCCRef );

    /*-------------------------------------------------------------------------
    Lock out the core timer configuration settings to prevent dangerous changes
    that might damage the power stage of the inverter.
    -------------------------------------------------------------------------*/
    // lockoutTimer( cb->timer, LockoutLevel::LOCK_LEVEL_3 );
    // RT_HARD_ASSERT( isLockedOut( cb->timer ) );

    return result;
  }


  Chimera::Status_t Driver::enableOutput()
  {
    using namespace Thor::LLD::TIMER;

    ControlBlock *cb = reinterpret_cast<ControlBlock *>( mTimerImpl );

    /*-------------------------------------------------------------------------
    Disable the counter
    -------------------------------------------------------------------------*/
    disableCounter( cb->timer );

    /*-------------------------------------------------------------------------
    Reset the timer to a known state
    -------------------------------------------------------------------------*/
    assignCounter( cb->timer, 0 );
    setOCReference( cb->timer, Chimera::Timer::Channel::CHANNEL_1, 0 );
    setOCReference( cb->timer, Chimera::Timer::Channel::CHANNEL_2, 0 );
    setOCReference( cb->timer, Chimera::Timer::Channel::CHANNEL_3, 0 );

    /*-------------------------------------------------------------------------
    Enable the outputs
    -------------------------------------------------------------------------*/
    enableAllOutput( cb->timer );
    enableCounter( cb->timer );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::disableOutput()
  {
    return this->emergencyBreak();
  }


  Chimera::Status_t Driver::setCarrierFrequency( const float freq )
  {
    /*-------------------------------------------------------------------------
    Set the PWM frequency by controlling the timer overflow rate
    -------------------------------------------------------------------------*/
    ControlBlock *cb = reinterpret_cast<ControlBlock *>( mTimerImpl );
    return setEventRate( cb->timer, ( 1.0f / freq ) * 1e9f );
  }


  Chimera::Status_t Driver::svmUpdate( const float drive, const float theta)
  {
    /*-------------------------------------------------------------------------
    Local Constants
    -------------------------------------------------------------------------*/
    constexpr float PI_OVER_3 = static_cast<float>( M_PI ) / 3.0f;

    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    RT_DBG_ASSERT( drive >= 0.0f && drive <= 0.866f );
    RT_DBG_ASSERT( theta >= 0.0f && theta <= 2.0f * static_cast<float>( M_PI ) );

    /*-------------------------------------------------------------------------
    Compute the current sector and angular offset inside that sector
    -------------------------------------------------------------------------*/
    const uint32_t sector = static_cast<uint32_t>( theta / PI_OVER_3 );
    float          alpha  = theta - ( sector * PI_OVER_3 );

    RT_DBG_ASSERT( sector <= 6 );
    RT_DBG_ASSERT( alpha <= PI_OVER_3 && alpha >= 0.0f );

    /*-------------------------------------------------------------------------
    Massage alpha a bit to prevent getting into weird edge cases
    -------------------------------------------------------------------------*/
    if( alpha < 0.0174533 ) // 1 deg
    {
      alpha = 0.0174533;
    }
    else if( alpha > ( PI_OVER_3 - 0.0174533 ) )
    {
      alpha = PI_OVER_3 - 0.0174533;
    }

    /*-------------------------------------------------------------------------
    Calculate the timing windows used to generate the PWM signals
    -------------------------------------------------------------------------*/
    ControlBlock *cb = reinterpret_cast<ControlBlock *>( mTimerImpl );

    const float ta = cb->T_half * drive * fast_sin( PI_OVER_3 - alpha );
    const float tb = cb->T_half * drive * fast_sin( alpha );
    const float tn = cb->T_half - ta - tb;
    const float tn_half = 0.5f * tn;

    /*-------------------------------------------------------------------------
    Compute the PWM compare values for each channel
    -------------------------------------------------------------------------*/
    const uint32_t ton_tn_half = static_cast<uint32_t>( tn_half * cb->T_freq );
    const uint32_t ton_1       = static_cast<uint32_t>( ( tn_half + ta ) * cb->T_freq );
    const uint32_t ton_2       = static_cast<uint32_t>( ( tn_half + tb ) * cb->T_freq );
    const uint32_t ton_3       = static_cast<uint32_t>( ( tn_half + ta + tb) * cb->T_freq );

    RT_DBG_ASSERT( ton_tn_half <= cb->timer->registers->ARR );
    RT_DBG_ASSERT( ton_1 <= cb->timer->registers->ARR );
    RT_DBG_ASSERT( ton_2 <= cb->timer->registers->ARR );
    RT_DBG_ASSERT( ton_3 <= cb->timer->registers->ARR );

    /*-------------------------------------------------------------------------
    Set the PWM compare values for each channel depending on the sector
    -------------------------------------------------------------------------*/
    uint32_t a, b, c;
    switch ( sector )
    {
      case 0:
        a = ton_tn_half;
        b = ton_1;
        c = ton_3;
        break;

      case 1:
        a = ton_2;
        b = ton_tn_half;
        c = ton_3;
        break;

      case 2:
        a = ton_3;
        b = ton_tn_half;
        c = ton_1;
        break;

      case 3:
        a = ton_3;
        b = ton_2;
        c = ton_tn_half;
        break;

      case 4:
        a = ton_1;
        b = ton_3;
        c = ton_tn_half;
        break;

      case 5:
        a = ton_tn_half;
        b = ton_3;
        c = ton_2;
        break;

      default:
        RT_HARD_ASSERT( false );
        break;
    }

    setOCReference( cb->timer, Chimera::Timer::Channel::CHANNEL_1, a );
    setOCReference( cb->timer, Chimera::Timer::Channel::CHANNEL_2, b );
    setOCReference( cb->timer, Chimera::Timer::Channel::CHANNEL_3, c );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::emergencyBreak()
  {
    using namespace Thor::LLD::TIMER;

    /*-------------------------------------------------------------------------
    Hook into the break event to safely set the outputs to a known state
    -------------------------------------------------------------------------*/
    ControlBlock *cb = reinterpret_cast<ControlBlock *>( mTimerImpl );
    generateBreakEvent( cb->timer, BreakChannel::BREAK_INPUT_1 );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::updateTriggerTiming( const uint32_t adc_sample_time_ns, const uint32_t trigger_offset_ns )
  {
    using namespace Thor::LLD::TIMER;

    ControlBlock *const cb = reinterpret_cast<ControlBlock *>( mTimerImpl );

    /*-------------------------------------------------------------------------
    Calculate the reference value for the TRGO signal
    -------------------------------------------------------------------------*/
    const uint32_t timer_clock_period_ns = static_cast<uint32_t>( getBaseTickPeriod( cb->timer ) );

    /* Core sample time plus a boundary layer on either side */
    const uint32_t min_pwm_period_ns = ( 2u * trigger_offset_ns ) + adc_sample_time_ns;

    /* Compute the timer ticks required to reach at least the minimum width, possibly a little larger */
    const uint32_t min_timer_ticks = ( min_pwm_period_ns / timer_clock_period_ns ) + 1u;

    /* Timer runs as centered up/down counter, so the compare match is half of the total width required */
    const uint32_t new_cc4_ref_val = getAutoReload( cb->timer ) - ( min_timer_ticks / 2 );

    /*-------------------------------------------------------------------------
    Compute the new maximum capture compare reference for output channels 1-3.
    Because we are counting center-aligned, setting the max CC value results in
    specifying the minimum PWM period we can command without violating the ADC
    timing constraints. Back off the CC4 value by the trigger offset to ensure
    there is enough setting time between power stage turning on and the ADC
    starting it's sampling.
    -------------------------------------------------------------------------*/
    cb->maxCCRef = new_cc4_ref_val - ( trigger_offset_ns / timer_clock_period_ns );

    /*-------------------------------------------------------------------------
    Update the TRGO capture compare reference
    -------------------------------------------------------------------------*/
    return setOCReference( cb->timer, Chimera::Timer::Channel::CHANNEL_4, new_cc4_ref_val );
  }

}    // namespace Chimera::Timer::Inverter
