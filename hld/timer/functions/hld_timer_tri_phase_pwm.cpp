/******************************************************************************
 *  File Name:
 *    hld_timer_tri_phase_pwm.cpp
 *
 *  Description:
 *    Thor implementation of the 3-phase PWM driver
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
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
  static constexpr uint32_t
      s_all_output_channel_bf =    //::Thor::LLD::TIMER::EnableFlagGenerator<Chimera::Timer::Output::OUTPUT_1P>;
      /* clang-format off */
    ( 1u << EnumValue( Chimera::Timer::Output::OUTPUT_1P ) ) |
    ( 1u << EnumValue( Chimera::Timer::Output::OUTPUT_1N ) ) |
    ( 1u << EnumValue( Chimera::Timer::Output::OUTPUT_2P ) ) |
    ( 1u << EnumValue( Chimera::Timer::Output::OUTPUT_2N ) ) |
    ( 1u << EnumValue( Chimera::Timer::Output::OUTPUT_3P ) ) |
    ( 1u << EnumValue( Chimera::Timer::Output::OUTPUT_3N ) ) |
    ( 1u << EnumValue( Chimera::Timer::Output::OUTPUT_5P ) );
  /* clang-format on */

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct ControlBlock
  {
    Thor::LLD::TIMER::Handle_rPtr timer;                  /**< Handle to the timer */
    Chimera::Timer::Output        pinMap[ NUM_SWITCHES ]; /**< Which channel each pin maps to */
  };

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static Chimera::DeviceManager<ControlBlock, Chimera::Timer::Instance, EnumValue( Chimera::Timer::Instance::NUM_OPTIONS )>
      s_timer_data;


  static uint32_t s_ccer_fwd_comm_table[ 7 ] = {
    /* clang-format off */
    ( Thor::LLD::TIMER::CCER_CC1E  | Thor::LLD::TIMER::CCER_CC2NE ),  /* STATE_0 */
    ( Thor::LLD::TIMER::CCER_CC1E  | Thor::LLD::TIMER::CCER_CC3NE ),  /* STATE_1 */
    ( Thor::LLD::TIMER::CCER_CC2E  | Thor::LLD::TIMER::CCER_CC3NE ),  /* STATE_2 */
    ( Thor::LLD::TIMER::CCER_CC1NE | Thor::LLD::TIMER::CCER_CC2E  ),  /* STATE_3 */
    ( Thor::LLD::TIMER::CCER_CC1NE | Thor::LLD::TIMER::CCER_CC3E  ),  /* STATE_4 */
    ( Thor::LLD::TIMER::CCER_CC2NE | Thor::LLD::TIMER::CCER_CC3E  ),  /* STATE_5 */
    ( 0 ), // All off
  };/* clang-format on */

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

    /* Center-aligned up/down counting with output compare flags set on both count directions */
    setAlignment( cb->timer, AlignMode::CENTER_ALIGNED_3 );

    /* Set the pwm output frequency that drives the IO pins. Multiply by two b/c of the
       center aligned counting mode, whose period is defined by BOTH up/down cycles. */
    result |= setCarrierFrequency( cfg.pwmFrequency * 2.0f );

    /*-------------------------------------------------------------------------
    Break signal(s) configuration to control emergency shutdowns
    -------------------------------------------------------------------------*/
    setBreakPolarity( cb->timer, BreakSource::BREAK_SOURCE_INTERNAL, BreakChannel::BREAK_INPUT_1,
                      BreakPolarity::BREAK_ACTIVE_HIGH );
    setBreakPolarity( cb->timer, BreakSource::BREAK_SOURCE_INTERNAL, BreakChannel::BREAK_INPUT_2,
                      BreakPolarity::BREAK_ACTIVE_HIGH );
    breakEnable( cb->timer, BreakSource::BREAK_SOURCE_INTERNAL, BreakChannel::BREAK_INPUT_1 );
    breakEnable( cb->timer, BreakSource::BREAK_SOURCE_INTERNAL, BreakChannel::BREAK_INPUT_2 );

    /*-------------------------------------------------------------------------
    3-Phase configuration
    -------------------------------------------------------------------------*/
    /* Buffer the phase PWM set-point updates for seamless transitions */
    useOCPreload( cb->timer, Chimera::Timer::Channel::CHANNEL_1, true );
    useOCPreload( cb->timer, Chimera::Timer::Channel::CHANNEL_2, true );
    useOCPreload( cb->timer, Chimera::Timer::Channel::CHANNEL_3, true );

    // Use PWM mode 1 to logically line up duty cycle programming, which assumes high side
    // on-time is equal to some percentage of the ARR. Mode 1 has the channel set to "active"
    // while CNT < CCRx.
    setOCMode( cb->timer, Chimera::Timer::Channel::CHANNEL_1, OCMode::OC_MODE_PWM_MODE_1 );
    setOCMode( cb->timer, Chimera::Timer::Channel::CHANNEL_2, OCMode::OC_MODE_PWM_MODE_1 );
    setOCMode( cb->timer, Chimera::Timer::Channel::CHANNEL_3, OCMode::OC_MODE_PWM_MODE_1 );

    // Use PWM mode 2 here to set the rising edge once CNT > CCR5
    setOCMode( cb->timer, Chimera::Timer::Channel::CHANNEL_5, OCMode::OC_MODE_PWM_MODE_2 );

    /* Set output idle (safe) states. Assumes positive logic for the power stage drive signals. */
    setRunModeOffState( cb->timer, OffStateMode::TIMER_CONTROL );
    setIdleModeOffState( cb->timer, OffStateMode::TIMER_CONTROL );
    setOutputIdleStateBulk( cb->timer, s_all_output_channel_bf, Chimera::GPIO::State::LOW );

    /* Assign dead-time during complementary output transitions */
    RT_HARD_ASSERT( setDeadTime( cb->timer, cfg.deadTimeNs ) );

    /* Set capture/compare mode */
    setCCMode( cb->timer, Chimera::Timer::Channel::CHANNEL_1, CCMode::CCM_OUTPUT );
    setCCMode( cb->timer, Chimera::Timer::Channel::CHANNEL_2, CCMode::CCM_OUTPUT );
    setCCMode( cb->timer, Chimera::Timer::Channel::CHANNEL_3, CCMode::CCM_OUTPUT );

    /* Set output polarity */
    setCCOutputPolarityBulk( cb->timer, s_all_output_channel_bf, CCPolarity::CCP_OUT_ACTIVE_HIGH );

    /* Enable the outputs */
    enableCCOutputBulk( cb->timer, s_all_output_channel_bf );

    /* Set the initial duty-cycles for each phase */
    result |= setPhaseDutyCycle( 0.0f, 0.0f, 0.0f );

    /*-------------------------------------------------------------------------
    ADC trigger configuration
    -------------------------------------------------------------------------*/
    /* Use this channel's OC to drive ADC sample. Requires external ADC configuration. */
    Thor::LLD::TIMER::useOCPreload( cb->timer, Chimera::Timer::Channel::CHANNEL_5, true );

    /* Configure the TRGO2 signal to match the OC channel */
    setMasterMode2( cb->timer, MasterMode2::COMPARE_OC5REF );

    /* Set the trigger timing to be in the center of the low-side (complementary output) ON sequence */
    const uint32_t arr_val = Thor::LLD::TIMER::getAutoReload( cb->timer );
    setOCReference( cb->timer, Chimera::Timer::Channel::CHANNEL_5, 25 );

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
    setForwardCommState( STATE_OFF );

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


  Chimera::Status_t Driver::setPhaseDutyCycle( const float a, const float b, const float c )
  {
    /*-------------------------------------------------------------------------
    Set the output compare reference for each phase
    -------------------------------------------------------------------------*/
    Chimera::Status_t         result    = Chimera::Status::OK;
    const ControlBlock *const cb        = reinterpret_cast<ControlBlock *>( mTimerImpl );
    const uint32_t            arr_val   = Thor::LLD::TIMER::getAutoReload( cb->timer );
    const float               arr_val_f = static_cast<float>( arr_val );

    /*-------------------------------------------------------------------------
    Phase A
    -------------------------------------------------------------------------*/
    auto phase_a_ref = static_cast<uint32_t>( arr_val_f * a );
    result |= Thor::LLD::TIMER::setOCReference( cb->timer, Chimera::Timer::Channel::CHANNEL_1, phase_a_ref );

    /*-------------------------------------------------------------------------
    Phase B
    -------------------------------------------------------------------------*/
    auto phase_b_ref = static_cast<uint32_t>( arr_val_f * b );
    result |= Thor::LLD::TIMER::setOCReference( cb->timer, Chimera::Timer::Channel::CHANNEL_2, phase_b_ref );

    /*-------------------------------------------------------------------------
    Phase C
    -------------------------------------------------------------------------*/
    auto phase_c_ref = static_cast<uint32_t>( arr_val_f * c );
    result |= Thor::LLD::TIMER::setOCReference( cb->timer, Chimera::Timer::Channel::CHANNEL_3, phase_c_ref );

    return result;
  }


  Chimera::Status_t Driver::setPhaseDutyCycle( const uint32_t a, const uint32_t b, const uint32_t c )
  {
    /*-------------------------------------------------------------------------
    Set the output compare reference for each phase
    -------------------------------------------------------------------------*/
    Chimera::Status_t         result  = Chimera::Status::OK;
    const ControlBlock *const cb      = reinterpret_cast<ControlBlock *>( mTimerImpl );
    const uint32_t            arr_val = Thor::LLD::TIMER::getAutoReload( cb->timer );

    RT_DBG_ASSERT( a <= arr_val );
    RT_DBG_ASSERT( b <= arr_val );
    RT_DBG_ASSERT( c <= arr_val );

    result |= Thor::LLD::TIMER::setOCReference( cb->timer, Chimera::Timer::Channel::CHANNEL_1, a );
    result |= Thor::LLD::TIMER::setOCReference( cb->timer, Chimera::Timer::Channel::CHANNEL_2, b );
    result |= Thor::LLD::TIMER::setOCReference( cb->timer, Chimera::Timer::Channel::CHANNEL_3, c );

    return result;
  }


  Chimera::Status_t Driver::setForwardCommState( const int state )
  {
    using namespace Thor::LLD::TIMER;
    RT_DBG_ASSERT( state < CommutationState::NUM_STATES );
    RT_DBG_ASSERT( state < ARRAY_COUNT( s_ccer_fwd_comm_table ) );

    ControlBlock *cb = reinterpret_cast<ControlBlock *>( mTimerImpl );

    uint32_t tmp = cb->timer->registers->CCER;

    tmp &= ~( CCER_CC1E | CCER_CC1NE | CCER_CC2E | CCER_CC2NE | CCER_CC3E | CCER_CC3NE );
    tmp |= s_ccer_fwd_comm_table[ state ];

    cb->timer->registers->CCER = tmp;

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


  uint32_t Driver::getAutoReloadValue() const
  {
    return reinterpret_cast<ControlBlock *>( mTimerImpl )->timer->registers->ARR;
  }

}    // namespace Chimera::Timer::Inverter
