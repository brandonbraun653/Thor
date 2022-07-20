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

/*-----------------------------------------------------------------------------
Constants
-----------------------------------------------------------------------------*/
static constexpr size_t REQ_HW_TIMER_TYPES = Thor::LLD::TIMER::HardwareType::TIMER_HW_ADVANCED;


namespace Chimera::Timer::Inverter
{
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


  static uint32_t s_ccer_fwd_comm_table[ 6 ] = {
    /* clang-format off */
    ( Thor::LLD::TIMER::CCER_CC1E  | Thor::LLD::TIMER::CCER_CC2NE ),
    ( Thor::LLD::TIMER::CCER_CC1E  | Thor::LLD::TIMER::CCER_CC3NE ),
    ( Thor::LLD::TIMER::CCER_CC2E  | Thor::LLD::TIMER::CCER_CC3NE ),
    ( Thor::LLD::TIMER::CCER_CC1NE | Thor::LLD::TIMER::CCER_CC2E  ),
    ( Thor::LLD::TIMER::CCER_CC1NE | Thor::LLD::TIMER::CCER_CC3E  ),
    ( Thor::LLD::TIMER::CCER_CC2NE | Thor::LLD::TIMER::CCER_CC3E  )
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
      mTimerImpl  = std::make_shared<void *>();
      *mTimerImpl = reinterpret_cast<void *>( cb );
    }

    /*-------------------------------------------------------------------------
    Initialize the control block data
    -------------------------------------------------------------------------*/
    RT_HARD_ASSERT( Chimera::Status::OK == allocate( cfg.coreCfg.instance ) );

    cb->timer  = Thor::LLD::TIMER::getHandle( cfg.coreCfg.instance );
    memcpy( cb->pinMap, cfg.pinMap, sizeof( cb->pinMap ) );

    /*-------------------------------------------------------------------------
    Basic timer configuration
    -------------------------------------------------------------------------*/
    auto result = Chimera::Status::OK;

    /* Power on the timer core */
    result |= Thor::LLD::TIMER::Master::initCore( cb->timer, cfg.coreCfg );

    /* Set the pwm output frequency that drives the IO pins */
    result |= setCarrierFrequency( cfg.pwmFrequency );

    /*-------------------------------------------------------------------------
    Break signal(s) configuration to control emergency shutdowns
    -------------------------------------------------------------------------*/
    setBreakPolarity( cb->timer, BreakSource::BREAK_SOURCE_INTERNAL, BreakChannel::BREAK_INPUT_1, BreakPolarity::BREAK_ACTIVE_HIGH );
    setBreakPolarity( cb->timer, BreakSource::BREAK_SOURCE_INTERNAL, BreakChannel::BREAK_INPUT_2, BreakPolarity::BREAK_ACTIVE_HIGH );
    breakEnable( cb->timer, BreakSource::BREAK_SOURCE_INTERNAL, BreakChannel::BREAK_INPUT_1 );
    breakEnable( cb->timer, BreakSource::BREAK_SOURCE_INTERNAL, BreakChannel::BREAK_INPUT_2 );

    /*-------------------------------------------------------------------------
    3-Phase configuration
    -------------------------------------------------------------------------*/
    /* Center-aligned up/down counting with output compare flags set on both count directions */
    setAlignment( cb->timer, AlignMode::CENTER_ALIGNED_3 );

    /* Buffer the phase PWM set-point updates for seamless transitions */
    useOCPreload( cb->timer, Chimera::Timer::Channel::CHANNEL_1, true ); /* Output Phase A */
    useOCPreload( cb->timer, Chimera::Timer::Channel::CHANNEL_2, true ); /* Output Phase B */
    useOCPreload( cb->timer, Chimera::Timer::Channel::CHANNEL_3, true ); /* Output Phase C */

    /* Configure PWM mode */
    setCountDirection( cb->timer, CountDir::COUNT_UP );

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

    setOutputIdleState( cb->timer, Chimera::Timer::Output::OUTPUT_1P, Chimera::GPIO::State::LOW );
    setOutputIdleState( cb->timer, Chimera::Timer::Output::OUTPUT_1N, Chimera::GPIO::State::LOW );
    setOutputIdleState( cb->timer, Chimera::Timer::Output::OUTPUT_2P, Chimera::GPIO::State::LOW );
    setOutputIdleState( cb->timer, Chimera::Timer::Output::OUTPUT_2N, Chimera::GPIO::State::LOW );
    setOutputIdleState( cb->timer, Chimera::Timer::Output::OUTPUT_3P, Chimera::GPIO::State::LOW );
    setOutputIdleState( cb->timer, Chimera::Timer::Output::OUTPUT_3N, Chimera::GPIO::State::LOW );
    setOutputIdleState( cb->timer, Chimera::Timer::Output::OUTPUT_5P, Chimera::GPIO::State::LOW );

    /* Assign dead-time during complementary output transitions */
    RT_HARD_ASSERT( setDeadTime( cb->timer, cfg.deadTimeNs ) );

    /* Set capture/compare mode */
    setCCMode( cb->timer, Chimera::Timer::Channel::CHANNEL_1, CCMode::CCM_OUTPUT );
    setCCMode( cb->timer, Chimera::Timer::Channel::CHANNEL_2, CCMode::CCM_OUTPUT );
    setCCMode( cb->timer, Chimera::Timer::Channel::CHANNEL_3, CCMode::CCM_OUTPUT );
    setCCMode( cb->timer, Chimera::Timer::Channel::CHANNEL_5, CCMode::CCM_OUTPUT );

    /* Set output polarity */
    setCCOutputPolarity( cb->timer, Chimera::Timer::Output::OUTPUT_1P, CCPolarity::CCP_OUT_ACTIVE_HIGH );
    setCCOutputPolarity( cb->timer, Chimera::Timer::Output::OUTPUT_1N, CCPolarity::CCP_OUT_ACTIVE_HIGH );
    setCCOutputPolarity( cb->timer, Chimera::Timer::Output::OUTPUT_2P, CCPolarity::CCP_OUT_ACTIVE_HIGH );
    setCCOutputPolarity( cb->timer, Chimera::Timer::Output::OUTPUT_2N, CCPolarity::CCP_OUT_ACTIVE_HIGH );
    setCCOutputPolarity( cb->timer, Chimera::Timer::Output::OUTPUT_3P, CCPolarity::CCP_OUT_ACTIVE_HIGH );
    setCCOutputPolarity( cb->timer, Chimera::Timer::Output::OUTPUT_3N, CCPolarity::CCP_OUT_ACTIVE_HIGH );
    setCCOutputPolarity( cb->timer, Chimera::Timer::Output::OUTPUT_5P, CCPolarity::CCP_OUT_ACTIVE_HIGH );

    /* Enable the outputs */
    enableCCOutput( cb->timer, Chimera::Timer::Output::OUTPUT_1P );
    enableCCOutput( cb->timer, Chimera::Timer::Output::OUTPUT_1N );
    enableCCOutput( cb->timer, Chimera::Timer::Output::OUTPUT_2P );
    enableCCOutput( cb->timer, Chimera::Timer::Output::OUTPUT_2N );
    enableCCOutput( cb->timer, Chimera::Timer::Output::OUTPUT_3P );
    enableCCOutput( cb->timer, Chimera::Timer::Output::OUTPUT_3N );
    enableCCOutput( cb->timer, Chimera::Timer::Output::OUTPUT_5P );

    /* Set the initial duty-cycles for each phase */
    result |= setPhaseDutyCycle( 0.0f, 0.0f, 0.0f );

    /*-------------------------------------------------------------------------
    ADC trigger configuration
    -------------------------------------------------------------------------*/
    /* Use this channel's OC to drive ADC sample. Requires external ADC configuration. */
    Thor::LLD::TIMER::useOCPreload( cb->timer, Chimera::Timer::Channel::CHANNEL_5, true );

    /* Configure the TRGO2 signal to match the OC channel */
    setMasterMode2( cb->timer, MasterMode2::COMPARE_OC5REF );

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

    // Reset anything that needs reset
    // Set the IO to the safe state levels
    // Start the timer back up and let the ISRs do the rest
    ControlBlock *cb = reinterpret_cast<ControlBlock *>( *mTimerImpl );

    enableAllOutput( cb->timer );
    // Reset counter? Set pins to idle?
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
    ControlBlock *cb = reinterpret_cast<ControlBlock *>( *mTimerImpl );
    return setEventRate( cb->timer, ( 1.0f / freq ) * 1e9f );
  }


  Chimera::Status_t Driver::setPhaseDutyCycle( const float a, const float b, const float c )
  {
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    // Bound the floating point inputs

    /*-------------------------------------------------------------------------
    Set the output compare reference for each phase
    -------------------------------------------------------------------------*/
    Chimera::Status_t       result        = Chimera::Status::OK;
    ControlBlock           *cb            = reinterpret_cast<ControlBlock *>( *mTimerImpl );
    float                   arr_val       = static_cast<float>( Thor::LLD::TIMER::getAutoReload( cb->timer ) );
    float                   dutyIn[ 3 ]   = { a, b, c };
    Chimera::Timer::Channel phaseMap[ 3 ] = { Chimera::Timer::Channel::CHANNEL_1, Chimera::Timer::Channel::CHANNEL_2,
                                              Chimera::Timer::Channel::CHANNEL_3 };

    for ( size_t phase_idx = 0; phase_idx < 3; phase_idx++ )
    {
      float    dutyPercent = dutyIn[ phase_idx ] / 100.0f;
      uint32_t new_ref     = static_cast<uint32_t>( roundf( arr_val * dutyPercent ) );

      result |= Thor::LLD::TIMER::setOCReference( cb->timer, phaseMap[ phase_idx ], new_ref );
    }

    /*-------------------------------------------------------------------------
    Update the reference for the ADC trigger
    -------------------------------------------------------------------------*/
    setOCReference( cb->timer, Chimera::Timer::Channel::CHANNEL_5, arr_val - 1 );

    return result;
  }


  Chimera::Status_t Driver::setForwardCommState( const uint8_t phase )
  {
    using namespace Thor::LLD::TIMER;
    static bool was_set = false;

    ControlBlock *cb = reinterpret_cast<ControlBlock *>( *mTimerImpl );

    // if( !was_set )
    // {
    //   was_set = true;

    //   uint32_t tmp = cb->timer->registers->CCER;
    //   tmp |= ( CCER_CC1E | CCER_CC1NE );
    //   cb->timer->registers->CCER = tmp;
    // }
    // else
    // {
    //   was_set = false;

    //   uint32_t tmp = cb->timer->registers->CCER;
    //   tmp &= ~( CCER_CC1E | CCER_CC1NE );
    //   cb->timer->registers->CCER = tmp;
    // }

    uint32_t tmp = cb->timer->registers->CCER;

    tmp &= ~( CCER_CC1E | CCER_CC1NE | CCER_CC2E | CCER_CC2NE | CCER_CC3E | CCER_CC3NE );
    tmp |= s_ccer_fwd_comm_table[ phase ];

    cb->timer->registers->CCER = tmp;

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::emergencyBreak()
  {
    using namespace Thor::LLD::TIMER;

    ControlBlock *cb = reinterpret_cast<ControlBlock *>( *mTimerImpl );
    generateBreakEvent( cb->timer, BreakChannel::BREAK_INPUT_1 );

    return Chimera::Status::OK;
  }

}    // namespace Chimera::Timer::Inverter
