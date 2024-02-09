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
#include <Chimera/assert>
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
    float                         pwm_half_period_hz;     /**< Half of the PWM period in seconds*/
    float                         frequency_hz;           /**< Core timer frequency */
    float                         period_ns;              /**< Timer period in ns */
    int32_t                       dt_ticks;               /**< Dead-time in timer ticks */
    int32_t                       tn_ticks;               /**< Duration of the shunt resistor noise due to switching */
    int32_t                       tr_ticks;               /**< Rise time of the input signal to the ADC */
    int32_t                       ts_ticks;               /**< Sample time of the ADC */
    int32_t                       tstable_ticks;          /**< Offset from switching when current should be less noisy */
    volatile SVMState             svmState;               /**< Current SVM state */
  };

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static Chimera::DeviceManager<ControlBlock, Chimera::Timer::Instance, EnumValue( Chimera::Timer::Instance::NUM_OPTIONS )>
      s_timer_data;

  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Checks if the given SwitchIO is for the high-side switch.
   *
   * @param io The SwitchIO to check.
   * @return True if the SwitchIO is for the high-side switch, false otherwise.
   */
  static constexpr bool isHiSideSwitch( const SwitchIO io )
  {
    return ( io == SWITCH_1_HI ) || ( io == SWITCH_2_HI ) || ( io == SWITCH_3_HI );
  }


  /**
   * @brief Checks if the given SwitchIO is a low side switch.
   *
   * @param io The SwitchIO to check.
   * @return true if the SwitchIO is for the low side switch, false otherwise.
   */
  static constexpr bool isLoSideSwitch( const SwitchIO io )
  {
    return ( io == SWITCH_1_LO ) || ( io == SWITCH_2_LO ) || ( io == SWITCH_3_LO );
  }


  /**
   * @brief Checks if the given duty cycle is within the valid range.
   *
   * @param dutyCycle The duty cycle to be checked.
   * @return True if the duty cycle is within the valid range, false otherwise.
   */
  static constexpr bool isDutyCycleInRange( const float dutyCycle )
  {
    return ( dutyCycle >= 0.0f ) && ( dutyCycle <= 1.0f );
  }


  /**
   * @brief Calculates the sine of the given angle using a fast approximation algorithm.
   *
   * @param angle The angle in radians.
   * @return The sine of the angle.
   */
  static float fastSine( float angle )
  {
    constexpr float M_PI_F = static_cast<float>( M_PI );

    /*-------------------------------------------------------------------------
    Wrap the angle from -PI to PI
    -------------------------------------------------------------------------*/
    while( angle < -M_PI_F )
    {
      angle += 2.0f * M_PI_F;
    }

    while( angle > M_PI_F )
    {
      angle -= 2.0f * M_PI_F;
    }

    /*-------------------------------------------------------------------------
    Compute Sine
    -------------------------------------------------------------------------*/
    if( angle < 0.0f )
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
    if( !( getHardwareType( cfg.coreCfg.instance ) & REQ_HW_TIMER_TYPES ) )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }

    /*-------------------------------------------------------------------------
    Grab the driver for this instance and register it with the class
    -------------------------------------------------------------------------*/
    ControlBlock *cb = s_timer_data.getOrCreate( cfg.coreCfg.instance );
    RT_HARD_ASSERT( cb );

    if( !mTimerImpl )
    {
      mTimerImpl = reinterpret_cast<void *>( cb );
    }

    /*-------------------------------------------------------------------------
    Initialize the control block data
    -------------------------------------------------------------------------*/
    RT_HARD_ASSERT( Chimera::Status::OK == allocate( cfg.coreCfg.instance ) );

    cb->timer                 = Thor::LLD::TIMER::getHandle( cfg.coreCfg.instance );
    cb->frequency_hz          = 0.0f;
    cb->pwm_half_period_hz    = 0.0f;
    cb->dt_ticks              = 0;
    cb->tn_ticks              = 0;
    cb->tr_ticks              = 0;
    cb->ts_ticks              = 0;
    cb->tstable_ticks         = 0;
    cb->svmState.phase1       = Chimera::Timer::Channel::CHANNEL_1;
    cb->svmState.phase2       = Chimera::Timer::Channel::CHANNEL_2;
    cb->svmState.adc_polarity = true;
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
    Compute some timing parameters for the SVM update step
    -------------------------------------------------------------------------*/
    cb->period_ns          = getBaseTickPeriod( cb->timer );
    cb->frequency_hz       = 1.0f / ( cb->period_ns / 1e9f );
    cb->pwm_half_period_hz = 0.5f / cfg.pwmFrequency;
    cb->dt_ticks           = static_cast<uint32_t>( cfg.deadTimeNs / cb->period_ns );
    cb->tn_ticks           = static_cast<uint32_t>( cfg.settleTimeNs / cb->period_ns );
    cb->ts_ticks           = static_cast<uint32_t>( cfg.adcSampleTimeNs / cb->period_ns );
    cb->tstable_ticks      = cb->dt_ticks + cb->tn_ticks;

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
    Set PWM mode 2 for each channel, which sets the non-complementary
    output active when the counter is above the compare value.
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
    Set capture compare behavior to outputs with positive logic.
    -------------------------------------------------------------------*/
    setCCMode( cb->timer, Chimera::Timer::Channel::CHANNEL_1, CCMode::CCM_OUTPUT );
    setCCMode( cb->timer, Chimera::Timer::Channel::CHANNEL_2, CCMode::CCM_OUTPUT );
    setCCMode( cb->timer, Chimera::Timer::Channel::CHANNEL_3, CCMode::CCM_OUTPUT );
    setCCOutputPolarityBulk( cb->timer, s_all_output_channel_bf, CCPolarity::CCP_OUT_ACTIVE_HIGH );
    enableCCOutputBulk( cb->timer, s_all_output_channel_bf );

    /*-------------------------------------------------------------------------
    Output trigger configuration (TRGO) for synchronization. This is just a
    preliminary setting and will be updated each interrupt period.
    -------------------------------------------------------------------------*/
    setOCMode( cb->timer, Chimera::Timer::Channel::CHANNEL_4, OCMode::OC_MODE_TOGGLE_MATCH );
    useOCPreload( cb->timer, Chimera::Timer::Channel::CHANNEL_4, true );
    setMasterMode( cb->timer, MasterMode::COMPARE_OC4REF );
    setOCReference( cb->timer, Chimera::Timer::Channel::CHANNEL_4, 0 );

    /*-------------------------------------------------------------------------
    Lock out the core timer configuration settings to prevent dangerous changes
    that might damage the power stage of the inverter.
    -------------------------------------------------------------------------*/
    // lockoutTimer( cb->timer, LockoutLevel::LOCK_LEVEL_3 );
    // RT_HARD_ASSERT( isLockedOut( cb->timer ) );

    return result;
  }


  void Driver::reset()
  {
    using namespace Thor::LLD::RCC;
    using namespace Chimera::Peripheral;

    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if( !mTimerImpl )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Inject the hardware reset signal
    -------------------------------------------------------------------------*/
    ControlBlock *cb = reinterpret_cast<ControlBlock *>( mTimerImpl );

    auto result = getPeriphClockCtrl()->reset( Type::PERIPH_TIMER, cb->timer->globalIndex );
    RT_DBG_ASSERT( result == Chimera::Status::OK );
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
    using namespace Thor::LLD::TIMER;

    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if( !mTimerImpl )
    {
      return Chimera::Status::NOT_INITIALIZED;
    }

    ControlBlock *cb = reinterpret_cast<ControlBlock *>( mTimerImpl );
    disableAllOutput( cb->timer );
    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::setCarrierFrequency( const float freq )
  {
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if( !mTimerImpl )
    {
      return Chimera::Status::NOT_INITIALIZED;
    }

    /*-------------------------------------------------------------------------
    Set the PWM frequency by controlling the timer overflow rate
    -------------------------------------------------------------------------*/
    ControlBlock *cb = reinterpret_cast<ControlBlock *>( mTimerImpl );
    return setEventRate( cb->timer, ( 1.0f / freq ) * 1e9f );
  }


  Chimera::Status_t Driver::energizeWinding( const SwitchIO hiSide, const SwitchIO loSide, const float dutyCycle )
  {
    using namespace Thor::LLD::TIMER;

    constexpr Chimera::Timer::Channel switch_ch_lut[] = {
      Chimera::Timer::Channel::CHANNEL_1, Chimera::Timer::Channel::CHANNEL_1, Chimera::Timer::Channel::CHANNEL_2,
      Chimera::Timer::Channel::CHANNEL_2, Chimera::Timer::Channel::CHANNEL_3, Chimera::Timer::Channel::CHANNEL_3,
    };

    constexpr Chimera::Timer::Output switch_out_lut[] = {
      Chimera::Timer::Output::OUTPUT_1P, Chimera::Timer::Output::OUTPUT_1N, Chimera::Timer::Output::OUTPUT_2P,
      Chimera::Timer::Output::OUTPUT_2N, Chimera::Timer::Output::OUTPUT_3P, Chimera::Timer::Output::OUTPUT_3N,
    };

    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if( !isHiSideSwitch( hiSide ) || !isLoSideSwitch( loSide ) || !isDutyCycleInRange( dutyCycle ) )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------------------------------
    Reset the timer to a known state: No outputs enabled and all OC references
    set to an effectively 0% duty cycle.
    -------------------------------------------------------------------------*/
    ControlBlock *cb      = reinterpret_cast<ControlBlock *>( mTimerImpl );
    uint32_t      arr_val = getAutoReload( cb->timer );
    auto          result  = reset_to_idle();

    /*-------------------------------------------------------------------------
    Drive the requested switches with the expected duty cycle. This possibly
    could drive an undesired switch, but output channel masking will take care
    of this before it reaches the power stage.
    -------------------------------------------------------------------------*/
    uint32_t ocref = arr_val - static_cast<uint32_t>( arr_val * dutyCycle );
    result |= setOCReference( cb->timer, switch_ch_lut[ EnumValue( hiSide ) ], ocref );
    result |= setOCReference( cb->timer, switch_ch_lut[ EnumValue( loSide ) ], ocref );

    /*-------------------------------------------------------------------------
    Enable only the outputs requested.
    -------------------------------------------------------------------------*/
    enableCCOutput( cb->timer, switch_out_lut[ EnumValue( hiSide ) ] );
    enableCCOutput( cb->timer, switch_out_lut[ EnumValue( loSide ) ] );

    return result;
  }


  Chimera::Status_t Driver::svmUpdate( const float alpha, const float beta, const float theta )
  {
    /*-------------------------------------------------------------------------
    Local Constants
    -------------------------------------------------------------------------*/
    constexpr float PI_OVER_3      = static_cast<float>( M_PI / 3.0 );
    constexpr float SQRT_3_OVER_2  = static_cast<float>( M_SQRT3 / 2.0 );
    constexpr float ONE_DEG_IN_RAD = 0.0174533f;

    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    RT_DBG_ASSERT( theta >= 0.0f && theta <= 2.0f * static_cast<float>( M_PI ) );

    /*-------------------------------------------------------------------------
    Compute the drive strength from the magnitude of the inverse park transform
    output, saturating at max theoretical value.
    -------------------------------------------------------------------------*/
    float drive = hypotf( alpha, beta );
    if( drive > SQRT_3_OVER_2 )
    {
      drive = SQRT_3_OVER_2;
    }

    /*-------------------------------------------------------------------------
    Compute the current sector and angular offset inside that sector
    -------------------------------------------------------------------------*/
    const uint32_t sector       = static_cast<uint32_t>( theta / PI_OVER_3 );
    float          sector_angle = theta - ( sector * PI_OVER_3 );

    RT_DBG_ASSERT( sector <= 6 );
    RT_DBG_ASSERT( sector_angle <= PI_OVER_3 && sector_angle >= 0.0f );

    /*-------------------------------------------------------------------------
    Massage sector_angle a bit to prevent getting into weird edge cases with
    the trig functions. This is a bit of a hack, but it works alright.
    -------------------------------------------------------------------------*/
    if( sector_angle < ONE_DEG_IN_RAD )
    {
      sector_angle = ONE_DEG_IN_RAD;
    }
    else if( sector_angle > ( PI_OVER_3 - ONE_DEG_IN_RAD ) )
    {
      sector_angle = PI_OVER_3 - ONE_DEG_IN_RAD;
    }

    /*-------------------------------------------------------------------------
    Calculate the timing windows used to generate the PWM signals
    -------------------------------------------------------------------------*/
    ControlBlock *cb = reinterpret_cast<ControlBlock *>( mTimerImpl );

    const float ta      = cb->pwm_half_period_hz * drive * fastSine( PI_OVER_3 - sector_angle );
    const float tb      = cb->pwm_half_period_hz * drive * fastSine( sector_angle );
    const float tn      = cb->pwm_half_period_hz - ta - tb;
    const float tn_half = 0.5f * tn;

    /*-------------------------------------------------------------------------
    Compute the PWM compare values for each channel
    -------------------------------------------------------------------------*/
    const uint32_t ton_tn_half = static_cast<uint32_t>( tn_half * cb->frequency_hz );
    const uint32_t ton_1       = static_cast<uint32_t>( ( tn_half + ta ) * cb->frequency_hz );
    const uint32_t ton_2       = static_cast<uint32_t>( ( tn_half + tb ) * cb->frequency_hz );
    const uint32_t ton_3       = static_cast<uint32_t>( ( tn_half + ta + tb ) * cb->frequency_hz );

    RT_DBG_ASSERT( ton_tn_half <= cb->timer->registers->ARR );
    RT_DBG_ASSERT( ton_1 <= cb->timer->registers->ARR );
    RT_DBG_ASSERT( ton_2 <= cb->timer->registers->ARR );
    RT_DBG_ASSERT( ton_3 <= cb->timer->registers->ARR );

    /*-------------------------------------------------------------------------
    Set the PWM compare values for each channel depending on the sector.
    Additionally, decide references for computing the ADC sample time update.

    See the SpaceVectorModulation.ipynb notebook for how this came about.
    -------------------------------------------------------------------------*/
    uint32_t ref_phase_x_cc = 0; /**< Phase with the high side ON the longest time */
    uint32_t ref_phase_y_cc = 0; /**< Phase with the high side ON the 2nd longest time */

    switch( sector )
    {
      case 0:
        cb->svmState.t1_cc  = ton_tn_half;
        cb->svmState.t2_cc  = ton_1;
        cb->svmState.t3_cc  = ton_3;
        cb->svmState.phase1 = Chimera::Timer::Channel::CHANNEL_1;
        cb->svmState.phase2 = Chimera::Timer::Channel::CHANNEL_2;
        ref_phase_x_cc      = cb->svmState.t1_cc;
        ref_phase_y_cc      = cb->svmState.t2_cc;
        break;

      case 1:
        cb->svmState.t1_cc  = ton_2;
        cb->svmState.t2_cc  = ton_tn_half;
        cb->svmState.t3_cc  = ton_3;
        cb->svmState.phase1 = Chimera::Timer::Channel::CHANNEL_1;
        cb->svmState.phase2 = Chimera::Timer::Channel::CHANNEL_2;
        ref_phase_x_cc      = cb->svmState.t2_cc;
        ref_phase_y_cc      = cb->svmState.t1_cc;
        break;

      case 2:
        cb->svmState.t1_cc  = ton_3;
        cb->svmState.t2_cc  = ton_tn_half;
        cb->svmState.t3_cc  = ton_1;
        cb->svmState.phase1 = Chimera::Timer::Channel::CHANNEL_2;
        cb->svmState.phase2 = Chimera::Timer::Channel::CHANNEL_3;
        ref_phase_x_cc      = cb->svmState.t2_cc;
        ref_phase_y_cc      = cb->svmState.t3_cc;
        break;

      case 3:
        cb->svmState.t1_cc  = ton_3;
        cb->svmState.t2_cc  = ton_2;
        cb->svmState.t3_cc  = ton_tn_half;
        cb->svmState.phase1 = Chimera::Timer::Channel::CHANNEL_2;
        cb->svmState.phase2 = Chimera::Timer::Channel::CHANNEL_3;
        ref_phase_x_cc      = cb->svmState.t3_cc;
        ref_phase_y_cc      = cb->svmState.t2_cc;
        break;

      case 4:
        cb->svmState.t1_cc  = ton_1;
        cb->svmState.t2_cc  = ton_3;
        cb->svmState.t3_cc  = ton_tn_half;
        cb->svmState.phase1 = Chimera::Timer::Channel::CHANNEL_1;
        cb->svmState.phase2 = Chimera::Timer::Channel::CHANNEL_3;
        ref_phase_x_cc      = cb->svmState.t3_cc;
        ref_phase_y_cc      = cb->svmState.t1_cc;
        break;

      case 5:
        cb->svmState.t1_cc  = ton_tn_half;
        cb->svmState.t2_cc  = ton_3;
        cb->svmState.t3_cc  = ton_2;
        cb->svmState.phase1 = Chimera::Timer::Channel::CHANNEL_1;
        cb->svmState.phase2 = Chimera::Timer::Channel::CHANNEL_3;
        ref_phase_x_cc      = cb->svmState.t1_cc;
        ref_phase_y_cc      = cb->svmState.t3_cc;
        break;

      default:
        RT_HARD_ASSERT( false );
        break;
    }

    /*-------------------------------------------------------------------------
    Compute the ADC sample time reference. This is an extension of UM1052 5.2
    to all sectors. This assumes that the low side switches are driven while
    the timer is counting down from tmax, then back up.
    -------------------------------------------------------------------------*/
    RT_DBG_ASSERT( ref_phase_x_cc <= ref_phase_y_cc );
    const int32_t duty_a  = ref_phase_x_cc;
    const int32_t duty_a2 = 2 * duty_a;
    const int32_t duty_ab = ref_phase_y_cc - ref_phase_x_cc;

    if( duty_a >= cb->tstable_ticks ) /* Case 1 */
    {
      /*-----------------------------------------------------------------------
      ADC is sampled just after the phase with the longest high side ON time
      -----------------------------------------------------------------------*/
      cb->svmState.t4_cc        = duty_a - cb->tstable_ticks;
      cb->svmState.adc_polarity = true;
    }
    else
    {
      if( duty_ab >= duty_a2 ) /* Case 2 */
      {
        /*---------------------------------------------------------------------
        Since we're counting down to zero, we can use the negative value to
        detect if the ADC should be triggered on the rising or falling edge.
        ---------------------------------------------------------------------*/
        const int raw_offset = ref_phase_y_cc - duty_ab - cb->dt_ticks - cb->tn_ticks;
        if( raw_offset < 0 )
        {
          cb->svmState.t4_cc        = -1 * raw_offset;
          cb->svmState.adc_polarity = false;
        }
        else
        {
          cb->svmState.t4_cc        = raw_offset;
          cb->svmState.adc_polarity = true;
        }
      }
      else if( duty_ab < duty_a2 ) /* Case 3 */
      {
        cb->svmState.t4_cc        = ref_phase_y_cc - cb->tstable_ticks;
        cb->svmState.adc_polarity = true;
      }
      else /* Case 4: Panic */
      {
        /*-----------------------------------------------------------------------
        Impossible to sample! You need to lower the PWM frequency, decrease the
        modulation index, or decrease the ADC sample time.
        -----------------------------------------------------------------------*/
        emergencyBreak();
        RT_DBG_ASSERT( false );
        return Chimera::Status::FAIL;
      }
    }

    /*-------------------------------------------------------------------------
    Assign the computed compare values to the timer
    -------------------------------------------------------------------------*/
    setOCReference( cb->timer, Chimera::Timer::Channel::CHANNEL_1, cb->svmState.t1_cc );
    setOCReference( cb->timer, Chimera::Timer::Channel::CHANNEL_2, cb->svmState.t2_cc );
    setOCReference( cb->timer, Chimera::Timer::Channel::CHANNEL_3, cb->svmState.t3_cc );
    setOCReference( cb->timer, Chimera::Timer::Channel::CHANNEL_4, cb->svmState.t4_cc );

    return Chimera::Status::OK;
  }


  SVMState Driver::svmState()
  {
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if( !mTimerImpl )
    {
      return {};
    }

    /*-------------------------------------------------------------------------
    Get the phase channels that are being sampled
    -------------------------------------------------------------------------*/
    ControlBlock *cb = reinterpret_cast<ControlBlock *>( mTimerImpl );

    SVMState copy;
    copy.adc_polarity = cb->svmState.adc_polarity;
    copy.phase1       = cb->svmState.phase1;
    copy.phase2       = cb->svmState.phase2;
    copy.t1_cc        = cb->svmState.t1_cc;
    copy.t2_cc        = cb->svmState.t2_cc;
    copy.t3_cc        = cb->svmState.t3_cc;
    copy.t4_cc        = cb->svmState.t4_cc;

    return copy;
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


  /**
   * @brief Resets the timer to an inert state.
   *
   * No outputs are enabled and all OC references are set to an effectively 0% duty cycle.
   *
   */
  Chimera::Status_t Driver::reset_to_idle()
  {
    ControlBlock *cb      = reinterpret_cast<ControlBlock *>( mTimerImpl );
    uint32_t      arr_val = getAutoReload( cb->timer );
    auto          result  = Chimera::Status::OK;

    disableCCOutputBulk( cb->timer, s_all_output_channel_bf );
    result |= setOCReference( cb->timer, Chimera::Timer::Channel::CHANNEL_1, arr_val );
    result |= setOCReference( cb->timer, Chimera::Timer::Channel::CHANNEL_2, arr_val );
    result |= setOCReference( cb->timer, Chimera::Timer::Channel::CHANNEL_3, arr_val );

    return result;
  }

}    // namespace Chimera::Timer::Inverter
