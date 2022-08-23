/******************************************************************************
 *  File Name:
 *    hld_timer_pwm.cpp
 *
 *  Description:
 *    Thor implementation of the Chimera PWM timer driver
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
static constexpr size_t REQ_HW_TIMER_TYPES =
    ( Thor::LLD::TIMER::HardwareType::TIMER_HW_ADVANCED | Thor::LLD::TIMER::HardwareType::TIMER_HW_BASIC |
      Thor::LLD::TIMER::HardwareType::TIMER_HW_GENERAL );

/*-----------------------------------------------------------------------------
Structures
-----------------------------------------------------------------------------*/
/**
 * @brief PWM controller data
 */
struct PWMControlBlock
{
  Thor::LLD::TIMER::Handle_rPtr timer;   /**< Handle to the timer */
  Chimera::Timer::Channel       channel; /**< Which channel is in use */
  Chimera::Timer::Output        output;  /**< Which output is in use */
};

/*-----------------------------------------------------------------------------
Static Data
-----------------------------------------------------------------------------*/
static Chimera::DeviceManager<PWMControlBlock, Chimera::Timer::Instance, EnumValue( Chimera::Timer::Instance::NUM_OPTIONS )>
    s_timer_data;


namespace Chimera::Timer::PWM
{
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
    PWMControlBlock *cb = s_timer_data.getOrCreate( cfg.coreCfg.instance );
    RT_HARD_ASSERT( cb );

    if ( !mTimerImpl )
    {
      mTimerImpl = reinterpret_cast<void *>( cb );
    }

    /*-------------------------------------------------------------------------
    Initialize the control block data
    -------------------------------------------------------------------------*/
    RT_HARD_ASSERT( Chimera::Status::OK == allocate( cfg.coreCfg.instance ) );

    cb->timer   = Thor::LLD::TIMER::getHandle( cfg.coreCfg.instance );
    cb->channel = cfg.channel;
    cb->output  = cfg.output;

    /*-------------------------------------------------------------------------
    Configure the timer for desired PWM operation.
    -------------------------------------------------------------------------*/
    auto result = Chimera::Status::OK;
    result |= Thor::LLD::TIMER::Master::initCore( cb->timer, cfg.coreCfg );
    result |= Thor::LLD::TIMER::disableCCOutput( cb->timer, cfg.output );

    result |= setPolarity( cfg.polarity );
    result |= setFrequency( cfg.frequency );
    result |= setDutyCycle( cfg.dutyCycle );

    /*-------------------------------------------------------------------------
    Start the timer, enable generic output gate, but leave the channel output
    disabled. Set future PWM duty cycle updates to transition seamlessly.
    -------------------------------------------------------------------------*/
    Thor::LLD::TIMER::useOCPreload( cb->timer, cb->channel, true );
    Thor::LLD::TIMER::enableCounter( cb->timer );
    Thor::LLD::TIMER::enableAllOutput( cb->timer );

    return result;
  }


  Chimera::Status_t Driver::enableOutput()
  {
    /*-------------------------------------------------------------------------
    Enable the output compare channel. Assumes the timer is running.
    -------------------------------------------------------------------------*/
    PWMControlBlock *cb = reinterpret_cast<PWMControlBlock *>( mTimerImpl );
    return Thor::LLD::TIMER::enableCCOutput( cb->timer, cb->output );
  }


  Chimera::Status_t Driver::disableOutput()
  {
    /*-------------------------------------------------------------------------
    Disable the output compare channel, but don't stop the timer
    -------------------------------------------------------------------------*/
    PWMControlBlock *cb = reinterpret_cast<PWMControlBlock *>( mTimerImpl );
    return Thor::LLD::TIMER::disableCCOutput( cb->timer, cb->output );
  }


  Chimera::Status_t Driver::setFrequency( const float freq )
  {
    /*-------------------------------------------------------------------------
    Set the PWM frequency by controlling the timer overflow rate
    -------------------------------------------------------------------------*/
    PWMControlBlock *cb = reinterpret_cast<PWMControlBlock *>( mTimerImpl );
    return setEventRate( cb->timer, ( 1.0f / freq ) * 1e9f );
  }


  Chimera::Status_t Driver::setDutyCycle( const float dutyCycle )
  {
    /*-------------------------------------------------------------------------
    Calculate the new reference based on a percentage of the current TIMx_ARR.
    -------------------------------------------------------------------------*/
    PWMControlBlock *cb          = reinterpret_cast<PWMControlBlock *>( mTimerImpl );
    float            dutyPercent = dutyCycle / 100.0f;
    float            arr_val     = static_cast<float>( Thor::LLD::TIMER::getAutoReload( cb->timer ) );
    uint32_t         new_ref     = static_cast<uint32_t>( roundf( arr_val * dutyPercent ) );

    /*-------------------------------------------------------------------------
    Apply the new PWM reference
    -------------------------------------------------------------------------*/
    return Thor::LLD::TIMER::setOCReference( cb->timer, cb->channel, new_ref );
  }


  Chimera::Status_t Driver::setPolarity( const Chimera::Timer::PWM::Polarity polarity )
  {
    PWMControlBlock *cb = reinterpret_cast<PWMControlBlock *>( mTimerImpl );

    /*-------------------------------------------------------------------------
    On compare match, set the output to the desired active state
    -------------------------------------------------------------------------*/
    auto result = Thor::LLD::TIMER::setOCMode( cb->timer, cb->channel, Thor::LLD::TIMER::OC_MODE_PWM_MODE_1 );

    /*-------------------------------------------------------------------------
    Now set the desired active state (polarity)
    -------------------------------------------------------------------------*/
    switch ( polarity )
    {
      case Chimera::Timer::PWM::Polarity::ACTIVE_HIGH:
        result |= Thor::LLD::TIMER::setCCOutputPolarity( cb->timer, cb->output, Thor::LLD::TIMER::CCPolarity::CCP_OUT_ACTIVE_HIGH );
        break;

      case Chimera::Timer::PWM::Polarity::ACTIVE_LOW:
        result |= Thor::LLD::TIMER::setCCOutputPolarity( cb->timer, cb->output, Thor::LLD::TIMER::CCPolarity::CCP_OUT_ACTIVE_LOW );
        break;

      default:
        return Chimera::Status::NOT_SUPPORTED;
    };

    return result;
  }

}    // namespace Chimera::Timer::PWM
