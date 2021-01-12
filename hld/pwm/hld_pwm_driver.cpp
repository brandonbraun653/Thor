/********************************************************************************
 *  File Name:
 *    hld_pwm_driver.cpp
 *
 *  Description:
 *    HLD driver implementation for Thor PWM
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* STL Includes */
#include <cstring>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/pwm>
#include <Chimera/thread>

/* Thor Includes */
#include <Thor/gpio>
#include <Thor/pwm>
#include <Thor/timer>

namespace Thor::PWM
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    return Chimera::Status::OK;
  }


  Chimera::Status_t reset()
  {
    return Chimera::Status::OK;
  }


  Chimera::PWM::IPWM_sPtr getDriver( const size_t channel )
  {
    return nullptr;
  }


  size_t numSupportedChannels()
  {
    return 0;
  }

/*-------------------------------------------------------------------------------
Classes
-------------------------------------------------------------------------------*/
#if 0
  Driver::Driver() : mInitialized( false ), mpOutputPin( nullptr ), mpTimerDriver( nullptr )
  {

  }

  Driver::~Driver()
  {

  }

  Chimera::Status_t Driver::init( const Chimera::PWM::DriverConfig &cfg )
  {
    using namespace Chimera::Timer;

    /*------------------------------------------------
    Input validity checks
    ------------------------------------------------*/
    if ( !cfg.validity )
    {
      return Chimera::Status::FAIL;
    }

    /*------------------------------------------------
    Check if a timer peripheral exists for the requested PWM channel.
    If it does, grab the driver, creating one if it doesn't exist yet.
    ------------------------------------------------*/
    mpTimerDriver = nullptr; //Chimera::Timer::getSharedInstance( cfg.timer.peripheral );
    if ( !mpTimerDriver )
    {
      return Chimera::Status::NOT_AVAILABLE;
    }

    /*------------------------------------------------
    Configure the timer peripheral. This could already be in use
    by another entity, so try not to blow away configuration settings.
    ------------------------------------------------*/
    auto result = Chimera::Status::OK;
    bool configured = false;
    CoreFeatureInit tmp;

    //mpTimerDriver->requestData( DriverData::IS_CONFIGURED, &configured, sizeof( configured ) );

    /* Configure the base timer if needed */
    if ( !configured || cfg.timer.overwrite )
    {
      memset( &tmp, 0, sizeof( tmp ) );
      tmp.base = cfg.timer;
      //result |= mpTimerDriver->initializeCoreFeature( CoreFeature::BASE_TIMER, tmp );
    }

    /* Configure the PWM portion of the timer */
    memset( &tmp, 0, sizeof( tmp ) );
    tmp.pwm = cfg.pwm;
    //result |= mpTimerDriver->initializeCoreFeature( CoreFeature::PWM_OUTPUT, tmp );

    /*------------------------------------------------
    Configure the GPIO for timer output
    ------------------------------------------------*/
    mpOutputPin = std::make_unique<Thor::GPIO::Driver>();
    result |= mpOutputPin->init( cfg.outputPin );

    /*------------------------------------------------
    Assuming everything is OK, save class data
    ------------------------------------------------*/
    if ( result == Chimera::Status::OK )
    {
      mInitialized = true;
      mPWMConfig   = cfg.pwm;
      mPeripheral  = cfg.timer.peripheral;
    }

    return result;
  }

  Chimera::Status_t Driver::toggleOutput( const bool state )
  {
    using namespace Chimera::Timer;

    /*------------------------------------------------
    HW Protection and Thread Safety
    ------------------------------------------------*/
    if ( !mInitialized )
    {
      return Chimera::Status::NOT_INITIALIZED;
    }
    else if( !try_lock_for( Chimera::Threading::TIMEOUT_25MS ) )
    {
      return Chimera::Status::LOCKED;
    }

    /*------------------------------------------------
    Enable the PWM output
    ------------------------------------------------*/
    auto action = state ? DriverAction::PWM_ENABLE_CHANNEL : DriverAction::PWM_DISABLE_CHANNEL;
    //auto result = mpTimerDriver->invokeAction( action, &mPWMConfig.outputChannel, sizeof( mPWMConfig.outputChannel ) );

    /*------------------------------------------------
    Unlock the driver and return the result
    ------------------------------------------------*/
    unlock();
    return Chimera::Status::NOT_AVAILABLE;
  }

  Chimera::Status_t Driver::setFrequency( const size_t freq )
  {
    /*
    1. Grab the current base timer configuration
    2. Grab information about the timer (clock bus)
    3. Use the RCC driver to grab the source clock
    4. Update the clock prescaler and auto-reload value (converging algorithm)
        a) Allowed to be naive approach due to infrequent calls
    */
    return Chimera::Status::NOT_AVAILABLE;
  }

  Chimera::Status_t Driver::setDutyCyle( const size_t dutyCycle )
  {
    using namespace Chimera::Timer;

    DriverAction_PWMDutyCycle_t action;
    action.channel = mPWMConfig.outputChannel;
    action.dutyCycle = dutyCycle;

    //return mpTimerDriver->invokeAction( DriverAction::PWM_SET_DUTY_CYCLE, &action, sizeof( DriverAction_PWMDutyCycle_t ) );
    return Chimera::Status::NOT_AVAILABLE;
  }

  Chimera::Status_t Driver::setPolarity( const Chimera::Timer::PWM::Polarity polarity )
  {
    /*
    1. This one should be pretty simple. Just send a command with the desired state.
    */
    return Chimera::Status::NOT_AVAILABLE;
  }

  Chimera::Status_t Driver::applyConfig( const size_t freq, const size_t dutyCycle,
                                         const Chimera::Timer::PWM::Polarity polarity )
  {
    using namespace Chimera::Timer;

    /*------------------------------------------------
    HW Protection and Thread Safety
    ------------------------------------------------*/
    if ( !mInitialized )
    {
      return Chimera::Status::NOT_INITIALIZED;
    }
    else if( !try_lock_for( Chimera::Threading::TIMEOUT_25MS ) )
    {
      return Chimera::Status::LOCKED;
    }

    //    /*------------------------------------------------
    //    Set the new frequency/duty cycle
    //    ------------------------------------------------*/
    //    mPWMConfig.frequency = freq;
    //    mPWMConfig.dutyCycle = dutyCycle;
    //
    //
    //    CoreFeatureInit tmp;
    //    memset( &tmp, 0, sizeof( tmp ) );
    //    tmp.pwm = mPWMConfig;
    //    auto result = mpTimerDriver->initializeCoreFeature( CoreFeature::PWM_OUTPUT, tmp );
    //
    //    /*------------------------------------------------
    //    Unlock the driver and return the result
    //    ------------------------------------------------*/
    //    unlock();
    //    return result;

    return Chimera::Status::NOT_AVAILABLE;
  }

#endif
}    // namespace Thor::PWM
