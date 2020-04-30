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
  Free Function Definitions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    return Chimera::CommonStatusCodes::OK;
  }

  /*-------------------------------------------------------------------------------
  HLD Driver Definitions
  -------------------------------------------------------------------------------*/
  Driver::Driver() : mInitialized( false ), mpOutputPin( nullptr ), mpPWMDriver( nullptr ), mpTimerBaseDriver( nullptr )
  {

  }

  Driver::~Driver()
  {

  }

  Chimera::Status_t Driver::init( const Chimera::PWM::DriverConfig &cfg )
  {
    /*------------------------------------------------
    Input validity checks
    ------------------------------------------------*/
    if ( !cfg.validity )
    {
      return Chimera::CommonStatusCodes::FAIL;
    }

    /*------------------------------------------------
    Check if a timer peripheral exists for the requested PWM channel.
    If it does, grab the driver, creating one if it doesn't exist yet.
    ------------------------------------------------*/
    #if defined( VIRTUAL_FUNC )
    mpTimerBaseDriver = Thor::TIMER::getTimerAsBase( cfg.timer.peripheral );
    mpPWMDriver = Thor::TIMER::getTimerAsPWM( cfg.timer.peripheral );
    #else 
    mpTimerBaseDriver = Chimera::Timer::getTimerAsBase( cfg.timer.peripheral );
    mpPWMDriver = Chimera::Timer::getTimerAsPWM( cfg.timer.peripheral );
    #endif 


    if ( !mpPWMDriver )
    {
      return Chimera::CommonStatusCodes::NOT_AVAILABLE;
    }

    /*------------------------------------------------
    Configure the timer peripheral. This could already be in use
    by another entity, so try not to blow away configuration settings.
    ------------------------------------------------*/
    auto result = Chimera::CommonStatusCodes::OK;
    if ( !mpTimerBaseDriver->configured() || cfg.timer.overwrite )
    {
      result |= mpTimerBaseDriver->initPeripheral( cfg.timer );
    }

    result |= mpPWMDriver->pwmInit( cfg.pwm );

    /*------------------------------------------------
    Configure the GPIO for timer output
    ------------------------------------------------*/
    mpOutputPin = std::make_unique<Thor::GPIO::Driver>();
    result |= mpOutputPin->init( cfg.outputPin, Chimera::Threading::TIMEOUT_DONT_WAIT );

    /*------------------------------------------------
    Assuming everything is OK, save class data
    ------------------------------------------------*/
    if ( result == Chimera::CommonStatusCodes::OK )
    {
      mInitialized = true;
      mPWMConfig   = cfg.pwm;
      mPeripheral  = cfg.timer.peripheral;
    }

    return result;
  }

  Chimera::Status_t Driver::enableOutput()
  {
    /*------------------------------------------------
    HW Protection and Thread Safety
    ------------------------------------------------*/
    if ( !mInitialized )
    {
      return Chimera::CommonStatusCodes::NOT_INITIALIZED;
    }
    else if( !try_lock_for( Chimera::Threading::TIMEOUT_25MS ) )
    {
      return Chimera::CommonStatusCodes::LOCKED;
    }

    /*------------------------------------------------
    Enable the PWM output
    ------------------------------------------------*/
    auto result = mpTimerBaseDriver->enable( mPWMConfig.outputChannel );

    /*------------------------------------------------
    Unlock the driver and return the result
    ------------------------------------------------*/
    unlock();
    return result;
  }

  Chimera::Status_t Driver::disableOutput()
  {
    /*------------------------------------------------
    HW Protection and Thread Safety
    ------------------------------------------------*/
    if ( !mInitialized )
    {
      return Chimera::CommonStatusCodes::NOT_INITIALIZED;
    }
    else if( !try_lock_for( Chimera::Threading::TIMEOUT_25MS ) )
    {
      return Chimera::CommonStatusCodes::LOCKED;
    }

    /*------------------------------------------------
    Disable the PWM output
    ------------------------------------------------*/
    auto result = mpTimerBaseDriver->disable( mPWMConfig.outputChannel );

    /*------------------------------------------------
    Unlock the driver and return the result
    ------------------------------------------------*/
    unlock();
    return result;
  }

  Chimera::Status_t Driver::setFrequency( const size_t freq )
  {
    return applyConfig( freq, mPWMConfig.dutyCycle, mPWMConfig.polarity );
  }

  Chimera::Status_t Driver::setDutyCyle( const size_t dutyCycle )
  {
    return applyConfig( mPWMConfig.frequency, dutyCycle, mPWMConfig.polarity );
  }

  Chimera::Status_t Driver::setPolarity( const Chimera::Timer::PWM::Polarity polarity )
  {
    return applyConfig( mPWMConfig.frequency, mPWMConfig.dutyCycle, polarity );
  }

  Chimera::Status_t Driver::applyConfig( const size_t freq, const size_t dutyCycle,
                                         const Chimera::Timer::PWM::Polarity polarity )
  {
    /*------------------------------------------------
    HW Protection and Thread Safety
    ------------------------------------------------*/
    if ( !mInitialized )
    {
      return Chimera::CommonStatusCodes::NOT_INITIALIZED;
    }
    else if( !try_lock_for( Chimera::Threading::TIMEOUT_25MS ) )
    {
      return Chimera::CommonStatusCodes::LOCKED;
    }

    /*------------------------------------------------
    Set the new frequency/duty cycle
    ------------------------------------------------*/
    mPWMConfig.frequency = freq;
    mPWMConfig.dutyCycle = dutyCycle;
    auto result = mpPWMDriver->pwmInit( mPWMConfig );

    /*------------------------------------------------
    Unlock the driver and return the result
    ------------------------------------------------*/
    unlock();
    return result;
  }
}
