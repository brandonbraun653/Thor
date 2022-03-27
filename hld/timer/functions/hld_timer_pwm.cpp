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
#include <Chimera/common>
#include <Chimera/timer>


namespace Chimera::Timer::PWM
{
  Driver::Driver()
  {
  }


  Driver::~Driver()
  {
  }


  Chimera::Status_t Driver::init( const DriverConfig &cfg )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::enableOutput()
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::disableOutput()
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::setFrequency( const size_t freq )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::setDutyCycle( const size_t dutyCycle )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::setPolarity( const Chimera::Timer::PWM::OutputPolarity polarity )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }

}    // namespace Chimera::Timer::PWM
