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

/* Thor Includes */
#include <Thor/pwm>

namespace Thor::PWM
{
  /*-------------------------------------------------
  Free Function Implementation
  -------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    return Chimera::CommonStatusCodes::OK;
  }

  /*-------------------------------------------------
  Driver Implementation
  -------------------------------------------------*/
  Driver::Driver()
  {

  }

  Driver::~Driver()
  {

  }

  Chimera::Status_t Driver::init( const Chimera::PWM::DriverConfig &cfg )
  {
    return Chimera::CommonStatusCodes::FAIL;
  }

  void Driver::enableOutput()
  {
  }

  void Driver::disableOutput()
  {
  }

  Chimera::Status_t Driver::setFrequency( const size_t freq )
  {
    return Chimera::CommonStatusCodes::FAIL;
  }

  Chimera::Status_t Driver::setDutyCyle( const size_t dutyCycle )
  {
    return Chimera::CommonStatusCodes::FAIL;
  }
}
