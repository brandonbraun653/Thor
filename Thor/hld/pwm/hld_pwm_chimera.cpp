/********************************************************************************
 *  File Name:
 *    hld_pwm_chimera.cpp
 *
 *	 Description:
 *    Implementation of Chimera PWM driver hooks
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* STL Includes */
#include <memory>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/pwm>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/pwm>

namespace Chimera::PWM::Backend
{
  Chimera::Status_t initialize()
  {
    return Thor::PWM::initialize();
  }

  Chimera::Status_t reset()
  {
    return Thor::PWM::reset();
  }

  Chimera::PWM::IPWM_sPtr getDriver( const size_t channel )
  {
    return Thor::PWM::getDriver( channel );
  }

  size_t numSupportedChannels()
  {
    return Thor::PWM::numSupportedChannels();
  }

  Chimera::Status_t registerDriver( Chimera::PWM::Backend::DriverRegistration &registry )
  {
#if defined( THOR_HLD_PWM )
    registry.isSupported          = true;
    registry.getDriver            = getDriver;
    registry.initialize           = initialize;
    registry.reset                = reset;
    registry.numSupportedChannels = numSupportedChannels;
    return Chimera::Status::OK;
#else
    registry.isSupported          = false;
    registry.getDriver            = nullptr;
    registry.initialize           = nullptr;
    registry.reset                = nullptr;
    registry.numSupportedChannels = nullptr;
    return Chimera::Status::NOT_SUPPORTED;
#endif /* THOR_HLD_PWM */
  }
}    // namespace Chimera::PWM::Backend
