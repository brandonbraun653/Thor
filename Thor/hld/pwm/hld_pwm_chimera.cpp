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
    return Thor::PWM::initializeModule();
  }

  Chimera::Status_t reset()
  {
    return Chimera::Status::OK;
  }

  Chimera::PWM::IPWM_sPtr getDriver( const size_t channel )
  {
    return std::make_shared<Thor::PWM::Driver>();
  }

  size_t numSupportedChannels()
  {
    return 0;    // TODO
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
    registry.createShared         = nullptr;
    registry.createUnique         = nullptr;
    registry.initialize           = nullptr;
    registry.reset                = nullptr;
    registry.numSupportedChannels = nullptr;
    return Chimera::Status::NOT_SUPPORTED;
#endif /* THOR_DRIVER_PWM == 1*/
  }
}    // namespace Chimera::PWM::Backend