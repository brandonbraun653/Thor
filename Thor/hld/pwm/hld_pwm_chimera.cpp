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
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::PWM::PWM_sPtr create_shared_ptr()
  {
    return std::make_shared<Thor::PWM::Driver>();
  }

  Chimera::PWM::PWM_uPtr create_unique_ptr()
  {
    return std::make_unique<Thor::PWM::Driver>();
  }

  size_t numSupportedChannels()
  {
    return 0;    // TODO
  }

  Chimera::Status_t registerDriver( Chimera::PWM::Backend::DriverRegistration &registry )
  {
#if defined( THOR_HLD_PWM )
    registry.isSupported          = true;
    registry.createShared         = create_shared_ptr;
    registry.createUnique         = create_unique_ptr;
    registry.initialize           = initialize;
    registry.reset                = reset;
    registry.numSupportedChannels = numSupportedChannels;
    return Chimera::CommonStatusCodes::OK;
#else
    registry.isSupported          = false;
    registry.createShared         = nullptr;
    registry.createUnique         = nullptr;
    registry.initialize           = nullptr;
    registry.reset                = nullptr;
    registry.numSupportedChannels = nullptr;
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
#endif /* THOR_DRIVER_PWM == 1*/
  }
}    // namespace Chimera::PWM::Backend