/********************************************************************************
 *  File Name:
 *    hld_timer_chimera.cpp
 *
 *	 Description:
 *    Implementation of Chimera Timer driver hooks
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* STL Includes */
#include <memory>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/timer>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/timer>

namespace Chimera::Timer::Backend
{
  Chimera::Status_t registerDriver( Chimera::Timer::Backend::DriverConfig &registry )
  {
#if defined( THOR_HLD_TIMER )
    registry.isSupported  = true;
    registry.createShared = create_shared_ptr;
    registry.createUnique = create_unique_ptr;
    registry.initialize   = initialize;
    registry.reset        = reset;
    return Chimera::CommonStatusCodes::OK;
#else
    registry.isSupported       = false;
    registry.initialize        = nullptr;
    registry.reset             = nullptr;
    registry.delayMicroseconds = nullptr;
    registry.delayMilliseconds = nullptr;
    registry.millis            = nullptr;
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
#endif /* THOR_DRIVER_TIMER == 1*/
  }

  // Chimera::Status_t initialize()
  // {
  //   return Thor::TIMER::initialize();
  // }

  // Chimera::Status_t reset()
  // {
  //   return Chimera::CommonStatusCodes::OK;
  // }

  // Chimera::TIMER::TIMER_sPtr create_shared_ptr()
  // {
  //   return std::make_shared<Thor::TIMER::Driver>();
  // }

  // Chimera::TIMER::TIMER_uPtr create_unique_ptr()
  // {
  //   return std::make_unique<Thor::TIMER::Driver>();
  // }
}    // namespace Chimera::Timer::Backend