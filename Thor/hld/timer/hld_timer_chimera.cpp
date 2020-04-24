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
  Chimera::Status_t initialize()
  {
    return Thor::Timer::initialize();
  }

  Chimera::Status_t reset()
  {
    return Thor::Timer::reset();
  }

  Chimera::Timer::ITimer_sPtr create_shared_ptr()
  {
    return std::make_shared<Thor::Timer::Driver>();
  }

  Chimera::Timer::ITimer_uPtr create_unique_ptr()
  {
    return std::make_unique<Thor::Timer::Driver>();
  }

  size_t millis()
  {
    return Thor::Timer::millis();
  }

  void delayMilliseconds( const size_t val )
  {
    Thor::Timer::delayMilliseconds( val );
  }

  void delayMicroseconds( const size_t val )
  {
    Thor::Timer::delayMicroseconds( val );
  }

  Chimera::Status_t registerDriver( Chimera::Timer::Backend::DriverRegistration &registry )
  {
#if defined( THOR_HLD_TIMER )
    registry.isSupported       = true;
    registry.initialize        = initialize;
    registry.reset             = reset;
    registry.create_shared_ptr = create_shared_ptr;
    registry.create_unique_ptr = create_unique_ptr;
    registry.delayMicroseconds = delayMicroseconds;
    registry.delayMilliseconds = delayMilliseconds;
    registry.millis            = millis;
    return Chimera::CommonStatusCodes::OK;
#else
    registry.isSupported       = false;
    registry.initialize        = nullptr;
    registry.reset             = nullptr;
    registry.create_shared_ptr = nullptr;
    registry.create_unique_ptr = nullptr;
    registry.delayMicroseconds = nullptr;
    registry.delayMilliseconds = nullptr;
    registry.millis            = nullptr;
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
#endif /* THOR_HLD_TIMER */
  }

}    // namespace Chimera::Timer::Backend