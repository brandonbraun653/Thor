/********************************************************************************
 *  File Name:
 *    hld_clock_driver.cpp
 *
 *  Description:
 *    Implements the HLD clock driver for Thor
 *
 *  2020-2022 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <limits>
#include <Aurora/constants>
#include <Chimera/common>
#include <Chimera/clock>
#include <Thor/cfg>
#include <Thor/clock>
#include <Thor/lld/interface/inc/rcc>

#if defined( THOR_CLK )
namespace Chimera::Clock::Backend
{
  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  static Chimera::Status_t initialize()
  {
    return Chimera::Status::OK;
  }

  static Chimera::Status_t periphEnable( const Chimera::Peripheral::Type periph )
  {
    return Chimera::Status::NOT_AVAILABLE;
  }

  static Chimera::Status_t periphDisable( const Chimera::Peripheral::Type periph )
  {
    return Chimera::Status::NOT_AVAILABLE;
  }

  static Chimera::Status_t enableClock( const Chimera::Clock::Bus bus )
  {
    return Chimera::Status::NOT_AVAILABLE;
  }

  static Chimera::Status_t disableClock( const Chimera::Clock::Bus bus )
  {
    return Chimera::Status::NOT_AVAILABLE;
  }

  static bool isEnabled( const Chimera::Clock::Bus bus )
  {
    return false;
  }

  static size_t getFrequency( const Chimera::Clock::Bus bus )
  {
    return Thor::LLD::RCC::getBusFrequency( bus );
  }

  static Chimera::Status_t setFrequency( const Chimera::Clock::Bus bus, const size_t freq )
  {
    return Chimera::Status::NOT_AVAILABLE;
  }

  Chimera::Status_t registerDriver( Chimera::Clock::Backend::DriverConfig &registry )
  {
    registry.isSupported           = true;
    registry.disableClock          = disableClock;
    registry.enableClock           = enableClock;
    registry.getFrequency          = getFrequency;
    registry.initialize            = initialize;
    registry.isEnabled             = isEnabled;
    registry.periphDisable         = periphDisable;
    registry.periphEnable          = periphEnable;
    registry.setFrequency          = setFrequency;
    return Chimera::Status::OK;
  }
}    // namespace Thor::Clock

#endif /* THOR_CLK */
