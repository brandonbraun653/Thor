/********************************************************************************
 *  File Name:
 *    hld_clock_chimera.cpp
 *
 *  Description:
 *    Implemenation of the Chimera GPIO driver hooks
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Chimera Includes */
#include <Chimera/clock>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/clock>

namespace Chimera::Clock::Backend
{
  Chimera::Status_t initialize()
  {
    return Thor::Clock::initialize();
  }

  Chimera::Status_t periphEnable( const Chimera::Peripheral::Type periph )
  {
    return Thor::Clock::periphEnable( periph );
  }

  Chimera::Status_t periphDisable( const Chimera::Peripheral::Type periph )
  {
    return Thor::Clock::periphDisable( periph );
  }

  Chimera::Status_t enableClock( const Chimera::Clock::Bus bus )
  {
    return Thor::Clock::enableClock( bus );
  }

  Chimera::Status_t disableClock( const Chimera::Clock::Bus bus )
  {
    return Thor::Clock::disableClock( bus );
  }

  bool isEnabled( const Chimera::Clock::Bus bus )
  {
    return Thor::Clock::isEnabled( bus );
  }

  size_t getFrequency( const Chimera::Clock::Bus bus )
  {
    return Thor::Clock::getFrequency( bus );
  }

  Chimera::Status_t setFrequency( const Chimera::Clock::Bus bus, const size_t freq )
  {
    return Thor::Clock::setFrequency( bus, freq );
  }

  Chimera::Status_t registerDriver( Chimera::Clock::Backend::DriverConfig &registry )
  {
#if defined( THOR_HLD_CLK )
    registry.isSupported           = true;
    registry.disableClock          = disableClock;
    registry.enableClock           = enableClock;
    registry.getFrequency          = getFrequency;
    registry.initialize            = initialize;
    registry.isEnabled             = isEnabled;
    registry.periphDisable         = periphDisable;
    registry.periphEnable          = periphEnable;
    registry.setFrequency          = setFrequency;
    return Chimera::CommonStatusCodes::OK;
#else
    registry.isSupported           = false;
    registry.disableClock          = nullptr;
    registry.enableClock           = nullptr;
    registry.getFrequency          = nullptr;
    registry.initialize            = nullptr;
    registry.isEnabled             = nullptr;
    registry.periphDisable         = nullptr;
    registry.periphEnable          = nullptr;
    registry.setFrequency          = nullptr;
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
#endif /* THOR_HLD_CLK */
  }
}    // namespace Chimera::Clock::Backend
