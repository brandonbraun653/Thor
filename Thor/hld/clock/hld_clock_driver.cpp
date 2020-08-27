/********************************************************************************
 *  File Name:
 *    hld_clock_driver.cpp
 *
 *  Description:
 *    Implements the HLD clock driver for Thor
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* STL Includes */
#include <limits>

/* Aurora Includes */
#include <Aurora/constants/common.hpp>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/clock>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/clock>
#include <Thor/lld/interface/rcc/rcc_intf.hpp>
#include <Thor/lld/interface/rcc/rcc_detail.hpp>

#if defined( THOR_HLD_CLK )

namespace Thor::Clock
{
  // Tracks if the module data has been initialized correctly
  static size_t s_driver_initialized;

  /*------------------------------------------------
  High Level Driver Free Functions
  ------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    /*------------------------------------------------
    Prevent re-initialization from occurring
    ------------------------------------------------*/
    auto result = Chimera::Status::OK;
    if ( s_driver_initialized == Chimera::DRIVER_INITIALIZED_KEY )
    {
      return result;
    }

    /*------------------------------------------------
    Initialize local memory
    ------------------------------------------------*/
    s_driver_initialized = ~Chimera::DRIVER_INITIALIZED_KEY;


    /*------------------------------------------------
    Make sure we can't be initialized again, then exit
    ------------------------------------------------*/
    s_driver_initialized = Chimera::DRIVER_INITIALIZED_KEY;
    return result;
  }

  Chimera::Status_t periphEnable( const Chimera::Peripheral::Type periph )
  {
    return Chimera::Status::NOT_AVAILABLE;
  }

  Chimera::Status_t periphDisable( const Chimera::Peripheral::Type periph )
  {
    return Chimera::Status::NOT_AVAILABLE;
  }

  Chimera::Status_t enableClock( const Chimera::Clock::Bus bus )
  {
    return Chimera::Status::NOT_AVAILABLE;
  }

  Chimera::Status_t disableClock( const Chimera::Clock::Bus bus )
  {
    return Chimera::Status::NOT_AVAILABLE;
  }

  bool isEnabled( const Chimera::Clock::Bus bus )
  {
    return false;
  }

  size_t getFrequency( const Chimera::Clock::Bus bus )
  {
    using namespace Thor::LLD::RCC;

    auto rcc = getCoreClock();
    return rcc->getClockFrequency( bus );
  }

  Chimera::Status_t setFrequency( const Chimera::Clock::Bus bus, const size_t freq )
  {
    using namespace Thor::LLD::RCC;

    auto rcc = getCoreClock();
    return rcc->setClockFrequency( bus, freq, false );
  }

}    // namespace Thor::Clock

#endif /* THOR_HLD_CLK */
