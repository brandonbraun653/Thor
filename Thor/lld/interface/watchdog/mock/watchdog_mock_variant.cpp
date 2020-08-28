/********************************************************************************
 *  File Name:
 *    watchdog_mock_variant.cpp
 *
 *  Description:
 *    Thor WATCHDOG mock variant
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* STL Includes */
#include <cstddef>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/watchdog/mock/watchdog_mock_variant.hpp>

#if defined( THOR_LLD_WATCHDOG_MOCK )

namespace Thor::LLD::WATCHDOG
{
  /*-------------------------------------------------------------------------------
  External Variables
  -------------------------------------------------------------------------------*/
  std::array<RegisterMap*, NUM_WATCHDOG_PERIPHS> PeripheralRegisterMaps;

  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  void initializeRegisters()
  {
    for( size_t x=0; x<PeripheralRegisterMaps.size(); x++)
    {
      if( PeripheralRegisterMaps[x])
      {
        delete PeripheralRegisterMaps[ x ];
      }

      PeripheralRegisterMaps[x] = new RegisterMap();
    }
  }


  void initializeMapping()
  {

  }
}  // namespace Thor::LLD::WATCHDOG

#endif /* THOR_LLD_WATCHDOG_MOCK */
