/********************************************************************************
 *  File Name:
 *    timer_mock_variant.cpp
 *
 *  Description:
 *    Thor TIMER mock variant
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* STL Includes */
#include <cstddef>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/timer/mock/timer_mock_variant.hpp>

#if defined( THOR_LLD_TIMER_MOCK )

namespace Thor::LLD::TIMER
{
  /*-------------------------------------------------------------------------------
  External Variables
  -------------------------------------------------------------------------------*/
  std::array<RegisterMap*, NUM_TIMER_PERIPHS> PeripheralRegisterMaps;

  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/

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
}  // namespace Thor::LLD::TIMER

#endif /* THOR_LLD_TIMER_MOCK */
