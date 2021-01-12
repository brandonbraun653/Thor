/********************************************************************************
 *  File Name:
 *    uart_mock_variant.cpp
 *
 *  Description:
 *    Thor UART mock variant
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* STL Includes */
#include <cstddef>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/uart/mock/uart_mock_variant.hpp>

#if defined( THOR_LLD_UART_MOCK )

namespace Thor::LLD::UART
{
  /*-------------------------------------------------------------------------------
  External Variables
  -------------------------------------------------------------------------------*/
  std::array<RegisterMap*, NUM_UART_PERIPHS> PeripheralRegisterMaps;

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
}  // namespace Thor::LLD::UART

#endif /* THOR_LLD_UART_MOCK */
