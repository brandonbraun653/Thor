/********************************************************************************
 *  File Name:
 *    usart_mock_variant.cpp
 *
 *  Description:
 *    Thor USART mock variant
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* STL Includes */
#include <cstddef>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/usart/mock/usart_mock_variant.hpp>

#if defined( THOR_LLD_USART_MOCK )

namespace Thor::LLD::USART
{
  /*-------------------------------------------------------------------------------
  External Variables
  -------------------------------------------------------------------------------*/
  std::array<RegisterMap*, NUM_USART_PERIPHS> PeripheralRegisterMaps;

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
}  // namespace Thor::LLD::USART

#endif /* THOR_LLD_USART_MOCK */
