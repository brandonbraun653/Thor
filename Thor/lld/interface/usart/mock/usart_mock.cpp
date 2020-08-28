/********************************************************************************
 *  File Name:
 *    usart_mock.cpp
 *
 *  Description:
 *    Mocks the USART driver interface
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* STL Includes */
#include <array>

/* Mock Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/usart/mock/usart_mock.hpp>

#if defined( THOR_LLD_USART_MOCK )

namespace Thor::LLD::USART
{
  /*-------------------------------------------------------------------------------
  Static Data
  -------------------------------------------------------------------------------*/


  /*-------------------------------------------------------------------------------
  Mock Public Functions
  -------------------------------------------------------------------------------*/
  namespace Mock
  {
    static ModuleMock moduleMock;

    ModuleMock &getModuleMockObject()
    {
      return moduleMock;
    }

    DriverMock &getDriverMockObject( const size_t channel )
    {
      return s_usart_drivers[ channel ];
    }
  }    // namespace Mock


  /*-------------------------------------------------------------------------------
  Mock C-Style Interface
  -------------------------------------------------------------------------------*/

}    // namespace Thor::LLD::USART

#endif /* THOR_LLD_USART_MOCK */
