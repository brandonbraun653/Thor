/********************************************************************************
 *  File Name:
 *    uart_mock.cpp
 *
 *  Description:
 *    Mocks the UART driver interface
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* STL Includes */
#include <array>

/* Mock Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/uart/mock/uart_mock.hpp>

#if defined( THOR_LLD_UART_MOCK )

namespace Thor::LLD::UART
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
      return s_uart_drivers[ channel ];
    }
  }    // namespace Mock


  /*-------------------------------------------------------------------------------
  Mock C-Style Interface
  -------------------------------------------------------------------------------*/

}    // namespace Thor::LLD::UART

#endif /* THOR_LLD_UART_MOCK */
