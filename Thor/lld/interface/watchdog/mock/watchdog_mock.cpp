/********************************************************************************
 *  File Name:
 *    watchdog_mock.cpp
 *
 *  Description:
 *    Mocks the WATCHDOG driver interface
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* STL Includes */
#include <array>

/* Mock Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/watchdog/mock/watchdog_mock.hpp>

#if defined( THOR_LLD_WATCHDOG_MOCK )

namespace Thor::LLD::WATCHDOG
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
      return s_watchdog_drivers[ channel ];
    }
  }    // namespace Mock


  /*-------------------------------------------------------------------------------
  Mock C-Style Interface
  -------------------------------------------------------------------------------*/

}    // namespace Thor::LLD::WATCHDOG

#endif /* THOR_LLD_WATCHDOG_MOCK */
