/********************************************************************************
 *  File Name:
 *    startup_mock.cpp
 *
 *  Description:
 *    Mocks the STARTUP driver interface
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* STL Includes */
#include <array>

/* Mock Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/startup/mock/startup_mock.hpp>

#if defined( THOR_LLD_STARTUP_MOCK )

namespace Thor::LLD::STARTUP
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
      return s_startup_drivers[ channel ];
    }
  }    // namespace Mock


  /*-------------------------------------------------------------------------------
  Mock C-Style Interface
  -------------------------------------------------------------------------------*/

}    // namespace Thor::LLD::STARTUP

#endif /* THOR_LLD_STARTUP_MOCK */
