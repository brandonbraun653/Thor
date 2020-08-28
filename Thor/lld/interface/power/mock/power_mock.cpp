/********************************************************************************
 *  File Name:
 *    power_mock.cpp
 *
 *  Description:
 *    Mocks the PWR driver interface
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* STL Includes */
#include <array>

/* Mock Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/power/mock/power_mock.hpp>

#if defined( THOR_LLD_PWR_MOCK )

namespace Thor::LLD::PWR
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
      return s_power_drivers[ channel ];
    }
  }    // namespace Mock


  /*-------------------------------------------------------------------------------
  Mock C-Style Interface
  -------------------------------------------------------------------------------*/

}    // namespace Thor::LLD::PWR

#endif /* THOR_LLD_PWR_MOCK */
