/********************************************************************************
 *  File Name:
 *    flash_mock.cpp
 *
 *  Description:
 *    Mocks the FLASH driver interface
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* STL Includes */
#include <array>

/* Mock Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/flash/mock/flash_mock.hpp>

#if defined( THOR_LLD_FLASH_MOCK )

namespace Thor::LLD::FLASH
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
      return s_flash_drivers[ channel ];
    }
  }    // namespace Mock


  /*-------------------------------------------------------------------------------
  Mock C-Style Interface
  -------------------------------------------------------------------------------*/

}    // namespace Thor::LLD::FLASH

#endif /* THOR_LLD_FLASH_MOCK */
