/********************************************************************************
 *  File Name:
 *    watchdog_mock.hpp
 *
 *  Description:
 *    Mock interface for WATCHDOG
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#ifndef THOR_LLD_WATCHDOG_MOCK_HPP
#define THOR_LLD_WATCHDOG_MOCK_HPP

/* STL Includes */
#include <memory>

/* LLD Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/watchdog/watchdog_intf.hpp>
#include <Thor/lld/interface/watchdog/watchdog_types.hpp>

#if defined( THOR_LLD_WATCHDOG_MOCK )

/* Mock Includes */
#include "gmock/gmock.h"

namespace Thor::LLD::WATCHDOG::Mock
{
  /*-------------------------------------------------------------------------------
  Mock Interfaces
  -------------------------------------------------------------------------------*/
  class IModule
  {
  public:
    virtual ~IModule() = default;
  };


  /*-------------------------------------------------------------------------------
  Mock Classes
  -------------------------------------------------------------------------------*/
  class ModuleMock : public IModule
  {
  public:
  };

  class DriverMock : virtual public Thor::LLD::WATCHDOG::IDriver
  {
  public:
  };


  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  ModuleMock &getModuleMockObject();
  DriverMock &getDriverMockObject( const size_t channel );

}    // namespace Thor::LLD::WATCHDOG::Mock

#endif /* THOR_LLD_WATCHDOG_MOCK */
#endif /* !THOR_LLD_WATCHDOG_MOCK_HPP */
