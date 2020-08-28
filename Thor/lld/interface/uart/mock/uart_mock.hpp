/********************************************************************************
 *  File Name:
 *    uart_mock.hpp
 *
 *  Description:
 *    Mock interface for UART
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#ifndef THOR_LLD_UART_MOCK_HPP
#define THOR_LLD_UART_MOCK_HPP

/* STL Includes */
#include <memory>

/* LLD Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/uart/uart_intf.hpp>
#include <Thor/lld/interface/uart/uart_types.hpp>

#if defined( THOR_LLD_UART_MOCK )

/* Mock Includes */
#include "gmock/gmock.h"

namespace Thor::LLD::UART::Mock
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

  class DriverMock : virtual public Thor::LLD::UART::IDriver
  {
  public:
  };


  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  ModuleMock &getModuleMockObject();
  DriverMock &getDriverMockObject( const size_t channel );

}    // namespace Thor::LLD::UART::Mock

#endif /* THOR_LLD_UART_MOCK */
#endif /* !THOR_LLD_UART_MOCK_HPP */
