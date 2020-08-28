/********************************************************************************
 *  File Name:
 *    usart_mock.hpp
 *
 *  Description:
 *    Mock interface for USART
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#ifndef THOR_LLD_USART_MOCK_HPP
#define THOR_LLD_USART_MOCK_HPP

/* STL Includes */
#include <memory>

/* LLD Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/usart/usart_intf.hpp>
#include <Thor/lld/interface/usart/usart_types.hpp>

#if defined( THOR_LLD_USART_MOCK )

/* Mock Includes */
#include "gmock/gmock.h"

namespace Thor::LLD::USART::Mock
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

  class DriverMock : virtual public Thor::LLD::USART::IDriver
  {
  public:
  };


  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  ModuleMock &getModuleMockObject();
  DriverMock &getDriverMockObject( const size_t channel );

}    // namespace Thor::LLD::USART::Mock

#endif /* THOR_LLD_USART_MOCK */
#endif /* !THOR_LLD_USART_MOCK_HPP */
