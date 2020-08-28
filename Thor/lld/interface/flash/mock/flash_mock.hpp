/********************************************************************************
 *  File Name:
 *    flash_mock.hpp
 *
 *  Description:
 *    Mock interface for FLASH
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#ifndef THOR_LLD_FLASH_MOCK_HPP
#define THOR_LLD_FLASH_MOCK_HPP

/* STL Includes */
#include <memory>

/* LLD Includes */
#include <Thor/cfg>

#if defined( THOR_LLD_FLASH_MOCK )

/* Mock Includes */
#include "gmock/gmock.h"

namespace Thor::LLD::FLASH::Mock
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

  class DriverMock //: virtual public Thor::LLD::FLASH::IDriver
  {
  public:
  };


  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  ModuleMock &getModuleMockObject();
  DriverMock &getDriverMockObject( const size_t channel );

}    // namespace Thor::LLD::FLASH::Mock

#endif /* THOR_LLD_FLASH_MOCK */
#endif /* !THOR_LLD_FLASH_MOCK_HPP */
