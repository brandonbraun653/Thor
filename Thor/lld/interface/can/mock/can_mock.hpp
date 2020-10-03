/********************************************************************************
 *  File Name:
 *    can_mock.hpp
 *
 *  Description:
 *    Mock interface for CAN
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#ifndef THOR_LLD_CAN_MOCK_HPP
#define THOR_LLD_CAN_MOCK_HPP

/* STL Includes */
#include <memory>

/* LLD Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/can/can_intf.hpp>
#include <Thor/lld/interface/can/can_types.hpp>

#if defined( THOR_LLD_CAN_MOCK )

/* Mock Includes */
#include "gmock/gmock.h"

namespace Thor::LLD::CAN::Mock
{
  /*-------------------------------------------------------------------------------
  Mock Interfaces
  -------------------------------------------------------------------------------*/
  class IModule
  {
  public:
    virtual ~IModule()                                                   = default;
    virtual void initialize()                                            = 0;
    virtual Driver_rPtr getDriver( const Chimera::CAN::Channel channel ) = 0;
    virtual size_t availableChannels()                                   = 0;
  };


  /*-------------------------------------------------------------------------------
  Mock Classes
  -------------------------------------------------------------------------------*/
  class ModuleMock : public IModule
  {
  public:
    MOCK_METHOD( void, initialize, (), ( override ) );
    MOCK_METHOD( Driver_rPtr, getDriver, ( const Chimera::CAN::Channel ), ( override ) );
    MOCK_METHOD( size_t, availableChannels, (), ( override ) );
  };

  class DriverMock : virtual public Thor::LLD::CAN::IDriver
  {
  public:
    MOCK_METHOD( void, attach, ( RegisterMap *const x ), ( override ) );
  };


  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  ModuleMock &getModuleMockObject();
  DriverMock &getDriverMockObject( const size_t channel );

}    // namespace Thor::LLD::CAN::Mock

#endif /* THOR_LLD_CAN_MOCK */
#endif /* !THOR_LLD_CAN_MOCK_HPP */
