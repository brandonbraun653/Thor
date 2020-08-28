/********************************************************************************
 *  File Name:
 *    gpio_mock.hpp
 *
 *  Description:
 *    Mock interface for GPIO
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#ifndef THOR_LLD_GPIO_MOCK_HPP
#define THOR_LLD_GPIO_MOCK_HPP

/* STL Includes */
#include <memory>

/* LLD Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/gpio/gpio_intf.hpp>
#include <Thor/lld/interface/gpio/gpio_types.hpp>

#if defined( THOR_LLD_GPIO_MOCK )

/* Mock Includes */
#include "gmock/gmock.h"

namespace Thor::LLD::GPIO::Mock
{
  /*-------------------------------------------------------------------------------
  Mock Interfaces
  -------------------------------------------------------------------------------*/
  class IModule
  {
  public:
    virtual ~IModule()                                     = default;
    virtual void initialize()                              = 0;
    virtual IDriver_rPtr getDriver( const size_t channel ) = 0;
    virtual size_t availableChannels()                     = 0;
  };


  /*-------------------------------------------------------------------------------
  Mock Classes
  -------------------------------------------------------------------------------*/
  class ModuleMock : public IModule
  {
  public:
    MOCK_METHOD( void, initialize, (), ( override ) );
    MOCK_METHOD( IDriver_rPtr, getDriver, ( const size_t ), ( override ) );
    MOCK_METHOD( size_t, availableChannels, (), ( override ) );
  };

  class DriverMock : virtual public Thor::LLD::GPIO::IDriver
  {
  public:
    MOCK_METHOD( void, attach, ( RegisterMap *const x ), ( override ) );
    MOCK_METHOD( void, clockDisable, (), ( override ) );
    MOCK_METHOD( void, clockEnable, (), ( override ) );
    MOCK_METHOD( Chimera::Status_t, driveSet, ( const uint8_t, const Chimera::GPIO::Drive ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, speedSet, ( const uint8_t, const Thor::LLD::GPIO::Speed ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, pullSet, ( const uint8_t, const Chimera::GPIO::Pull ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, write, ( const uint8_t, const Chimera::GPIO::State ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, alternateFunctionSet, ( const uint8_t, const Chimera::GPIO::Alternate ), ( override ) );
    MOCK_METHOD( Chimera::GPIO::State, read, ( const uint8_t ), ( override ) );
    MOCK_METHOD( Chimera::GPIO::Drive, driveGet, ( const uint8_t ), ( override ) );
    MOCK_METHOD( Thor::LLD::GPIO::Speed, speedGet, ( const uint8_t ), ( override ) );
    MOCK_METHOD( Chimera::GPIO::Pull, pullGet, ( const uint8_t ), ( override ) );
    MOCK_METHOD( Chimera::GPIO::Alternate, alternateFunctionGet, ( const uint8_t ), ( override ) );
  };


  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  ModuleMock &getModuleMockObject();
  DriverMock &getDriverMockObject( const size_t channel );

}    // namespace Thor::LLD::GPIO::Mock

#endif /* THOR_LLD_GPIO_MOCK */
#endif /* !THOR_LLD_GPIO_MOCK_HPP */
