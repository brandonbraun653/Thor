/********************************************************************************
 *  File Name:
 *    gpio_mock.hpp
 *
 *  Description:
 *
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#ifndef THOR_HLD_GPIO_MOCK_HPP
#define THOR_HLD_GPIO_MOCK_HPP


/* STL Includes */
#include <memory>

/* LLD Includes */
#include <Thor/lld/interface/gpio/gpio_intf.hpp>
#include <Thor/lld/interface/gpio/gpio_types.hpp>

#if defined( THOR_HLD_GPIO_MOCK )

/* Mock Includes */
#include "gmock/gmock.h"

namespace Thor::LLD::GPIO
{
  class DriverMock : virtual public IDriver
  {
  public:
    MOCK_METHOD( void, attach, ( RegisterMap *const x ), ( override ) );
    MOCK_METHOD( void, clockDisable, (), ( override ) );
    MOCK_METHOD( void, clockEnable, (), ( override ) );
    MOCK_METHOD( Chimera::Status_t, driveSet, ( const uint8_t, const Chimera::GPIO::Drive, const size_t ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, speedSet, ( const uint8_t, const Thor::LLD::GPIO::Speed, const size_t ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, pullSet, ( const uint8_t, const Chimera::GPIO::Pull, const size_t ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, write, ( const uint8_t, const Chimera::GPIO::State, const size_t ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, alternateFunctionSet, ( const uint8_t, const size_t, const size_t ), ( override ) );
    MOCK_METHOD( size_t, read, ( const size_t ), ( override ) );
    MOCK_METHOD( size_t, driveGet, ( const uint8_t, const size_t ), ( override ) );
    MOCK_METHOD( size_t, speedGet, ( const uint8_t, const size_t ), ( override ) );
    MOCK_METHOD( size_t, pullGet, ( const uint8_t, const size_t ), ( override ) );
    MOCK_METHOD( size_t, alternateFunctionGet, ( const uint8_t, const size_t ), ( override ) );
  };


  /**
   *  LLD module level interface that describes how the
   */
  class IModule
  {
  public:
    virtual ~IModule() = default;

    virtual Chimera::Status_t initialize() = 0;

    virtual IGPIO_sPtr getDriver( const size_t channel ) = 0;

    virtual size_t availableChannels() = 0;
  };

  /**
   *  Concrete class that the
   */
  class ModuleMock : public IModule
  {
  public:
    MOCK_METHOD( Chimera::Status_t, initialize, (), ( override ) );
    MOCK_METHOD( IGPIO_sPtr, getDriver, ( const size_t channel ), ( override ) );
    MOCK_METHOD( size_t, availableChannels, (), ( override ) );
  };

  void assignModuleMock( ModuleMock *mock );

}    // namespace Thor::LLD::GPIO

#endif /* THOR_HLD_GPIO_MOCK */
#endif /* !THOR_HLD_GPIO_MOCK_HPP */
