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

#include "gmock/gmock.h"

#include <Thor/lld/interface/gpio/gpio_intf.hpp>
#include <Thor/lld/interface/gpio/gpio_types.hpp>

namespace Thor::LLD::GPIO
{

  class DriverMock : virtual public IDriver
  {
  public:
   MOCK_METHOD( void, attach, ( RegisterMap *const x ) );
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
}    // namespace Thor::LLD::GPIO

#endif /* !THOR_HLD_GPIO_MOCK_HPP */

