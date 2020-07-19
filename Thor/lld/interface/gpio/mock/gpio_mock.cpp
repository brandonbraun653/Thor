/********************************************************************************
 *  File Name:
 *    gpio_mock.cpp
 *
 *  Description:
 *    Mocks the GPIO driver interface
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Mock Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/gpio/gpio_intf.hpp>
#include <Thor/lld/interface/gpio/gpio_types.hpp>
#include <Thor/lld/interface/gpio/mock/gpio_mock.hpp>
#include <Thor/lld/interface/gpio/mock/gpio_mock_variant.hpp>

#if defined( THOR_LLD_GPIO_MOCK )

/* STL Includes */
#include <array>
#include <iostream>

namespace Thor::LLD::GPIO
{
  static std::array<IDriver_sPtr, NUM_GPIO_PERIPHS> s_gpio_drivers;

  /*-------------------------------------------------
  LLD->HLD Interface Implementation
  -------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    initializeRegisters();
    initializeMapping();

    return Chimera::CommonStatusCodes::OK;
  }

  IDriver_sPtr getDriver( const size_t channel )
  {
    if ( !( channel < NUM_GPIO_PERIPHS ) )
    {
      return nullptr;
    }
    else if ( !s_gpio_drivers[ channel ] )
    {
      s_gpio_drivers[ channel ] = std::make_shared<DriverMock>();
      s_gpio_drivers[ channel ]->attach( PeripheralRegisterMaps[ channel ] );
    }

    return s_gpio_drivers[ channel ];
  }

  size_t availableChannels()
  {
    return NUM_GPIO_PERIPHS;
  }
}    // namespace Thor::LLD::GPIO

#endif /* THOR_LLD_GPIO_MOCK */
