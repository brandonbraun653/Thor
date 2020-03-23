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
#include <Thor/lld/interface/gpio/gpio_intf.hpp>
#include <Thor/lld/interface/gpio/gpio_types.hpp>
#include <Thor/lld/interface/gpio/mock/gpio_mock.hpp>

#if defined( THOR_HLD_GPIO_MOCK )

/* STL Includes */
#include <array>
#include <iostream>

namespace Thor::LLD::GPIO
{
  static ModuleMock *gpio_module_mock;

  void assignModuleMock( ModuleMock *mock )
  {
    gpio_module_mock = mock;
  }

  Chimera::Status_t initialize()
  {
    return gpio_module_mock->initialize();
  }

  IGPIO_sPtr getDriver( const size_t channel )
  {
    gpio_module_mock->getDriver( channel );
    return std::make_shared<DriverMock>();
  }

  size_t availableChannels()
  {
    return gpio_module_mock->availableChannels();
  }
}    // namespace Thor::LLD::GPIO

#endif /* THOR_HLD_GPIO_MOCK */
