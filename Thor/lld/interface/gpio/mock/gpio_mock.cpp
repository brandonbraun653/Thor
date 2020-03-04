/********************************************************************************
 *  File Name:
 *    gpio_mock.cpp
 *
 *  Description:
 *    Mocks the GPIO driver interface
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* STL Includes */
#include <array>

/* Mock Includes */
#include <Thor/lld/interface/gpio/gpio_intf.hpp>
#include <Thor/lld/interface/gpio/gpio_types.hpp>
#include <Thor/lld/interface/gpio/mock/gpio_mock.hpp>

namespace Thor::LLD::GPIO
{
  static constexpr size_t MAX_NUM_CHANNELS = 8;

  std::array<IGPIO_sPtr, MAX_NUM_CHANNELS> channels;

  IGPIO_sPtr getDriver( const size_t channel )
  {
    if( !channels[ channel ])
    {
      channels[ channel ] = std::make_shared<DriverMock>();
    }

    return channels[ channel ];
  }

  size_t availableChannels()
  {
    return MAX_NUM_CHANNELS;
  }
}