/********************************************************************************
 *  File Name:
 *    test_fixture_gpio.cpp
 *
 *  Description:
 *    GPIO Test Fixture Implementation
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/


/* Test Includes */
#include <Thor/hld/gpio/test/test_fixture_gpio.hpp>
#include <Thor/lld/interface/gpio/mock/gpio_mock.hpp>

namespace Thor::HLD::GPIO
{
  void TestFixture::SetUp()
  {
    Thor::LLD::GPIO::gpio_module_mock = new Thor::LLD::GPIO::ModuleMock();
  }

  void TestFixture::TearDown()
  {
    delete Thor::LLD::GPIO::gpio_module_mock;
  }

}    // namespace Thor::HLD::GPIO