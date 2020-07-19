/********************************************************************************
 *  File Name:
 *    test_fixture_gpio.cpp
 *
 *  Description:
 *    GPIO Test Fixture Implementation
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#if defined( THOR_HLD_TEST ) || defined( THOR_HLD_TEST_GPIO )

/* STL Includes */
#include <iostream>

/* Test Includes */
#include <Thor/hld/gpio/test/test_fixture_gpio.hpp>
#include <Thor/lld/interface/gpio/mock/gpio_mock.hpp>

namespace Thor::HLD::GPIO
{
  void TestFixture::SetUp()
  {
    
  }

  void TestFixture::TearDown()
  {
    
  }

}    // namespace Thor::HLD::GPIO

#endif 