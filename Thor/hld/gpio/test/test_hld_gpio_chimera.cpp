/********************************************************************************
 *  File Name:
 *    test_hld_gpio_chimera.cpp
 *
 *	 Description:
 *    Tests the Chimera interface to Thor GPIO
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#if defined( THOR_HLD_TEST ) || defined( THOR_HLD_TEST_GPIO )

/* GTest Includes */
#include "gtest/gtest.h"

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/gpio>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/gpio>
#include <Thor/hld/gpio/hld_gpio_chimera.hpp>
#include <Thor/hld/gpio/test/test_fixture_gpio.hpp>

namespace Thor::HLD::GPIO
{
  TEST_F( TestFixture, PrettyPlease )
  {
    EXPECT_EQ( 0, 0 );

    Chimera::GPIO::Backend::initialize();
  }
}    // namespace Thor::HLD::GPIO
#endif

