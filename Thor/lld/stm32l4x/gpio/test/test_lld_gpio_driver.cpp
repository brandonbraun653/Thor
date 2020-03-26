/********************************************************************************
 *  File Name:
 *    test_lld_gpio_driver.cpp
 *
 *	 Description:
 *    Tests the Thor GPIO low level driver
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#if defined( THOR_LLD_TEST_GPIO )
/* GTest Includes */
#include "gtest/gtest.h"

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/gpio>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/gpio/gpio.hpp>

TEST(Compiler, CanCompile)
{
  EXPECT_EQ(0, 0);
  Thor::LLD::GPIO::initialize();
}

#endif  /* THOR_LLD_TEST_GPIO */
