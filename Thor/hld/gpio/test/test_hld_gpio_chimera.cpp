/********************************************************************************
 *  File Name:
 *    test_hld_gpio_chimera.cpp
 *
 *	 Description:
 *    Tests the Chimera interface to Thor GPIO
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* GTest Includes */
#include "gtest/gtest.h"

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/gpio>

/* Thor Includes */
#include <Thor/gpio>
#include <Thor/hld/gpio/hld_gpio_chimera.hpp>


TEST(PleasePass, PrettyPlease)
{
  EXPECT_EQ(0, 0);

  Chimera::GPIO::Backend::initialize();
}

