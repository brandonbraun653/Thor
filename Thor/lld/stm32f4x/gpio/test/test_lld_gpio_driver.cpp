/********************************************************************************
 *  File Name:
 *    test_lld_gpio_chimera.cpp
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
#include <Thor/lld/interface/gpio/gpio.hpp>


TEST(PleasePass, PrettyPlease)
{
  EXPECT_EQ(0, 0);
  Thor::LLD::GPIO::initialize();
}

