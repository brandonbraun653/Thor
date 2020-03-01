/********************************************************************************
 *  File Name:
 *    test_hld_gpio_driver.cpp
 *
 *	 Description:
 *    Tests the Thor GPIO high level driver
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


TEST(PleasePass, PrettyPlease2)
{
  EXPECT_EQ(0, 0);
}

#endif
