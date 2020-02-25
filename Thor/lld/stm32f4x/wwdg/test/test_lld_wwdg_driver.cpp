/********************************************************************************
 *  File Name:
 *    test_lld_wwdg_driver.cpp
 *
 *	 Description:
 *    Tests the Thor Watchdog low level driver
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* GTest Includes */
#include "gtest/gtest.h"

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/watchdog>

/* Thor Includes */
#include <Thor/lld/interface/watchdog/watchdog.hpp>


TEST(PleasePass, PrettyPlease)
{
  EXPECT_EQ(0, 0);
  Thor::LLD::WWDG::initialize();
}
