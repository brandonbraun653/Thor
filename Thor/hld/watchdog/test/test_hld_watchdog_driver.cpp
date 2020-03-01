/********************************************************************************
 *  File Name:
 *    test_hld_watchdog_driver.cpp
 *
 *	Description:
 *    Tests the Thor Watchdog high level driver
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#if defined( THOR_HLD_TEST )
/* GTest Includes */
#include "gtest/gtest.h"

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/watchdog>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/watchdog>

TEST(PleasePass, PrettyPlease2)
{
  EXPECT_EQ(0, 0);
  Thor::Watchdog::initializeWWDG();
  Thor::Watchdog::initializeIWDG();
}

#endif