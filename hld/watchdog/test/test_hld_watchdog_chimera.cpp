/********************************************************************************
 *  File Name:
 *    test_hld_watchdog_chimera.cpp
 *
 *	Description:
 *    Tests the Chimera interface to Thor Watchdog
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
#include <Thor/hld/watchdog/hld_watchdog_chimera.hpp>


TEST(PleasePass, PrettyPlease)
{
  EXPECT_EQ(0, 0);

  Chimera::Watchdog::Backend::initialize();
}

#endif
