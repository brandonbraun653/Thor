/********************************************************************************
 *  File Name:
 *    test_hld_system_driver.cpp
 *
 *	 Description:
 *    Tests the Thor Timer high level driver
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#if defined( THOR_HLD_TEST )
/* GTest Includes */
#include "gtest/gtest.h"

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/system>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/system>
#include <Thor/hld/timer/hld_timer_chimera.hpp>


TEST(PleasePass, PrettyPlease2)
{
  EXPECT_EQ(0, 0);
}

#endif