/********************************************************************************
 *  File Name:
 *    test_lld_iwdg_driver.cpp
 *
 *	 Description:
 *    Tests the Thor Watchdog low level driver
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#if defined( THOR_LLD_TEST )
/* GTest Includes */
#include "gtest/gtest.h"

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/watchdog>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/watchdog/watchdog.hpp>


TEST(PleasePass, PrettyPlease)
{
  EXPECT_EQ(0, 0);
  Thor::LLD::IWDG::initialize();
}
#endif 
