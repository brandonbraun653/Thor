/********************************************************************************
 *  File Name:
 *    test_lld_power_driver.cpp
 *
 *	 Description:
 *    Tests the Thor POWER low level driver
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#if defined( THOR_LLD_TEST_POWER )
/* GTest Includes */
#include "gtest/gtest.h"

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/power>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/power/power.hpp>

TEST(Compiler, CanCompile)
{
  EXPECT_EQ(0, 0);
  Thor::LLD::POWER::initialize();
}

#endif  /* THOR_LLD_TEST_POWER */
