/********************************************************************************
 *  File Name:
 *    test_lld_timer_driver.cpp
 *
 *	 Description:
 *    Tests the Thor TIMER low level driver
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#if defined( THOR_LLD_TEST_TIMER )
/* GTest Includes */
#include "gtest/gtest.h"

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/timer>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/timer/timer.hpp>

TEST(Compiler, CanCompile)
{
  EXPECT_EQ(0, 0);
  Thor::LLD::TIMER::initialize();
}

#endif  /* THOR_LLD_TEST_TIMER */
