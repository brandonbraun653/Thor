/********************************************************************************
 *  File Name:
 *    test_lld_flash_driver.cpp
 *
 *	 Description:
 *    Tests the Thor FLASH low level driver
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#if defined( THOR_LLD_TEST_FLASH )
/* GTest Includes */
#include "gtest/gtest.h"

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/flash>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/flash/flash.hpp>

TEST(Compiler, CanCompile)
{
  EXPECT_EQ(0, 0);
  Thor::LLD::FLASH::initialize();
}

#endif  /* THOR_LLD_TEST_FLASH */
