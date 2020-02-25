/********************************************************************************
 *  File Name:
 *    test_lld_flash_driver.cpp
 *
 *	Description:
 *    Tests the Thor Flash low level driver
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* GTest Includes */
#include "gtest/gtest.h"

/* Chimera Includes */
#include <Chimera/common>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/flash/flash.hpp>


TEST(PleasePass, PrettyPlease)
{
  EXPECT_EQ(0, 0);
  Thor::LLD::FLASH::initialize();
}

