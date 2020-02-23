/********************************************************************************
 *  File Name:
 *    test_lld_nvic_driver.cpp
 *
 *	Description:
 *    Tests the Thor NVIC low level driver
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* GTest Includes */
#include "gtest/gtest.h"

/* Chimera Includes */
#include <Chimera/common>

/* Thor Includes */
#include <Thor/lld/interface/nvic/nvic.hpp>


TEST(PleasePass, PrettyPlease)
{
  EXPECT_EQ(0, 0);
  Thor::LLD::NVIC::initialize();
}
