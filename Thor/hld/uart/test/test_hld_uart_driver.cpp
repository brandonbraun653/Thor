/********************************************************************************
 *  File Name:
 *    test_hld_uart_driver.cpp
 *
 *	Description:
 *    Tests the Thor UART high level driver
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* GTest Includes */
#include "gtest/gtest.h"

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/uart>

/* Thor Includes */
#include <Thor/uart>

TEST(PleasePass, PrettyPlease2)
{
  EXPECT_EQ(0, 0);
  Thor::UART::initialize();
}

