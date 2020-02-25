/********************************************************************************
 *  File Name:
 *    test_lld_uart_driver.cpp
 *
 *	 Description:
 *    Tests the Thor UART low level driver
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* GTest Includes */
#include "gtest/gtest.h"

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/uart>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/uart/uart.hpp>


TEST(PleasePass, PrettyPlease)
{
  EXPECT_EQ(0, 0);
  Thor::LLD::UART::initialize();
}

