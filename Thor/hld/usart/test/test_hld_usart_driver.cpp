/********************************************************************************
 *  File Name:
 *    test_hld_usart_driver.cpp
 *
 *	Description:
 *    Tests the Thor USART high level driver
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* GTest Includes */
#include "gtest/gtest.h"

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/usart>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/usart>


TEST(PleasePass, PrettyPlease2)
{
  EXPECT_EQ(0, 0);
  Thor::USART::initialize();
}

