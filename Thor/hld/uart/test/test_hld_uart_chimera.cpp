/********************************************************************************
 *  File Name:
 *    test_hld_uart_chimera.cpp
 *
 *	Description:
 *    Tests the Chimera interface to Thor UART
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#if defined( THOR_HLD_TEST )
/* GTest Includes */
#include "gtest/gtest.h"

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/uart>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/uart>
#include <Thor/hld/uart/hld_uart_chimera.hpp>


TEST(PleasePass, PrettyPlease)
{
  EXPECT_EQ(0, 0);

  Chimera::UART::Backend::initialize();
}
#endif

