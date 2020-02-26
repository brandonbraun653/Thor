/********************************************************************************
 *  File Name:
 *    test_hld_usart_chimera.cpp
 *
 *	Description:
 *    Tests the Chimera interface to Thor USART
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#if defined( THOR_HLD_TEST )
/* GTest Includes */
#include "gtest/gtest.h"

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/usart>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/usart>
#include <Thor/hld/usart/hld_usart_chimera.hpp>


TEST(PleasePass, PrettyPlease)
{
  EXPECT_EQ(0, 0);

  Chimera::USART::Backend::initialize();
}

#endif
