/********************************************************************************
 *  File Name:
 *    test_hld_spi_chimera.cpp
 *
 *	 Description:
 *    Tests the Chimera interface to Thor SPI
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* GTest Includes */
#include "gtest/gtest.h"

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/spi>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/spi>
#include <Thor/hld/spi/hld_spi_chimera.hpp>


TEST(PleasePass, PrettyPlease)
{
  EXPECT_EQ(0, 0);

  Chimera::SPI::Backend::initialize();
}

