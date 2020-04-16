/********************************************************************************
 *  File Name:
 *    test_lld_spi_driver.cpp
 *
 *	 Description:
 *    Tests the Thor SPI low level driver
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#if defined( THOR_LLD_TEST_SPI )
/* GTest Includes */
#include "gtest/gtest.h"

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/spi>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/spi/spi.hpp>

TEST(Compiler, CanCompile)
{
  EXPECT_EQ(0, 0);
  Thor::LLD::SPI::initialize();
}

#endif  /* THOR_LLD_TEST_SPI */
