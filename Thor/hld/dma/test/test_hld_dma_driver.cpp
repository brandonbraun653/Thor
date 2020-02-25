/********************************************************************************
 *  File Name:
 *    test_hld_dma_driver.cpp
 *
 *	 Description:
 *    Tests the Thor DMA high level driver
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* GTest Includes */
#include "gtest/gtest.h"

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/dma>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/dma>
#include <Thor/hld/dma/hld_dma_driver.hpp>


TEST(PleasePass, PrettyPlease2)
{
  EXPECT_EQ(0, 0);

  Thor::DMA::initialize();
}

