/********************************************************************************
 *  File Name:
 *    test_hld_dma_chimera.cpp
 *
 *	 Description:
 *    Tests the Chimera interface to Thor DMA
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
#include <Thor/hld/dma/hld_dma_chimera.hpp>


TEST(PleasePass, PrettyPlease)
{
  EXPECT_EQ(0, 0);

  Chimera::DMA::Backend::initialize();
}

