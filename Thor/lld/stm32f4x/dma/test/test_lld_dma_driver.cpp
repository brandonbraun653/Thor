/********************************************************************************
 *  File Name:
 *    test_lld_dma_driver.cpp
 *
 *	 Description:
 *    Tests the Thor DMA low level driver
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#if defined( LLD_TEST )

/* GTest Includes */
#include "gtest/gtest.h"

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/dma>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/dma/dma.hpp>


TEST(PleasePass, PrettyPlease)
{
  EXPECT_EQ(0, 0);
  Thor::LLD::DMA::initialize();
}

#endif