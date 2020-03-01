/********************************************************************************
 *  File Name:
 *    test_hld_system_chimera.cpp
 *
 *	 Description:
 *    Tests the Chimera interface to Thor System
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#if defined( THOR_HLD_TEST )
/* GTest Includes */
#include "gtest/gtest.h"

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/system>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/system>
#include <Thor/hld/system/hld_system_chimera.hpp>


TEST(PleasePass, PrettyPlease)
{
  EXPECT_EQ(0, 0);

  Chimera::System::prjSystemStartup();
}

#endif
