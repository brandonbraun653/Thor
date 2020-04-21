/********************************************************************************
 *  File Name:
 *    test_hld_pwm_chimera.cpp
 *
 *	 Description:
 *    Tests the Chimera interface to Thor PWM
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#if defined( THOR_HLD_TEST )
/* GTest Includes */
#include "gtest/gtest.h"

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/pwm>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/pwm>
#include <Thor/hld/pwm/hld_pwm_chimera.hpp>


TEST(PleasePass, PrettyPlease)
{
  EXPECT_EQ(0, 0);

  Chimera::PWM::Backend::initialize();
}

#endif