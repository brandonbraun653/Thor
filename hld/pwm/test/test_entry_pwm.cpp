/********************************************************************************
 *  File Name:
 *    test_entry_pwm.cpp
 *
 *	 Description:
 *    Entry into the PWM HLD tests
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#if defined( THOR_HLD_TEST )
#include "gtest/gtest.h"

int main( int argc, char **argv )
{
  ::testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}

#endif