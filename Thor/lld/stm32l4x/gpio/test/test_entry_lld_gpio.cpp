/********************************************************************************
 *  File Name:
 *    test_entry_lld_gpio.cpp
 *
 *	 Description:
 *    Entry into the test suite for LLD GPIO
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#if defined( THOR_LLD_TEST_GPIO )
#include "gtest/gtest.h"

int main( int argc, char **argv )
{
  ::testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}

#endif /* THOR_LLD_TEST_GPIO */
