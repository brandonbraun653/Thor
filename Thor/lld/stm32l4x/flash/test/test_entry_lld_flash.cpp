/********************************************************************************
 *  File Name:
 *    test_entry_lld_flash.cpp
 *
 *	 Description:
 *    Entry into the test suite for LLD FLASH
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#if defined( THOR_LLD_TEST_FLASH )
#include "gtest/gtest.h"

int main( int argc, char **argv )
{
  ::testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}

#endif /* THOR_LLD_TEST_FLASH */
