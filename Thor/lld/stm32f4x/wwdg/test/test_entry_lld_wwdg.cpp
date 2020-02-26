/********************************************************************************
 *  File Name:
 *    test_entry.cpp
 *
 *	 Description:
 *    Entry into the test suite
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#if defined( LLD_TEST )
#include "gtest/gtest.h"

int main( int argc, char **argv )
{
  ::testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
#endif