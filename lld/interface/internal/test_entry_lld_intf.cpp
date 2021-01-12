/********************************************************************************
 *  File Name:
 *    test_entry_lld_intf.cpp
 *
 *	 Description:
 *    Entry into the test suite for the LLD GPIO interface
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#include "gtest/gtest.h"

int main( int argc, char **argv )
{
  ::testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
