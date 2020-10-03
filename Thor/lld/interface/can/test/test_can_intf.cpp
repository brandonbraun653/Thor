/********************************************************************************
 *  File Name:
 *    test_can_intf.cpp
 *
 *  Description:
 *    Tests the interface layer for the low level CAN drivers
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Chimera Includes */
#include <Chimera/utility>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/can/can_intf.hpp>
#include <Thor/lld/interface/can/can_prv_data.hpp>

#if defined( TARGET_LLD_TEST ) && defined( THOR_LLD_CAN )

/* GTest Includes */
#include <gtest/gtest.h>

namespace Thor::LLD::CAN
{
  static Driver fakeDriverList[ NUM_CAN_PERIPHS ];

  /*-------------------------------------------------------------------------------
  FUT: isSupported()
  Range: Normal & Robust
  -------------------------------------------------------------------------------*/
  TEST( Robust_CAN_DriverSupport, InvalidInputs )
  {
  }

  TEST( Normal_CAN_DriverSupport, ValidInputs )
  {
    /*-------------------------------------------------
    This section uses hard coded values defined in the
    can_sim_variant.cpp file.
    -------------------------------------------------*/
  }


  /*-------------------------------------------------------------------------------
  FUT: getResourceIndex()
  Range: Normal & Robust
  -------------------------------------------------------------------------------*/
  TEST( Robust_CAN_ResourceIndex, InvalidInputs )
  {
  }

  TEST( Normal_CAN_ResourceIndex, ValidInputs )
  {
  }

  /*-------------------------------------------------------------------------------
  FUT: attachDriverInstances()
  Range: Normal & Robust
  -------------------------------------------------------------------------------*/
  TEST( Robust_CAN_AttachDriver, InvalidInputs )
  {
    EXPECT_EQ( false, attachDriverInstances( nullptr, 1 ) );
    EXPECT_EQ( false, attachDriverInstances( fakeDriverList, 0 ) );
    EXPECT_EQ( false, attachDriverInstances( fakeDriverList, DRIVER_MAX_PORTS + 1 ) );
    EXPECT_EQ( false, attachDriverInstances( fakeDriverList, ARRAY_COUNT( fakeDriverList ) - 5 ) );
    EXPECT_EQ( false, attachDriverInstances( fakeDriverList, NUM_CAN_PINS + 1 ) );
  }

  TEST( Normal_CAN_AttachDriver, ValidInputs )
  {
    EXPECT_EQ( true, attachDriverInstances( fakeDriverList, ARRAY_COUNT( fakeDriverList ) ) );
  }

}    // namespace Thor::Test::LLD

#endif /* THOR_LLD_TEST_CAN_INTF */
