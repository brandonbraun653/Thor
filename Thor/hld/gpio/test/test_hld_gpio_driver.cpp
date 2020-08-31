/********************************************************************************
 *  File Name:
 *    test_hld_gpio_driver.cpp
 *
 *	 Description:
 *    Tests the Thor GPIO high level driver
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#if defined( THOR_HLD_TEST ) || defined( THOR_HLD_TEST_GPIO )

/* GTest Includes */
#include "gtest/gtest.h"

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/gpio>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/gpio>
#include <Thor/hld/gpio/hld_gpio_chimera.hpp>
#include <Thor/hld/gpio/test/test_fixture_gpio.hpp>
#include <Thor/lld/interface/gpio/mock/gpio_mock.hpp>

namespace Thor::HLD::GPIO
{
  void reset_test()
  {
    ::Thor::GPIO::reset();

    ASSERT_TRUE( ::testing::Mock::VerifyAndClearExpectations( &Thor::LLD::GPIO::Mock::getModuleMockObject() ) );
  }


  TEST_F(TestFixture, DriverConstructor)
  {
    auto DoIFailCreation = Thor::GPIO::Driver();
    SUCCEED();
  }

  TEST_F(TestFixture, InitFromConfig)
  {
    auto driver = Thor::GPIO::Driver();
    Chimera::GPIO::PinInit pinInit;

    EXPECT_EQ( 0, 0 );
  }

  TEST_F(TestFixture, FreeFunc_Initialize)
  {
    using ::testing::Return;

    Thor::LLD::GPIO::Mock::ModuleMock& lld = Thor::LLD::GPIO::Mock::getModuleMockObject();

    
    reset_test();
    EXPECT_CALL( lld, initialize() ).Times(1);

    Thor::GPIO::initialize();
  }

  TEST_F(TestFixture, SanityCheck_Access_LLD_MOCK)
  {
    reset_test();
    Thor::GPIO::initialize();
    Thor::LLD::GPIO::Mock::DriverMock& driver = Thor::LLD::GPIO::Mock::getDriverMockObject( 0 );

    EXPECT_CALL( driver, clockDisable() ).Times( 1 );
    driver.clockDisable();
  }
}

#endif
