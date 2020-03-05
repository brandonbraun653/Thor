/********************************************************************************
 *  File Name:
 *    test_fixture_gpio.hpp
 *
 *  Description:
 *    GPIO Test Fixture
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_HLD_GPIO_TEST_FIXTURE
#define THOR_HLD_GPIO_TEST_FIXTURE

/* Google Includes */
#include "gtest/gtest.h"

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/gpio>


namespace Thor::HLD::GPIO
{

  class TestFixture : public ::testing::Test
  {
  protected:
    virtual void SetUp() override;
    virtual void TearDown() override;


  };
}


#endif  /* !THOR_HLD_GPIO_TEST_FIXTURE */
