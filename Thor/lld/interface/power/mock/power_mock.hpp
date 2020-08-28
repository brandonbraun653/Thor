/********************************************************************************
 *  File Name:
 *    power_mock.hpp
 *
 *  Description:
 *    Mock interface for PWR
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#ifndef THOR_LLD_PWR_MOCK_HPP
#define THOR_LLD_PWR_MOCK_HPP

/* LLD Includes */
#include <Thor/cfg>

#if defined( THOR_LLD_PWR_MOCK )

/* Mock Includes */
#include "gmock/gmock.h"

namespace Thor::LLD::PWR::Mock
{
  // Not like needed to mock yet
}    // namespace Thor::LLD::PWR::Mock

#endif /* THOR_LLD_PWR_MOCK */
#endif /* !THOR_LLD_PWR_MOCK_HPP */
