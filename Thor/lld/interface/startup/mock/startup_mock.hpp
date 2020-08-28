/********************************************************************************
 *  File Name:
 *    startup_mock.hpp
 *
 *  Description:
 *    Mock interface for STARTUP
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#ifndef THOR_LLD_STARTUP_MOCK_HPP
#define THOR_LLD_STARTUP_MOCK_HPP

/* LLD Includes */
#include <Thor/cfg>

#if defined( THOR_LLD_STARTUP_MOCK )

/* Mock Includes */
#include "gmock/gmock.h"

namespace Thor::LLD::STARTUP::Mock
{
  // Not like needed to mock yet
}    // namespace Thor::LLD::STARTUP::Mock

#endif /* THOR_LLD_STARTUP_MOCK */
#endif /* !THOR_LLD_STARTUP_MOCK_HPP */
