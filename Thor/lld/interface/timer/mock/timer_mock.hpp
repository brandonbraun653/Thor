/********************************************************************************
 *  File Name:
 *    timer_mock.hpp
 *
 *  Description:
 *    Mock interface for TIMER
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#ifndef THOR_LLD_TIMER_MOCK_HPP
#define THOR_LLD_TIMER_MOCK_HPP

/* LLD Includes */
#include <Thor/cfg>

#if defined( THOR_LLD_TIMER_MOCK )

/* Mock Includes */
#include "gmock/gmock.h"

namespace Thor::LLD::TIMER::Mock
{
  // Not like needed to mock yet
}    // namespace Thor::LLD::TIMER::Mock

#endif /* THOR_LLD_TIMER_MOCK */
#endif /* !THOR_LLD_TIMER_MOCK_HPP */
