/********************************************************************************
 *  File Name:
 *    timer_mock_variant.hpp
 *
 *  Description:
 *    Mock interface for TIMER
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#ifndef THOR_LLD_TIMER_MOCK_VARIANT_HPP
#define THOR_LLD_TIMER_MOCK_VARIANT_HPP

/* LLD Includes */
#include <Thor/cfg>

#if defined( THOR_LLD_TIMER_MOCK_VARIANT )

/* Mock Includes */
#include "gmock/gmock.h"

namespace Thor::LLD::TIMER::Mock
{
  // Not like needed to mock yet
}    // namespace Thor::LLD::TIMER::Mock

#endif /* THOR_LLD_TIMER_MOCK_VARIANT */
#endif /* !THOR_LLD_TIMER_MOCK_VARIANT_HPP */
