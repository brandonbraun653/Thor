/********************************************************************************
 *  File Name:
 *    nvic_mock.hpp
 *
 *  Description:
 *    Mock interface for NVIC
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#ifndef THOR_LLD_NVIC_MOCK_HPP
#define THOR_LLD_NVIC_MOCK_HPP

/* LLD Includes */
#include <Thor/cfg>

#if defined( THOR_LLD_NVIC_MOCK )

/* Mock Includes */
#include "gmock/gmock.h"

namespace Thor::LLD::NVIC::Mock
{
  // Not like needed to mock yet
}    // namespace Thor::LLD::NVIC::Mock

#endif /* THOR_LLD_NVIC_MOCK */
#endif /* !THOR_LLD_NVIC_MOCK_HPP */
