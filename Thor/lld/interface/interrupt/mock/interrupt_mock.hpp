/********************************************************************************
 *  File Name:
 *    interrupt_mock.hpp
 *
 *  Description:
 *    Mock interface for Interrupt
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_INTERRUPT_MOCK_HPP
#define THOR_LLD_INTERRUPT_MOCK_HPP

/* LLD Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/interrupt/interrupt_intf.hpp>
#include <Thor/lld/interface/interrupt/interrupt_types.hpp>

#if defined( THOR_LLD_IT_MOCK )

/* Mock Includes */
#include "gmock/gmock.h"

namespace Thor::LLD::IT
{
  // Nothing for now
}  // namespace Thor::LLD::IT

#endif  /* THOR_LLD_IT_MOCK */
#endif  /* !THOR_LLD_INTERRUPT_MOCK_HPP */
