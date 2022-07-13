/******************************************************************************
 *  File Name:
 *    debug.hpp
 *
 *  Description:
 *    CortexM4 Debug Interface
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef CORTEX_M4_DEBUG_HPP
#define CORTEX_M4_DEBUG_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Thor/cfg>
#include <Thor/lld/common/cortex-m4/register.hpp>

#if defined( CORTEX_M4 )

namespace CortexM4::Debug
{
  static inline void enableCounter()
  {
    *SCB_REG_DEMCR = *SCB_REG_DEMCR | 0x01000000;    // enable trace
    *DWT_CYCCNT    = 0;                              // clear DWT cycle counter
    *DWT_CONTROL   = *DWT_CONTROL | 1;               // enable DWT cycle counter
  }

  static inline void disableCounter()
  {
    *DWT_CONTROL = *DWT_CONTROL & ~1;
  }

  static inline uint32_t sampleCounter()
  {
    return *DWT_CYCCNT;
  }
}    // namespace CortexM4::Debug

#endif /* CORTEX_M4 */
#endif /* !CORTEX_M4_DEBUG_HPP */
