/********************************************************************************
 *  File Name:
 *    cmsis_windows.h
 *
 *  Description:
 *
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef __CMSIS_WINDOWS_H
#define __CMSIS_WINDOWS_H

#if defined( _SIM ) && ( defined( WIN32 ) || defined( WIN64 ) )

/* C/C++ Includes */
#include <cstdint>

/* Thor Includes */
#include <Thor/drivers/common/cmsis/configuration.hpp>


/*------------------------------------------------
CMSIS compiler specific defines
------------------------------------------------*/
#ifndef __ASM
#define __ASM __asm
#endif
#ifndef __INLINE
#define __INLINE inline
#endif
#ifndef __STATIC_INLINE
#define __STATIC_INLINE static inline
#endif
#ifndef __STATIC_FORCEINLINE
#define __STATIC_FORCEINLINE
#endif
#ifndef __NO_RETURN
#define __NO_RETURN
#endif
#ifndef __USED
#define __USED
#endif
#ifndef __WEAK
#define __WEAK
#endif
#ifndef __PACKED
#define __PACKED
#endif
#ifndef __PACKED_STRUCT
#define __PACKED_STRUCT
#endif
#ifndef __PACKED_UNION
#define __PACKED_UNION
#endif

/*------------------------------------------------
ARM NVIC Macros
------------------------------------------------*/
#define NVIC_SetPriorityGrouping __NVIC_SetPriorityGrouping
#define NVIC_GetPriorityGrouping __NVIC_GetPriorityGrouping
#define NVIC_EnableIRQ __NVIC_EnableIRQ
#define NVIC_GetEnableIRQ __NVIC_GetEnableIRQ
#define NVIC_DisableIRQ __NVIC_DisableIRQ
#define NVIC_GetPendingIRQ __NVIC_GetPendingIRQ
#define NVIC_SetPendingIRQ __NVIC_SetPendingIRQ
#define NVIC_ClearPendingIRQ __NVIC_ClearPendingIRQ
#define NVIC_GetActive __NVIC_GetActive
#define NVIC_SetPriority __NVIC_SetPriority
#define NVIC_GetPriority __NVIC_GetPriority
#define NVIC_SystemReset __NVIC_SystemReset


__STATIC_INLINE void __NVIC_SetPriorityGrouping( uint32_t PriorityGroup )
{
}

__STATIC_INLINE uint32_t __NVIC_GetPriorityGrouping( void )
{
  return std::numeric_limits<uint32_t>::max();
}

__STATIC_INLINE void __NVIC_EnableIRQ( IRQn_Type IRQn )
{
}

__STATIC_INLINE uint32_t __NVIC_GetEnableIRQ( IRQn_Type IRQn )
{
  return std::numeric_limits<uint32_t>::max();
}

__STATIC_INLINE void __NVIC_DisableIRQ( IRQn_Type IRQn )
{
}

__STATIC_INLINE uint32_t __NVIC_GetPendingIRQ( IRQn_Type IRQn )
{
  return std::numeric_limits<uint32_t>::max();
}

__STATIC_INLINE void __NVIC_SetPendingIRQ( IRQn_Type IRQn )
{
}

__STATIC_INLINE void __NVIC_ClearPendingIRQ( IRQn_Type IRQn )
{
}

__STATIC_INLINE uint32_t __NVIC_GetActive( IRQn_Type IRQn )
{
  return std::numeric_limits<uint32_t>::max();
}

__STATIC_INLINE void __NVIC_SetPriority( IRQn_Type IRQn, uint32_t priority )
{
}

__STATIC_INLINE uint32_t __NVIC_GetPriority( IRQn_Type IRQn )
{
  return std::numeric_limits<uint32_t>::max();
}

__STATIC_INLINE uint32_t NVIC_EncodePriority( uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority )
{
  return std::numeric_limits<uint32_t>::max();
}

__STATIC_INLINE void NVIC_DecodePriority( uint32_t Priority, uint32_t PriorityGroup, uint32_t *const pPreemptPriority,
                                          uint32_t *const pSubPriority )
{
}

__STATIC_INLINE void __NVIC_SetVector( IRQn_Type IRQn, uint32_t vector )
{
}

__STATIC_INLINE uint32_t __NVIC_GetVector( IRQn_Type IRQn )
{
  return std::numeric_limits<uint32_t>::max();
}

__STATIC_INLINE void __NVIC_SystemReset( void )
{
}

#endif /* _SIM && ( WIN32 || WIN64 ) */

#endif /* !__CMSIS_WINDOWS_H */