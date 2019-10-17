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

#include <Thor/drivers/common/cmsis/configuration.hpp>

/* CMSIS compiler specific defines */
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
#define NVIC_SetPriorityGrouping
#define NVIC_GetPriorityGrouping
#define NVIC_EnableIRQ
#define NVIC_GetEnableIRQ
#define NVIC_DisableIRQ
#define NVIC_GetPendingIRQ
#define NVIC_SetPendingIRQ
#define NVIC_ClearPendingIRQ
#define NVIC_GetActive
#define NVIC_SetPriority
#define NVIC_GetPriority
#define NVIC_SystemReset

#endif /* !__CMSIS_WINDOWS_H */