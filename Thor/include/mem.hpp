#pragma once
#ifndef MEM_H_
#define MEM_H_

#include <Thor/include/config.h>

/*----------------------------------------------------------------
 * Globally overload the new/delete operators to use the FreeRTOS
 * mem management functions. Without this, all hell will break loose.
 *---------------------------------------------------------------*/
#if	defined(USING_FREERTOS) && defined __cplusplus
#include <Thor/FreeRTOS/include/FreeRTOS.h>
#include <Thor/FreeRTOS/include/portable.h>
#include "new"

void * operator new(size_t size);

void * operator new[](size_t size);

void operator delete(void *p) noexcept;

#endif /* USING_FREERTOS */

#endif /* MEM_H_*/