#pragma once
#ifndef MEM_H_
#define MEM_H_

#include "thor_config.h"

/*----------------------------------------------------------------
 * Globally overload the new/delete operators to use the FreeRTOS
 * mem management functions. Without this, all hell will break loose.
 *---------------------------------------------------------------*/
#if	defined(USING_FREERTOS) && defined __cplusplus
#include "FreeRTOS.h"
#include "portable.h"
#include "new"

void * operator new(size_t size);

void * operator new[](size_t size);

void operator delete(void *p) noexcept;

#endif /* USING_FREERTOS */

#endif /* MEM_H_*/