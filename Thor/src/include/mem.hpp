#pragma once
#ifndef THOR_MEM_HPP
#define THOR_MEM_HPP

#include <Thor/include/config.hpp>

/* Globally overload the new/delete operators to use the FreeRTOS mem management functions. Without this, all hell will break loose.
 * If using the Embedded Remote Procedural Call library (eRPC), it provides the necessary overload definitions and thus the ones below
 * can be disabled safely. */
#if	defined(USING_FREERTOS) && !defined(USING_ERPC) && defined __cplusplus
#include <Thor/FreeRTOS/include/FreeRTOS.h>
#include <Thor/FreeRTOS/include/portable.h>
#include "new"

void * operator new(size_t size);

void * operator new[](size_t size);

void operator delete(void *p) noexcept;

#endif /* USING_FREERTOS */

#endif /* THOR_MEM_HPP*/