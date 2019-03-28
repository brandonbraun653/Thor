#pragma once
#ifndef THOR_HEADERS_HPP
#define THOR_HEADERS_HPP

#include <Thor/preprocessor.hpp>

#if defined( USING_CHIMERA )
#include <Chimera/chimera.hpp>
#endif

#ifdef __cplusplus
 extern "C" {
#endif

#if defined( TARGET_STM32F7 )
#include "stm32f7xx_hal.h"
#endif

#if defined( TARGET_STM32F4 )
#include "stm32f4xx_hal.h"
#endif

#if defined( USING_FREERTOS )
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"

#if configUSE_TICK_HOOK != 1
#warning Please set "configUSE_TICK_HOOK" in FreeRTOSConfig.h or some HAL Libs will break.
#endif
#endif /* USING_FREERTOS */

#ifdef __cplusplus
}
#endif

#endif
