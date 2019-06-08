/********************************************************************************
 * File Name:
 *   headers.hpp
 *
 * Description:
 *   Includes critical headers needed for the operation of Thor
 *
 * 2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/
#pragma once
#ifndef THOR_HEADERS_HPP
#define THOR_HEADERS_HPP

/* Thor Includes */
#include <Thor/preprocessor.hpp>
#include <Thor/definitions/system_checks.hpp>

/* Chimera Includes */
#include <Chimera/chimera.hpp>

#ifdef __cplusplus
extern "C"
{
#endif

#if defined( TARGET_STM32F7 )
#include <stm32f7/STM32F7xx_HAL_Driver/Inc/stm32f7xx_hal.h>
#endif /* TARGET_STM32F7 */

#if defined( TARGET_STM32F4 )
#include <stm32f4/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h>
#endif /* TARGET_STM32F4 */

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
