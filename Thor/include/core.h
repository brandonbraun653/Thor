#ifndef CORE_H_
#define CORE_H_

#include "thor_config.h"
#include "exceptions.h"

#ifdef TARGET_STM32F7
#include <stm32f7xx_hal.h>
#endif

#ifdef TARGET_STM32F4
#include <stm32f4xx_hal.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif
	void SysTick_Handler();
#ifdef __cplusplus
}
#endif

extern void ThorSystemClockConfig();

#if defined(USING_FREERTOS)

#endif




#endif /*! CORE_H_ */