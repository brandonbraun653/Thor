#ifndef CORE_H_
#define CORE_H_

#include <Thor/include/config.hpp>
#include <Thor/include/exceptions.hpp>
#include <Thor/include/macro.hpp>

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
	
	#if defined(USING_FREERTOS)
	void vApplicationTickHook(void);
	#endif 
#ifdef __cplusplus
}
#endif

extern void ThorSystemClockConfig();






#endif /*! CORE_H_ */