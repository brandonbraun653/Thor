#pragma once
#ifndef THOR_THREADING_HPP
#define THOR_THREADING_HPP
	
	#include <Thor/include/thor.hpp>

	#ifdef USING_FREERTOS
		/* FreeRTOS Includes */
		#include "FreeRTOS.h"
		#include "task.h"
		#include "queue.h"
		#include "semphr.h"

		namespace Thor
		{
			const uint32_t INIT_THREAD = 0;
			const uint32_t MAX_THREADS = 10;
			extern TaskHandle_t TaskHandle[];
		
			extern BaseType_t xRegisterTaskHandle(const uint32_t idx, TaskHandle_t handle);
			extern BaseType_t xRemoveTaskHandle(const uint32_t idx);
			extern BaseType_t xTaskSendMessage(const uint32_t idx, const uint32_t);
			extern BaseType_t xTaskSendMessageFromISR(const uint32_t idx, const uint32_t);
		}
	#endif /* !USING_FREERTOS */
#endif /* !THOR_THREADING_HPP */