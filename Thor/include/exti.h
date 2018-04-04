#pragma once
#ifndef EXTI_H_
#define EXTI_H_

/* Boost Includes */
#include <boost/circular_buffer.hpp>
#include <boost/container/vector.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <Thor/include/config.h>
#include <Thor/include/interrupt.h>

#if defined(TARGET_STM32F4)
#include "stm32f4xx_ll_exti.h"

#elif defined(TARGET_STM32F7)
#include "stm32f7xx_ll_exti.h"
#endif

#if defined(USING_FREERTOS)
#include "FreeRTOS.h"
#include "semphr.h"




/*--------------------------------------------------------------------------
 * Ensures that the EXTI0 interrupt will always be able to preempt most other 
 * threads running on the kernel. This is important for quick response times. 
 *-------------------------------------------------------------------------*/
#define EXTI0_IRQn_Priority (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY)
extern void setupEXTI0_Interrupt();

class TaskTrigger
{
public:
	bool logEventGenerator(Thor::Interrupt::TriggerSource source, uint32_t instance);
	bool logEventConsumer(Thor::Interrupt::TriggerSource source, uint32_t instance, SemaphoreHandle_t* sem);
	SemaphoreHandle_t* getNextEvent();

	TaskTrigger();
	~TaskTrigger();
private:
	struct EventSource
	{
		Thor::Interrupt::TriggerSource periph_src;
		uint32_t periph_instance;
	};
	
	
	SemaphoreHandle_t USARTSemaphore_Mutex;
	SemaphoreHandle_t SPISemaphore_Mutex;
	
	volatile bool pendingTask_lock;
	const int taskBufferSize = 10;
	boost::circular_buffer<EventSource> pendingTask;
	
	boost::container::vector<SemaphoreHandle_t*> vecUART_USART_Semaphores;
	boost::container::vector<SemaphoreHandle_t*> vecSPI_Semaphores;

};
extern boost::shared_ptr<TaskTrigger> EXTI0_TaskMGR;
#endif /* USING_FREERTOS */


#ifdef __cplusplus
extern "C" {
#endif
	void EXTI0_IRQHandler();
	void EXTI1_IRQHandler();
	void EXTI2_IRQHandler();
	void EXTI3_IRQHandler();
	void EXTI4_IRQHandler();
	void EXTI9_5_IRQHandler();
	void EXTI15_10_IRQHandler();
#ifdef __cplusplus
}
#endif

#endif