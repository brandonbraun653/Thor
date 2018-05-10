#include <Thor/include/exti.h>
#include <Thor/include/definitions.h>

using namespace Thor::Definitions::Serial;
using namespace Thor::Definitions::SPI;
using namespace Thor::Definitions::Interrupt;
using namespace Thor::Interrupt;

boost::circular_buffer<TaskTrigger*>TriggerBuffer;

#if defined(USING_FREERTOS)
TaskTrigger* trigger_obj = nullptr;
SemaphoreHandle_t* trigger_semphr = nullptr; 

void setupEXTI0_Interrupt()
{
	TriggerBuffer.set_capacity(5);
	
	LL_EXTI_InitTypeDef exti_init;

	exti_init.LineCommand	= ENABLE;
	exti_init.Line_0_31		= LL_EXTI_LINE_0;
	exti_init.Mode			= LL_EXTI_MODE_IT;
	exti_init.Trigger		= LL_EXTI_TRIGGER_RISING;

	LL_EXTI_Init(&exti_init);

	NVIC_SetPriority(EXTI0_IRQn, EXTI0_IRQn_Priority);
	NVIC_EnableIRQ(EXTI0_IRQn);
}

void EXTI0_IRQHandler()
{
	/* Clear the flag to prevent infinite looping */
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);

	/* Grab the latest instance that triggered this ISR */
	trigger_obj = TriggerBuffer.front();
	
	if (trigger_obj)
	{
		/* Remove the reference and grab the appropriate semaphore */
		TriggerBuffer.pop_front();
		trigger_semphr = trigger_obj->getEventSemaphore();
		
		if (trigger_semphr)
		{
			/* Give a semaphore to the waiting task */
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			xSemaphoreGiveFromISR(*trigger_semphr, &xHigherPriorityTaskWoken);

			/* Request a context switch if xHPTW is true */
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
	}
}
#else
void EXTI0_IRQHandler()
{
}
#endif

void EXTI1_IRQHandler()
{
}

void EXTI2_IRQHandler()
{
}

void EXTI3_IRQHandler()
{
}

void EXTI4_IRQHandler()
{
}

void EXTI9_5_IRQHandler()
{
}

void EXTI15_10_IRQHandler()
{
}