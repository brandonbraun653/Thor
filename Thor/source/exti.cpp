#include <Thor/include/exti.h>
#include <Thor/include/definitions.h>

/*-------------------------------------------------------------
 * Take over EXTI0 to use for passing messages from high
 * priority ISRs down to this lower one so it can trigger
 * a task to run in the FreeRTOS kernel.
 *------------------------------------------------------------*/
#if defined(USING_FREERTOS)
boost::shared_ptr<TaskTrigger> EXTI0_TaskMGR = boost::make_shared<TaskTrigger>();

void setupEXTI0_Interrupt()
{
	LL_EXTI_InitTypeDef exti_init;

	exti_init.LineCommand = ENABLE;
	exti_init.Line_0_31 = LL_EXTI_LINE_0;
	exti_init.Mode = LL_EXTI_MODE_IT;
	exti_init.Trigger = LL_EXTI_TRIGGER_RISING;

	LL_EXTI_Init(&exti_init);

	NVIC_SetPriority(EXTI0_IRQn, EXTI0_IRQn_Priority);
	NVIC_EnableIRQ(EXTI0_IRQn);
}

TaskTrigger::TaskTrigger()
{
	/* Interestingly, set_capacity MUST be used instead of resize...unsure why*/
	pendingTask.set_capacity(taskBufferSize);

	/* Normal resize works fine here */
	vecUART_USART_Semaphores.resize(Thor::Libraries::Serial::MAX_SERIAL_CHANNELS);
	vecSPI_Semaphores.resize(Thor::Definitions::SPI::MAX_SPI_CHANNELS);

	/* Create the locking mechanisms */
	pendingTask_lock = false;
	USARTSemaphore_Mutex = xSemaphoreCreateMutex();
	SPISemaphore_Mutex = xSemaphoreCreateMutex();
}

TaskTrigger::~TaskTrigger()
{
}

/*-----------------------------------------------------------------------------------
 * This function is called from the high priority interrupt and thus can only
 * lock through simple flags. If a lock cannot be obtained, it returns false.
 * In practice this is not likely due to the short duration where lock==true
 *----------------------------------------------------------------------------------*/
bool TaskTrigger::logEventGenerator(Thor::Interrupt::TriggerSource source, uint32_t instance)
{
	if (pendingTask_lock)
		return false;

	pendingTask_lock = true;
	pendingTask.push_back({ source, instance });
	pendingTask_lock = false;

	/* Trigger the EXTI0 Interrupt to signal data ready. Because it has a lower priority
	 * than whatever ISR is calling, it will not immediately fire. */
	LL_EXTI_GenerateSWI_0_31(LL_EXTI_LINE_0);
	return true;
}

/*-----------------------------------------------------------------------------------
 * Attaches a semaphore to a specific interrupt source. Currently only supports
 * one semaphore per source, but may expand to multiple in the future.
 *----------------------------------------------------------------------------------*/
bool TaskTrigger::logEventConsumer(Thor::Interrupt::TriggerSource source, uint32_t instance, SemaphoreHandle_t* sem)
{
	/* Instance is correlated to the physical hardware peripherals, starting at index 1.*/
	switch (source)
	{
	case Thor::Interrupt::SRC_UART:
	case Thor::Interrupt::SRC_USART:
		if ((instance > 0) && (instance <= Thor::Libraries::Serial::MAX_SERIAL_CHANNELS))
		{
			vecUART_USART_Semaphores[instance - 1] = sem;
			return true;
		}
		else
			return false;
		break;

	case Thor::Interrupt::SRC_SPI:
		if ((instance > 0) && (instance <= Thor::Definitions::SPI::MAX_SPI_CHANNELS))
		{
			vecSPI_Semaphores[instance - 1] = sem;
			return true;
		}
		else
			return false;
		break;

	default: return false;
	}
}

/*-----------------------------------------------------------------------------------
 * This function is called from the EXTI interrupt, which holds the highest
 * priority allowed in FreeRTOS in order to utilize the FreeRTOS API. This is defined
 * by EXTI0_IRQn_Priority
 *----------------------------------------------------------------------------------*/
SemaphoreHandle_t* TaskTrigger::getNextEvent()
{
	SemaphoreHandle_t* tempSem = nullptr;

	if (!pendingTask.empty())
	{
		EventSource nextTask = pendingTask.front();

		/* Grab the correct semaphore needed to wake up a process. Don't need
		 * to check accessor bounds due to constraints in logEventConsumerTask() */
		switch (nextTask.periph_src)
		{
		case Thor::Interrupt::SRC_UART:
		case Thor::Interrupt::SRC_USART:
			xSemaphoreTakeFromISR(USARTSemaphore_Mutex, NULL);
			tempSem = vecUART_USART_Semaphores[nextTask.periph_instance - 1];
			xSemaphoreGiveFromISR(USARTSemaphore_Mutex, NULL);
			break;

		case Thor::Interrupt::SRC_SPI:
			xSemaphoreTakeFromISR(SPISemaphore_Mutex, NULL);
			tempSem = vecSPI_Semaphores[nextTask.periph_instance - 1];
			xSemaphoreGiveFromISR(SPISemaphore_Mutex, NULL);
			break;

		default: break;
		}

		pendingTask.pop_front();
	}

	return tempSem;
}

/*-----------------------------------------------------------------------------------
 * Handles all requests from very high priority interrupts (aka peripherals) that 
 * cannot directly make use of the FreeRTOS API to wake up a thread used to process
 * some data. Instead, the EXTI0 handler is used as a FIFO proxy.
 *----------------------------------------------------------------------------------*/
void EXTI0_IRQHandler()
{
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);

	SemaphoreHandle_t* newTask = EXTI0_TaskMGR->getNextEvent();

	if (newTask == nullptr)
		return;

	/* Give a semaphore to the waiting task */
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(*newTask, &xHigherPriorityTaskWoken);

	/* Request a context switch if xHPTW is true */
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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