#include "test_uart_settings.hpp"
#include "test_uart.hpp"

#define LED_THREAD_IDX 1
#define RPC_THREAD_IDX 2
#define SER_THREAD_IDX 3

void init(void* argument);

int main(void)
{
	HAL_Init();
	ThorInit();

	setupRPCServer();

	TaskHandle_t initHandle;
	xTaskCreate(init, "init", 500, NULL, 1, &initHandle);
	Thor::xRegisterTaskHandle(Thor::INIT_THREAD, initHandle);
	vTaskStartScheduler();

	/* Should never reach here as scheduler should be running */
	for (;;)
	{

	}
}

void init(void* argument)
{
	using namespace Thor;

	volatile BaseType_t error = pdPASS;
	TickType_t lastTimeWoken = xTaskGetTickCount();
	TaskHandle_t threadHandle;

	/* Create the LED Thread  */
	error = xTaskCreate(ledTask, "led", 350, NULL, 2, &threadHandle);
	while (!ulTaskNotifyTake(pdTRUE, 0))
		vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(10));

	xRegisterTaskHandle(LED_THREAD_IDX, threadHandle);

	/* Create the Serial Thread */
	error = xTaskCreate(serverTask, "rpcServer", 250, NULL, 4, &threadHandle);
	while (!ulTaskNotifyTake(pdTRUE, 0))
		vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(10));

	xRegisterTaskHandle(RPC_THREAD_IDX, threadHandle);


	vTaskResume(TaskHandle[LED_THREAD_IDX]);
	vTaskResume(TaskHandle[RPC_THREAD_IDX]);
	xRemoveTaskHandle(INIT_THREAD);
}