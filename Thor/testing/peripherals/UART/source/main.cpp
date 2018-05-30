#include "test_uart_settings.hpp"
#include "test_uart.hpp"

#include <Thor/include/threads.hpp>
using namespace Thor::Threading;

TaskHandle_t ledThread;
TaskHandle_t rpcThread;
TaskHandle_t serThread;

int main(void)
{
	HAL_Init();
	ThorInit();

	setupRPCServer();

	//TaskHandle_t initHandle;
	//xTaskCreate(init, "init", 500, NULL, 1, &initHandle);
	//Thor::xRegisterTaskHandle(Thor::INIT_THREAD, initHandle);
	//vTaskStartScheduler();

	addThread(ledTask, "led", 350, NULL, 2, &ledThread);
	addThread(serverTask, "rpcServer", 250, NULL, 4, &rpcThread);
	startScheduler();

	/* Should never reach here as scheduler should be running */
	for (;;)
	{

	}
}