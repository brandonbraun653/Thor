/* FreeRTOS Includes */
#include "FreeRTOS.h"
#include "task.h"

/* Thor Includes */
#include <Thor/include/thor.hpp>
#include <Thor/include/config.hpp>
#include <Thor/include/definitions.hpp>
#include <Thor/include/gpio.hpp>

/* Project Includes */
#include "led_thread.hpp"
#include "spi_thread.hpp"
#include "serial_thread.hpp"

/*
Blue Led: PB7
Red Led: PB14
Green Led: PB0
*/
void redLED(void* arguments);
void greenLED(void* arguments);

int main(void)
{
	HAL_Init();
	ThorInit();

	//yoooos

	xTaskCreate(redLED, "ledThread1", 350, NULL, 2, NULL);
	xTaskCreate(greenLED, "ledThread2", 350, NULL, 2, NULL);
	xTaskCreate(ledThread, "ledThread3", 350, NULL, 2, NULL);
	//xTaskCreate(serialThread, "serialThread", 1000, NULL, 2, NULL);
	vTaskStartScheduler();
	
	/* Should never reach here as scheduler should be running */
	for (;;)
	{
		
	}
}

void redLED(void* arguments)
{
	using namespace Thor::Peripheral::GPIO;
	using namespace Thor::Definitions::GPIO;

	GPIOClass red_led(GPIOB, PIN_14);
	
	red_led.mode(OUTPUT_PP);
	red_led.write(HIGH);

	
	TickType_t lastTimeWoken = xTaskGetTickCount();
	for (;;)
	{
		red_led.toggle();
		vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(100));
	}
}

void greenLED(void* arguments)
{
	using namespace Thor::Peripheral::GPIO;
	using namespace Thor::Definitions::GPIO;

	GPIOClass green_led(GPIOB, PIN_0);
	
	green_led.mode(OUTPUT_PP);
	green_led.write(HIGH);

	
	TickType_t lastTimeWoken = xTaskGetTickCount();
	for (;;)
	{
		green_led.toggle();
		vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(1000));
	}
}