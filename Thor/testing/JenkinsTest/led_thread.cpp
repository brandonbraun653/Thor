/* Boost Includes */
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

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

void ledThread(void* argument)
{
	using namespace Thor::Peripheral::GPIO;
	using namespace Thor::Definitions::GPIO;

	GPIOClass blue_led(GPIOB, PIN_7);
	
	blue_led.mode(OUTPUT_PP);
	blue_led.write(HIGH);

	
	TickType_t lastTimeWoken = xTaskGetTickCount();
	for (;;)
	{
		blue_led.toggle();
		vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(500));
	}
}