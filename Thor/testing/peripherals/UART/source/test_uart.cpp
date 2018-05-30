#include "test_uart.hpp"

/* eRPC Includes */
#include "uart_rpc.h"
#include "erpc_server_setup.h"
#include "serial_thor_transport.h"
#include "uart_rpc_server.h"

using namespace Thor;
using namespace Thor::Nucleo;
using namespace Thor::Threading;
using namespace Thor::Peripheral::GPIO;
using namespace Thor::Peripheral::Serial;



GPIOClass_sPtr greenLed = boost::make_shared<GPIOClass>(GREEN_LED_PORT, GREEN_LED_PIN);
GPIOClass_sPtr blueLed = boost::make_shared<GPIOClass>(BLUE_LED_PORT, BLUE_LED_PIN);

void ledTask(void* arguments)
{
	greenLed->mode(OUTPUT_PP);
	blueLed->mode(OUTPUT_PP);
	
	signalThreadSetupComplete();

	TickType_t lastTimeWoken = xTaskGetTickCount();
	for (;;)
	{
		blueLed->toggle();
		vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(500));
	}
}


void serverTask(void* arguments)
{
	volatile erpc_status_t status;
	signalThreadSetupComplete();

	TickType_t lastTimeWoken = xTaskGetTickCount();
	for (;;)
	{
		xSemaphoreTake(serverWakeup, portMAX_DELAY);
		status = erpc_server_poll();
	}
}

void setupRPCServer()
{
	/* Init the eRPC server environment */
	erpc_transport_t transport = erpc_transport_thor_serial_init(RPC_SERIAL_CHANNEL);

	/* Message buffer factory initialization */
	erpc_mbf_t message_buffer_factory = erpc_mbf_dynamic_init();

	/* eRPC server side initialization */
	erpc_server_init(transport, message_buffer_factory);

	/* Connect generated service into server */
	erpc_add_service_to_server(create_TEST_UART_service());
}


void turnGreenLEDON()
{
	greenLed->write(HIGH);
}

void turnGreenLEDOFF()
{
	greenLed->write(LOW);
}