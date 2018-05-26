/* Thor Includes */
#include <Thor/include/thor.hpp>
#include <Thor/include/nucleo.hpp>
#include <Thor/include/gpio.hpp>
#include <Thor/include/serial.hpp>
#include <Thor/include/uart.hpp>

/* eRPC Includes */
#include "led_server.h"
#include "erpc_server_setup.h"
#include "uart_thor_transport.h"

/* FreeRTOS Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define SERIAL_CHANNEL 7
#define RX_BUFF_SIZE 10

using namespace Thor::Nucleo;
using namespace Thor::Peripheral::GPIO;
using namespace Thor::Peripheral::UART;
using namespace Thor::Peripheral::Serial;


/*
Blue Led: PB7
Red Led: PB14
Green Led: PB0
*/
GPIOClass_sPtr greenLed = boost::make_shared<GPIOClass>(GREEN_LED_PORT, GREEN_LED_PIN);
GPIOClass_sPtr blueLed = boost::make_shared<GPIOClass>(BLUE_LED_PORT, BLUE_LED_PIN);

SerialClass_sPtr serial;
UARTClass_sPtr uart;
SemaphoreHandle_t uartSem;

void turnGreenLEDON()
{
	greenLed->write(HIGH);
}

void turnGreenLEDOFF()
{
	greenLed->write(LOW);
}

void ledTask(void* arguments)
{
	TickType_t lastTimeWoken = xTaskGetTickCount();
	for (;;)
	{
		blueLed->toggle();
		vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(500));
	}
}

void serverTask(void* arguments)
{
	//turnGreenLEDON();
	
	volatile erpc_status_t status;
	TickType_t lastTimeWoken = xTaskGetTickCount();
	for (;;)
	{
		xSemaphoreTake(serverWakeup, portMAX_DELAY);
		status = erpc_server_poll();
		//vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(100));
	}
}

void uartTest(void* arguments)
{
	uartSem = xSemaphoreCreateBinary();
	uart = UARTClass::create(7);
	
	uart->begin();
	uart->setMode(TX, DMA);
	uart->setMode(RX, DMA);

	uart->attachThreadTrigger(TX_COMPLETE, &uartSem);
	
	size_t rxSize = 5;
	uint8_t rxBuff[RX_BUFF_SIZE];
	memset(rxBuff, 0, RX_BUFF_SIZE);
	
	
	/* Kicks off the python script */
	uart->write("Waiting for 5 characters...\r\n");
	//uart->read(rxBuff, rxSize);
	

	size_t packetSize = 0;
	
	TickType_t lastTimeWoken = xTaskGetTickCount();
	vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(500));
	for (;;)
	{
		//Explicit version 
// 		if (rxBuff[0] != 0)
// 		{
// 			uart->write("\tReceived: ");
// 			uart->write(rxBuff, rxSize);
// 			uart->write("\r\n");
// 			
// 			memset(rxBuff, 0, RX_BUFF_SIZE);
// 			
// 			uart->write("Waiting for 5 characters...\r\n");
// 			uart->read(rxBuff, rxSize);
// 		}
		
		//Asynchronous version
//		if(uart->availablePackets())
//		{
//			packetSize = uart->nextPacketSize();
//			uart->readPacket(rxBuff, packetSize);
//			
//			uart->write("\tReceived: ");
//			uart->write(rxBuff, rxSize);
//			uart->write("\r\n");
//			
//			uart->write("Waiting for 5 characters...\r\n");
//		}
		
		vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(500));
	}
}

void serialTest(void* arguments)
{
	uartSem = xSemaphoreCreateBinary();
	serial = boost::make_shared<SerialClass>(7);
	
	serial->begin();
	serial->setMode(TX, DMA);
	serial->setMode(RX, DMA);

	//serial->attachThreadTrigger(TX_COMPLETE, &uartSem);
	
	size_t rxSize = 5;
	uint8_t rxBuff[RX_BUFF_SIZE];
	memset(rxBuff, 0, RX_BUFF_SIZE);
	
	
	/* Kicks off the python script */
	serial->write("Waiting for 5 characters...\r\n");
	//serial->read(rxBuff, rxSize);
	

	size_t packetSize = 0;
	
	TickType_t lastTimeWoken = xTaskGetTickCount();
	vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(500));
	for (;;)
	{
		//Explicit version 
// 		if (rxBuff[0] != 0)
// 		{
// 			serial->write("\tReceived: ");
// 			serial->write(rxBuff, rxSize);
// 			serial->write("\r\n");
// 			
// 			memset(rxBuff, 0, RX_BUFF_SIZE);
// 			
// 			serial->write("Waiting for 5 characters...\r\n");
// 			serial->read(rxBuff, rxSize);
// 		}
		
		//Asynchronous version
		if(serial->availablePackets())
		{
			packetSize = serial->nextPacketSize();
			serial->readPacket(rxBuff, packetSize);
			
			serial->write("\tReceived: ");
			serial->write(rxBuff, rxSize);
			serial->write("\r\n");
			
			serial->write("Waiting for 5 characters...\r\n");
		}
		
		vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(500));
	}
}

void triggered(void* arguments)
{
	uint8_t rxBuff[RX_BUFF_SIZE];
	memset(rxBuff, 0, RX_BUFF_SIZE);

	size_t rxSize = 5;
	size_t packetSize = 0;
	
	TickType_t lastTimeWoken = xTaskGetTickCount();
	for (;;)
	{
		//Block the task from running until the semaphore has been 'given'
		xSemaphoreTake(uartSem, portMAX_DELAY);
		serial->removeThreadTrigger(TX_COMPLETE);
		serial->write("\tTOTES MADE IT.\r\n");
		
		
// 		if (uart->availablePackets())
// 		{
// 			packetSize = uart->nextPacketSize();
// 			uart->readPacket(rxBuff, packetSize);
// 			
// 			uart->write("\tReceived: ");
// 			uart->write(rxBuff, rxSize);
// 			uart->write("\r\n");
// 			
// 			uart->write("Waiting for 5 characters...\r\n");
// 		}
		
		vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(500));
	}
}

int main(void)
{
	using namespace Thor::Peripheral::GPIO;

	HAL_Init();
	ThorInit();

	greenLed->mode(OUTPUT_PP);
	blueLed->mode(OUTPUT_PP);

	turnGreenLEDON();
	turnGreenLEDOFF();

	/* Init the eRPC server environment */
	erpc_transport_t transport = erpc_transport_thor_uart_init(SERIAL_CHANNEL);
	
	/* Message buffer factory initialization */
	erpc_mbf_t message_buffer_factory = erpc_mbf_dynamic_init();
	
	/* eRPC server side initialization */
	erpc_server_init(transport, message_buffer_factory);
	
	/* Connect generated service into server, look @ "led_server.h" for details */
	erpc_add_service_to_server(create_IO_service());

	/* Start up the tasks */
	xTaskCreate(ledTask, "led", 250, NULL, 2, NULL);
	xTaskCreate(serverTask, "server", 250, NULL, 2, NULL);
	//xTaskCreate(serialTest, "serial", 500, NULL, 2, NULL);
	//xTaskCreate(triggered, "trig", 250, NULL, 2, NULL);
	
	vTaskStartScheduler();

	/* Should never reach here as scheduler should be running */
	for (;;)
	{
	}
}