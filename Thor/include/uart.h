#pragma once
#ifndef UART_H_
#define UART_H_

/************************************************************************/
/*							   Includes                                 */
/************************************************************************/
#include <stdlib.h>
#include <stdint.h>
#include <string>

#include "../libraries/Boost/boost/bind.hpp"
#include "../libraries/Boost/boost/shared_ptr.hpp"
#include "../libraries/Boost/boost/make_shared.hpp"
#include "../libraries/Boost/boost/circular_buffer.hpp"

#include "thor_config.h"
#if defined(STM32F7)
#include <stm32f7xx_hal.h>
#elif defined(STM32F4)
#include <stm32f4xx_hal.h>
#endif

#include "thor_definitions.h"
#include "exceptions.h"
#include "defaults.h"
#include "gpio.h"
#include "ringbuffer.h"
#include "interrupt.h"

#if defined(USING_FREERTOS)
#include "FreeRTOS.h"
#include "semphr.h"
#include "exti.h"
#endif



class UARTClass;
typedef boost::shared_ptr<UARTClass> UARTClass_sPtr;

typedef enum
{
	UART_NOT_INITIALIZED = -3,
	UART_ERROR = -2,
	UART_NOT_READY = -1,
	UART_READY = 0,
	UART_TX_IN_PROGRESS,
	UART_RX_OK
} UART_Status;

class UARTClass
{
public:
	/*-------------------------------
	* Initialization
	*------------------------------*/
	UART_Status begin();
	UART_Status begin(uint32_t baud);
	UART_Status begin(uint32_t baud, uint32_t tx_mode, uint32_t rx_mode);
	void end();

	/*-------------------------------
	* Generic IO and Processing
	*------------------------------*/
	UART_Status write(uint8_t* val, size_t length);
	UART_Status write(char* string, size_t length);
	UART_Status write(const char* string);
	UART_Status write(std::string string);
	
	int availablePackets();
	int nextPacketSize();
	int readPacket(uint8_t* buff, size_t buff_length);
	void flush();
	

	/*-------------------------------
	* Dynamic Reconfiguring
	*------------------------------*/
	void attachSettings(UART_InitTypeDef config);

	void setTxModeBlock();
	void setTxModeIT();
	void setTxModeDMA();

	void setRxModeBlock();
	void setRxModeIT();
	void setRxModeDMA();

	/*-------------------------------
	* Interrupt Handlers
	*------------------------------*/
	void UART_IRQHandler();
	void UART_IRQHandler_TXDMA();
	void UART_IRQHandler_RXDMA();

	/*-------------------------------
	* Class Constructors/Deconstructor
	*------------------------------*/
	UARTClass(int channel);
	~UARTClass();

	/*-------------------------------
	* Status Flags for User
	*------------------------------*/
	volatile bool* isInitialized;
	bool tx_complete = true;
	bool rx_complete = true;
	
	bool RX_ASYNC;
	
	uint32_t txMode;
	uint32_t rxMode;
	
	/*-------------------------------
	* Buffers
	*------------------------------*/
	struct UARTPacket
	{
		uint8_t *data_ptr;
		uint8_t data;
		size_t length;
	};
	UARTPacket TX_tempPacket, RX_tempPacket;
	boost::circular_buffer<UARTPacket> TXPacketBuffer, RXPacketBuffer;

	//TODO: ADD MUTEX TO PROTECT READ ACCESS DURING ISR ROUTINES
	/* Asynchronous RX buffer for many packets */
	uint8_t packetQueue[ThorDef::UART::UART_PACKET_QUEUE_SIZE][ThorDef::UART::UART_BUFFER_SIZE];
	uint8_t currentQueuePacket;
	uint32_t rxAsyncPacketSize;
	int totalWaitingPackets;

	/*-------------------------------
	* Threaded Support
	*------------------------------*/
	

private:
	/*-------------------------------
	* Class Variables / Flags
	*------------------------------*/
	int uart_channel;
	struct UARTClassStatus
	{
		bool gpio_enabled;
		bool uart_enabled;
		bool uart_interrupts_enabled;
		bool dma_enabled_tx;
		bool dma_enabled_rx;
		bool dma_interrupts_enabled_tx;
		bool dma_interrupts_enabled_rx;
	} UART_PeriphState;

	/*-------------------------------
	* Object Pointers / Handles
	*------------------------------*/
	UART_HandleTypeDef uart_handle;
	DMA_HandleTypeDef hdma_uart_tx;
	DMA_HandleTypeDef hdma_uart_rx;
	GPIOClass_sPtr tx_pin;
	GPIOClass_sPtr rx_pin;

	/* Local copy of interrupt settings */
	IT_Initializer ITSettings_HW, ITSettings_DMA_TX, ITSettings_DMA_RX;

	/*-------------------------------
	* Low Level Setup/Teardown Functions
	*------------------------------*/
	void UART_GPIO_Init();
	void UART_GPIO_DeInit();

	void UART_Init();
	void UART_DeInit();
	void UART_EnableClock();
	void UART_DisableClock();
	void UART_EnableInterrupts();
	void UART_DisableInterrupts();

	void UART_DMA_Init_RX();
	void UART_DMA_Init_TX();
	void UART_DMA_DeInit_TX();
	void UART_DMA_DeInit_RX();
	void UART_DMA_EnableClock();
	void UART_DMA_EnableInterrupts_TX();
	void UART_DMA_EnableInterrupts_RX();
	void UART_DMA_DisableInterrupts_TX();
	void UART_DMA_DisableInterrupts_RX();
};

/************************************************************************/
/*						    Exported Classes                            */
/************************************************************************/
#ifdef ENABLE_UART4
extern UARTClass_sPtr uart4;
#endif

#ifdef ENABLE_UART5
extern UARTClass_sPtr uart5;
#endif

#ifdef ENABLE_UART7
extern UARTClass_sPtr uart7;
#endif

#ifdef ENABLE_UART8
extern UARTClass_sPtr uart8;
#endif


#ifdef __cplusplus
extern "C" {
#endif
	#ifdef ENABLE_UART4
	void UART4_IRQHandler();
	#endif

	#ifdef ENABLE_UART5
	void UART5_IRQHandler();
	#endif

	#ifdef ENABLE_UART7
	void UART7_IRQHandler();
	#endif

	#ifdef ENABLE_UART8
	void UART8_IRQHandler();
	#endif
#ifdef __cplusplus
}
#endif

#endif /* !UART_H_ */
