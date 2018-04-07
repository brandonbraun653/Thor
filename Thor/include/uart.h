#pragma once
#ifndef UART_H_
#define UART_H_

/* C/C++ Includes */
#include <stdlib.h>
#include <stdint.h>
#include <string>

/* Boost Includes */
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/container/flat_map.hpp>

/* Thor Includes */
#include <Thor/include/config.h>
#include <Thor/include/definitions.h>
#include <Thor/include/exceptions.h>
#include <Thor/include/defaults.h>
#include <Thor/include/gpio.h>
#include <Thor/include/ringbuffer.h>
#include <Thor/include/interrupt.h>

#if defined(USING_FREERTOS)
#include "FreeRTOS.h"
#include "semphr.h"
#include "exti.h"
#endif

namespace Thor
{
	namespace Peripheral
	{
		namespace UART
		{
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
				void IRQHandler();
				void IRQHandler_TXDMA();
				void IRQHandler_RXDMA();

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
				uint8_t packetQueue[Thor::Definitions::Serial::UART_PACKET_QUEUE_SIZE][Thor::Definitions::Serial::UART_BUFFER_SIZE];
				uint8_t currentQueuePacket = 0;
				uint32_t rxAsyncPacketSize = 0;
				int totalWaitingPackets = 0;

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
					bool gpio_enabled = false;
					bool uart_enabled = false;
					bool dma_enabled_tx = false;
					bool dma_enabled_rx = false;
					bool uart_interrupts_enabled = false;
					bool dma_interrupts_enabled_tx = false;
					bool dma_interrupts_enabled_rx = false;
				} UART_PeriphState;

				/*-------------------------------
				* Object Pointers / Handles
				*------------------------------*/
				UART_HandleTypeDef uart_handle;
				DMA_HandleTypeDef hdma_uart_tx;
				DMA_HandleTypeDef hdma_uart_rx;
				Thor::Peripheral::GPIO::GPIOClass_sPtr tx_pin;
				Thor::Peripheral::GPIO::GPIOClass_sPtr rx_pin;


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
			typedef boost::shared_ptr<UARTClass> UARTClass_sPtr;
		}
	}

	namespace Definitions
	{
		namespace Serial
		{
			extern boost::container::flat_map<USART_TypeDef*, Thor::Peripheral::UART::UARTClass_sPtr> uart_periph_to_class;
			//extern boost::container::flat_map<USART_TypeDef*, Thor::Peripheral::UART::USARTClass_sPtr> usart_periph_to_class;
		}
	}
}


#if defined(ENABLE_UART1)
extern Thor::Peripheral::UART::UARTClass_sPtr uart1;
#endif
#if defined(ENABLE_UART2)
extern Thor::Peripheral::UART::UARTClass_sPtr uart2;
#endif
#if defined(ENABLE_UART3)
extern Thor::Peripheral::UART::UARTClass_sPtr uart3;
#endif
#if defined(ENABLE_UART4)
extern Thor::Peripheral::UART::UARTClass_sPtr uart4;
#endif
#if defined(ENABLE_UART5)
extern Thor::Peripheral::UART::UARTClass_sPtr uart5;
#endif
#if defined(ENABLE_UART6)
extern Thor::Peripheral::UART::UARTClass_sPtr uart6;
#endif 
#if defined(ENABLE_UART7)
extern Thor::Peripheral::UART::UARTClass_sPtr uart7;
#endif
#if defined(ENABLE_UART8)
extern Thor::Peripheral::UART::UARTClass_sPtr uart8;
#endif


#endif /* !UART_H_ */
