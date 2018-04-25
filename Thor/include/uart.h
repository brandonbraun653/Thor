#pragma once
#ifndef UART_H_
#define UART_H_

/* C/C++ Includes */
#include <stdlib.h>
#include <stdint.h>

/* Boost Includes */
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/container/static_vector.hpp>

/* Thor Includes */
#include <Thor/include/config.h>
#include <Thor/include/definitions.h>
#include <Thor/include/defaults.h>
#include <Thor/include/gpio.h>
#include <Thor/include/ringbuffer.h>

#if defined(USING_FREERTOS)
#include "FreeRTOS.h"
#include "semphr.h"
#include <Thor/include/exti.h>
#endif

/** @namespace Thor */
namespace Thor
{
	/** @namespace Thor::Peripheral */
	namespace Peripheral
	{
		/** @namespace Thor::Peripheral::UART */
		namespace UART
		{
			/** @enum UART_Status
			 *	Indicates various possible states of the uart. This includes general 
			 *	messages as well as error codes.
			 **/
			enum UART_Status : int
			{
				#if defined(USING_FREERTOS)
				UART_LOCKED                      = -4,
				#endif
				UART_NOT_INITIALIZED             = -3,
				UART_ERROR                       = -2,
				UART_NOT_READY                   = -1,
				UART_OK                          = 0,
				UART_TX_IN_PROGRESS,
				UART_RX_OK,
				UART_PACKET_TOO_LARGE_FOR_BUFFER
			};
			
			/** @enum UARTPeriph
			 *	Explicitly defines a uart peripheral type
			 **/
			enum class UARTPeriph : bool
			{
				RX = false,
				TX = true
			};

			/** @class UARTClass
			 * A higher level uart interface built ontop of the STM32 HAL that abstracts away most 
			 * of the details associated with setup and general usage.
			 * 
			 * @note If using FreeRTOS, the class is threadsafe and allows multiple sources to write and
			 * read on a class object up to an internal buffer limit defined by Thor::Definitions::Serial::UART_BUFFER_SIZE
			 **/
			class UARTClass
			{
			public:
				
				/** Initializes UART with default parameters. 
				 *	Baud rate is set to 115200 and both TX and RX modes are set to blocking. 
				 *	
				 *	@return Status code indicating UART state. Will read 'UART_OK' if everything is fine.
				 **/
				UART_Status begin();
				
				/** Initializes UART with a given baud rate. 
				 *	Both TX and RX modes are set to blocking.
				 *
				 *  @param[in] baud Desired baud rate. Accepts standard rates from 110-921600.
				 *  @return Status code indicating UART state. Will read 'UART_OK' if everything is fine.
				 **/
				UART_Status begin(Thor::Definitions::Serial::BaudRate baud);
				
				/** Initializes UART with a given baud rate and TX/RX modes.
				 *	@param[in] baud		Desired baud rate. Accepts standard rates are Thor::Definitions::Serial::Modes
				 *	@param[in] tx_mode	Sets the TX mode to Blocking, Interrupt, or DMA
				 *	@param[in] rx_mode	Sets the RX mode to Blocking, Interrupt, or DMA
				 *	@return	Status code indicating UART state. Will read 'UART_OK' if everything is fine.
				 **/
				UART_Status begin(Thor::Definitions::Serial::BaudRate baud, 
									Thor::Definitions::Serial::Modes tx_mode, 
									Thor::Definitions::Serial::Modes rx_mode);

				UART_Status write(uint8_t* val, size_t length);
				UART_Status write(char* string, size_t length);
				UART_Status write(const char* string);
				UART_Status write(const char* string, size_t length);
				UART_Status readPacket(uint8_t* buff, size_t buff_length);

				int availablePackets();
				size_t nextPacketSize();
				void flush();
				void end();

				void attachSettings(UART_InitTypeDef config);

				void setBlockMode(const UARTPeriph& periph);
				void setITMode(const UARTPeriph& periph);
				void setDMAMode(const UARTPeriph& periph);

				void IRQHandler();
				void IRQHandler_TXDMA();
				void IRQHandler_RXDMA();

			private:
				UARTClass(int channel);

			public:
				/** A factory method to create a new UARTClass object.
				 *
				 *	This method intentionally replaces the typical constructor due to the need to register\n
				 *	the shared_ptr with a static_vector that allows runtime deduction of which class to call\n
				 *	inside of an ISR. This is done for simplicity.
				 *	
				 *	@param[in] channel Hardware peripheral channel number (i.e. 1 for UART1, 4 for UART4, etc)
				 *	@return Shared pointer to the new object
				 **/
				static boost::shared_ptr<UARTClass> create(int channel);
				~UARTClass();

				struct UARTPacket
				{
					uint8_t *data_ptr;
					uint8_t data;
					size_t length;
				};

				/*-------------------------------
				* ISR Stubs (To protect private variables while still allowing access)
				*------------------------------*/
				int _getChannel(){ return uart_channel; }
				void _setTxComplete(){ tx_complete = true; }
				void _setRxComplete(){ rx_complete = true; }
				bool _getRxComplete(){ return rx_complete; }
				void _setRxAsync(){ RX_ASYNC = true; }
				bool _getInitStatus(){ return UART_PeriphState.uart_enabled; }
				uint32_t _getTxMode(){ return txMode; }
				uint32_t _getRxMode(){ return rxMode; }
				bool _txBufferEmpty(){ return TXPacketBuffer.empty(); }
				void _txBufferRemoveFrontPacket(){ TXPacketBuffer.pop_front(); }
				const UARTPacket& _txBufferNextPacket(){ return TXPacketBuffer.front(); }
				void _rxBufferPushBack(const UARTPacket& newPkt){ RXPacketBuffer.push_back(newPkt); }
				uint8_t* _rxCurrentQueuePacketRef(){ return packetQueue[currentQueuePacket]; }

			private:
				int uart_channel;
				bool tx_complete = true;
				bool rx_complete = true;
				bool RX_ASYNC = true;
				uint32_t txMode = Thor::Definitions::Serial::TX_MODE_NONE;
				uint32_t rxMode = Thor::Definitions::Serial::RX_MODE_NONE;

				UARTPacket TX_tempPacket, RX_tempPacket;
				boost::circular_buffer<UARTPacket> TXPacketBuffer, RXPacketBuffer;

				/* Asynchronous RX buffer for many packets */
				uint8_t packetQueue[Thor::Definitions::Serial::UART_PACKET_QUEUE_SIZE][Thor::Definitions::Serial::UART_BUFFER_SIZE];
				uint8_t currentQueuePacket = 0;
				uint32_t rxAsyncPacketSize = 0;
				int totalWaitingPackets = 0;

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
				void UART_DMA_EnableClock();
				void UART_EnableInterrupts();
				void UART_DisableInterrupts();

				void UART_DMA_Init(const UARTPeriph& periph);
				void UART_DMA_DeInit(const UARTPeriph& periph);
				void UART_DMA_EnableIT(const UARTPeriph& periph);
				void UART_DMA_DisableIT(const UARTPeriph& periph);
			};
			typedef boost::shared_ptr<UARTClass> UARTClass_sPtr;
		}
	}	
}
#endif /* !UART_H_ */
