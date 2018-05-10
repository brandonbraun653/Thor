/** @example uart_basic_example.cpp
 *	This demonstrates a simple way to get an instance of Thor::Peripheral::UART::UARTClass up and running quickly. */

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
			using namespace Thor::Definitions::Serial;
			using namespace Thor::Definitions::UART;

			/** A higher level uart interface built ontop of the STM32 HAL that abstracts away most
			 *	of the details associated with setup and general usage. It supports both transmission\n
			 *	and reception in 3 modes [blocking, interrupt, dma] and does not require that TX and RX
			 *	share the same mode for proper operation.
			 *
			 *	@note If using FreeRTOS, the class is threadsafe and allows multiple sources to write and
			 *		  read on a class object up to an internal buffer limit defined by Thor::Definitions::Serial::UART_BUFFER_SIZE
			 **/
			class UARTClass
			{
			public:
				/** Initializes with a given baud rate and TX/RX modes. If no parameters are given it will default to a
				 *	baudrate of 115200 and set both TX and RX modes to blocking.
				 *
				 *	@param[in] baud		Desired baud rate. Accepts standard rates from Thor::Definitions::Serial::BaudRate
				 *	@param[in] tx_mode	Sets the TX mode to Blocking, Interrupt, or DMA from Thor::Definitions::Serial::Modes
				 *	@param[in] rx_mode	Sets the RX mode to Blocking, Interrupt, or DMA from Thor::Definitions::Serial::Modes
				 *	@return	Status code indicating peripheral state. Will read 'PERIPH_OK' if everything is fine. Otherwise it
				 *			will return a code from Thor::Peripheral::Serial::Status
				 **/
				Status begin(const BaudRate& baud = SERIAL_BAUD_115200,
					const Modes& tx_mode = BLOCKING,
					const Modes& rx_mode = BLOCKING);

				/** Places the specified peripheral into a given mode
				 *	@param[in] periph	Explicitly states which peripheral subsystem (TX or RX) to set from Thor::Peripheral::Serial::SubPeripheral
				 *	@param[in] mode		The corresponding mode for the peripheral to enter, from Thor::Peripheral::Serial::Modes
				 *	@return	Status code indicating peripheral state. Will read 'PERIPH_OK' if everything is fine. Otherwise it
				 *			will return a code from Thor::Peripheral::Serial::Status
				 **/
				Status setMode(const SubPeripheral& periph, const Modes& mode);

				/** Writes data to the serial output
				*	@param[in] val		Pointer to a mutable array
				*	@param[in] length	The length of data to be sent out
				*	@return	Status code indicating peripheral state. Will read 'PERIPH_OK' if everything is fine. Otherwise it
				*			will return a code from Thor::Peripheral::Serial::Status
				**/
				Status write(uint8_t* val, size_t length);

				/** Writes data to the serial output
				*	@param[in] string	Pointer to a mutable character array
				*	@param[in] length	The length of data to be sent out
				*	@return	Status code indicating peripheral state. Will read 'PERIPH_OK' if everything is fine. Otherwise it
				*			will return a code from Thor::Peripheral::Serial::Status
				**/
				Status write(char* string, size_t length);

				/** Writes data to the serial output
				*	@param[in] string	Pointer to an immutable character array. The length is internally calculated with strlen()
				*	@return	Status code indicating peripheral state. Will read 'PERIPH_OK' if everything is fine. Otherwise it
				*			will return a code from Thor::Peripheral::Serial::Status
				**/
				Status write(const char* string);

				/** Writes data to the serial output
				*	@param[in] string	Pointer to an immutable character array
				*	@param[in] length	The length of data to be sent out
				*	@return	Status code indicating peripheral state. Will read 'PERIPH_OK' if everything is fine. Otherwise it
				*			will return a code from Thor::Peripheral::Serial::Status
				**/
				Status write(const char* string, size_t length);

				/** Reads a single transmission of known length into the provided buffer.
				 *	@param[out] buff	An external buffer to write the received data to
				 *	@param[in]  length	The number of bytes to be received
				 *
				 *	@note Only use this for receptions that have a fixed, known length. For transmissions that last longer than
				 *		  the given 'length' value, it will simply be ignored and lost forever. Poor data.
				 **/
				Status read(uint8_t* buff, size_t length);

				/** Reads the next packet received into a buffer
				 *	@param[out] buff		Address of an external buffer to read data into
				 *	@param[in]	buff_length	The size of the external buffer
				 *	@return Status code indicating peripheral state. Will read 'UART_OK' if everything is fine. Otherwise it
				 *			will return a code from Thor::Peripheral::UART::UART_Status
				 *
				 *  @note This is for asynchronous data reception of unknown length in Interrupt or DMA mode only. If the length
				 *		  is known and only one transmission is to be received, use the provided read function instead.
				 **/
				Status readPacket(uint8_t* buff, size_t buff_length);

				/** How many unread asynchronously received packets are available
				 *	@return number of available packets
				 **/
				int availablePackets();

				/** Gets the size of the next received packet in the buffer
				 *	@return next packet size
				 **/
				size_t nextPacketSize();

				/** Clears the receive buffer entirely and waits for all buffered transmissions to complete */
				void flush();

				/** Deinitializes and cleans up the peripheral */
				void end();

				/** Provides a convenient way for the user to specifiy advanced configuration settings.
				 *	@param[in] config Configuration settings customized from the STM32 UART HAL struct defintion
				 **/
				void attachSettings(UART_InitTypeDef config);

				/** Primary handler for interrupt mode in TX or RX.
				 *	Additionally, will be called in DMA mode after UARTClass::IRQHandler_TXDMA or UARTClass::IRQHandler_RXDMA
				 **/
				void IRQHandler();

				/** DMA Handler for TX. It is a simple wrapper that calls the correct STM32 HAL DMA Handler. */
				void IRQHandler_TXDMA();

				/** DMA Handler for RX. It is a simple wrapper that calls the correct STM32 HAL DMA Handler. */
				void IRQHandler_RXDMA();

			private:
				UARTClass(const int& channel);

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
				static boost::shared_ptr<UARTClass> create(const int channel);
				~UARTClass();

				/** Easily contains references to buffered data for TX or RX */
				struct UARTPacket
				{
					uint8_t* data_ptr; /**< Contains the buffer address where data is stored */
					size_t length; /**< Number of bytes contained in data_ptr */
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
				uint32_t txMode = Thor::Definitions::Serial::MODE_UNDEFINED;
				uint32_t rxMode = Thor::Definitions::Serial::MODE_UNDEFINED;

				UARTPacket TX_tempPacket, RX_tempPacket;
				boost::circular_buffer<UARTPacket> TXPacketBuffer, RXPacketBuffer;

				/* Asynchronous RX buffer for many packets */
				boost::circular_buffer<uint8_t*> temp;
				
				uint8_t packetQueue[UART_PACKET_QUEUE_SIZE][UART_PACKET_BUFFER_SIZE];
				
				
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

				void UART_DMA_Init(const SubPeripheral& periph);
				void UART_DMA_DeInit(const SubPeripheral& periph);
				void UART_DMA_EnableIT(const SubPeripheral& periph);
				void UART_DMA_DisableIT(const SubPeripheral& periph);
				
				/*-------------------------------
				* Error Handler Functions 
				*------------------------------*/
				void UART_OverrunHandler();
			};

			/** The most common way of referencing an instance of UARTClass. It is intended that future
			 *	libraries will want to pass around copies of a class and a shared_ptr was chosen for ease\n
			 *	of use and safe destruction. */
			typedef boost::shared_ptr<UARTClass> UARTClass_sPtr;
		}
	}
}
#endif /* !UART_H_ */
