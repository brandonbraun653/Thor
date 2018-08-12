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
#include <Thor/include/config.hpp>
#include <Thor/include/definitions.hpp>
#include <Thor/include/defaults.hpp>
#include <Thor/include/gpio.hpp>

#if defined(USING_FREERTOS)
#include "FreeRTOS.h"
#include "semphr.h"
#include <Thor/include/exti.hpp>
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
			using namespace Thor::Definitions;
			using namespace Thor::Definitions::Serial;
			using namespace Thor::Definitions::Interrupt;
			using namespace Thor::Definitions::UART;

			/** A higher level uart interface built ontop of the STM32 HAL that abstracts away most
			 *	of the details associated with setup and general usage. It supports both transmission\n
			 *	and reception in 3 modes [blocking, interrupt, dma] and does not require that TX and RX
			 *	share the same mode for proper operation. In addition, data is copied to an internal buffer\n
			 *	for all non-blocking transmissions (IT/DMA) so that the user doesn't have to worry about 
			 *	destroyed or mutable data.
			 *
			 *	@note If using FreeRTOS, the class is threadsafe and allows multiple sources to write and
			 *		  read on a class object up to an internal buffer limit defined by Thor::Definitions::Serial::UART_BUFFER_SIZE
			 **/
			class UARTClass : public SerialBase
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
				Status begin(const uint32_t& baud = 115200u,
					const Modes& tx_mode = Modes::BLOCKING,
					const Modes& rx_mode = Modes::BLOCKING) override;

				/** Places the specified peripheral into a given mode
				 *	@param[in] periph	Explicitly states which peripheral subsystem (TX or RX) to set from Thor::Peripheral::Serial::SubPeripheral
				 *	@param[in] mode		The corresponding mode for the peripheral to enter, from Thor::Peripheral::Serial::Modes
				 *	@return	Status code indicating peripheral state. Will read 'PERIPH_OK' if everything is fine. Otherwise it
				 *			will return a code from Thor::Peripheral::Serial::Status
				 *			
				 *	@note When setting the RX peripheral to IT or DMA mode, it automatically enables asynchronous data reception
				 **/
				Status setMode(const SubPeripheral& periph, const Modes& mode) override;

				/**
				 *
				 **/
				Status setBaud(const uint32_t& baud) override;


				/** Writes data to the serial output
				*	@param[in] val		Pointer to a mutable array
				*	@param[in] length	The length of data to be sent out
				*	@return	Status code indicating peripheral state. Will read 'PERIPH_OK' if everything is fine. Otherwise it
				*			will return a code from Thor::Peripheral::Serial::Status
				**/
				Status write(uint8_t* val, size_t length) override;

				/** Writes data to the serial output
				*	@param[in] string	Pointer to a mutable character array
				*	@param[in] length	The length of data to be sent out
				*	@return	Status code indicating peripheral state. Will read 'PERIPH_OK' if everything is fine. Otherwise it
				*			will return a code from Thor::Peripheral::Serial::Status
				**/
				Status write(char* string, size_t length) override;

				/** Writes data to the serial output
				*	@param[in] string	Pointer to an immutable character array. The length is internally calculated with strlen()
				*	@return	Status code indicating peripheral state. Will read 'PERIPH_OK' if everything is fine. Otherwise it
				*			will return a code from Thor::Peripheral::Serial::Status
				**/
				Status write(const char* string) override;

				/** Writes data to the serial output
				*	@param[in] string	Pointer to an immutable character array
				*	@param[in] length	The length of data to be sent out
				*	@return	Status code indicating peripheral state. Will read 'PERIPH_OK' if everything is fine. Otherwise it
				*			will return a code from Thor::Peripheral::Serial::Status
				**/
				Status write(const char* string, size_t length) override;

				/** Commands the RX peripheral to read a single transmission of known length into the provided buffer.
				 *	@param[out] buff	An external buffer to write the received data to
				 *	@param[in]  length	The number of bytes to be received
				 *
				 *	@note Only use this for receptions that have a fixed, known length. For transmissions that last longer than
				 *		  the given 'length' value, it will simply be ignored and lost forever. Poor data.
				 **/
				Status readSync(uint8_t* buff, size_t length) override;

				/** Reads the next packet received into a buffer 
				 *	@param[out] buff		Address of an external buffer to read data into
				 *	@param[in]	length		Number of bytes to read out from the packet
				 *	@return Status code indicating peripheral state. Will read 'UART_OK' if everything is fine. Otherwise it
				 *			will return a code from Thor::Peripheral::UART::UART_Status
				 *
				 *  @note This grabs data from an asynchronous data reception of unknown length in Interrupt or DMA mode only. If the length
				 *		  is known and only one transmission is to be received, use the provided readSync function instead.
				 **/
				Status readPacket(uint8_t* buff, size_t length) override;

				/** How many unread asynchronously received packets are available
				 *	@return number of available packets
				 **/
				uint32_t availablePackets() override;

				/** Gets the size of the next asynchronously received packet in the buffer
				 *	@return next packet size
				 **/
				size_t nextPacketSize() override;

				/** Deinitializes and cleans up the peripheral */
				void end() override;

				/** Provides a convenient way for the user to specify advanced configuration settings.
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
				
				#if defined(USING_FREERTOS)
				/** Attaches a semaphore to a specific trigger source. When an event is triggered on that source,
				 *	the semaphore will be 'given' to and any task waiting on that semaphore will become unblocked.
				 *	@param[in] trig		The source to be triggered on, of type Thor::Definitions::Interrupt::Trigger
				 *	@param[in] semphr	The address of the semaphore that will be 'given' to upon triggering 
				 **/
				void attachThreadTrigger(Trigger trig, SemaphoreHandle_t* semphr) override;
				
				/** Removes a trigger source 
				 *	@param[in] trig	The source to be removed, of type Thor::Definitions::Interrupt::Trigger
				 **/
				void removeThreadTrigger(Trigger trig) override;
				#endif 
				

			private:
				/** The real constructor used by UARTClass::create */
				UARTClass(const int& channel, Thor::Definitions::Serial::SerialPins* pinConfig);

			public:
				/** A factory method to create a new UARTClass object.
				 *
				 *	This method intentionally replaces the typical constructor due to the need to register
				 *	the shared_ptr with a static_vector that allows runtime deduction of which class to call
				 *	inside of an ISR. This is done for simplicity.
				 *
				 *	@param[in] channel Hardware peripheral channel number (i.e. 1 for UART1, 4 for UART4, etc)
				 *	@return Shared pointer to the new object
				 **/
				static boost::shared_ptr<UARTClass> create(const int channel, Thor::Definitions::Serial::SerialPins* pinConfig = nullptr);
				~UARTClass();

				/** Easily references buffered data for TX or RX */
				struct UARTPacket
				{
					uint8_t* data_ptr = nullptr;	/**< Contains the buffer address where data is stored */
					uint16_t bytesRead = 0;			/**< Number of bytes already read from the packet (currently used in eRPC calls) */
					size_t length = 0;				/**< Number of bytes contained in data_ptr */
				};

				/*-------------------------------
				* ISR Stubs 
				*------------------------------*/
				int _getChannel(){ return uart_channel; }
				
				void _setTxComplete(){ tx_complete = true; }
				
				void _setRxComplete(){ rx_complete = true; }
				
				bool _getRxComplete(){ return rx_complete; }
				
				void _setRxAsync(){ RX_ASYNC = true; }
				
				bool _getInitStatus(){ return UART_PeriphState.uart_enabled; }
				
				Modes _getTxMode(){ return txMode; }
				
				Modes _getRxMode(){ return rxMode; }
				
				bool _txBufferEmpty(){ return TXPacketBuffer.empty(); }
				
				void _txBufferRemoveFrontPacket(){ TXPacketBuffer.pop_front(); }
				
				const UARTPacket& _txBufferNextPacket(){ return TXPacketBuffer.front(); }
				
				void _rxBufferPushBack(const UARTPacket& newPkt){ RXPacketBuffer.push_back(newPkt); }
				
				uint8_t* _rxCurrentQueueAddr(){ return RX_Queue[RXQueueIdx]; }
				
				void _rxIncrQueueIdx()
				{
					RXQueueIdx++;

					if (RXQueueIdx == UART_QUEUE_SIZE)
						RXQueueIdx = 0;
				}

			private:
				int uart_channel;																/**< Numerical representation of the UART instance, zero is invalid */
				bool tx_complete = true;														/**< Indicates if a transmission has been completed */
				bool rx_complete = true;														/**< Indicates if a reception has been completed */
				bool RX_ASYNC = true;															/**< Enables/Disables asynchronous reception of data */
				Modes txMode = Modes::MODE_UNDEFINED;										/**< Logs which mode the TX peripheral is currently in */
				Modes rxMode = Modes::MODE_UNDEFINED;										/**< Logs which mode the RX peripheral is currently in */

				UARTPacket TX_tempPacket, RX_tempPacket;										/**< Used in ISR routines to prevent creation/deletion on the stack and help cleanup code a bit */
				boost::circular_buffer<UARTPacket> TXPacketBuffer, RXPacketBuffer;				/**< User level buffers for queuing data to transmit or holding data that was received */
				uint8_t RX_Queue[UART_QUEUE_SIZE][UART_QUEUE_BUFFER_SIZE];						/**< The raw data buffer that stores all receptions. RXPacketBuffer references this for the user. */
				uint8_t TX_Queue[UART_QUEUE_SIZE][UART_QUEUE_BUFFER_SIZE];						/**< The raw data buffer that stores all transmissions. TXPacketBuffer references this. */
				uint8_t RXQueueIdx = 0;															/**< Indicates which array in RX_Queue[x] is currently selected to hold the next RX data */
				uint8_t TXQueueIdx = 0;															/**< Indicates which array in TX_Queue[x] is currently selected to hold the next RX data */
				uint8_t asyncRXDataSize = 0;													/**< Temporarily holds how large (in bytes) an RX data reception is */
				uint32_t totalUnreadPackets = 0;													/**< Counter to inform the user how many unread packets are waiting */
				
				//boost::circular_buffer<uint8_t> runningTally;
				
				struct UARTClassStatus
				{
					bool gpio_enabled = false;
					bool uart_enabled = false;
					bool dma_enabled_tx = false;
					bool dma_enabled_rx = false;
					bool uart_interrupts_enabled = false;
					bool dma_interrupts_enabled_tx = false;
					bool dma_interrupts_enabled_rx = false;
				} UART_PeriphState;															/**< Flags that allow more precise configuring of the low level hardware during init/de-init */

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
				* Misc Functions 
				*------------------------------*/
				/** Copies data to the internal TX buffer. This allows for successive IT or DMA transmissions from the same external array
				 *	without having to worry about the user changing the data in runtime.
				 *	
				 *	@param[in] data		Address of the data to be copied 
				 *	@param[in] length	Length of data
				 *	@return TX_Queue address that the data was copied to 
				 **/
				uint8_t* assignTXBuffer(const uint8_t* data, const size_t length);
				
				uint8_t* txCurrentQueueAddr(){ return TX_Queue[TXQueueIdx]; }
				
				void txIncrQueueIdx()
				{
					TXQueueIdx++;
					
					if (TXQueueIdx == UART_QUEUE_SIZE)
						TXQueueIdx = 0;
				}

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
		
		
		/*-------------------------------
		* Utility functions
		*------------------------------*/
		extern void UART_EnableIT_IDLE(UART_HandleTypeDef *UartHandle);
		extern void UART_DisableIT_IDLE(UART_HandleTypeDef *UartHandle);
		extern void UART_ClearIT_IDLE(UART_HandleTypeDef *UartHandle);
	}
}
#endif /* !UART_H_ */
