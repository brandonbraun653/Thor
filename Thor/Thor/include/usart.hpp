#pragma once
#ifndef USART_H_
#define USART_H_

/* C/C++ Includes */
#include <stdlib.h>
#include <stdint.h>

/* Boost Includes */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/circular_buffer.hpp>
#pragma GCC diagnostic pop

/* Thor Includes */
#include <Thor/include/config.hpp>
#include <Thor/include/definitions.hpp>
#include <Thor/include/defaults.hpp>
#include <Thor/include/gpio.hpp>

/** @namespace Thor */
namespace Thor
{
	/** @namespace Thor::Peripheral */
	namespace Peripheral
	{
		/** @namespace Thor::Peripheral::USART */
		namespace USART
		{
			using namespace Thor::Peripheral;
			using namespace Thor::Definitions;

			class USARTClass : public Thor::Definitions::Serial::SerialBase
			{
			public:
				/** Initializes with a given baud rate and TX/RX modes.
				*	@param[in] baud		Desired baud rate. Accepts standard rates from Thor::Definitions::Serial::BaudRate
				*	@param[in] tx_mode	Sets the TX mode to Blocking, Interrupt, or DMA from Thor::Definitions::Serial::Modes
				*	@param[in] rx_mode	Sets the RX mode to Blocking, Interrupt, or DMA from Thor::Definitions::Serial::Modes
				*	@return	Status code indicating peripheral state. Will read 'PERIPH_OK' if everything is fine. Otherwise it
				*			will return a code from Thor::Peripheral::Serial::Status
				**/
				Status begin(const Thor::Definitions::Serial::BaudRate& baud = Thor::Definitions::Serial::BaudRate::SERIAL_BAUD_115200,
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

				/**
				*
				**/
				Status setBaud(const Thor::Definitions::Serial::BaudRate& baud) override;



				/** Writes data to the serial output gpio
				*	@param[in] val		Pointer to an array of data to be sent out
				*	@param[in] length	The length of data to be sent out
				*	@return	Status code indicating peripheral state. Will read 'PERIPH_OK' if everything is fine. Otherwise it
				*			will return a code from Thor::Peripheral::Serial::Status
				**/
				Status write(uint8_t* val, size_t length) override;

				/** Writes data to the serial output gpio
				*	@param[in] string	Pointer to a mutable character array
				*	@param[in] length	The length of data to be sent out
				*	@return	Status code indicating peripheral state. Will read 'PERIPH_OK' if everything is fine. Otherwise it
				*			will return a code from Thor::Peripheral::Serial::Status
				**/
				Status write(char* string, size_t length) override;

				/** Writes data to the serial output gpio
				*	@param[in] string	Pointer to a immutable character array. The length is internally calculated with strlen()
				*	@return	Status code indicating peripheral state. Will read 'PERIPH_OK' if everything is fine. Otherwise it
				*			will return a code from Thor::Peripheral::Serial::Status
				**/
				Status write(const char* string) override;

				/** Writes data to the serial output gpio
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
				*	@param[in]	buff_length	The size of the external buffer
				*	@return Status code indicating peripheral state. Will read 'UART_OK' if everything is fine. Otherwise it
				*			will return a code from Thor::Peripheral::UART::UART_Status
				**/
				Status readPacket(uint8_t* buff, size_t length) override;

				/** Returns how many unread received packets are available
				*	@return number of available packets
				**/
				uint32_t availablePackets() override;

				/** Gets the size of the next received packet in the buffer
				*	@return next packet size
				**/
				size_t nextPacketSize() override;

				/** Deinitializes and cleans up the peripheral */
				void end() override;

				/** Primary handler for interrupt mode in TX or RX.
				*	Additionally, will be called in DMA mode after USARTClass::IRQHandler_TXDMA or USARTClass::IRQHandler_RXDMA
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
				void attachThreadTrigger(Thor::Definitions::Interrupt::Trigger trig, SemaphoreHandle_t* semphr) override;

				/** Removes a trigger source
				 *	@param[in] trig	The source to be removed, of type Thor::Definitions::Interrupt::Trigger
				 **/
				void removeThreadTrigger(Thor::Definitions::Interrupt::Trigger trig) override;
				#endif

			private:
				USARTClass(const int& channel, Thor::Definitions::Serial::SerialPins* pinConfig);

			public:
				static boost::shared_ptr<USARTClass> create(const int channel, Thor::Definitions::Serial::SerialPins* pinConfig = nullptr);
				~USARTClass();

				/** Easily references buffered data for TX or RX */
				struct USARTPacket
				{
					uint8_t* data_ptr = nullptr;	/**< Contains the buffer address where data is stored */
					uint16_t bytesRead = 0;			/**< Number of bytes already read from the packet (currently used in eRPC calls) */
					size_t length = 0;				/**< Number of bytes contained in data_ptr */
				};

				/*-------------------------------
				 * ISR Stubs
				 *------------------------------*/
				int _getChannel() { return usartChannel; }

				void _setTxComplete() { tx_complete = true; }

				void _setRxComplete() { rx_complete = true; }

				bool _getRxComplete() { return rx_complete; }

				void _setRxAsync() { RX_ASYNC = true; }

				bool _getInitStatus() { return USARTPeriphState.usart_enabled; }

				Modes _getTxMode() { return txMode; }

				Modes _getRxMode() { return rxMode; }

				bool _txBufferEmpty() { return TXPacketBuffer.empty(); }

				void _txBufferRemoveFrontPacket() { TXPacketBuffer.pop_front(); }

				const USARTPacket& _txBufferNextPacket() { return TXPacketBuffer.front(); }

				void _rxBufferPushBack(const USARTPacket& newPkt) { RXPacketBuffer.push_back(newPkt); }

				uint8_t* _rxCurrentQueueAddr() { return RX_Queue[RXQueueIdx]; }

				void _rxIncrQueueIdx()
				{
					RXQueueIdx++;

					if (RXQueueIdx == Thor::Definitions::USART::USART_QUEUE_SIZE)
						RXQueueIdx = 0;
				}

			private:
                
    			friend void(::HAL_USART_TxCpltCallback)(USART_HandleTypeDef *UsartHandle);
    			friend void(::HAL_USART_RxCpltCallback)(USART_HandleTypeDef *UsartHandle);
                

				int usartChannel;											/* Which peripheral hardware channel this class is mapped to (ie USART1, USART2, etc ...) */
				bool tx_complete = true;									/**< Indicates if a transmission has been completed */
				bool rx_complete = true;									/**< Indicates if a reception has been completed */
				bool RX_ASYNC = true;										/**< Enables/Disables asynchronous reception of data */

				Modes txMode = Modes::MODE_UNDEFINED;						/**< Logs which mode the TX peripheral is currently in */
				Modes rxMode = Modes::MODE_UNDEFINED;						/**< Logs which mode the RX peripheral is currently in */

				boost::circular_buffer<USARTPacket> TXPacketBuffer;			/* User level buffers for queuing data to transmit or holding data that was received */
				boost::circular_buffer<USARTPacket> RXPacketBuffer;				

				USARTPacket TX_tempPacket, RX_tempPacket;										/**< Used in ISR routines to prevent creation/deletion on the stack and help cleanup code a bit */
				uint8_t RX_Queue[Thor::Definitions::USART::USART_QUEUE_SIZE][Thor::Definitions::USART::USART_QUEUE_BUFFER_SIZE];						/**< The raw data buffer that stores all receptions. RXPacketBuffer references this for the user. */
				uint8_t TX_Queue[Thor::Definitions::USART::USART_QUEUE_SIZE][Thor::Definitions::USART::USART_QUEUE_BUFFER_SIZE];						/**< The raw data buffer that stores all transmissions. TXPacketBuffer references this. */
				uint8_t RXQueueIdx = 0;															/**< Indicates which array in RX_Queue[x] is currently selected to hold the next RX data */
				uint8_t TXQueueIdx = 0;															/**< Indicates which array in TX_Queue[x] is currently selected to hold the next RX data */
				uint8_t asyncRXDataSize = 0;													/**< Temporarily holds how large (in bytes) an RX data reception is */
				uint32_t totalUnreadPackets = 0;													/**< Counter to inform the user how many unread packets are waiting */


				struct USARTClassStatus
				{
					bool gpio_enabled = false;
					bool usart_enabled = false;
					bool dma_enabled_tx = false;
					bool dma_enabled_rx = false;
					bool usart_interrupts_enabled = false;
					bool dma_interrupts_enabled_tx = false;
					bool dma_interrupts_enabled_rx = false;
				} USARTPeriphState;

				/*-------------------------------
				 * Object Pointers / Handles
				 *------------------------------*/
				USART_HandleTypeDef usartHandle;
				DMA_HandleTypeDef hdma_usart_tx;
				DMA_HandleTypeDef hdma_usart_rx;

				IT_Initializer ITSettings_HW;
				IT_Initializer ITSettings_DMA_TX;
				IT_Initializer ITSettings_DMA_RX;

				Thor::Peripheral::GPIO::GPIOClass_sPtr tx_pin;
				Thor::Peripheral::GPIO::GPIOClass_sPtr rx_pin;


				/*-------------------------------
				 * Misc Functions
				 *------------------------------*/
				uint8_t* assignTXBuffer(const uint8_t* data, const size_t length);

				uint8_t* txCurrentQueueAddr() 
				{ 
					return TX_Queue[TXQueueIdx]; 
				}

				void txIncrQueueIdx()
				{
					TXQueueIdx++;

					if (TXQueueIdx == Thor::Definitions::USART::USART_QUEUE_SIZE)
						TXQueueIdx = 0;
				}


				void USART_GPIO_Init();
				void USART_GPIO_DeInit();

				void USART_Init();
				void USART_DeInit();
				void USART_EnableClock();
				void USART_DisableClock();
				void USART_DMA_EnableClock();
				void USART_EnableInterrupts();
				void USART_DisableInterrupts();

				void USART_DMA_Init(const Thor::Definitions::SubPeripheral& periph);
				void USART_DMA_DeInit(const Thor::Definitions::SubPeripheral& periph);
				void USART_DMA_EnableIT(const Thor::Definitions::SubPeripheral& periph);
				void USART_DMA_DisableIT(const Thor::Definitions::SubPeripheral& periph);

				void USART_OverrunHandler();
			};
			typedef boost::shared_ptr<USARTClass> USARTClass_sPtr;
		}
	}
}

#endif // !USART_H_