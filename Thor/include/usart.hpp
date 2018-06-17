#pragma once
#ifndef USART_H_
#define USART_H_

/* C/C++ Includes */
#include <stdlib.h>
#include <stdint.h>

/* Boost Includes */
#include <boost/shared_ptr.hpp>
#include <boost/circular_buffer.hpp>

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
				Status readPacket(uint8_t* buff, size_t buff_length) override;

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

			private:

				int usartChannel;											/* Which peripheral hardware channel this class is mapped to (ie USART1, USART2, etc ...) */

				boost::circular_buffer<USARTPacket> TXPacketBuffer;			/* User level buffers for queuing data to transmit or holding data that was received */
				boost::circular_buffer<USARTPacket> RXPacketBuffer;				


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

			};
			typedef boost::shared_ptr<USARTClass> USARTClass_sPtr;
		}
	}
}

#endif // !USART_H_