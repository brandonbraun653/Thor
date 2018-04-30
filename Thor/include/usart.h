#pragma once
#ifndef USART_H_
#define USART_H_

/* C/C++ Includes */
#include <stdlib.h>
#include <stdint.h>

/* Boost Includes */
#include <boost/shared_ptr.hpp>

/* Thor Includes */
#include <Thor/include/config.h>
#include <Thor/include/definitions.h>
#include <Thor/include/defaults.h>
#include <Thor/include/gpio.h>
#include <Thor/include/ringbuffer.h>

/** @namespace Thor */
namespace Thor
{
	/** @namespace Thor::Peripheral */
	namespace Peripheral
	{
		/** @namespace Thor::Peripheral::USART */
		namespace USART
		{
			using namespace Thor::Definitions::Serial;
			
			
			class USARTClass
			{
			public:
				/** Initializes with default parameters.
				*	Baudrate is set to 115200 and both TX and RX modes are set to blocking.
				*
				*	@return Status code indicating peripheral state. Will read 'PERIPH_OK' if everything is fine. Otherwise it
				*			will return a code from Thor::Peripheral::Serial::Status
				**/
				Status begin();

				/** Initializes with a given baud rate.
				*	Both TX and RX modes are set to blocking.
				*
				*  @param[in] baud Desired baud rate. Accepts standard rates from Thor::Definitions::Serial::Modes
				*  @return Status code indicating peripheral state. Will read 'PERIPH_OK' if everything is fine. Otherwise it
				*			will return a code from Thor::Peripheral::Serial::Status
				**/
				Status begin(const BaudRate& baud);

				/** Initializes with a given baud rate and TX/RX modes.
				*	@param[in] baud		Desired baud rate. Accepts standard rates from Thor::Definitions::Serial::BaudRate
				*	@param[in] tx_mode	Sets the TX mode to Blocking, Interrupt, or DMA from Thor::Definitions::Serial::Modes
				*	@param[in] rx_mode	Sets the RX mode to Blocking, Interrupt, or DMA from Thor::Definitions::Serial::Modes
				*	@return	Status code indicating peripheral state. Will read 'PERIPH_OK' if everything is fine. Otherwise it
				*			will return a code from Thor::Peripheral::Serial::Status
				**/
				Status begin(const BaudRate& baud, const Modes& tx_mode, const Modes& rx_mode);

				/** Writes data to the serial output gpio
				*	@param[in] val		Pointer to an array of data to be sent out
				*	@param[in] length	The length of data to be sent out
				*	@return	Status code indicating peripheral state. Will read 'PERIPH_OK' if everything is fine. Otherwise it
				*			will return a code from Thor::Peripheral::Serial::Status
				**/
				Status write(uint8_t* val, size_t length);
				
				/** Writes data to the serial output gpio
				*	@param[in] string	Pointer to a mutable character array
				*	@param[in] length	The length of data to be sent out
				*	@return	Status code indicating peripheral state. Will read 'PERIPH_OK' if everything is fine. Otherwise it
				*			will return a code from Thor::Peripheral::Serial::Status
				**/
				Status write(char* string, size_t length);

				/** Writes data to the serial output gpio
				*	@param[in] string	Pointer to a immutable character array. The length is internally calculated with strlen()
				*	@return	Status code indicating peripheral state. Will read 'PERIPH_OK' if everything is fine. Otherwise it
				*			will return a code from Thor::Peripheral::Serial::Status
				**/
				Status write(const char* string);

				/** Writes data to the serial output gpio
				*	@param[in] string	Pointer to an immutable character array
				*	@param[in] length	The length of data to be sent out
				*	@return	Status code indicating peripheral state. Will read 'PERIPH_OK' if everything is fine. Otherwise it
				*			will return a code from Thor::Peripheral::Serial::Status
				**/
				Status write(const char* string, size_t length);

				/** Reads the next packet received into a buffer
				*	@param[out] buff		Address of an external buffer to read data into
				*	@param[in]	buff_length	The size of the external buffer
				*	@return Status code indicating peripheral state. Will read 'UART_OK' if everything is fine. Otherwise it
				*			will return a code from Thor::Peripheral::UART::UART_Status
				**/
				Status readPacket(uint8_t* buff, size_t buff_length);

				/** Returns how many unread received packets are available
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

				/** Sets the specified peripheral to blocking mode. It also takes into account any settings changes that might
				*	be necessary.
				*	@param[in] periph Explicitly states which peripheral subsystem (RX or TX) to set from Thor::Peripheral::UART::UARTPeriph
				**/
				void setBlockMode(const SubPeripheral& periph);

				/** Sets the specified peripheral to interrupt mode. It also takes into account any settings changes that might
				*	be necessary.
				*	@param[in] periph Explicitly states which peripheral subsystem (RX or TX) to set from Thor::Peripheral::UART::UARTPeriph
				**/
				void setITMode(const SubPeripheral& periph);

				/** Sets the specified peripheral to dma mode. It also takes into account any settings changes that might
				*	be necessary.
				*	@param[in] periph Explicitly states which peripheral subsystem (RX or TX) to set from Thor::Peripheral::UART::UARTPeriph
				**/
				void setDMAMode(const SubPeripheral& periph);

			private:
				USARTClass(const int& channel);

			public:
				static boost::shared_ptr<USARTClass> create(const int channel);
				~USARTClass();
				
			private:
				
			};
			
			typedef boost::shared_ptr<USARTClass> USARTClass_sPtr;
		}
	}
}





#endif // !USART_H_