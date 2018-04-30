#pragma once
#ifndef SERIAL_H_
#define SERIAL_H_

/* C/C++ Includes */
#include <stdlib.h>
#include <stdint.h>

/* Boost Includes */
#include <boost/shared_ptr.hpp>

/* Thor Includes */
#include <Thor/include/config.h>
#include <Thor/include/definitions.h>
#include <Thor/include/uart.h>
#include <Thor/include/usart.h>

/** @namespace Thor */
namespace Thor
{
	/** @namespace Thor::Peripheral */
	namespace Peripheral
	{
		/** @namespace Thor::Peripheral::Serial */
		namespace Serial
		{
			using namespace Thor::Definitions::Serial;


			/** A high level basic serial interface for rapid prototyping. This is essentially a wrapper over the existing 
			 *	Thor::Peripheral::UART::UARTClass and Thor::Peripheral::USART::USARTClass interfaces and implements much of
			 *	the same features found there. The goal of this class is to abstract away from the specific UART and USART 
			 *	peripherals so that the user only has to call 'SerialClass(1)' or 'SerialClass(x)' to generate a serial object 
			 *	quickly without having to worry about the underlying hardware type.
			 *
			 *  @note Just like the USART and UART classes, this class is thread safe when using FreeRTOS.
			 **/
			class SerialClass
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
				*		   will return a code from Thor::Peripheral::Serial::Status
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

				/** Gives how many unread packets are available in the receive buffer
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

				
				SerialClass(const int& channel);
				~SerialClass() = default;


				/* Specialized functions used only when interfacing with the Chimera HAL and are not intended be called by the user. */
				#if defined(USING_CHIMERA)
				Chimera::Serial::Status begin(Chimera::Serial::BaudRate baud = Chimera::Serial::BaudRate::SERIAL_BAUD_115200,
												Chimera::Serial::Modes tx_mode = Chimera::Serial::Modes::TX_MODE_BLOCKING,
												Chimera::Serial::Modes rx_mode = Chimera::Serial::Modes::RX_MODE_BLOCKING);

				Chimera::Serial::Status setMode(Chimera::Serial::SubPeripheral sp, Chimera::Serial::Modes mode);

				Chimera::Serial::Status cwrite(uint8_t* val, size_t length);

				Chimera::Serial::Status cwrite(char* string, size_t length);

				Chimera::Serial::Status cwrite(const char* string);

				Chimera::Serial::Status cwrite(const char* string, size_t length);

				Chimera::Serial::Status creadPacket(uint8_t* buff, size_t buff_length);

				Chimera::Serial::Status convertStatus(Thor::Definitions::Serial::Status status);
				#endif 

			private:
				int serial_channel = 0;
				Thor::Peripheral::UART::UARTClass_sPtr uart;
				Thor::Peripheral::USART::USARTClass_sPtr usart;
			};
			
			typedef boost::shared_ptr<SerialClass> Serial_sPtr;
		}
	}
}




#endif /* !SERIAL_H_ */