#pragma once
#ifndef SERIAL_H_
#define SERIAL_H_

/* C/C++ Includes */
#include <stdlib.h>
#include <stdint.h>

/* Boost Includes */
#include <boost/shared_ptr.hpp>
#include <boost/move/unique_ptr.hpp>

/* Thor Includes */
#include <Thor/include/config.hpp>
#include <Thor/include/definitions.hpp>
#include <Thor/include/uart.hpp>
#include <Thor/include/usart.hpp>

/** @namespace Thor */
namespace Thor
{
	/** @namespace Thor::Peripheral */
	namespace Peripheral
	{
		/** @namespace Thor::Peripheral::Serial */
		namespace Serial
		{
			using namespace Thor::Definitions;
			using namespace Thor::Definitions::Serial;


			/** A high level, basic serial interface for rapid prototyping. This is essentially a wrapper over the existing 
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
					const Modes& tx_mode = Modes::BLOCKING,
					const Modes& rx_mode = Modes::BLOCKING);
				
				/** Places the specified peripheral into a given mode
				 *	@param[in] periph	Explicitly states which peripheral subsystem (TX or RX) to set from Thor::Peripheral::Serial::SubPeripheral
				 *	@param[in] mode		The corresponding mode for the peripheral to enter, from Thor::Peripheral::Serial::Modes
				 *	@return	Status code indicating peripheral state. Will read 'PERIPH_OK' if everything is fine. Otherwise it
				 *			will return a code from Thor::Peripheral::Serial::Status
				 *			
				 *	@note When setting the RX peripheral to IT or DMA mode, it automatically enables asynchronous data reception
				 **/
				Status setMode(const SubPeripheral& periph, const Modes& mode);

				/** Change the baudrate of the peripheral
				 *	@param[in] baud	Desired buadrate
				 *	@return SERIAL_OK if everything is alright, error code from Thor::Peripheral::Serial::Status if not
				 **/
				Status setBaud(const BaudRate& baud);

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
				
				/** Commands the RX peripheral to read a single transmission of known length into the provided buffer.
				 *	@param[out] buff	An external buffer to write the received data to
				 *	@param[in]  length	The number of bytes to be received
				 *
				 *	@note Only use this for receptions that have a fixed, known length. For transmissions that last longer than
				 *		  the given 'length' value, it will simply be ignored and lost forever. Poor data.
				 **/
				Status readSync(uint8_t* buff, size_t length);

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
				uint32_t availablePackets();

				/** Gets the size of the next received packet in the buffer
				*	@return next packet size
				**/
				size_t nextPacketSize();

				/** Deinitializes and cleans up the peripheral */
				void end();

				#if defined(USING_FREERTOS)
				/** Attaches a semaphore to a specific trigger source. When an event is triggered on that source,
				 *	the semaphore will be 'given' to and any task waiting on that semaphore will become unblocked.
				 *	@param[in] trig		The source to be triggered on, of type Thor::Definitions::Interrupt::Trigger
				 *	@param[in] semphr	The address of the semaphore that will be 'given' to upon triggering 
				 **/
				void attachThreadTrigger(Trigger trig, SemaphoreHandle_t* semphr);
				
				/** Removes a trigger source 
				 *	@param[in] trig	The source to be removed, of type Thor::Definitions::Interrupt::Trigger
				 **/
				void removeThreadTrigger(Trigger trig);
				#endif 
				
				/** Constructor used with Thor */
				SerialClass(const int& channel, SerialPins* pinConfig = nullptr);
				~SerialClass() = default;


				/* Specialized functions used only when interfacing with the Chimera HAL and are not intended be called by the user. */
				#if defined(USING_CHIMERA)
				Chimera::Serial::Status cbegin(uint32_t baud = Chimera::Serial::BaudRate::SERIAL_BAUD_115200,
												Chimera::Serial::Modes tx_mode = Chimera::Serial::Modes::BLOCKING,
												Chimera::Serial::Modes rx_mode = Chimera::Serial::Modes::BLOCKING);

				Chimera::Serial::Status csetMode(Chimera::Serial::SubPeripheral periph, Chimera::Serial::Modes mode);

				Chimera::Serial::Status csetBaud(uint32_t baud);

				Chimera::Serial::Status cwrite(uint8_t* val, size_t length);

				Chimera::Serial::Status cwrite(char* string, size_t length);

				Chimera::Serial::Status cwrite(const char* string);

				Chimera::Serial::Status cwrite(const char* string, size_t length);

				Chimera::Serial::Status creadPacket(uint8_t* buff, size_t buff_length);

				Chimera::Serial::Status convertStatus(Status status);
				#endif 

			private:			
				int serialChannel = 0;
				boost::shared_ptr<SerialBase> serialObject;	//Absolutely must be a shared_ptr. Multiple references stored elsewhere with static scope.
			};
			typedef boost::shared_ptr<SerialClass> SerialClass_sPtr;
			typedef boost::movelib::unique_ptr<SerialClass> SerialClass_uPtr;
		}
	}
}




#endif /* !SERIAL_H_ */