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

				Status write(uint8_t* val, size_t length);
				Status write(char* string, size_t length);
				Status write(const char* string);
				Status write(const char* string, size_t length);

					
					
				#ifdef USING_CHIMERA


				#endif 


				SerialClass(const int& channel);
				~SerialClass() = default;

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