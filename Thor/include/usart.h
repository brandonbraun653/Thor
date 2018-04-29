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

				Status write(uint8_t* val, size_t length);
				Status write(char* string, size_t length);
				Status write(const char* string);
				Status write(const char* string, size_t length);

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