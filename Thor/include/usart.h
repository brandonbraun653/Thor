#pragma once
#ifndef USART_H_
#define USART_H_

/* C/C++ Includes */
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

/* Thor Includes */
#include <Thor/include/config.h>
#include <Thor/include/definitions.h>
#include <Thor/include/defaults.h>
#include <Thor/include/gpio.h>
#include <Thor/include/ringbuffer.h>


#ifndef MAX_SERIAL_CHANNELS
#define MAX_SERIAL_CHANNELS 8
#endif // !MAX_SERIAL_CHANNELS

#define USART_BUFFER_SIZE 16

class USARTClass : public RingBufferClass
{
public:
	
	void	begin();
	void	write(uint32_t val, uint32_t length);
	void	read(uint32_t length);
	void	flush();
	
	
	
	void IrqHandler(void);
	
	/*-------------------------------
	* Constructors
	*------------------------------*/
	USARTClass();
	USARTClass(uint32_t channel, Thor::Peripheral::GPIO::GPIOClass *txPin, Thor::Peripheral::GPIO::GPIOClass *rxPin);
	
private:
	int usart_channel;
	
	/*-------------------------------
	* Data Buffers
	*------------------------------*/
	int _txBuff[USART_BUFFER_SIZE];
	int _rxBuff[USART_BUFFER_SIZE];
	RingBufferClass tx_buffer = RingBufferClass(_txBuff, USART_BUFFER_SIZE);
	RingBufferClass rx_buffer = RingBufferClass(_rxBuff, USART_BUFFER_SIZE);
	
	/*-------------------------------
	* Object Pointers
	*------------------------------*/
	Thor::Peripheral::GPIO::GPIOClass  *tx_pin;
	Thor::Peripheral::GPIO::GPIOClass  *rx_pin;
	
	/*-------------------------------
	* Testing Functions: TEMPORARY
	*------------------------------*/
	void USART_Init();
	
	
};
#endif // !USART_H_