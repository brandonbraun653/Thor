#pragma once
#ifndef USART_H_
#define USART_H_


/************************************************************************/
/*							   Includes                                 */
/************************************************************************/
#include "thor_config.h"
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#ifdef TARGET_STM32F7
#include <stm32f7xx_hal.h>
#endif

#ifdef TARGET_STM32F4
#include <stm32f4xx_hal.h>
#endif
#include "thor_definitions.h"
#include "defaults.h"
#include "gpio.h"
#include "ringbuffer.h"

/************************************************************************/
/*								Defines                                 */
/************************************************************************/
#ifndef MAX_SERIAL_CHANNELS
#define MAX_SERIAL_CHANNELS 8
#endif // !MAX_SERIAL_CHANNELS

#define USART_BUFFER_SIZE 16

/************************************************************************/
/*								 Types                                  */
/************************************************************************/

/************************************************************************/
/*							Exported Variables                          */
/************************************************************************/

/************************************************************************/
/*							Exported Functions                          */
/************************************************************************/

/************************************************************************/
/*								Classes                                 */
/************************************************************************/
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
	USARTClass(uint32_t channel, GPIOClass *txPin, GPIOClass *rxPin);
	
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
	GPIOClass  *tx_pin;
	GPIOClass  *rx_pin;
	
	/*-------------------------------
	* Testing Functions: TEMPORARY
	*------------------------------*/
	void USART_Init();
	
	
};
#endif // !USART_H_