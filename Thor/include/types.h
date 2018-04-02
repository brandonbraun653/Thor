#ifndef TYPES_H_
#define TYPES_H_

#include "thor_config.h"

#ifdef TARGET_STM32F7
#include <stm32f7xx_hal.h>
#endif

#ifdef TARGET_STM32F4
#include <stm32f4xx_hal.h>
#endif

#include <stdlib.h>
#include <stdint.h>

#include <boost/function.hpp>

#include "thor_definitions.h"
#include "gpio.h"

typedef boost::function<void()> func_void;

typedef void(*func_ptr_void_t)();
typedef void(*func_ptr_uart_t)(UART_HandleTypeDef *UartHandle);


struct GPIO_Initializer
{
	GPIO_TypeDef*			GPIOx;
	GPIO_PinNum_TypeDef		PinNum;
	GPIO_Mode_TypeDef		Mode;
	GPIO_Speed_TypeDef		Speed;
	GPIO_Pull_TypeDef		Pull;
	uint8_t					Alternate;
};

struct SPI_Initializer
{

};

struct IT_Initializer
{
	IRQn_Type IRQn;
	uint32_t groupPriority;
	uint32_t preemptPriority;
	uint32_t subPriority;
};

struct DMA_Initializer
{
	DMA_Stream_TypeDef* Instance;
	uint32_t channel;
	uint32_t direction;
};

#endif // !TYPES_H_