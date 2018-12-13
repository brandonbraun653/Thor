#ifndef TYPES_H_
#define TYPES_H_

/* C/C++ Includes */
#include <stdlib.h>
#include <stdint.h>

/* Boost Includes */
#include <boost/function.hpp>

/* Thor Includes */
#include <Thor/include/config.hpp>
#include <Thor/include/definitions.hpp>
#include <Thor/include/gpio.hpp>


typedef boost::function<void()> func_void;

typedef void(*func_ptr_void_t)();
typedef void(*func_ptr_uart_t)(UART_HandleTypeDef *UartHandle);


struct GPIO_Initializer
{
	GPIO_TypeDef*			GPIOx;
	Thor::Definitions::GPIO::PinNum		PinNum;
	Thor::Definitions::GPIO::PinMode		Mode;
	Thor::Definitions::GPIO::PinSpeed		Speed;
	Thor::Definitions::GPIO::PinPull		Pull;
	uint8_t					Alternate;
};

struct SPI_Initializer
{

};

struct IT_Initializer
{
	IRQn_Type IRQn;
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
