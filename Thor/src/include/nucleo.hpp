#ifndef THOR_NUCLEO_HPP
#define THOR_NUCLEO_HPP

#include <Thor/include/thor.hpp>
#include <Thor/include/gpio.hpp>

/*
Blue Led: PB7
Red Led: PB14
Green Led: PB0
*/
namespace Thor
{
	namespace Nucleo
	{
		using PinNum = Thor::Peripheral::GPIO::PinNum;
		
		#if defined(STM32F7)
		static GPIO_TypeDef* RED_LED_PORT = GPIOB;
		static GPIO_TypeDef* GREEN_LED_PORT	= GPIOB;
		static GPIO_TypeDef* BLUE_LED_PORT = GPIOB;
		
		static const PinNum RED_LED_PIN = Thor::Peripheral::GPIO::PIN_14;
		static const PinNum GREEN_LED_PIN = Thor::Peripheral::GPIO::PIN_0;
		static const PinNum BLUE_LED_PIN = Thor::Peripheral::GPIO::PIN_7;
		#endif
		
		#if defined(STM32F4)
		static GPIO_TypeDef* GREEN_LED_PORT = GPIOA;
		static const PinNum GREEN_LED_PIN = Thor::Peripheral::GPIO::PIN_5;
		#endif
	}
}

#endif