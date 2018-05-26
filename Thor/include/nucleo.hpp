#ifndef THOR_NUCLEO_HPP
#define THOR_NUCLEO_HPP

#include <Thor/include/thor.hpp>
#include <Thor/include/gpio.hpp>

namespace Thor
{
	namespace Nucleo
	{
		using PinNum = Thor::Peripheral::GPIO::PinNum;
		
		#if defined(STM32F7)
		#define RED_LED_PORT	GPIOB
		#define GREEN_LED_PORT	GPIOA
		#define BLUE_LED_PORT	GPIOB
		#define GREEN_LED_PIN	5
		#endif
		
		#if defined(STM32F4)
		static GPIO_TypeDef* GREEN_LED_PORT = GPIOA;
		static const PinNum GREEN_LED_PIN = Thor::Peripheral::GPIO::PIN_5;
		#endif
	}
}

#endif