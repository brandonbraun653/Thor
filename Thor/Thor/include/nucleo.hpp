#ifndef THOR_NUCLEO_HPP
#define THOR_NUCLEO_HPP

#include <Thor/include/thor.hpp>
#include <Thor/include/gpio.hpp>

/*
Blue Led: PB7
Red Led: PB14
Green Led: PB0
*/
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
namespace Thor
{
	namespace Nucleo
	{
		#if defined(STM32F7)
		static GPIO_TypeDef* RED_LED_PORT = GPIOB;
		static GPIO_TypeDef* GREEN_LED_PORT	= GPIOB;
		static GPIO_TypeDef* BLUE_LED_PORT = GPIOB;

        static const Thor::Definitions::GPIO::PinNum RED_LED_PIN = Thor::Definitions::GPIO::PinNum::PIN_14;
        static const Thor::Definitions::GPIO::PinNum GREEN_LED_PIN = Thor::Definitions::GPIO::PinNum::PIN_0;
        static const Thor::Definitions::GPIO::PinNum BLUE_LED_PIN = Thor::Definitions::GPIO::PinNum::PIN_7;
		#endif

		#if defined(STM32F4)
		static GPIO_TypeDef* GREEN_LED_PORT = GPIOA;
        static const Thor::Definitions::GPIO::PinNum GREEN_LED_PIN = Thor::Definitions::GPIO::PinNum::PIN_5;
		#endif
	}
}
#pragma GCC diagnostic pop

#endif
