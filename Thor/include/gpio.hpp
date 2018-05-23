#pragma once
#ifndef THOR_GPIO_H_
#define THOR_GPIO_H_

/* C/C++ Includes */
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

/* Boost Includes */
#include <boost/shared_ptr.hpp>
#include <boost/move/unique_ptr.hpp>

/* Thor Includes */
#include <Thor/include/config.hpp>
#include <Thor/include/definitions.hpp>
#include <Thor/include/utilities.hpp>


namespace Thor
{
	namespace Peripheral
	{
		namespace GPIO
		{
			using PinPort = GPIO_TypeDef*;
			using namespace Thor::Definitions::GPIO;
			
			const int NOALTERNATE = (0x08000CC8);	//Default value for the alternate configuration var

			enum PinNum : uint32_t
			{
				PIN_0   = GPIO_PIN_0,
				PIN_1   = GPIO_PIN_1,
				PIN_2   = GPIO_PIN_2,
				PIN_3   = GPIO_PIN_3,
				PIN_4   = GPIO_PIN_4,
				PIN_5   = GPIO_PIN_5,
				PIN_6   = GPIO_PIN_6,
				PIN_7   = GPIO_PIN_7,
				PIN_8   = GPIO_PIN_8,
				PIN_9   = GPIO_PIN_9,
				PIN_10  = GPIO_PIN_10,
				PIN_11  = GPIO_PIN_11,
				PIN_12  = GPIO_PIN_12,
				PIN_13  = GPIO_PIN_13,
				PIN_14  = GPIO_PIN_14,
				PIN_15  = GPIO_PIN_15,
				PIN_ALL = GPIO_PIN_All,
				MAX_PINS = 16
			};

			enum PinMode : uint32_t
			{
				INPUT              = GPIO_MODE_INPUT,
				OUTPUT_PP          = GPIO_MODE_OUTPUT_PP,
				OUTPUT_OD          = GPIO_MODE_OUTPUT_OD,
				ALT_PP             = GPIO_MODE_AF_PP,
				ALT_OD             = GPIO_MODE_AF_OD,
				ANALOG             = GPIO_MODE_ANALOG,
				IT_RISING          = GPIO_MODE_IT_RISING,
				IT_FALLING         = GPIO_MODE_IT_FALLING,
				IT_RISING_FALLING  = GPIO_MODE_IT_RISING_FALLING,
				EVT_RISING         = GPIO_MODE_EVT_RISING,
				EVT_FALLING        = GPIO_MODE_EVT_FALLING,
				EVT_RISING_FALLING = GPIO_MODE_EVT_RISING_FALLING
			};

			enum PinSpeed : uint32_t
			{
				LOW_SPD    = GPIO_SPEED_FREQ_LOW,
				MEDIUM_SPD = GPIO_SPEED_FREQ_MEDIUM,
				HIGH_SPD   = GPIO_SPEED_FREQ_HIGH,
				ULTRA_SPD  = GPIO_SPEED_FREQ_VERY_HIGH
			};

			enum PinPull : uint32_t
			{	
				NOPULL = GPIO_NOPULL,
				PULLUP = GPIO_PULLUP,
				PULLDN = GPIO_PULLDOWN
			};

			struct PinConfig
			{
				PinPort		GPIOx		= GPIOA;
				PinSpeed	speed		= HIGH_SPD;
				PinMode		mode		= INPUT;
				PinNum		pinNum		= PIN_0;
				PinPull     pull		= NOPULL;
				uint32_t	alternate	= NOALTERNATE;

				GPIO_InitTypeDef getHALInit()
				{
					GPIO_InitTypeDef InitStruct;

					InitStruct.Pin = pinNum;
					InitStruct.Speed = speed;
					InitStruct.Mode = mode;
					InitStruct.Pull = pull;
					InitStruct.Alternate = alternate;

					return InitStruct;
				}
			};

			class GPIOClass
			{
			public:
				void mode(PinMode Mode, PinPull Pull = NOPULL);
				void write(const LogicLevel& state);
				void toggle();
				bool read();

				void reconfigure(PinPort GPIOx, PinNum PIN_x, PinSpeed SPEED = HIGH_SPD, uint32_t ALTERNATE = NOALTERNATE);
	
				GPIOClass() = default;
				~GPIOClass() = default;
				GPIOClass(PinPort GPIOx, PinNum PIN_x, PinSpeed SPEED = HIGH_SPD, uint32_t ALTERNATE = NOALTERNATE);


				#ifdef USING_CHIMERA
				Chimera::GPIO::Status init(Chimera::GPIO::Port port, uint8_t pin);
				Chimera::GPIO::Status mode(Chimera::GPIO::Mode pin_mode, bool pullup);
				Chimera::GPIO::Status write(Chimera::GPIO::State state);
				#endif 
	
			private:
				PinConfig pinConfig;

				#ifdef USING_CHIMERA
				bool chimera_settings_recorded = false;
				#endif

				void GPIO_Init(PinPort GPIOx, GPIO_InitTypeDef *InitStruct);
				void GPIO_ClockEnable(PinPort GPIOx);
				void GPIO_ClockDisable(PinPort GPIOx);
			};
			typedef boost::shared_ptr<GPIOClass> GPIOClass_sPtr;
			typedef boost::movelib::unique_ptr<GPIOClass> GPIOClass_uPtr;
		}
	}
}


#endif // !_GPIO_H_
