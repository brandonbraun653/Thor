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

			constexpr uint32_t NOALTERNATE = (0x08000CC8);	//Default value for the alternate configuration var

			struct PinConfig
			{
				PinPort	GPIOx		= GPIOA;
				PinSpeed speed		= PinSpeed::MEDIUM_SPD;
				PinMode	mode		= PinMode::INPUT;
                PinNum pinNum		= PinNum::NOT_A_PIN;
				PinPull pull		= PinPull::NOPULL;
				uint32_t alternate	= NOALTERNATE;
			};

            GPIO_InitTypeDef getHALInit(const PinConfig &config);




            class GPIOClass
			{
			public:
				void mode(PinMode Mode, PinPull Pull = PinPull::NOPULL);
				void write(const LogicLevel& state);
				void toggle();
				bool read();

				void reconfigure(PinPort GPIOx, PinNum PIN_x, PinSpeed SPEED = PinSpeed::HIGH_SPD, uint32_t ALTERNATE = NOALTERNATE);

				GPIOClass() = default;
				~GPIOClass() = default;
				GPIOClass(PinPort GPIOx, PinNum PIN_x, PinSpeed SPEED = PinSpeed::HIGH_SPD, uint32_t ALTERNATE = NOALTERNATE);


				#ifdef USING_CHIMERA
				Chimera::GPIO::Status cinit(Chimera::GPIO::Port port, uint8_t pin);
				Chimera::GPIO::Status cmode(Chimera::GPIO::Drive pin_mode, bool pullup);
				Chimera::GPIO::Status cwrite(Chimera::GPIO::State state);
				void ctoggle();
				bool cread();
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
			typedef boost::shared_ptr<Thor::Peripheral::GPIO::GPIOClass> GPIOClass_sPtr;
            typedef boost::movelib::unique_ptr<Thor::Peripheral::GPIO::GPIOClass> GPIOClass_uPtr;


            #if defined(USING_CHIMERA)
            class ChimeraGPIO
            {
            public:
                static PinNum convertPinNum(const uint8_t &num);
                static PinPort convertPort(const Chimera::GPIO::Port &port);
                static PinMode convertDrive(const Chimera::GPIO::Drive &drive);
                static PinPull convertPull(const Chimera::GPIO::Pull &pull);

                static PinConfig convertPinInit(const Chimera::GPIO::PinInit &pin);

            private:

            };
            #endif
		}
	}
}


#endif // !_GPIO_H_
