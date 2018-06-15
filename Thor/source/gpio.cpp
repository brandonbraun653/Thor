/* Boost Includes */
#include <boost/container/flat_map.hpp>

/* Project Includes */
#include <Thor/include/gpio.hpp>

namespace Thor
{
	namespace Peripheral
	{
		namespace GPIO
		{
			using namespace Thor::Definitions::GPIO;
			
			boost::container::flat_map<GPIO_TypeDef*, uint32_t> rcc_gpio_mask =
			{ 
				//STM32F7 PDF: RM0410 pg.181
				//STM32F4 PDF: RM0390 pg.143
				#if defined(TARGET_STM32F7) || defined(TARGET_STM32F4)
				{GPIOA, (1u << 0)}, 
				{GPIOB, (1u << 1)}, 
				{GPIOC, (1u << 2)}, 
				{GPIOD, (1u << 3)},
				{GPIOE, (1u << 4)}, 
				{GPIOF, (1u << 5)}, 
				{GPIOG, (1u << 6)}, 
				{GPIOH, (1u << 7)},
				#endif								#if defined(TARGET_STM32F7)				{GPIOI, (1u << 8)}, 				{GPIOJ, (1u << 9)}, 				{GPIOK, (1u << 10)}				#endif
			};

			GPIOClass::GPIOClass(PinPort GPIOx, PinNum PIN_x, PinSpeed SPEED, uint32_t ALTERNATE)
			{
				pinConfig.GPIOx		= GPIOx;
				pinConfig.pinNum	= PIN_x;
				pinConfig.speed		= SPEED;
				pinConfig.alternate = ALTERNATE;
			}

			void GPIOClass::mode(PinMode Mode, PinPull Pull)
			{
				pinConfig.mode = Mode;
				pinConfig.pull = Pull;
				
				auto cfg = pinConfig.getHALInit();
				GPIO_Init(pinConfig.GPIOx, &cfg);
			}

			void GPIOClass::write(const LogicLevel& state)
			{
				HAL_GPIO_WritePin(pinConfig.GPIOx, pinConfig.pinNum, static_cast<GPIO_PinState>(state));
			}

			bool GPIOClass::read()
			{
				return HAL_GPIO_ReadPin(pinConfig.GPIOx, pinConfig.pinNum);
			}

			void GPIOClass::toggle()
			{
				HAL_GPIO_TogglePin(pinConfig.GPIOx, pinConfig.pinNum);
			}

			void GPIOClass::reconfigure(PinPort GPIOx, PinNum PIN_x, PinSpeed SPEED, uint32_t ALTERNATE)
			{
				/* Update local copy of config  */
				pinConfig.GPIOx		= GPIOx;
				pinConfig.pinNum	= PIN_x;
				pinConfig.speed		= SPEED;
				pinConfig.alternate = ALTERNATE;

				auto cfg = pinConfig.getHALInit();
				GPIO_Init(pinConfig.GPIOx, &cfg);
			}
			
			void GPIOClass::GPIO_Init(PinPort GPIOx, GPIO_InitTypeDef *InitStruct)
			{	
				GPIO_ClockEnable(GPIOx);
				HAL_GPIO_Init(GPIOx, InitStruct);
			}

			void GPIOClass::GPIO_ClockEnable(PinPort GPIOx)
			{
				#if defined(TARGET_STM32F7) || defined(TARGET_STM32F4)
				SET_BIT(RCC->AHB1ENR, rcc_gpio_mask[GPIOx]);
				#endif 
			}
			
			void GPIOClass::GPIO_ClockDisable(PinPort GPIOx)
			{
				#if defined(TARGET_STM32F7) || defined(TARGET_STM32F4)
				CLEAR_BIT(RCC->AHB1ENR, rcc_gpio_mask[GPIOx]);
				#endif 
			}
			

			#ifdef USING_CHIMERA
			boost::container::flat_map<Chimera::GPIO::Port, GPIO_TypeDef*> portMap = 
			{
				#if defined(TARGET_STM32F7) || defined(TARGET_STM32F4)
				{ Chimera::GPIO::Port::PORTA, GPIOA },
				{ Chimera::GPIO::Port::PORTB, GPIOB },
				{ Chimera::GPIO::Port::PORTC, GPIOC },
				{ Chimera::GPIO::Port::PORTD, GPIOD },
				{ Chimera::GPIO::Port::PORTE, GPIOE },
				{ Chimera::GPIO::Port::PORTF, GPIOF },
				{ Chimera::GPIO::Port::PORTG, GPIOG },
				{ Chimera::GPIO::Port::PORTH, GPIOH },
				#endif

				#if defined(TARGET_STM32F7)				{ Chimera::GPIO::Port::PORTI, GPIOI },				{ Chimera::GPIO::Port::PORTJ, GPIOJ },				{ Chimera::GPIO::Port::PORTK, GPIOK }				#endif
			};

			boost::container::flat_map<uint8_t, Thor::Peripheral::GPIO::PinNum> pinMap =
			{
				{ 0, PIN_0 },
				{ 1, PIN_1 },
				{ 2, PIN_2 },
				{ 3, PIN_3 },
				{ 4, PIN_4 },
				{ 5, PIN_5 },
				{ 6, PIN_6 },
				{ 7, PIN_7 },
				{ 8, PIN_8 },
				{ 9, PIN_9 },
				{ 10, PIN_10 },
				{ 11, PIN_11 },
				{ 12, PIN_12 },
				{ 13, PIN_13 },
				{ 14, PIN_14 },
				{ 15, PIN_15 }
			};

			boost::container::flat_map<Chimera::GPIO::Mode, Thor::Peripheral::GPIO::PinMode> modeMap =
			{
				{ Chimera::GPIO::Mode::INPUT,					INPUT },
				{ Chimera::GPIO::Mode::OUTPUT_PUSH_PULL,		OUTPUT_PP },
				{ Chimera::GPIO::Mode::OUTPUT_OPEN_DRAIN,		OUTPUT_OD },
				{ Chimera::GPIO::Mode::ALTERNATE_PUSH_PULL,		ALT_PP },
				{ Chimera::GPIO::Mode::ALTERNATE_OPEN_DRAIN,	ALT_OD },
				{ Chimera::GPIO::Mode::ANALOG,					ANALOG }
			};

			Chimera::GPIO::Status GPIOClass::cinit(Chimera::GPIO::Port port, uint8_t pin)
			{
				pinConfig.pinNum = pinMap[pin];
				if (pinConfig.pinNum == (PinNum)0)
					return Chimera::GPIO::Status::GPIO_ERROR_INVALID_PIN;

				pinConfig.GPIOx = portMap[port];
				if (pinConfig.GPIOx == (GPIO_TypeDef*)0)
					return Chimera::GPIO::Status::GPIO_ERROR_INVALID_PORT;


				chimera_settings_recorded = true;
				return Chimera::GPIO::Status::GPIO_OK;
			}

			Chimera::GPIO::Status GPIOClass::cmode(Chimera::GPIO::Mode mode, bool pullup = false)
			{
				if (chimera_settings_recorded)
				{
					//Note: PinMode::INPUT and output of modeMap are the same when "mode" is invalid (0u). No good way to check for invalid input.
					PinMode pinMode = modeMap[mode];
					PinPull pinPull = (pullup) ? PULLUP : NOPULL;

					this->mode(pinMode, pinPull);
					return Chimera::GPIO::Status::GPIO_OK;
				}
				else
					return Chimera::GPIO::Status::GPIO_ERROR_UNINITIALIZED;
			}

			Chimera::GPIO::Status GPIOClass::cwrite(Chimera::GPIO::State state)
			{
				if (chimera_settings_recorded)
				{
					HAL_GPIO_WritePin(pinConfig.GPIOx, pinConfig.pinNum, static_cast<GPIO_PinState>(state));
					return Chimera::GPIO::Status::GPIO_OK;
				}
				else
					return Chimera::GPIO::Status::GPIO_ERROR_UNINITIALIZED;
			}

			void GPIOClass::ctoggle()
			{
				this->toggle();
			}

			bool GPIOClass::cread()
			{
				return this->read();
			}
			#endif
		}
	}
}
	
