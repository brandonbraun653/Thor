/* Boost Includes */
#include <boost/container/flat_map.hpp>

/* Project Includes */
#include <Thor/include/gpio.h>

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
			Chimera::GPIO::Status GPIOClass::init(Chimera::GPIO::Port port, uint8_t pin)
			{
				/* Validate the input pin */
				if (!(pin < MAX_PINS))
					return Chimera::GPIO::Status::GPIO_ERROR_INVALID_PIN;
				else
					pinConfig.pinNum = static_cast<PinNum>(1u << pin);

				/* Grab the port from the rcc_gpio_mask map by a reverse lookup */
				PinPort portKey;
				uint32_t mask = (1u << port);

				if (!Thor::Util::findKeyFromVal(portKey, rcc_gpio_mask, mask))
					return Chimera::GPIO::Status::GPIO_ERROR_INVALID_PORT;
				else
					pinConfig.GPIOx = portKey;

				chimera_settings_recorded = true;
				return Chimera::GPIO::Status::GPIO_OK;
			}

			Chimera::GPIO::Status GPIOClass::mode(Chimera::GPIO::Mode pin_mode, bool pullup = false)
			{
				if (chimera_settings_recorded)
				{
					PinMode _mode;
					PinPull _pull;

					
					switch (pin_mode)
					{
					case Chimera::GPIO::Mode::OUTPUT:
						_mode = OUTPUT_PP;
						break;
					
					case Chimera::GPIO::Mode::INPUT:
						_mode = INPUT;
						break;

					//TODO: Going to need to do alt_pp at some point

					default:
						_mode = INPUT;
						break;
					}

					_pull = (pullup) ? PULLUP : NOPULL;

					this->mode(_mode, _pull);
				}
				else
					return Chimera::GPIO::Status::GPIO_ERROR_UNINITIALIZED;
			}

			Chimera::GPIO::Status GPIOClass::write(Chimera::GPIO::State state)
			{
				if (chimera_settings_recorded)
				{
					HAL_GPIO_WritePin(pinConfig.GPIOx, pinConfig.pinNum, static_cast<GPIO_PinState>(state));
					return Chimera::GPIO::Status::GPIO_OK;
				}
				else
					return Chimera::GPIO::Status::GPIO_ERROR_UNINITIALIZED;
			}
			#endif
		}
	}
}
	
