/* Project Includes */
#include <Thor/include/gpio.hpp>

namespace Thor
{
	namespace Peripheral
	{
		namespace GPIO
		{
			using namespace Thor::Definitions::GPIO;

			static uint32_t getRCCGPIOMask(GPIO_TypeDef* instance)
			{
				auto i = reinterpret_cast<std::uintptr_t>(instance);
				switch (i)
				{
				//STM32F7 PDF: RM0410 pg.181
				//STM32F4 PDF: RM0390 pg.143
				#if defined(TARGET_STM32F7) || defined(TARGET_STM32F4)
				case GPIOA_BASE:
					return (1u << 0);
					break;

				case GPIOB_BASE:
					return (1u << 1);
					break;

				case GPIOC_BASE:
					return (1u << 2);
					break;

				case GPIOD_BASE:
					return (1u << 3);
					break;

				case GPIOE_BASE:
					return (1u << 4);
					break;

				case GPIOF_BASE:
					return (1u << 5);
					break;

				case GPIOG_BASE:
					return (1u << 6);
					break;

				case GPIOH_BASE:
					return (1u << 7);
					break;
				#endif

				#if defined(TARGET_STM32F7)
				case GPIOI_BASE:
					return (1u << 8);
					break;

				case GPIOJ_BASE:
					return (1u << 9);
					break;

				case GPIOK_BASE:
					return (1u << 10);
					break;
				#endif

				default:
					return 0u;
					break;
				};
			}

            GPIO_InitTypeDef getHALInit(const PinConfig &config)
			{
				GPIO_InitTypeDef InitStruct;

				InitStruct.Pin = static_cast<uint32_t>(config.pinNum);
				InitStruct.Speed = static_cast<uint32_t>(config.speed);
				InitStruct.Mode = static_cast<uint32_t>(config.mode);
				InitStruct.Pull = static_cast<uint32_t>(config.pull);
				InitStruct.Alternate = config.alternate;

				return InitStruct;
			}

			GPIOClass::GPIOClass(PinPort GPIOx, PinNum PIN_x, PinSpeed SPEED, uint32_t ALTERNATE)
			{
				pinConfig.GPIOx = GPIOx;
				pinConfig.pinNum = PIN_x;
				pinConfig.speed = SPEED;
				pinConfig.alternate = ALTERNATE;
			}

			void GPIOClass::mode(PinMode Mode, PinPull Pull)
			{
				pinConfig.mode = Mode;
				pinConfig.pull = Pull;

				auto cfg = getHALInit(pinConfig);
				GPIO_Init(pinConfig.GPIOx, &cfg);
			}

			void GPIOClass::write(const LogicLevel& state)
			{
				HAL_GPIO_WritePin(pinConfig.GPIOx, static_cast<uint16_t>(pinConfig.pinNum), static_cast<GPIO_PinState>(state));
			}

			bool GPIOClass::read()
			{
				return HAL_GPIO_ReadPin(pinConfig.GPIOx, static_cast<uint16_t>(pinConfig.pinNum));
			}

			void GPIOClass::toggle()
			{
				HAL_GPIO_TogglePin(pinConfig.GPIOx, static_cast<uint16_t>(pinConfig.pinNum));
			}

			void GPIOClass::reconfigure(PinPort GPIOx, PinNum PIN_x, PinSpeed SPEED, uint32_t ALTERNATE)
			{
				/* Update local copy of config  */
				pinConfig.GPIOx		= GPIOx;
				pinConfig.pinNum	= PIN_x;
				pinConfig.speed		= SPEED;
				pinConfig.alternate = ALTERNATE;

                auto cfg = getHALInit(pinConfig);
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
				SET_BIT(RCC->AHB1ENR, getRCCGPIOMask(GPIOx));
				#endif
			}

			void GPIOClass::GPIO_ClockDisable(PinPort GPIOx)
			{
				#if defined(TARGET_STM32F7) || defined(TARGET_STM32F4)
				CLEAR_BIT(RCC->AHB1ENR, getRCCGPIOMask(GPIOx));
				#endif
			}


			#ifdef USING_CHIMERA
			static GPIO_TypeDef* portMap(Chimera::GPIO::Port port)
			{
				switch (port)
				{
				#if defined(TARGET_STM32F7) || defined(TARGET_STM32F4)
				case Chimera::GPIO::Port::PORTA:
					return GPIOA;
					break;

				case Chimera::GPIO::Port::PORTB:
					return GPIOB;
					break;

				case Chimera::GPIO::Port::PORTC:
					return GPIOC;
					break;

				case Chimera::GPIO::Port::PORTD:
					return GPIOD;
					break;

				case Chimera::GPIO::Port::PORTE:
					return GPIOE;
					break;

				case Chimera::GPIO::Port::PORTF:
					return GPIOF;
					break;

				case Chimera::GPIO::Port::PORTG:
					return GPIOG;
					break;

				case Chimera::GPIO::Port::PORTH:
					return GPIOH;
					break;
				#endif

				#if defined(TARGET_STM32F7)
				case Chimera::GPIO::Port::PORTI:
					return GPIOI;
					break;

				case Chimera::GPIO::Port::PORTJ:
					return GPIOJ;
					break;

				case Chimera::GPIO::Port::PORTK:
					return GPIOK;
					break;
				#endif

				/* If we get here, something is wrong */
				default:
					return GPIOA;
					break;
				};
			}


			static PinMode modeMap(Chimera::GPIO::Drive mode)
			{
				switch (mode)
				{
				case Chimera::GPIO::Drive::INPUT:
					return Thor::Peripheral::GPIO::PinMode::INPUT;
					break;
				case Chimera::GPIO::Drive::OUTPUT_PUSH_PULL:
					return Thor::Peripheral::GPIO::PinMode::OUTPUT_PP;
					break;
				case Chimera::GPIO::Drive::OUTPUT_OPEN_DRAIN:
					return Thor::Peripheral::GPIO::PinMode::OUTPUT_OD;
					break;
				case Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL:
					return Thor::Peripheral::GPIO::PinMode::ALT_PP;
					break;
				case Chimera::GPIO::Drive::ALTERNATE_OPEN_DRAIN:
					return Thor::Peripheral::GPIO::PinMode::ALT_OD;
					break;
				case Chimera::GPIO::Drive::ANALOG:
					return Thor::Peripheral::GPIO::PinMode::ANALOG;
					break;

				default:
					return Thor::Peripheral::GPIO::PinMode::INPUT;
					break;
				};
			}

			Chimera::GPIO::Status GPIOClass::cinit(Chimera::GPIO::Port port, uint8_t pin)
			{
				pinConfig.pinNum = ChimeraGPIO::convertPinNum(pin);
				if (pinConfig.pinNum == (PinNum)0)
					return Chimera::GPIO::Status::ERROR_INVALID_PIN;

				pinConfig.GPIOx = ChimeraGPIO::convertPort(port);
				if (pinConfig.GPIOx == (GPIO_TypeDef*)0)
					return Chimera::GPIO::Status::ERROR_INVALID_PORT;


				chimera_settings_recorded = true;
				return Chimera::GPIO::Status::OK;
			}

			Chimera::GPIO::Status GPIOClass::cmode(Chimera::GPIO::Drive mode, bool pullup = false)
			{
				if (chimera_settings_recorded)
				{
					//Note: PinMode::INPUT and output of modeMap are the same when "mode" is invalid (0u). No good way to check for invalid input.
					PinMode pinMode = modeMap(mode);
                    PinPull pinPull = (pullup) ? PinPull::PULLUP : PinPull::NOPULL;

					this->mode(pinMode, pinPull);
					return Chimera::GPIO::Status::OK;
				}
				else
					return Chimera::GPIO::Status::ERROR_UNINITIALIZED;
			}

			Chimera::GPIO::Status GPIOClass::cwrite(Chimera::GPIO::State state)
			{
				if (chimera_settings_recorded)
				{
					HAL_GPIO_WritePin(pinConfig.GPIOx, static_cast<uint16_t>(pinConfig.pinNum), static_cast<GPIO_PinState>(state));
					return Chimera::GPIO::Status::OK;
				}
				else
					return Chimera::GPIO::Status::ERROR_UNINITIALIZED;
			}

			void GPIOClass::ctoggle()
			{
				this->toggle();
			}

			bool GPIOClass::cread()
			{
				return this->read();
			}

            PinConfig ChimeraGPIO::convertPinInit(const Chimera::GPIO::PinInit &pin)
            {
                PinConfig cfg;

                cfg.GPIOx = convertPort(pin.port);
                cfg.mode = convertDrive(pin.mode);
                cfg.pinNum = convertPinNum(pin.number);
                cfg.pull = convertPull(pin.pull);

                return cfg;
            }

            PinNum ChimeraGPIO::convertPinNum(const uint8_t &num)
            {
                PinNum pinNum = PinNum::NOT_A_PIN;

                switch (num)
				{
					case 0:
						pinNum = Thor::Peripheral::GPIO::PinNum::PIN_0;
						break;
					case 1:
						pinNum = Thor::Peripheral::GPIO::PinNum::PIN_1;
						break;
					case 2:
						pinNum = Thor::Peripheral::GPIO::PinNum::PIN_2;
						break;
					case 3:
						pinNum = Thor::Peripheral::GPIO::PinNum::PIN_3;
						break;
					case 4:
						pinNum = Thor::Peripheral::GPIO::PinNum::PIN_4;
						break;
					case 5:
						pinNum = Thor::Peripheral::GPIO::PinNum::PIN_5;
						break;
					case 6:
						pinNum = Thor::Peripheral::GPIO::PinNum::PIN_6;
						break;
					case 7:
						pinNum = Thor::Peripheral::GPIO::PinNum::PIN_7;
						break;
					case 8:
						pinNum = Thor::Peripheral::GPIO::PinNum::PIN_8;
						break;
					case 9:
						pinNum = Thor::Peripheral::GPIO::PinNum::PIN_9;
						break;
					case 10:
						pinNum = Thor::Peripheral::GPIO::PinNum::PIN_10;
						break;
					case 11:
						pinNum = Thor::Peripheral::GPIO::PinNum::PIN_11;
						break;
					case 12:
						pinNum = Thor::Peripheral::GPIO::PinNum::PIN_12;
						break;
					case 13:
						pinNum = Thor::Peripheral::GPIO::PinNum::PIN_13;
						break;
					case 14:
						pinNum = Thor::Peripheral::GPIO::PinNum::PIN_14;
						break;
					case 15:
						pinNum = Thor::Peripheral::GPIO::PinNum::PIN_15;
						break;

					default:
                        pinNum = Thor::Peripheral::GPIO::PinNum::NOT_A_PIN;
                        break;
                };

                return pinNum;
            }

            PinPort ChimeraGPIO::convertPort(const Chimera::GPIO::Port &port)
            {
                PinPort pinPort = nullptr;

                switch (port)
                {
                #if defined(STM32F446xx) || defined(STM32F767xx)
                    case Chimera::GPIO::Port::PORTA:
                        pinPort = GPIOA;
                        break;

                    case Chimera::GPIO::Port::PORTB:
                        pinPort = GPIOB;
                        break;

                    case Chimera::GPIO::Port::PORTC:
                        pinPort = GPIOC;
                        break;

                    case Chimera::GPIO::Port::PORTD:
                        pinPort = GPIOD;
                        break;

                    case Chimera::GPIO::Port::PORTE:
                        pinPort = GPIOE;
                        break;

                    case Chimera::GPIO::Port::PORTF:
                        pinPort = GPIOF;
                        break;

                    case Chimera::GPIO::Port::PORTG:
                        pinPort = GPIOG;
                        break;

                    case Chimera::GPIO::Port::PORTH:
                        pinPort = GPIOH;
                        break;
                #endif

                #if defined(STM32F767xx)
                    case Chimera::GPIO::Port::PORTI:
                        pinPort = GPIOI;
                        break;

                    case Chimera::GPIO::Port::PORTJ:
                        pinPort = GPIOJ;
                        break;

                    case Chimera::GPIO::Port::PORTK:
                        pinPort = GPIOK;
                        break;

                    case Chimera::GPIO::Port::PORTL:
                        pinPort = GPIOL;
                        break;
                #endif

                    default:
                        pinPort = nullptr;
                        break;
                };

                return pinPort;
            }

            PinMode ChimeraGPIO::convertDrive(const Chimera::GPIO::Drive &drive)
            {
                PinMode mode = PinMode::UNKNOWN_MODE;

                switch(drive)
                {
                    case Chimera::GPIO::Drive::INPUT:
                        mode = PinMode::INPUT;
                        break;

			        case Chimera::GPIO::Drive::OUTPUT_PUSH_PULL:
                        mode = PinMode::OUTPUT_PP;
                        break;

			        case Chimera::GPIO::Drive::OUTPUT_OPEN_DRAIN:
                        mode = PinMode::OUTPUT_OD;
                        break;

			        case Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL:
                        mode = PinMode::ALT_PP;
                        break;

			        case Chimera::GPIO::Drive::ALTERNATE_OPEN_DRAIN:
                        mode = PinMode::ALT_OD;
                        break;

			        case Chimera::GPIO::Drive::ANALOG:
                        mode = PinMode::ANALOG;
                        break;

			        case Chimera::GPIO::Drive::HIZ:
                        mode = PinMode::INPUT;
                        break;

                    default:
                        mode = PinMode::UNKNOWN_MODE;
                        break;
                };

                return mode;
            }

            PinPull ChimeraGPIO::convertPull(const Chimera::GPIO::Pull &pull)
            {
                PinPull pinPull = PinPull::UNKNOWN_PULL;

                switch(pull)
                {
                    case Chimera::GPIO::Pull::NO_PULL:
                        pinPull = PinPull::NOPULL;
                        break;

                    case Chimera::GPIO::Pull::PULL_UP:
                        pinPull = PinPull::PULLUP;
                        break;

                    case Chimera::GPIO::Pull::PULL_DN:
                        pinPull = PinPull::PULLDN;
                        break;

                    default:
                        pinPull = PinPull::UNKNOWN_PULL;
                        break;
                };

                return pinPull;
            }


#endif
		}
	}
}

