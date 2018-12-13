/* Project Includes */
#include <Thor/include/gpio.hpp>

namespace Thor
{
	namespace Peripheral
	{
		namespace GPIO
		{
			using namespace Thor::Definitions::GPIO;

			static const uint32_t getRCCGPIOMask(const GPIO_TypeDef *const instance)
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

            void GPIOClass::init(const PinPort port, const PinNum pin)
            {
                pinConfig.GPIOx = port;
                pinConfig.pinNum = pin;

                auto cfg = getHALInit(pinConfig);
                GPIO_Init(pinConfig.GPIOx, &cfg);
            }

            void GPIOClass::initAdvanced(const PinPort port, const PinNum pin, const PinSpeed speed, const uint32_t alt)
            {
				pinConfig.GPIOx = port;
				pinConfig.pinNum = pin;
				pinConfig.speed = speed;
				pinConfig.alternate = alt;

                auto cfg = getHALInit(pinConfig);
                GPIO_Init(pinConfig.GPIOx, &cfg);
            }

            void GPIOClass::mode(const PinMode Mode, const PinPull Pull)
			{
				pinConfig.mode = Mode;
				pinConfig.pull = Pull;

				auto cfg = getHALInit(pinConfig);
				GPIO_Init(pinConfig.GPIOx, &cfg);
			}

			void GPIOClass::write(const LogicLevel state)
			{
				HAL_GPIO_WritePin(pinConfig.GPIOx, static_cast<uint16_t>(pinConfig.pinNum), static_cast<GPIO_PinState>(state));
			}

			bool GPIOClass::read()
			{
				return static_cast<bool>(HAL_GPIO_ReadPin(pinConfig.GPIOx, static_cast<uint16_t>(pinConfig.pinNum)));
			}

			void GPIOClass::toggle()
			{
				HAL_GPIO_TogglePin(pinConfig.GPIOx, static_cast<uint16_t>(pinConfig.pinNum));
			}

            GPIO_InitTypeDef GPIOClass::getHALInit(const PinConfig &config)
			{
				GPIO_InitTypeDef InitStruct;

				InitStruct.Pin = static_cast<uint32_t>(config.pinNum);
				InitStruct.Speed = static_cast<uint32_t>(config.speed);
				InitStruct.Mode = static_cast<uint32_t>(config.mode);
				InitStruct.Pull = static_cast<uint32_t>(config.pull);
				InitStruct.Alternate = config.alternate;

				return InitStruct;
			}

			void GPIOClass::GPIO_Init(PinPort port, GPIO_InitTypeDef *initStruct)
			{
				GPIO_ClockEnable(port);
				HAL_GPIO_Init(port, initStruct);
			}

			void GPIOClass::GPIO_ClockEnable(PinPort port)
			{
				#if defined(TARGET_STM32F7) || defined(TARGET_STM32F4)
				SET_BIT(RCC->AHB1ENR, getRCCGPIOMask(port));
				#endif
			}

			void GPIOClass::GPIO_ClockDisable(PinPort port)
			{
				#if defined(TARGET_STM32F7) || defined(TARGET_STM32F4)
				CLEAR_BIT(RCC->AHB1ENR, getRCCGPIOMask(port));
				#endif
			}


			#if defined(USING_CHIMERA)
			static const GPIO_TypeDef *const portMap(const Chimera::GPIO::Port port)
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

			static const PinMode modeMap(Chimera::GPIO::Drive mode)
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

			Chimera::GPIO::Status ChimeraGPIO::init(const Chimera::GPIO::Port port, const uint8_t pin)
			{
                PinNum _pin = ChimeraGPIO::convertPinNum(pin);
                if (_pin == PinNum::NOT_A_PIN)
                {
                    return Chimera::GPIO::Status::ERROR_INVALID_PIN;
                }

                PinPort _port = ChimeraGPIO::convertPort(port);
                if (_port == nullptr)
                {
                    return Chimera::GPIO::Status::ERROR_INVALID_PORT;
                }

                gpioPin.init(_port, _pin);

                return Chimera::GPIO::Status::OK;
			}

			Chimera::GPIO::Status ChimeraGPIO::setMode(const Chimera::GPIO::Drive drive, const bool pullup)
			{
				PinMode pinMode = modeMap(drive);
                PinPull pinPull = pullup ? PinPull::PULLUP : PinPull::NOPULL;

				gpioPin.mode(pinMode, pinPull);
				return Chimera::GPIO::Status::OK;
			}

			Chimera::GPIO::Status ChimeraGPIO::setState(const Chimera::GPIO::State state)
			{
                gpioPin.write(static_cast<LogicLevel>(state));
                return Chimera::GPIO::Status::OK;
			}

            Chimera::GPIO::Status ChimeraGPIO::getState(Chimera::GPIO::State &state)
            {
                state = static_cast<Chimera::GPIO::State>(gpioPin.read());
                return Chimera::GPIO::Status::OK;
            }

            const PinNum ChimeraGPIO::convertPinNum(const uint8_t num)
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

            const PinPort ChimeraGPIO::convertPort(const Chimera::GPIO::Port port)
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
                #endif

                    default:
                        pinPort = nullptr;
                        break;
                };

                return pinPort;
            }

            const PinMode ChimeraGPIO::convertDrive(const Chimera::GPIO::Drive drive)
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

            const PinPull ChimeraGPIO::convertPull(const Chimera::GPIO::Pull pull)
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

            const PinConfig ChimeraGPIO::convertPinInit(const Chimera::GPIO::PinInit &pin)
            {
                PinConfig cfg;

                cfg.GPIOx = convertPort(pin.port);
                cfg.mode = convertDrive(pin.mode);
                cfg.pinNum = convertPinNum(pin.pin);
                cfg.pull = convertPull(pin.pull);
                cfg.alternate = pin.alternate;

                return cfg;
            }

            #endif /* !USING_CHIMERA */
		}
	}
}

