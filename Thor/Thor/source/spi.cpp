/* C/C++ Includes */
#include <math.h>

/* Boost Includes */
#include <boost/bind.hpp>

/* Thor Includes */
#include <Thor/include/spi.hpp>
#include <Thor/include/exti.hpp>

#ifdef TARGET_STM32F7
#include <stm32f7xx_hal_dma.h>
#include <stm32f7xx_hal_spi.h>
#include <stm32f7xx_hal_rcc.h>
#endif

#ifdef TARGET_STM32F4
#include <stm32f4xx_hal_dma.h>
#include <stm32f4xx_hal_spi.h>
#include <stm32f4xx_hal_rcc.h>
#endif

using namespace Thor::Definitions;
using namespace Thor::Definitions::SPI;
using namespace Thor::Definitions::GPIO;
using namespace Thor::Definitions::Interrupt;
using namespace Thor::Defaults::SPI;

using namespace Thor::Peripheral::SPI;
using namespace Thor::Peripheral::GPIO;

using Status = Thor::Definitions::Status;
using Modes = Thor::Definitions::Modes;

#if defined(USING_FREERTOS)
TaskTrigger spiTaskTrigger;
#endif

/*------------------------------------------------
Stores references to available SPIClass objects
-------------------------------------------------*/
static SPIClass_sPtr spiObjects[MAX_SPI_CHANNELS + 1];

/*------------------------------------------------
Directly maps the HAL SPI Instance pointer to a possible SPIClass object
-------------------------------------------------*/
static const SPIClass_sPtr& getSPIClassRef(SPI_TypeDef* instance)
{
	auto i = reinterpret_cast<std::uintptr_t>(instance);

	switch (i)
	{
	#if defined(SPI1)
	case SPI1_BASE:
		return spiObjects[1];
		break;
	#endif
	#if defined(SPI2)
	case SPI2_BASE:
		return spiObjects[2];
		break;
	#endif
	#if defined(SPI3)
	case SPI3_BASE:
		return spiObjects[3];
		break;
	#endif
	#if defined(SPI4)
	case SPI4_BASE:
		return spiObjects[4];
		break;
	#endif
	#if defined(SPI5)
	case SPI5_BASE:
		return spiObjects[5];
		break;
	#endif
	#if defined(SPI6)
	case SPI6_BASE:
		return spiObjects[6];
		break;
	#endif

	/* If we get here, something went wrong and the program will likely crash */
	default:
		return spiObjects[0];
		break;
	};
}

/*------------------------------------------------
Directly maps the HAL SPI Instance pointer to the correct bit mask for
enabling/disabling the peripheral clock
-------------------------------------------------*/
static uint32_t spiClockMask(SPI_TypeDef* instance)
{
	auto i = reinterpret_cast<std::uintptr_t>(instance);

	switch (i)
	{
	#if defined(TARGET_STM32F4) || defined(TARGET_STM32F7)
	#if defined(SPI1)
	case SPI1_BASE:
		return RCC_APB2ENR_SPI1EN;
		break;
	#endif
	#if defined(SPI2)
	case SPI2_BASE:
		return RCC_APB1ENR_SPI2EN;
		break;
	#endif
	#if defined(SPI3)
	case SPI3_BASE:
		return RCC_APB1ENR_SPI3EN;
		break;
	#endif
	#if defined(SPI4)
	case SPI4_BASE:
		return RCC_APB2ENR_SPI4EN;
		break;
	#endif
	#endif

	#if defined(TARGET_STM32F7)
	#if defined(SPI5)
	case SPI5_BASE:
		return RCC_APB2ENR_SPI5EN;
		break;
		#endif
		#if defined(SPI6)
	case SPI6_BASE:
		return RCC_APB2ENR_SPI6EN;
		break;
		#endif
	#endif /* !TARGET_STM32F7 */

	/* If we get here, something went wrong */
	default:
		return 0u;
		break;
	};
}

/*------------------------------------------------
Directly maps the HAL SPI Instance pointer to the correct register for
enabling/disabling the peripheral clock.
-------------------------------------------------*/
static volatile uint32_t* spiClockRegister(SPI_TypeDef* instance)
{
	auto i = reinterpret_cast<std::uintptr_t>(instance);

	switch (i)
	{
#if defined(TARGET_STM32F4) || defined(TARGET_STM32F7)
	#if defined(SPI1)
	case SPI1_BASE:
		return &(RCC->APB2ENR);
		break;
	#endif
	#if defined(SPI2)
	case SPI2_BASE:
		return &(RCC->APB1ENR);
		break;
	#endif
	#if defined(SPI3)
	case SPI3_BASE:
		return &(RCC->APB1ENR);
		break;
	#endif
	#if defined(SPI4)
	case SPI4_BASE:
		return &(RCC->APB2ENR);
		break;
	#endif
#endif

#if defined(TARGET_STM32F7)
	#if defined(SPI5)
	case SPI5_BASE:
		return &(RCC->APB2ENR);
		break;
	#endif
	#if defined(SPI6)
	case SPI6_BASE:
		return &(RCC->APB2ENR);
		break;
	#endif
#endif /* !TARGET_STM32F7 */

	/* If we get here, something went wrong */
	default:
		return nullptr;
		break;
	};
}

namespace Thor
{
	namespace Peripheral
	{
		namespace SPI
		{

            SPIClass::SPIClass(const int& channel)
            {
				spi_channel = channel;
				spi_handle.Instance = spi_cfg[spi_channel].instance;
				spi_handle.Init = Defaults::SPI::dflt_SPI_Init;

				/*------------------------------------
				Interrupt
				------------------------------------*/
				ITSettingsHW.IRQn = spi_cfg[spi_channel].IT_HW.IRQn;
				ITSettingsHW.preemptPriority = spi_cfg[spi_channel].IT_HW.preemptPriority;
				ITSettingsHW.subPriority = spi_cfg[spi_channel].IT_HW.subPriority;

				ITSettings_DMA_TX.IRQn = spi_cfg[spi_channel].dmaIT_TX.IRQn;
				ITSettings_DMA_TX.preemptPriority = spi_cfg[spi_channel].dmaIT_TX.preemptPriority;
				ITSettings_DMA_TX.subPriority = spi_cfg[spi_channel].dmaIT_TX.subPriority;

				ITSettings_DMA_RX.IRQn = spi_cfg[spi_channel].dmaIT_RX.IRQn;
				ITSettings_DMA_RX.preemptPriority = spi_cfg[spi_channel].dmaIT_RX.preemptPriority;
				ITSettings_DMA_RX.subPriority = spi_cfg[spi_channel].dmaIT_RX.subPriority;

                /*------------------------------------
				DMA
				------------------------------------*/
                hdma_spi_tx.Init = Defaults::SPI::dflt_DMA_Init_TX;
				hdma_spi_rx.Init = Defaults::SPI::dflt_DMA_Init_RX;

				hdma_spi_tx.Init.Channel = spi_cfg[spi_channel].dmaTX.channel;
				hdma_spi_rx.Init.Channel = spi_cfg[spi_channel].dmaRX.channel;

				hdma_spi_tx.Instance = spi_cfg[spi_channel].dmaTX.Instance;
				hdma_spi_rx.Instance = spi_cfg[spi_channel].dmaRX.Instance;

				/*------------------------------------
				GPIO
				------------------------------------*/
                MOSI = boost::movelib::unique_ptr<GPIOClass>( new GPIOClass(
					spi_cfg[spi_channel].MOSI.GPIOx,
					spi_cfg[spi_channel].MOSI.PinNum,
					spi_cfg[spi_channel].MOSI.Speed,
					spi_cfg[spi_channel].MOSI.Alternate));

				MISO = boost::movelib::unique_ptr<GPIOClass>( new GPIOClass(
					spi_cfg[spi_channel].MISO.GPIOx,
					spi_cfg[spi_channel].MISO.PinNum,
					spi_cfg[spi_channel].MISO.Speed,
					spi_cfg[spi_channel].MISO.Alternate));

				SCK = boost::movelib::unique_ptr<GPIOClass>( new GPIOClass(
					spi_cfg[spi_channel].SCK.GPIOx,
					spi_cfg[spi_channel].SCK.PinNum,
					spi_cfg[spi_channel].SCK.Speed,
					spi_cfg[spi_channel].SCK.Alternate));

				CS = boost::movelib::unique_ptr<GPIOClass>( new GPIOClass(
					spi_cfg[spi_channel].NSS.GPIOx,
					spi_cfg[spi_channel].NSS.PinNum,
					spi_cfg[spi_channel].NSS.Speed,
					spi_cfg[spi_channel].NSS.Alternate));

			}

			const SPIClass_sPtr& SPIClass::create(const int channel)
			{
                /*------------------------------------------------
                Create a new object if none already there
                -------------------------------------------------*/
                if (!spiObjects[channel])
                {
                    /*------------------------------------------------
                    Forced to create the new object in this manner due to the
                    private constructor not being accessible by Boost
                    -------------------------------------------------*/
                    boost::shared_ptr<SPIClass> newClass(new SPIClass(channel));
                    spiObjects[channel] = newClass;
                }

				return spiObjects[channel];
			}



			Thor::Definitions::Status SPIClass::begin(const Config& settings, const bool &force)
			{

				SPI_GPIO_Init();
				SPI_Init();

				setMode(SubPeripheral::TX, Modes::BLOCKING);
				setMode(SubPeripheral::RX, Modes::BLOCKING);
			}

            void SPIClass::end()
            {
				SPI_GPIO_DeInit();
				SPI_DisableInterrupts();
				SPI_DMA_DeInit(SubPeripheral::TX);
				SPI_DMA_DeInit(SubPeripheral::RX);

				txMode = Modes::MODE_UNDEFINED;
				rxMode = Modes::MODE_UNDEFINED;
			}


			void SPIClass::attachChipSelect(const Thor::Peripheral::GPIO::GPIOClass_sPtr &slave_select)
			{
				extChipSelect = slave_select;
			}

			void SPIClass::detachChipSelect()
			{
				extChipSelect = nullptr;
			}


            Thor::Definitions::Status SPIClass::writeBytes(uint8_t *const txBuffer, size_t length, const bool &autoDisableCS)
            {
                return readWriteBytes(txBuffer, NULL, length, autoDisableCS);
            }

            Thor::Definitions::Status SPIClass::readWriteBytes(uint8_t *const txBuffer, uint8_t *const rxBuffer, size_t length, const bool &autoDisableCS)
            {
                /*------------------------------------------------
                Verify hardware is initialized and function inputs are ok
                -------------------------------------------------*/
                if (!hardwareStatus.gpio_enabled || !hardwareStatus.spi_enabled)
                {
                    return Thor::Definitions::Status::PERIPH_NOT_INITIALIZED;
                }
                else if (!length)
                {
                    return Thor::Definitions::Status::PERIPH_INVALID_PARAM;
                }

                /*------------------------------------------------
                Execute the transfer
                -------------------------------------------------*/
                switch (txMode)
                {
                case Modes::BLOCKING:
                    return transfer_blocking(txBuffer, rxBuffer, length, autoDisableCS);
                    break;

                case Modes::INTERRUPT:
                    return transfer_interrupt(txBuffer, rxBuffer, length, autoDisableCS);
                    break;

                case Modes::DMA:
                    return transfer_dma(txBuffer, rxBuffer, length, autoDisableCS);
                    break;

                default:
                    return Thor::Definitions::Status::PERIPH_ERROR;
                    break;
                }
            }






			Status SPIClass::setMode(const SubPeripheral& periph, const Modes& mode)
			{
				if ((periph == SubPeripheral::TXRX) || (periph == SubPeripheral::TX))
				{
					switch (mode)
					{
					case Modes::BLOCKING:
						txMode = mode;

						/*------------------------------------------------
                        Disable interrupts as both periphs are now in blocking mode
                        -------------------------------------------------*/
						if(rxMode == Modes::BLOCKING)
							SPI_DisableInterrupts();

						SPI_DMA_DeInit(SubPeripheral::TX);
						break;

					case Modes::INTERRUPT:
						txMode = mode;

						SPI_EnableInterrupts();
						SPI_DMA_DeInit(SubPeripheral::TX);
						break;

					case Modes::DMA:
						txMode = mode;

						SPI_EnableInterrupts();
						SPI_DMA_Init(SubPeripheral::TX);
						break;

					default:
						txMode = Modes::MODE_UNDEFINED;
						return Status::PERIPH_ERROR;
						break;
					}
				}

				if ((periph == SubPeripheral::TXRX) || (periph == SubPeripheral::RX))
				{
					switch (mode)
					{
					case Modes::BLOCKING:
                        rxMode = mode;

                        /*------------------------------------------------
                        Disable interrupts as both periphs are now in blocking mode
                        -------------------------------------------------*/
                        if (txMode == Modes::BLOCKING)
                        {
                            SPI_DisableInterrupts();
                        }

						SPI_DMA_DeInit(SubPeripheral::RX);
						break;

					case Modes::INTERRUPT:
						rxMode = mode;

						SPI_EnableInterrupts();
						SPI_DMA_DeInit(SubPeripheral::RX);
						break;

					case Modes::DMA:
						rxMode = mode;

						SPI_EnableInterrupts();
						SPI_DMA_Init(SubPeripheral::RX);
						break;

					default:
						rxMode = Modes::MODE_UNDEFINED;
						return Status::PERIPH_ERROR;
						break;
					}
				}

				return Status::PERIPH_OK;
			}

			void SPIClass::setChipSelect(LogicLevel state)
			{
				/*------------------------------------------------
                Use the external chip select line if the reference exists, otherwise
                control the default peripheral NSS (chip select) pin.
                -------------------------------------------------*/
                if (extChipSelect)
                {
                    extChipSelect->write(state);
                }
				else
				{
					/**
                     *  This is only useful when the peripheral's dedicated NSS pin is used, configured for hardware
					 *  management, and the SSOE bit in CR1 is set. (By default this is how Thor is set up)
					 *  In this configuration, the NSS pin is automatically driven low upon transmission start and
					 *  will STAY low until the SPI hardware is disabled. Rather quirky...
					 *
					 *  Note:   Don't bother trying to use software NSS management as described in the datasheet. It will
					 *		    cause a nasty mode fault if you are in master mode.
                     */
                    if (static_cast<bool>(state))
                    {
                        __HAL_SPI_DISABLE(&spi_handle);
                    }
				}
			}


			Status SPIClass::updateClockFrequency(uint32_t freq)
			{
				spi_handle.Init.BaudRatePrescaler = getPrescaler(spi_channel, freq);

                //TODO: Figure out how to do this without resetting the channel

				return Status::PERIPH_OK;
			}


            uint32_t SPIClass::getClockFrequency()
            {
                return getFrequency(spi_channel, spi_handle.Init.BaudRatePrescaler);
            }

            void SPIClass::IRQHandler()
			{
				HAL_SPI_IRQHandler(&spi_handle);
			}

			void SPIClass::IRQHandler_TXDMA()
			{
				HAL_DMA_IRQHandler(spi_handle.hdmatx);
			}

			void SPIClass::IRQHandler_RXDMA()
			{
				HAL_DMA_IRQHandler(spi_handle.hdmarx);
			}

            uint32_t SPIClass::getPrescaler(const int &channel, const uint32_t &freq)
            {
                uint32_t busFreq = 0u;
                uint32_t prescaler = 0u;
                const uint8_t numPrescalers = 8;

	            /*------------------------------------------------
                Figure out the APB bus frequency for this channel
                -------------------------------------------------*/
                if (spi_cfg[channel].clockBus == Thor::Definitions::ClockBus::APB1_PERIPH)
                {
                    busFreq = HAL_RCC_GetPCLK1Freq();
                }
                else if (spi_cfg[channel].clockBus == Thor::Definitions::ClockBus::APB2_PERIPH)
                {
                    busFreq = HAL_RCC_GetPCLK2Freq();
                }
                else
                {
                    return Thor::Defaults::SPI::dflt_SPI_Init.BaudRatePrescaler;
                }

                /*------------------------------------------------
                Calculate the error between the resultant pre-scaled clock and the desired clock
                -------------------------------------------------*/
	            int clockError[numPrescalers];
	            memset(clockError, INT_MAX, numPrescalers);

                for (int i = 0; i < numPrescalers; i++)
                {
                    clockError[i] = abs(static_cast<int>((busFreq / (1 << (i + 1)) - freq)));
                }

                /*------------------------------------------------
                Find the index of the element with lowest error
                -------------------------------------------------*/
	            auto idx = std::distance(clockError, std::min_element(clockError, clockError + numPrescalers - 1));

                /*------------------------------------------------
                Grab the correct prescaler
                -------------------------------------------------*/
	            switch (idx)
	            {
	            case 0:
		            prescaler = SPI_BAUDRATEPRESCALER_2;
                    break;

                case 1:
		            prescaler = SPI_BAUDRATEPRESCALER_4;
                    break;

	            case 2:
		            prescaler = SPI_BAUDRATEPRESCALER_8;
                    break;

	            case 3:
		            prescaler = SPI_BAUDRATEPRESCALER_16;
                    break;

	            case 4:
		            prescaler = SPI_BAUDRATEPRESCALER_32;
                    break;

	            case 5:
		            prescaler = SPI_BAUDRATEPRESCALER_64;
                    break;

	            case 6:
		            prescaler = SPI_BAUDRATEPRESCALER_128;
                    break;

	            case 7:
		            prescaler = SPI_BAUDRATEPRESCALER_256;
                    break;

	            default:
		            prescaler = Thor::Defaults::SPI::dflt_SPI_Init.BaudRatePrescaler;
                    break;
	            };

                return prescaler;
            }

            uint32_t SPIClass::getFrequency(const int &channel, const uint32_t &prescaler)
            {
                uint32_t apbFreq = 0u;
                uint32_t busFreq = 0u;

                /*------------------------------------------------
                Figure out the APB bus frequency for this channel
                -------------------------------------------------*/
                if (spi_cfg[channel].clockBus == Thor::Definitions::ClockBus::APB1_PERIPH)
                {
                    apbFreq = HAL_RCC_GetPCLK1Freq();
                }
                else if (spi_cfg[channel].clockBus == Thor::Definitions::ClockBus::APB2_PERIPH)
                {
                    apbFreq = HAL_RCC_GetPCLK2Freq();
                }
                else
                {
                    return 0u;
                }

                /*------------------------------------------------
                Now get the SPI clock frequency
                -------------------------------------------------*/
                switch (prescaler)
                {
                case SPI_BAUDRATEPRESCALER_2:
                    busFreq = apbFreq / 2;
                    break;

                case SPI_BAUDRATEPRESCALER_4:
                    busFreq = apbFreq / 4;
                    break;

                case SPI_BAUDRATEPRESCALER_8:
                    busFreq = apbFreq / 8;
                    break;

                case SPI_BAUDRATEPRESCALER_16:
                    busFreq = apbFreq / 16;
                    break;

                case SPI_BAUDRATEPRESCALER_32:
                    busFreq = apbFreq / 32;
                    break;

                case SPI_BAUDRATEPRESCALER_64:
                    busFreq = apbFreq / 64;
                    break;

                case SPI_BAUDRATEPRESCALER_128:
                    busFreq = apbFreq / 128;
                    break;

                case SPI_BAUDRATEPRESCALER_256:
                    busFreq = apbFreq / 256;
                    break;

                default:
                    busFreq = 0u;
                };

                return busFreq;
            }

            Thor::Definitions::Status SPIClass::transfer_blocking(uint8_t *const txBuffer, uint8_t *const rxBuffer, size_t length, const bool &autoDisableCS)
            {
                HAL_StatusTypeDef error = HAL_OK;
                Thor::Definitions::Status status = Thor::Definitions::Status::PERIPH_OK;

                uint32_t timeout = BLOCKING_TIMEOUT_MS;

                #if defined(USING_FREERTOS)
                timeout = pdMS_TO_TICKS(BLOCKING_TIMEOUT_MS);
                #endif

                /*------------------------------------------------
                Ensure the previous transmission has completed
                -------------------------------------------------*/
                if (tx_complete)
				{
                    /*------------------------------------------------
                    Activate the chip select line?
                    -------------------------------------------------*/
					if (1)//slaveSelectControl == SS_AUTOMATIC_CONTROL)
					{
						setChipSelect(LogicLevel::LOW);
                    }

                    /*------------------------------------------------
                    Execute the transfer
                    -------------------------------------------------*/
					if (txBuffer && !rxBuffer)
					{
						error = HAL_SPI_Transmit(&spi_handle, txBuffer, length, timeout);
					}
					else if (txBuffer && rxBuffer)
					{
                        /*------------------------------------------------
                        In order for TransmitReceive to work, both subperipherals must be in the same mode.
                        This will silently clobber whatever settings were previously there.
                        -------------------------------------------------*/
                        if (rxMode != txMode)
                        {
                            setMode(SubPeripheral::RX, txMode);
                        }

						error = HAL_SPI_TransmitReceive(&spi_handle, txBuffer, rxBuffer, length, timeout);
					}
                    else
                    {
                        status = Thor::Definitions::Status::PERIPH_INVALID_PARAM;
                    }

                    /*------------------------------------------------
                    De-activate the chip select line?
                    -------------------------------------------------*/
                    if (autoDisableCS)
                    {
                        setChipSelect(LogicLevel::HIGH);
                    }

                    /*------------------------------------------------
                    Handle any errors and exit gracefully
                    -------------------------------------------------*/
                    if (error != HAL_OK)
                    {
                        status = Thor::Definitions::Status::PERIPH_ERROR;
                    }
				}
				else
                {
                    status = Thor::Definitions::Status::PERIPH_BUSY;
                }

                return status;

            }   /* transfer_blocking() */

            Thor::Definitions::Status SPIClass::transfer_interrupt(uint8_t *const txBuffer, uint8_t *const rxBuffer, size_t length, const bool &autoDisableCS)
            {
                HAL_StatusTypeDef error = HAL_OK;
                Thor::Definitions::Status status = Thor::Definitions::Status::PERIPH_OK;

                /*------------------------------------------------
                Ensure the previous transmission has completed
                -------------------------------------------------*/
                if (tx_complete)
				{
					tx_complete = false;
                    isr_disable_chip_select = autoDisableCS;

                    /*------------------------------------------------
                    Activate the chip select line?
                    -------------------------------------------------*/
					if (1)//slaveSelectControl == SS_AUTOMATIC_CONTROL)
                    {
                        setChipSelect(LogicLevel::LOW);

                        //Queue up the auto disable CS for when the ISR hits.
                    }

				    /*------------------------------------------------
                    Start the transfer
                    -------------------------------------------------*/
					if (txBuffer && !rxBuffer)
                    {
                        error = HAL_SPI_Transmit_IT(&spi_handle, txBuffer, length);
                        status = Thor::Definitions::Status::PERIPH_TX_IN_PROGRESS;
                    }
					else if (txBuffer && rxBuffer)
					{
						/*------------------------------------------------
                        In order for TransmitReceive to work, both subperipherals must be in the same mode.
                        This will silently clobber whatever settings were previously there.
                        -------------------------------------------------*/
						if (rxMode != txMode)
                        {
                            setMode(SubPeripheral::RX, txMode);
                        }

                        status = Thor::Definitions::Status::PERIPH_TXRX_IN_PROGRESS;
						error = HAL_SPI_TransmitReceive_IT(&spi_handle, txBuffer, rxBuffer, length);
					}
		            else
                    {
                        status = Thor::Definitions::Status::PERIPH_INVALID_PARAM;
                    }

                    /*------------------------------------------------
                    Catch any errors and exit gracefully
                    -------------------------------------------------*/
                    if (error != HAL_OK)
                    {
                        status = Thor::Definitions::Status::PERIPH_ERROR;

                        if (autoDisableCS)
                        {
                            setChipSelect(LogicLevel::HIGH);
                        }
                    }
				}
				else
				{
					status = Thor::Definitions::Status::PERIPH_BUSY;
				}

                return status;

            }   /* transfer_interrupt() */

            Thor::Definitions::Status SPIClass::transfer_dma(uint8_t *const txBuffer, uint8_t *const rxBuffer, size_t length, const bool &autoDisableCS)
            {
                HAL_StatusTypeDef error = HAL_OK;
                Thor::Definitions::Status status = Thor::Definitions::Status::PERIPH_OK;

                /*------------------------------------------------
                Ensure the previous transmission has completed
                -------------------------------------------------*/
                if (tx_complete)
				{
					tx_complete = false;
                    isr_disable_chip_select = autoDisableCS;


                    /*------------------------------------------------
                    Activate the chip select line?
                    -------------------------------------------------*/
                    if (1)//slaveSelectControl == SS_AUTOMATIC_CONTROL)
                    {
                        setChipSelect(LogicLevel::LOW);

                        //Queue up the auto disable CS for when the ISR hits.
                    }

                    /*------------------------------------------------
                    Start the transfer
                    -------------------------------------------------*/
                    if (txBuffer && !rxBuffer)
                    {
                        status = Thor::Definitions::Status::PERIPH_TX_IN_PROGRESS;
                        error = HAL_SPI_Transmit_DMA(&spi_handle, txBuffer, length);
                    }
					else if (txBuffer && rxBuffer)
					{
                        /*------------------------------------------------
                        In order for TransmitReceive to work, both subperipherals must be in the same mode.
                        This will silently clobber whatever settings were previously there.
                        -------------------------------------------------*/
                        if (rxMode != txMode)
                        {
                            setMode(SubPeripheral::RX, txMode);
                        }

                        status = Thor::Definitions::Status::PERIPH_TXRX_IN_PROGRESS;
						error = HAL_SPI_TransmitReceive_DMA(&spi_handle, txBuffer, rxBuffer, length);
					}
                    else
                    {
                        status = Thor::Definitions::Status::PERIPH_INVALID_PARAM;
                    }

                    /*------------------------------------------------
                    Catch any errors and exit gracefully
                    -------------------------------------------------*/
                    if (error != HAL_OK)
                    {
                        status = Thor::Definitions::Status::PERIPH_ERROR;

                        if (autoDisableCS)
                        {
                            setChipSelect(LogicLevel::HIGH);
                        }
                    }
				}
				else
				{
                    status = Thor::Definitions::Status::PERIPH_BUSY;
				}

                return status;

            }   /* transfer_dma() */


			void SPIClass::SPI_GPIO_Init()
			{
				if (spi_handle.Init.Mode == SPI_MODE_MASTER)
				{
                    /*------------------------------------------------
                    Setup GPIO. Mode must be ALT_PP rather than Input for reads to work.
                    -------------------------------------------------*/
					MISO->mode(spi_cfg[spi_channel].MISO.Mode, spi_cfg[spi_channel].MISO.Pull);
					MOSI->mode(spi_cfg[spi_channel].MOSI.Mode, spi_cfg[spi_channel].MOSI.Pull);
					SCK->mode(spi_cfg[spi_channel].SCK.Mode, spi_cfg[spi_channel].SCK.Pull);

                    if (extChipSelect)
                    {
                        extChipSelect->mode(Thor::Definitions::GPIO::PinMode::OUTPUT_PP, Thor::Definitions::GPIO::PinPull::NOPULL);
                        extChipSelect->write(LogicLevel::HIGH);
                    }
                    else
                    {
                        CS->mode(spi_cfg[spi_channel].NSS.Mode, spi_cfg[spi_channel].NSS.Pull);
                    }
				}
				else
				{
					//TODO: Support slave mode
				}

                hardwareStatus.gpio_enabled = true;
			}

			void SPIClass::SPI_GPIO_DeInit()
			{
				//If there ever is a Thor::Peripheral::GPIO::GPIOClass deinit function, use it.
			}

			void SPIClass::SPI_Init()
			{
				SPI_EnableClock();

                if (HAL_SPI_Init(&spi_handle) != HAL_OK)
                {
                    BasicErrorHandler(logError("Failed SPI Init"));
                }

				actualClockFrequency = getFrequency(spi_channel, spi_handle.Init.BaudRatePrescaler);

				hardwareStatus.spi_enabled = true;
			}

			void SPIClass::SPI_DeInit()
			{
                if (HAL_SPI_DeInit(&spi_handle) != HAL_OK)
                {
                    BasicErrorHandler(logError("Failed SPI DeInit"));
                }

                SPI_DisableClock();

				hardwareStatus.spi_enabled = false;
			}

			void SPIClass::SPI_EnableClock()
			{
				*spiClockRegister(spi_handle.Instance) |= spiClockMask(spi_handle.Instance);
			}

			void SPIClass::SPI_DisableClock()
			{
				*spiClockRegister(spi_handle.Instance) &= ~spiClockMask(spi_handle.Instance);
			}

			void SPIClass::SPI_DMA_EnableClock()
			{
				__DMA1_CLK_ENABLE();
				__DMA2_CLK_ENABLE();
			}

			void SPIClass::SPI_EnableInterrupts()
			{
				HAL_NVIC_DisableIRQ(ITSettingsHW.IRQn);
				HAL_NVIC_SetPriority(ITSettingsHW.IRQn, ITSettingsHW.preemptPriority, ITSettingsHW.subPriority);
				HAL_NVIC_EnableIRQ(ITSettingsHW.IRQn);

                if (rxMode == Modes::INTERRUPT)
                {
                    __HAL_SPI_ENABLE_IT(&spi_handle, SPI_IT_RXNE);
                }

				hardwareStatus.spi_interrupts_enabled = true;
			}

			void SPIClass::SPI_DisableInterrupts()
			{
				__HAL_SPI_DISABLE_IT(&spi_handle, SPI_IT_RXNE);

				HAL_NVIC_ClearPendingIRQ(ITSettingsHW.IRQn);
				HAL_NVIC_DisableIRQ(ITSettingsHW.IRQn);

				hardwareStatus.spi_interrupts_enabled = false;
			}

			void SPIClass::SPI_DMA_Init(const SubPeripheral& periph)
			{
				if (periph == SubPeripheral::TX)
				{
					SPI_DMA_EnableClock();

                    if (HAL_DMA_Init(&hdma_spi_tx) != HAL_OK)
                    {
                        BasicErrorHandler(logError("Failed TX Modes::DMA Init"));
                    }

					__HAL_LINKDMA(&spi_handle, hdmatx, hdma_spi_tx);

					spiDMAManager.attachCallback_TXDMA(spi_channel, boost::bind(&SPIClass::IRQHandler_TXDMA, this));

					SPI_DMA_EnableInterrupts(periph);

					hardwareStatus.dma_enabled_tx = true;
				}
				else if (periph == SubPeripheral::RX)
				{
					SPI_DMA_EnableClock();

                    if (HAL_DMA_Init(&hdma_spi_rx) != HAL_OK)
                    {
                        BasicErrorHandler(logError("Failed RX Modes::DMA Init. Check Init settings."));
                    }

					__HAL_LINKDMA(&spi_handle, hdmarx, hdma_spi_rx);

					spiDMAManager.attachCallback_RXDMA(spi_channel, boost::bind(&SPIClass::IRQHandler_RXDMA, this));

					SPI_DMA_EnableInterrupts(periph);

					hardwareStatus.dma_enabled_rx = true;
				}
			}

			void SPIClass::SPI_DMA_DeInit(const SubPeripheral& periph)
			{
				if (periph == SubPeripheral::TX)
				{
                    if (!hardwareStatus.dma_enabled_tx)
                    {
                        return;
                    }

					HAL_DMA_Abort(spi_handle.hdmatx);
					HAL_DMA_DeInit(spi_handle.hdmatx);
					SPI_DMA_DisableInterrupts(periph);
					spiDMAManager.removeCallback_TXDMA(spi_channel);

					hardwareStatus.dma_enabled_tx = false;
				}
				else if (periph == SubPeripheral::RX)
				{
                    if (!hardwareStatus.dma_enabled_rx)
                    {
                        return;
                    }

					HAL_DMA_Abort(spi_handle.hdmarx);
					HAL_DMA_DeInit(spi_handle.hdmarx);
					SPI_DMA_DisableInterrupts(periph);
					spiDMAManager.removeCallback_RXDMA(spi_channel);

					hardwareStatus.dma_enabled_rx = false;
				}
			}

			void SPIClass::SPI_DMA_EnableInterrupts(const SubPeripheral& periph)
			{
				if (periph == SubPeripheral::TX)
				{
					HAL_NVIC_DisableIRQ(ITSettings_DMA_TX.IRQn);
					HAL_NVIC_ClearPendingIRQ(ITSettings_DMA_TX.IRQn);
					HAL_NVIC_SetPriority(ITSettings_DMA_TX.IRQn, ITSettings_DMA_TX.preemptPriority, ITSettings_DMA_TX.subPriority);
					HAL_NVIC_EnableIRQ(ITSettings_DMA_TX.IRQn);

					hardwareStatus.dma_interrupts_enabled_tx = true;
				}
				else if(periph == SubPeripheral::RX)
				{
					HAL_NVIC_DisableIRQ(ITSettings_DMA_RX.IRQn);
					HAL_NVIC_ClearPendingIRQ(ITSettings_DMA_RX.IRQn);
					HAL_NVIC_SetPriority(ITSettings_DMA_RX.IRQn, ITSettings_DMA_RX.preemptPriority, ITSettings_DMA_RX.subPriority);
					HAL_NVIC_EnableIRQ(ITSettings_DMA_RX.IRQn);

					hardwareStatus.dma_interrupts_enabled_rx = true;
				}
			}

			void SPIClass::SPI_DMA_DisableInterrupts(const SubPeripheral& periph)
			{
				if (periph == SubPeripheral::TX)
				{
					HAL_NVIC_ClearPendingIRQ(ITSettings_DMA_TX.IRQn);
					HAL_NVIC_DisableIRQ(ITSettings_DMA_TX.IRQn);

					hardwareStatus.dma_interrupts_enabled_tx = false;
				}
				else if(periph == SubPeripheral::RX)
				{
					HAL_NVIC_ClearPendingIRQ(ITSettings_DMA_RX.IRQn);
					HAL_NVIC_DisableIRQ(ITSettings_DMA_RX.IRQn);

					hardwareStatus.dma_interrupts_enabled_rx = false;
				}
			}

            #ifdef USING_FREERTOS
			void SPIClass::attachThreadTrigger(const Thor::Definitions::Interrupt::Trigger &trig, const SemaphoreHandle_t *const semphr)
			{
				//spiTaskTrigger.attachEventConsumer(trig, semphr);
			}

			void SPIClass::removeThreadTrigger(const SemaphoreHandle_t *const semphr)
			{
				//spiTaskTrigger.removeEventConsumer(trig);
			}
			#endif

            ChimeraSPI::ChimeraSPI(const int& channel)
            {
                this->channel = channel;
                spi = SPIClass::create(channel);

                conversionError = false;
            }

            Chimera::SPI::Status ChimeraSPI::init(const Chimera::SPI::Setup &setupStruct)
            {
                Chimera::SPI::Status initResult = Chimera::SPI::Status::ERROR;

                /*------------------------------------------------
                Convert the setup configuration and attempt to initialize the hardware
                -------------------------------------------------*/
                Thor::Peripheral::SPI::Config cfg = convertInitSettings(setupStruct);

                if (conversionError)
                {
                    initResult = Chimera::SPI::Status::FAILED_CONVERSION;
                }
                else if (spi->begin(cfg, false) == Thor::Definitions::Status::PERIPH_OK)
                {
                    initResult = Chimera::SPI::Status::OK;
                }
                else
                {
                    initResult = Chimera::SPI::Status::FAILED_INITIALIZATION;
                }

                return initResult;
            }

            Thor::Peripheral::SPI::Config ChimeraSPI::convertInitSettings(const Chimera::SPI::Setup &setup)
            {
                bool error = false;
                Thor::Peripheral::SPI::Config cfg;

                /*------------------------------------------------
                Convert over all the GPIO configuration info
                -------------------------------------------------*/
                if (setup.SCK.port != Chimera::GPIO::Port::UNKNOWN_PORT)
                {
                    cfg.SCK = ChimeraGPIO::convertPinInit(setup.SCK);
                }

                if (setup.MOSI.port != Chimera::GPIO::Port::UNKNOWN_PORT)
                {
                    cfg.MOSI = ChimeraGPIO::convertPinInit(setup.MOSI);
                }

                if (setup.MISO.port != Chimera::GPIO::Port::UNKNOWN_PORT)
                {
                    cfg.MISO = ChimeraGPIO::convertPinInit(setup.MISO);
                }

                if (setup.CS.port != Chimera::GPIO::Port::UNKNOWN_PORT)
                {
                    cfg.CS = ChimeraGPIO::convertPinInit(setup.CS);
                }

                /*------------------------------------------------
                Convert the hardware parameters
                -------------------------------------------------*/
                cfg.settings.Mode = convertMode(setup.mode, &error);
                cfg.settings.DataSize = convertDataSize(setup.dataSize, &error);
                cfg.settings.FirstBit = convertBitOrder(setup.bitOrder, &error);
                cfg.settings.CLKPhase = convertClockPhase(setup.clockMode, &error);
                cfg.settings.CLKPolarity = convertClockPolarity(setup.clockMode, &error);
                cfg.settings.BaudRatePrescaler = convertBaudRatePrescaler(channel, setup.clockFrequency, &error);

                conversionError = error;

                return cfg;
            }

            uint32_t ChimeraSPI::convertMode(const Chimera::SPI::Mode &mode, bool *const error)
            {
                uint32_t result = 0u;

                switch (mode)
                {
                case Chimera::SPI::Mode::MASTER:
                    result = SPI_MODE_MASTER;
                    break;

                case Chimera::SPI::Mode::SLAVE:
                    result = SPI_MODE_SLAVE;
                    break;

                default:
                    result = 0u;
                    *error = true;
                    break;
                };

                return result;
            }

            uint32_t ChimeraSPI::convertDataSize(const Chimera::SPI::DataSize &size, bool *const error)
            {
                uint32_t result = 0u;

                switch (size)
                {
                case Chimera::SPI::DataSize::SZ_8BIT:
                    result = SPI_DATASIZE_8BIT;
                    break;

                case Chimera::SPI::DataSize::SZ_16BIT:
                    result = SPI_DATASIZE_16BIT;
                    break;

                default:
                    result = 0u;
                    *error = true;
                    break;
                };

                return result;
            }

            uint32_t ChimeraSPI::convertBitOrder(const Chimera::SPI::BitOrder &order, bool *const error)
            {
                uint32_t result = 0u;

                switch (order)
                {
                case Chimera::SPI::BitOrder::LSB_FIRST:
                    result = SPI_FIRSTBIT_LSB;
                    break;

                case Chimera::SPI::BitOrder::MSB_FIRST:
                    result = SPI_FIRSTBIT_MSB;
                    break;

                default:
                    result = 0u;
                    *error = true;
                    break;
                };

                return result;
            }

            uint32_t ChimeraSPI::convertClockPhase(const Chimera::SPI::ClockMode &mode, bool *const error)
            {
                uint32_t result = 0u;

                switch (mode)
                {
                case Chimera::SPI::ClockMode::MODE0:
                case Chimera::SPI::ClockMode::MODE2:
                    result = SPI_PHASE_1EDGE;
                    break;

                case Chimera::SPI::ClockMode::MODE1:
                case Chimera::SPI::ClockMode::MODE3:
                    result = SPI_PHASE_2EDGE;
                    break;

                default:
                    result = 0u;
                    *error = true;
                    break;
                };

                return result;
            }

            uint32_t ChimeraSPI::convertClockPolarity(const Chimera::SPI::ClockMode &mode, bool *const error)
            {
                uint32_t result = 0u;

                switch (mode)
                {
                case Chimera::SPI::ClockMode::MODE0:
                case Chimera::SPI::ClockMode::MODE1:
                    result = SPI_POLARITY_LOW;
                    break;

                case Chimera::SPI::ClockMode::MODE2:
                case Chimera::SPI::ClockMode::MODE3:
                    result = SPI_POLARITY_HIGH;
                    break;

                default:
                    result = 0u;
                    *error = true;
                    break;
                };

                return result;
            }

            uint32_t ChimeraSPI::convertBaudRatePrescaler(const int &channel, const uint32_t &freq, bool *const error)
            {
               return SPIClass::getPrescaler(channel, freq);
            }

            Chimera::SPI::Status ChimeraSPI::setChipSelect(const Chimera::GPIO::State &value)
            {
                return Chimera::SPI::Status::OK;
            }

            Chimera::SPI::Status ChimeraSPI::setChipSelectControlMode(const Chimera::SPI::ChipSelectMode &mode)
            {
                return Chimera::SPI::Status::OK;
            }

            Chimera::SPI::Status ChimeraSPI::writeBytes(const uint8_t *const txBuffer, size_t length, const bool &disableCS)
            {
                return Chimera::SPI::Status::OK;
            }

            Chimera::SPI::Status ChimeraSPI::readBytes(uint8_t *const rxBuffer, size_t length, const bool &disableCS)
            {
                return Chimera::SPI::Status::OK;
            }

            Chimera::SPI::Status ChimeraSPI::readWriteBytes(const uint8_t *const txBuffer, uint8_t *const rxBuffer, size_t length, const bool& disableCS)
            {
                //spi->write(txBuffer, rxBuffer, length, disableCS);
                return Chimera::SPI::Status::OK;
            }

            Chimera::SPI::Status ChimeraSPI::setPeripheralMode(const Chimera::SPI::SubPeripheral &periph, const Chimera::SPI::SubPeripheralMode &mode)
            {
                return Chimera::SPI::Status::OK;
            }

            Chimera::SPI::Status ChimeraSPI::setClockFrequency(const uint32_t &freq)
            {
                return Chimera::SPI::Status::OK;
            }

            Chimera::SPI::Status ChimeraSPI::getClockFrequency(uint32_t *const freq)
            {
                return Chimera::SPI::Status::OK;
            }

            Chimera::SPI::Status ChimeraSPI::reserve(const uint32_t &timeout_ms)
            {
                return Chimera::SPI::Status::OK;
            }

            Chimera::SPI::Status ChimeraSPI::release(const uint32_t &timeout_ms)
            {
                return Chimera::SPI::Status::OK;
            }

            Chimera::SPI::Status ChimeraSPI::onWriteCompleteCallback(const Chimera::void_func_void func)
            {
                return Chimera::SPI::Status::OK;
            }

            Chimera::SPI::Status ChimeraSPI::onReadCompleteCallback(const Chimera::void_func_void func)
            {
                return Chimera::SPI::Status::OK;
            }

            Chimera::SPI::Status ChimeraSPI::onReadWriteCompleteCallback(const Chimera::void_func_void func)
            {
                return Chimera::SPI::Status::OK;
            }

            Chimera::SPI::Status ChimeraSPI::onErrorCallback(const Chimera::void_func_uint32_t func)
            {
                return Chimera::SPI::Status::OK;
            }

            Chimera::SPI::Status ChimeraSPI::attachEventWakeup(const Chimera::FreeRTOS::SPIEvent &event, const SemaphoreHandle_t *const semphr)
            {
                return Chimera::SPI::Status::OK;
            }

            Chimera::SPI::Status ChimeraSPI::removeEventWakeup(const SemaphoreHandle_t *const semphr)
            {
                return Chimera::SPI::Status::OK;
            }
        }
	}
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    /*------------------------------------------------
    Determine which object actually triggered this callback
    -------------------------------------------------*/
	const SPIClass_sPtr& spi = getSPIClassRef(hspi->Instance);

	if (spi && spi->hardwareStatus.spi_enabled)
	{
        spi->tx_complete = true;


		#if defined(USING_FREERTOS)
        /*------------------------------------------------
        Notify event occurred
        -------------------------------------------------*/
		spiTaskTrigger.logEvent(TX_COMPLETE, &spiTaskTrigger);
		#endif
	}
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	/*------------------------------------------------
    Determine which object actually triggered this callback
    -------------------------------------------------*/
	const SPIClass_sPtr& spi = getSPIClassRef(hspi->Instance);

	if (spi && spi->hardwareStatus.spi_enabled)
	{
		spi->tx_complete = true;


		#if defined(USING_FREERTOS)
        /*------------------------------------------------
        Notify event occurred
        -------------------------------------------------*/
		spiTaskTrigger.logEvent(TXRX_COMPLETE, &spiTaskTrigger);
		#endif
	}
}

void HAL_SPI_TxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
}

void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
}

void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
}

void SPI1_IRQHandler()
{
	if (spiObjects[1])
	{
		spiObjects[1]->IRQHandler();
	}
}

void SPI2_IRQHandler()
{
	if (spiObjects[2])
	{
		spiObjects[2]->IRQHandler();
	}
}

void SPI3_IRQHandler()
{
	if (spiObjects[3])
	{
		spiObjects[3]->IRQHandler();
	}
}

void SPI4_IRQHandler()
{
	if (spiObjects[4])
	{
		spiObjects[4]->IRQHandler();
	}
}

void SPI5_IRQHandler()
{
	if (spiObjects[5])
	{
		spiObjects[5]->IRQHandler();
	}
}

void SPI6_IRQHandler()
{
	if (spiObjects[6])
	{
		spiObjects[6]->IRQHandler();
	}
}

