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
using Options = Thor::Definitions::SPI::Options;

#if defined(USING_FREERTOS)
//static SemaphoreHandle_t spiSemphrs[MAX_SPI_CHANNELS + 1];
TaskTrigger spiTaskTrigger;
#endif 

/* Stores references to available SPIClass objects. Automatically initializes to empty shared_ptr. */
static SPIClass_sPtr spiObjects[MAX_SPI_CHANNELS + 1];

/* Directly maps the HAL SPI Instance pointer to a possible SPIClass object */
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

/* Directly maps the HAL SPI Instance pointer to the correct bit mask for enabling/disabling the peripheral clock */
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

/* Directly maps the HAL SPI Instance pointer to the correct register for enabling/disabling the peripheral clock.
 * For examples on how this variable memory mapping is accomplished, see: https://goo.gl/t9dgbs */
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


static const uint32_t getBaudRatePrescalerFromFreq(int channel, int freq)
{
	int busFreq = 1;
	const uint8_t numPrescalers = 8;
	
	/* Figure out what bus frequency is in use for this channel */
	if (spi_cfg[channel].clockBus == Thor::Definitions::ClockBus::APB1_PERIPH)
		busFreq = HAL_RCC_GetPCLK1Freq();
	else if (spi_cfg[channel].clockBus == Thor::Definitions::ClockBus::APB2_PERIPH)
		busFreq = HAL_RCC_GetPCLK2Freq();
	else
		return Thor::Defaults::SPI::dflt_SPI_Init.BaudRatePrescaler;

	/* Calculate the error between the resultant pre-scaled clocks and the desired clock */
	int clockError[numPrescalers];
	memset(clockError, INT_MAX, numPrescalers);

	for (int i = 0; i < numPrescalers; i++)
		clockError[i] = abs((busFreq / (1 << (i + 1)) - freq));

	/* Find the index of the element with lowest error */
	auto idx = std::distance(clockError, std::min_element(clockError, clockError + numPrescalers - 1));

	switch (idx)
	{
	case 0:
		return SPI_BAUDRATEPRESCALER_2;

	case 1:
		return SPI_BAUDRATEPRESCALER_4;

	case 2:
		return SPI_BAUDRATEPRESCALER_8;

	case 3:
		return SPI_BAUDRATEPRESCALER_16;

	case 4:
		return SPI_BAUDRATEPRESCALER_32;

	case 5:
		return SPI_BAUDRATEPRESCALER_64;

	case 6:
		return SPI_BAUDRATEPRESCALER_128;

	case 7:
		return SPI_BAUDRATEPRESCALER_256;

	default:	
		return Thor::Defaults::SPI::dflt_SPI_Init.BaudRatePrescaler;
	};
};

static const uint32_t getFreqFromBaudRatePrescaler(int channel, uint32_t baudRatePrescaler)
{
	uint32_t busFreq = 1;

	/* Figure out what bus frequency is in use for this channel */
	if (spi_cfg[channel].clockBus == Thor::Definitions::ClockBus::APB1_PERIPH)
		busFreq = HAL_RCC_GetPCLK1Freq();
	else if (spi_cfg[channel].clockBus == Thor::Definitions::ClockBus::APB2_PERIPH)
		busFreq = HAL_RCC_GetPCLK2Freq();


	switch (baudRatePrescaler)
	{
	case SPI_BAUDRATEPRESCALER_2:
		return busFreq / 2;

	case SPI_BAUDRATEPRESCALER_4:
		return busFreq / 4;

	case SPI_BAUDRATEPRESCALER_8:
		return busFreq / 8;

	case SPI_BAUDRATEPRESCALER_16:
		return busFreq / 16;

	case SPI_BAUDRATEPRESCALER_32:
		return busFreq / 32;

	case SPI_BAUDRATEPRESCALER_64:
		return busFreq / 64;

	case SPI_BAUDRATEPRESCALER_128:
		return busFreq / 128;

	case SPI_BAUDRATEPRESCALER_256:
		return busFreq / 256;

	default:
		return 0u;
	};

}

namespace Thor
{
	namespace Peripheral
	{
		namespace SPI
		{
			#ifdef USING_FREERTOS
			void SPIClass::attachThreadTrigger(Trigger trig, SemaphoreHandle_t* semphr)
			{
				spiTaskTrigger.attachEventConsumer(trig, semphr);
			}
			
			void SPIClass::removeThreadTrigger(Trigger trig)
			{
				spiTaskTrigger.removeEventConsumer(trig);
			}
			#endif 

			void SPIClass::SPI_IRQHandler()
			{
				HAL_SPI_IRQHandler(&spi_handle);
			}

			void SPIClass::SPI_IRQHandler_TXDMA()
			{
				HAL_DMA_IRQHandler(spi_handle.hdmatx);
			}

			void SPIClass::SPI_IRQHandler_RXDMA()
			{
				HAL_DMA_IRQHandler(spi_handle.hdmarx);
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
					if (slaveSelectControl == SS_AUTOMATIC_CONTROL)
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
                        if (!txRxModesEqual(txMode))
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
                    if (slaveSelectControl == SS_AUTOMATIC_CONTROL && autoDisableCS )
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

                    //TODO: This buffering feature is marked to be deprecated!

					TX_tempPacket.data_tx = nullptr;
					TX_tempPacket.data_rx = nullptr;
					TX_tempPacket.length = 0;
					TX_tempPacket.disableNSS = autoDisableCS;
					TXPacketBuffer.push_back(TX_tempPacket);

                    /*------------------------------------------------
                    Activate the chip select line?
                    -------------------------------------------------*/
					if (slaveSelectControl == SS_AUTOMATIC_CONTROL)
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
						if (!txRxModesEqual(txMode))
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

                        if (slaveSelectControl == SS_AUTOMATIC_CONTROL && autoDisableCS)
                        {
                            setChipSelect(LogicLevel::HIGH);
                        }
                    }
				}
				else
				{
                    //TODO: This buffering feature is marked to be deprecated!

					/* Hardware is currently busy. Buffer the data packet. */
					TX_tempPacket.data_tx = txBuffer;
					TX_tempPacket.data_rx = rxBuffer;
					TX_tempPacket.length = length;
					TX_tempPacket.disableNSS = autoDisableCS;
					TXPacketBuffer.push_back(TX_tempPacket);
						
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

                    //TODO: This buffering feature is marked to be deprecated!

					TX_tempPacket.data_tx = nullptr;
					TX_tempPacket.length = 0;
					TX_tempPacket.disableNSS = autoDisableCS;
					TXPacketBuffer.push_back(TX_tempPacket);

                    /*------------------------------------------------
                    Activate the chip select line?
                    -------------------------------------------------*/
                    if (slaveSelectControl == SS_AUTOMATIC_CONTROL)
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
                        if (!txRxModesEqual(txMode))
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

                        if (slaveSelectControl == SS_AUTOMATIC_CONTROL && autoDisableCS)
                        {
                            setChipSelect(LogicLevel::HIGH);
                        }
                    }
				}
				else
				{
                    //TODO: This buffering feature is marked to be deprecated!

                    /*------------------------------------------------
                    Hardware is busy. Buffer the data.
                    -------------------------------------------------*/
                    TX_tempPacket.data_tx = txBuffer;
					TX_tempPacket.data_rx = rxBuffer;
					TX_tempPacket.length = length;
					TX_tempPacket.disableNSS = autoDisableCS;
					TXPacketBuffer.push_back(TX_tempPacket);

                    status = Thor::Definitions::Status::PERIPH_BUSY;
				}

                return status;

            }   /* transfer_dma() */

            SPIClass::SPIClass(int channel)
            {
				spi_channel = channel;
				spi_handle.Instance = spi_cfg[spi_channel].instance;
				spi_handle.Init = Defaults::SPI::dflt_SPI_Init;

				/*------------------------------------
				Interrupt Setup
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
				DMA Setup
				------------------------------------*/
                hdma_spi_tx.Init = Defaults::SPI::dflt_DMA_Init_TX;
				hdma_spi_rx.Init = Defaults::SPI::dflt_DMA_Init_RX;

				hdma_spi_tx.Init.Channel = spi_cfg[spi_channel].dmaTX.channel;
				hdma_spi_rx.Init.Channel = spi_cfg[spi_channel].dmaRX.channel;

				hdma_spi_tx.Instance = spi_cfg[spi_channel].dmaTX.Instance;
				hdma_spi_rx.Instance = spi_cfg[spi_channel].dmaRX.Instance;

				/*------------------------------------
				GPIO Setup
				------------------------------------*/
				MOSI = boost::make_shared<Thor::Peripheral::GPIO::GPIOClass>(
					spi_cfg[spi_channel].MOSI.GPIOx,
					spi_cfg[spi_channel].MOSI.PinNum,
					spi_cfg[spi_channel].MOSI.Speed,
					spi_cfg[spi_channel].MOSI.Alternate);

				MISO = boost::make_shared<Thor::Peripheral::GPIO::GPIOClass>(
					spi_cfg[spi_channel].MISO.GPIOx,
					spi_cfg[spi_channel].MISO.PinNum,
					spi_cfg[spi_channel].MISO.Speed,
					spi_cfg[spi_channel].MISO.Alternate);

				SCK = boost::make_shared<Thor::Peripheral::GPIO::GPIOClass>(
					spi_cfg[spi_channel].SCK.GPIOx,
					spi_cfg[spi_channel].SCK.PinNum,
					spi_cfg[spi_channel].SCK.Speed,
					spi_cfg[spi_channel].SCK.Alternate);

				NSS = boost::make_shared<Thor::Peripheral::GPIO::GPIOClass>(
					spi_cfg[spi_channel].NSS.GPIOx,
					spi_cfg[spi_channel].NSS.PinNum,
					spi_cfg[spi_channel].NSS.Speed,
					spi_cfg[spi_channel].NSS.Alternate);

				/*------------------------------------
				Buffer Setup
				------------------------------------*/
				TXPacketBuffer.set_capacity(Thor::Definitions::SPI::SPI_BUFFER_SIZE);

				rxBufferedPackets = new SmartBuffer::RingBuffer<uint16_t>(_rxbuffpckt, Thor::Definitions::SPI::SPI_BUFFER_SIZE);
				rxBufferedPacketLengths = new SmartBuffer::RingBuffer<size_t>(_rxbuffpcktlen, Thor::Definitions::SPI::SPI_BUFFER_SIZE);
			}
			
			const boost::shared_ptr<SPIClass>& SPIClass::create(const int channel)
			{
				boost::shared_ptr<SPIClass> newClass(new SPIClass(channel));
				
				spiObjects[channel] = newClass;
				
				return spiObjects[channel];
			}

			void SPIClass::begin(Options options)
			{
				if (options != NO_OPTIONS)
				{
					if ((options & INTERNAL_SLAVE_SELECT) == INTERNAL_SLAVE_SELECT)
						SlaveSelectType = INTERNAL_SLAVE_SELECT;

					if ((options & EXTERNAL_SLAVE_SELECT) == EXTERNAL_SLAVE_SELECT)
						SlaveSelectType = EXTERNAL_SLAVE_SELECT;
				}

				/* Setup GPIO */
				SPI_GPIO_Init();

				/* Setup SPI */
				SPI_Init();

				setMode(SubPeripheral::TX, Modes::BLOCKING);
				setMode(SubPeripheral::RX, Modes::BLOCKING);
			}

			void SPIClass::attachPin(boost::shared_ptr<Thor::Peripheral::GPIO::GPIOClass> slave_select)
			{
				EXT_NSS = slave_select;
				EXT_NSS_ATTACHED = true;
				SLAVE_SELECT_MODE = EXTERNAL_SLAVE_SELECT;
			}

			void SPIClass::detachPin()
			{
				EXT_NSS = nullptr;
				EXT_NSS_ATTACHED = false;
				SLAVE_SELECT_MODE = INTERNAL_SLAVE_SELECT;
			}

			void SPIClass::setSSMode(Thor::Definitions::SPI::Options ss_mode)
			{
				if (ss_mode == SS_MANUAL_CONTROL)
					slaveSelectControl = SS_MANUAL_CONTROL;
				else
					slaveSelectControl = SS_AUTOMATIC_CONTROL;
			}

			void SPIClass::attachSettings(SPI_InitTypeDef& settings)
			{
				spi_handle.Init = settings;
			}

			SPI_InitTypeDef SPIClass::getSettings()
			{
				return spi_handle.Init;
			}

			void SPIClass::reInitialize()
			{
				auto oldRXMode = rxMode;
				auto oldTXMode = txMode;

				SPI_DeInit();
				SPI_Init();
			
				setMode(SubPeripheral::RX, oldRXMode);
				setMode(SubPeripheral::TX, oldTXMode);
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
                if (!SPI_PeriphState.gpio_enabled || !SPI_PeriphState.spi_enabled)
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




            void SPIClass::end()
            {
				SPI_GPIO_DeInit();
				SPI_DisableInterrupts();
				SPI_DMA_DeInit(SubPeripheral::TX);
				SPI_DMA_DeInit(SubPeripheral::RX);

				txMode = Modes::MODE_UNDEFINED;
				rxMode = Modes::MODE_UNDEFINED;
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
				/* Assumes we are using a random GPIO and not the dedicated peripheral NSS pin */
				if (SlaveSelectType == EXTERNAL_SLAVE_SELECT && EXT_NSS_ATTACHED)
					EXT_NSS->write(state);
				else
				{
					/* This is only useful when the peripheral's dedicated NSS pin is used, configured for hardware
					 * management, and the SSOE bit in CR1 is set. (By default this is how Thor is set up)
					 * In this configuration, the NSS pin is automatically driven low upon transmission start and
					 * will STAY low until the SPI hardware is disabled. Rather quirky...
					 * 
					 * Note: Don't bother trying to use software NSS management as described in the datasheet. It will
					 *		 cause a nasty mode fault if you are in master mode. */
					if (state)
						__HAL_SPI_DISABLE(&spi_handle);
				}
			}
			

			Status SPIClass::updateClockFrequency(uint32_t freq)
			{
				spi_handle.Init.BaudRatePrescaler = getBaudRatePrescalerFromFreq(spi_channel, freq);
				reInitialize();

				return Status::PERIPH_OK;
			}


            uint32_t SPIClass::getClockFrequency()
            {
                return getFreqFromBaudRatePrescaler(spi_channel, spi_handle.Init.BaudRatePrescaler);
            }

            bool SPIClass::txRxModesEqual(Modes mode)
			{
				if ((txMode == mode) && (rxMode == mode))
					return true;
				else
					return false;
			}
			
			void SPIClass::SPI_GPIO_Init()
			{
				if (spi_handle.Init.Mode == SPI_MODE_MASTER)
				{
					/* Setup the default IO modes */
					/* The mode must be ALT_PP rather than Input for correct reads...counterintuitive...*/
					MISO->mode(spi_cfg[spi_channel].MISO.Mode, spi_cfg[spi_channel].MISO.Pull);
					MOSI->mode(spi_cfg[spi_channel].MOSI.Mode, spi_cfg[spi_channel].MOSI.Pull);
					SCK->mode(spi_cfg[spi_channel].SCK.Mode, spi_cfg[spi_channel].SCK.Pull);

					if (EXT_NSS_ATTACHED && (EXT_NSS != nullptr) && (SlaveSelectType == EXTERNAL_SLAVE_SELECT))
					{
						EXT_NSS->mode(OUTPUT_PP, NOPULL);
						EXT_NSS->write(LogicLevel::HIGH);
					}
					else
						NSS->mode(spi_cfg[spi_channel].NSS.Mode, spi_cfg[spi_channel].NSS.Pull);

					SPI_PeriphState.gpio_enabled = true;
				}
				else
				{
					//Eventually support slave mode
				}
			}

			void SPIClass::SPI_GPIO_DeInit()
			{
				//If there ever is a Thor::Peripheral::GPIO::GPIOClass deinit function, use it.
			}
			
			void SPIClass::SPI_Init()
			{
				SPI_EnableClock();

				if (HAL_SPI_Init(&spi_handle) != HAL_OK)
					BasicErrorHandler(logError("Failed SPI Init"));

				actualClockFrequency = getFreqFromBaudRatePrescaler(spi_channel, spi_handle.Init.BaudRatePrescaler);

				SPI_PeriphState.spi_enabled = true;
			}

			void SPIClass::SPI_DeInit()
			{
				if (HAL_SPI_DeInit(&spi_handle) != HAL_OK)
					BasicErrorHandler(logError("Failed SPI DeInit"));

				SPI_DisableClock();

				SPI_PeriphState.spi_enabled = false;
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
				/* Global Modes::DMA Clock options. Only turn on capability is
				provided due to other peripherals possibly using DMA. */
				if (__DMA1_IS_CLK_DISABLED())
					__DMA1_CLK_ENABLE();

				if (__DMA2_IS_CLK_DISABLED())
					__DMA2_CLK_ENABLE();
			}

			void SPIClass::SPI_EnableInterrupts()
			{
				HAL_NVIC_DisableIRQ(ITSettingsHW.IRQn);
				HAL_NVIC_SetPriority(ITSettingsHW.IRQn, ITSettingsHW.preemptPriority, ITSettingsHW.subPriority);
				HAL_NVIC_EnableIRQ(ITSettingsHW.IRQn);

				/* Specific interrupts to enable */
				if (rxMode == Modes::INTERRUPT)
					__HAL_SPI_ENABLE_IT(&spi_handle, SPI_IT_RXNE); 	//RX Data Register not Empty

				SPI_PeriphState.spi_interrupts_enabled = true;
			}

			void SPIClass::SPI_DisableInterrupts()
			{
				__HAL_SPI_DISABLE_IT(&spi_handle, SPI_IT_RXNE);

				HAL_NVIC_ClearPendingIRQ(ITSettingsHW.IRQn);
				HAL_NVIC_DisableIRQ(ITSettingsHW.IRQn);

				SPI_PeriphState.spi_interrupts_enabled = false;
			}

			void SPIClass::SPI_DMA_Init(const SubPeripheral& periph)
			{
				if (periph == SubPeripheral::TX)
				{
					SPI_DMA_EnableClock();

					if (HAL_DMA_Init(&hdma_spi_tx) != HAL_OK)
						BasicErrorHandler(logError("Failed TX Modes::DMA Init"));

					__HAL_LINKDMA(&spi_handle, hdmatx, hdma_spi_tx);

					spiDMAManager.attachCallback_TXDMA(spi_channel, boost::bind(&SPIClass::SPI_IRQHandler_TXDMA, this));

					SPI_DMA_EnableInterrupts(periph);

					SPI_PeriphState.dma_enabled_tx = true;
				}
				else if (periph == SubPeripheral::RX)
				{
					SPI_DMA_EnableClock();

					if (HAL_DMA_Init(&hdma_spi_rx) != HAL_OK)
						BasicErrorHandler(logError("Failed RX Modes::DMA Init. Check Init settings."));

					__HAL_LINKDMA(&spi_handle, hdmarx, hdma_spi_rx);

					spiDMAManager.attachCallback_RXDMA(spi_channel, boost::bind(&SPIClass::SPI_IRQHandler_RXDMA, this));

					SPI_DMA_EnableInterrupts(periph);

					SPI_PeriphState.dma_enabled_rx = true;
				}
			}
			
			void SPIClass::SPI_DMA_DeInit(const SubPeripheral& periph)
			{
				if (periph == SubPeripheral::TX)
				{
					if (!SPI_PeriphState.dma_enabled_tx)
						return;

					HAL_DMA_Abort(spi_handle.hdmatx);
					HAL_DMA_DeInit(spi_handle.hdmatx);
					SPI_DMA_DisableInterrupts(periph);
					spiDMAManager.removeCallback_TXDMA(spi_channel);

					SPI_PeriphState.dma_enabled_tx = false;
				}
				else if (periph == SubPeripheral::RX)
				{
					if (!SPI_PeriphState.dma_enabled_rx)
						return;

					HAL_DMA_Abort(spi_handle.hdmarx);
					HAL_DMA_DeInit(spi_handle.hdmarx);
					SPI_DMA_DisableInterrupts(periph);
					spiDMAManager.removeCallback_RXDMA(spi_channel);

					SPI_PeriphState.dma_enabled_rx = false;
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

					SPI_PeriphState.dma_interrupts_enabled_tx = true;
				}
				else if(periph == SubPeripheral::RX)
				{
					HAL_NVIC_DisableIRQ(ITSettings_DMA_RX.IRQn);
					HAL_NVIC_ClearPendingIRQ(ITSettings_DMA_RX.IRQn);
					HAL_NVIC_SetPriority(ITSettings_DMA_RX.IRQn, ITSettings_DMA_RX.preemptPriority, ITSettings_DMA_RX.subPriority);
					HAL_NVIC_EnableIRQ(ITSettings_DMA_RX.IRQn);

					SPI_PeriphState.dma_interrupts_enabled_rx = true;
				}
			}
			
			void SPIClass::SPI_DMA_DisableInterrupts(const SubPeripheral& periph)
			{
				if (periph == SubPeripheral::TX)
				{
					HAL_NVIC_ClearPendingIRQ(ITSettings_DMA_TX.IRQn);
					HAL_NVIC_DisableIRQ(ITSettings_DMA_TX.IRQn);
					
					SPI_PeriphState.dma_interrupts_enabled_tx = false;
				}
				else if(periph == SubPeripheral::RX)
				{
					HAL_NVIC_ClearPendingIRQ(ITSettings_DMA_RX.IRQn);
					HAL_NVIC_DisableIRQ(ITSettings_DMA_RX.IRQn);
					
					SPI_PeriphState.dma_interrupts_enabled_rx = false;
				}

			}


            ChimeraSPI::ChimeraSPI(const int& channel)
            {
                this->channel = channel;
                spi = SPIClass::create(channel);
            }

            Chimera::SPI::Status ChimeraSPI::init(const Chimera::SPI::Setup &setupStruct)
            {
                return Chimera::SPI::Status::SPI_OK;
            }

            Chimera::SPI::Status ChimeraSPI::writeBytes(uint8_t *const txBuffer, size_t length, const bool &disableCS)
            {
                return Chimera::SPI::Status::SPI_OK;
            }

            Chimera::SPI::Status ChimeraSPI::readBytes(uint8_t *const rxBuffer, size_t length, const bool &disableCS)
            {
                return Chimera::SPI::Status::SPI_OK;
            }

            Chimera::SPI::Status ChimeraSPI::readWriteBytes(uint8_t *const txBuffer, uint8_t *const rxBuffer, size_t length, const bool &disableCS)
            {
                //spi->write(txBuffer, rxBuffer, length, disableCS);
                return Chimera::SPI::Status::SPI_OK;
            }

            Chimera::SPI::Status ChimeraSPI::setPeripheralMode(Chimera::SPI::SubPeripheral periph, Chimera::SPI::SubPeripheralMode mode)
            {
                return Chimera::SPI::Status::SPI_OK;
            }

            Chimera::SPI::Status ChimeraSPI::setClockFrequency(uint32_t freq)
            {
                return Chimera::SPI::Status::SPI_OK;
            }

            Chimera::SPI::Status ChimeraSPI::getClockFrequency(uint32_t *const freq)
            {
                return Chimera::SPI::Status::SPI_OK;
            }

            Chimera::SPI::Status ChimeraSPI::reserve(uint32_t timeout_ms)
            {
                return Chimera::SPI::Status::SPI_OK;
            }

            Chimera::SPI::Status ChimeraSPI::release(uint32_t timeout_ms)
            {
                return Chimera::SPI::Status::SPI_OK;
            }

            Chimera::SPI::Status ChimeraSPI::onWriteComplete(Chimera::void_func_void func)
            {
                return Chimera::SPI::Status::SPI_OK;
            }

            Chimera::SPI::Status ChimeraSPI::onReadComplete(Chimera::void_func_void func)
            {
                return Chimera::SPI::Status::SPI_OK;
            }

            Chimera::SPI::Status ChimeraSPI::onReadWriteComplete(Chimera::void_func_void func)
            {
                return Chimera::SPI::Status::SPI_OK;
            }

            Chimera::SPI::Status ChimeraSPI::onError(Chimera::void_func_uint32_t func)
            {
                return Chimera::SPI::Status::SPI_OK;
            }

            Chimera::SPI::Status ChimeraSPI::attachEventWakeup(Chimera::FreeRTOS::Event event, SemaphoreHandle_t *const semphr)
            {
                return Chimera::SPI::Status::SPI_OK;
            }

            Chimera::SPI::Status ChimeraSPI::removeEventWakeup(SemaphoreHandle_t *const semphr)
            {
                return Chimera::SPI::Status::SPI_OK;
            }

            Chimera::SPI::Status ChimeraSPI::setChipSelect(Chimera::GPIO::State value)
            {
                return Chimera::SPI::Status::SPI_OK;
            }
        }
	}
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	/* Determine at runtime which object actually triggered this callback. Because this function
	 * resides in an interrupt, grab a constant reference so we don't take time instantiating a new shared_ptr. */
	const SPIClass_sPtr& spi = getSPIClassRef(hspi->Instance);

	if (spi && spi->_getInitStatus())
	{
		//spi->_setTXComplete();

        spi->tx_complete = true;

		if (!spi->_txBufferEmpty())
		{
			/*-------------------------------
			* Handle any operations on the completed TX
			*-------------------------------*/
			auto packet = spi->_txBufferNextPacket();

			if (packet.disableNSS && (spi->_getSSControlType() == SS_AUTOMATIC_CONTROL))
				spi->setChipSelect(LogicLevel::HIGH);

			spi->_txBufferRemoveFrontPacket();

			/*-------------------------------
			* If more data is left to send, queue it up.
			*-------------------------------*/
			if (!spi->_txBufferEmpty())
			{
				packet = spi->_txBufferNextPacket();

				if (packet.data_tx != nullptr)
					spi->writeBytes(packet.data_tx, packet.length, packet.disableNSS);

				/* Don't pop_front() yet because the options are needed for when TX is complete */
			}
			#if defined(USING_FREERTOS)
			else
				spiTaskTrigger.logEvent(BUFFERED_TX_COMPLETE, &spiTaskTrigger);
			#endif 
		}

		/* Signal any waiting threads */
		#if defined(USING_FREERTOS)
		spiTaskTrigger.logEvent(TX_COMPLETE, &spiTaskTrigger);
		#endif
	}
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	/* Determine at runtime which object actually triggered this callback. Because this function
	 * resides in an interrupt, grab a constant reference so we don't take time instantiating a new shared_ptr. */
	const SPIClass_sPtr& spi = getSPIClassRef(hspi->Instance);

	if (spi && spi->_getInitStatus())
	{
		spi->_setTXComplete();

		if (!spi->_txBufferEmpty())
		{
			/*-------------------------------
			* Handle any operations on the completed TX
			*-------------------------------*/
			auto packet = spi->_txBufferNextPacket();

			if (packet.disableNSS && (spi->_getSSControlType() == SS_AUTOMATIC_CONTROL))
				spi->setChipSelect(LogicLevel::HIGH);

			spi->_txBufferRemoveFrontPacket();

			/*-------------------------------
			* If more data is left to send, queue it up.
			*-------------------------------*/
			if (!spi->_txBufferEmpty())
			{
				packet = spi->_txBufferNextPacket();

				if (packet.data_tx != nullptr && packet.data_rx != nullptr)
					spi->readWriteBytes(packet.data_tx, packet.data_rx, packet.length, packet.disableNSS);

				/* Don't pop_front() yet because the options are needed for when TX is complete */
			}
			#if defined(USING_FREERTOS)
			else
				spiTaskTrigger.logEvent(BUFFERED_TXRX_COMPLETE, &spiTaskTrigger);
			#endif 
		}

		/* Signal any waiting threads */
		#if defined(USING_FREERTOS)
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
		spiObjects[1]->SPI_IRQHandler();
	}
}

void SPI2_IRQHandler()
{
	if (spiObjects[2])
	{
		spiObjects[2]->SPI_IRQHandler();
	}
}

void SPI3_IRQHandler()
{
	if (spiObjects[3])
	{
		spiObjects[3]->SPI_IRQHandler();
	}
}

void SPI4_IRQHandler()
{
	if (spiObjects[4])
	{
		spiObjects[4]->SPI_IRQHandler();
	}
}

void SPI5_IRQHandler()
{
	if (spiObjects[5])
	{
		spiObjects[5]->SPI_IRQHandler();
	}
}

void SPI6_IRQHandler()
{
	if (spiObjects[6])
	{
		spiObjects[6]->SPI_IRQHandler();
	}
}

