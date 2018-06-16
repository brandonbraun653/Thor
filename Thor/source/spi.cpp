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
static SemaphoreHandle_t spiSemphrs[MAX_SPI_CHANNELS + 1];
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
		clockError[i] = abs((busFreq / (1 << i + 1)) - freq);

	/* Find the index of the element with lowest error */
	auto idx = std::distance(clockError, std::min_element(clockError, clockError + numPrescalers - 1));

	switch (idx)
	{
	case 0:		return SPI_BAUDRATEPRESCALER_2;
	case 1:		return SPI_BAUDRATEPRESCALER_4;
	case 2:		return SPI_BAUDRATEPRESCALER_8;
	case 3:		return SPI_BAUDRATEPRESCALER_16;
	case 4:		return SPI_BAUDRATEPRESCALER_32;
	case 5:		return SPI_BAUDRATEPRESCALER_64;
	case 6:		return SPI_BAUDRATEPRESCALER_128;
	case 7:		return SPI_BAUDRATEPRESCALER_256;
	default:	return Thor::Defaults::SPI::dflt_SPI_Init.BaudRatePrescaler;
	};
};


#if defined(USING_CHIMERA)
/* Status Code conversion */
using ThorStatus = Thor::Definitions::Status;
using ChimStatus = Chimera::SPI::Status;
static ChimStatus thorStatusToChimera(ThorStatus thorStatus)
{
	switch (thorStatus)
	{
	case ThorStatus::PERIPH_OK:								return ChimStatus::SPI_OK;
	case ThorStatus::PERIPH_LOCKED:							return ChimStatus::SPI_LOCKED;
	case ThorStatus::PERIPH_NOT_INITIALIZED:				return ChimStatus::SPI_NOT_INITIALIZED;
	case ThorStatus::PERIPH_ERROR:							return ChimStatus::SPI_ERROR;
	case ThorStatus::PERIPH_NOT_READY:						return ChimStatus::SPI_NOT_READY;
	case ThorStatus::PERIPH_TX_IN_PROGRESS:					return ChimStatus::SPI_TX_IN_PROGRESS;
	case ThorStatus::PERIPH_RX_IN_PROGRESS:					return ChimStatus::SPI_RX_IN_PROGRESS;
	case ThorStatus::PERIPH_PACKET_TOO_LARGE_FOR_BUFFER:	return ChimStatus::SPI_PACKET_TOO_LARGE_FOR_BUFFER;
	case ThorStatus::PERIPH_TIMEOUT:						return ChimStatus::SPI_TIMEOUT;
	default:												return ChimStatus::SPI_UNKNOWN_STATUS_CODE;
	};
}

using ChimMode = Chimera::SPI::Mode;
static uint32_t chimeraModeToHAL(ChimMode mode)
{
	switch (mode)
	{
	case ChimMode::MASTER:		return SPI_MODE_MASTER;
	case ChimMode::SLAVE:		return SPI_MODE_SLAVE;
	default:					return Thor::Defaults::SPI::dflt_SPI_Init.Mode;
	};
};

using ChimOrder = Chimera::SPI::BitOrder;
static uint32_t chimeraBitOrderToHAL(ChimOrder order)
{
	switch (order)
	{
	case ChimOrder::MSB_FIRST:	return SPI_FIRSTBIT_MSB;
	case ChimOrder::LSB_FIRST:	return SPI_FIRSTBIT_LSB;
	default:					return Thor::Defaults::SPI::dflt_SPI_Init.FirstBit;
	};
};

using ChimClockMode = Chimera::SPI::ClockMode;
static uint32_t chimeraClkPhaseToHAL(ChimClockMode mode)
{
	switch (mode)
	{
	case ChimClockMode::MODE0:
	case ChimClockMode::MODE1:	return SPI_PHASE_1EDGE;
	case ChimClockMode::MODE2:
	case ChimClockMode::MODE3:	return SPI_PHASE_2EDGE;
	default:					return Thor::Defaults::SPI::dflt_SPI_Init.CLKPhase;
	};
};

static uint32_t chimeraClkPolarityToHAL(ChimClockMode mode)
{
	switch (mode)
	{
	case ChimClockMode::MODE0:
	case ChimClockMode::MODE1:	return SPI_POLARITY_LOW;
	case ChimClockMode::MODE2:
	case ChimClockMode::MODE3:	return SPI_POLARITY_HIGH;
	default:					return Thor::Defaults::SPI::dflt_SPI_Init.CLKPolarity;
	};
};

using ChimDataSize = Chimera::SPI::DataSize;
static uint32_t chimeraDataSizeToHAL(ChimDataSize size)
{
	switch (size)
	{
	case ChimDataSize::DATASIZE_8BIT:	return SPI_DATASIZE_8BIT;
	case ChimDataSize::DATASIZE_16BIT:	return SPI_DATASIZE_16BIT;
	default:							return Thor::Defaults::SPI::dflt_SPI_Init.DataSize;
	};
};

/* Setup Parameter Conversion */
using ChimSetup = Chimera::SPI::Setup;
static SPI_InitTypeDef chimeraSetupToThor(int channel, const ChimSetup& setup)
{
	SPI_InitTypeDef tmp;

	tmp.Mode				= chimeraModeToHAL(setup.mode);
	tmp.Direction			= Thor::Defaults::SPI::dflt_SPI_Init.Direction;
	tmp.DataSize			= chimeraDataSizeToHAL(setup.dataSize);
	tmp.CLKPolarity			= chimeraClkPolarityToHAL(setup.clockMode);
	tmp.CLKPhase			= chimeraClkPhaseToHAL(setup.clockMode);
	tmp.NSS					= Thor::Defaults::SPI::dflt_SPI_Init.NSS;
	tmp.BaudRatePrescaler	= getBaudRatePrescalerFromFreq(channel, setup.clockFrequency);
	tmp.FirstBit			= chimeraBitOrderToHAL(setup.bitOrder);
	tmp.TIMode				= Thor::Defaults::SPI::dflt_SPI_Init.TIMode;
	tmp.CRCCalculation		= Thor::Defaults::SPI::dflt_SPI_Init.CRCCalculation;
	tmp.CRCPolynomial		= Thor::Defaults::SPI::dflt_SPI_Init.CRCPolynomial;

	return tmp;
};

/* Sub Peripheral Type Conversions */
using ChimSubPeriph = Chimera::SPI::SubPeripheral;
using ThorSubPeriph = Thor::Definitions::SubPeripheral;
static ThorSubPeriph chimeraSubPeriphToThor(ChimSubPeriph periph)
{
	switch (periph)
	{
	case ChimSubPeriph::TX:		return ThorSubPeriph::TX;
	case ChimSubPeriph::RX:		return ThorSubPeriph::RX;
	case ChimSubPeriph::TXRX:	return ThorSubPeriph::TXRX;
	default:					return ThorSubPeriph::TXRX;
	};
};

/* Sub Peripheral Operation Mode Conversions */
using ChimSubPeriphMode = Chimera::SPI::SubPeripheralMode;
using ThorSubPeriphMode = Thor::Definitions::Modes;
static ThorSubPeriphMode chimeraSubPeriphModeToThor(ChimSubPeriphMode periphMode)
{
	switch (periphMode)
	{
	case ChimSubPeriphMode::BLOCKING:		return ThorSubPeriphMode::BLOCKING;
	case ChimSubPeriphMode::INTERRUPT:		return ThorSubPeriphMode::INTERRUPT;
	case ChimSubPeriphMode::DMA:			return ThorSubPeriphMode::DMA;
	case ChimSubPeriphMode::UNKOWN_MODE:	return ThorSubPeriphMode::MODE_UNDEFINED;
	default:								return ThorSubPeriphMode::MODE_UNDEFINED;
	};
};

#endif


namespace Thor
{
	namespace Peripheral
	{
		namespace SPI
		{
			#ifdef USING_CHIMERA
			ChimStatus SPIClass::cbegin(const ChimSetup& setupStruct)
			{
				SPI_InitTypeDef config = chimeraSetupToThor(spi_channel, setupStruct);
				attachSettings(config);
				begin();

				return thorStatusToChimera(Status::PERIPH_OK);
			}

			ChimStatus SPIClass::cwrite(uint8_t* in, size_t length, const bool& nssDisableAfterTX)
			{
				return thorStatusToChimera(write(in, length, nssDisableAfterTX));
			}

			ChimStatus SPIClass::cwrite(uint8_t* in, uint8_t* out, size_t length, const bool& nssDisableAfterTX)
			{
				return thorStatusToChimera(write(in, out, length, nssDisableAfterTX));
			}

			ChimStatus SPIClass::csetMode(ChimSubPeriph periph, ChimSubPeriphMode mode)
			{
				auto p = chimeraSubPeriphToThor(periph);
				auto m = chimeraSubPeriphModeToThor(mode);

				return thorStatusToChimera(setMode(p, m));
			}

			ChimStatus SPIClass::cupdateClockFrequency(uint32_t freq)
			{
				return thorStatusToChimera(updateClockFrequency(freq));
			}

			#endif
			
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

			SPIClass::SPIClass(int channel)
			{
				spi_channel = channel;
				spi_handle.Instance = spi_cfg[spi_channel].instance;

				/*------------------------------------
				 * Interrupt Setup
				 *-----------------------------------*/
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
				 * DMA Setup
				 *-----------------------------------*/
				spi_handle.Init = Defaults::SPI::dflt_SPI_Init;
				hdma_spi_tx.Init = Defaults::SPI::dflt_DMA_Init_TX;
				hdma_spi_rx.Init = Defaults::SPI::dflt_DMA_Init_RX;

				hdma_spi_tx.Init.Channel = spi_cfg[spi_channel].dmaTX.channel;
				hdma_spi_rx.Init.Channel = spi_cfg[spi_channel].dmaRX.channel;

				hdma_spi_tx.Instance = spi_cfg[spi_channel].dmaTX.Instance;
				hdma_spi_rx.Instance = spi_cfg[spi_channel].dmaRX.Instance;

				/*------------------------------------
				 * GPIO Setup
				 *-----------------------------------*/
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
				 * Buffer Setup
				 *-----------------------------------*/
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

			Status SPIClass::write(uint8_t* data_in, size_t length, const bool& nssDisableAfterTX)
			{
				return this->write(data_in, nullptr, length, nssDisableAfterTX);
			}

			Status SPIClass::write(uint8_t* data_in, uint8_t* data_out, size_t length, const bool& nssDisableAfterTX)
			{
				if (!SPI_PeriphState.gpio_enabled || !SPI_PeriphState.spi_enabled)
					return Status::PERIPH_NOT_INITIALIZED;

				volatile HAL_StatusTypeDef error = HAL_OK;

				switch (txMode)
				{
				case Modes::BLOCKING:
					if (tx_complete)
					{
						tx_complete = false;
						if (slaveSelectControl == SS_AUTOMATIC_CONTROL)
						{
							writeSS(LogicLevel::LOW);
							
							if (data_out == nullptr)
							{
								#if defined(USING_FREERTOS)
								error = HAL_SPI_Transmit(&spi_handle, data_in, length, pdMS_TO_TICKS(BLOCKING_TIMEOUT_MS));
								#else
								error = HAL_SPI_Transmit(&spi_handle, data_in, length, BLOCKING_TIMEOUT_MS);
								#endif
							}
							else
							{
								/* In order for TransmitReceive to work, both subperipherals must be in the same mode. This will
								 * silently clobber whatever settings were previously there. */
								if (!txRxModesEqual(txMode))
									setMode(SubPeripheral::RX, txMode);
								
								#if defined(USING_FREERTOS)
								error = HAL_SPI_TransmitReceive(&spi_handle, data_in, data_out, length, pdMS_TO_TICKS(BLOCKING_TIMEOUT_MS));
								#else
								error = HAL_SPI_TransmitReceive(&spi_handle, data_in, data_out, length, BLOCKING_TIMEOUT_MS);
								#endif
							}
								
							if (nssDisableAfterTX)
								writeSS(LogicLevel::HIGH);
						}
						else
						{
							if (data_out == nullptr)
							{
								#if defined(USING_FREERTOS)
								error = HAL_SPI_Transmit(&spi_handle, data_in, length, pdMS_TO_TICKS(BLOCKING_TIMEOUT_MS));
								#else
								error = HAL_SPI_Transmit(&spi_handle, data_in, length, BLOCKING_TIMEOUT_MS);
								#endif
							}
							else
							{
								/* In order for TransmitReceive to work, both subperipherals must be in the same mode. This will
								 * silently clobber whatever settings were previously there. */
								if (!txRxModesEqual(txMode))
									setMode(SubPeripheral::RX, txMode);
								
								#if defined(USING_FREERTOS)
								error = HAL_SPI_TransmitReceive(&spi_handle, data_in, data_out, length, pdMS_TO_TICKS(BLOCKING_TIMEOUT_MS));
								#else
								error = HAL_SPI_TransmitReceive(&spi_handle, data_in, data_out, length, BLOCKING_TIMEOUT_MS);
								#endif
							}
						}
							
						tx_complete = true;

						return Status::PERIPH_READY;
					}
					else
						return Status::PERIPH_TX_IN_PROGRESS;
					break;

				case Modes::INTERRUPT:
					if (tx_complete)
					{
						tx_complete = false;

						TX_tempPacket.data_tx = nullptr;
						TX_tempPacket.data_rx = nullptr;
						TX_tempPacket.length = 0;
						TX_tempPacket.disableNSS = nssDisableAfterTX;
						TXPacketBuffer.push_back(TX_tempPacket);

						if (slaveSelectControl == SS_AUTOMATIC_CONTROL)
							writeSS(LogicLevel::LOW);
						
						if (data_out == nullptr)
							error = HAL_SPI_Transmit_IT(&spi_handle, data_in, length);
						else
						{
							/* In order for TransmitReceive to work, both subperipherals must be in the same mode. This will
							 * silently clobber whatever settings were previously there. */
							if (!txRxModesEqual(txMode))
								setMode(SubPeripheral::RX, txMode);
							
							error = HAL_SPI_TransmitReceive_IT(&spi_handle, data_in, data_out, length);
						}
						
						return Status::PERIPH_TX_IN_PROGRESS;
					}
					else
					{
						/* Hardware is currently busy. Buffer the data packet. */
						TX_tempPacket.data_tx = data_in;
						TX_tempPacket.data_rx = data_out;
						TX_tempPacket.length = length;
						TX_tempPacket.disableNSS = nssDisableAfterTX;
						TXPacketBuffer.push_back(TX_tempPacket);
						
						return Status::PERIPH_NOT_READY;
					}
					break;

				case Modes::DMA:
					if (tx_complete)
					{
						tx_complete = false;

						TX_tempPacket.data_tx = nullptr;
						TX_tempPacket.length = 0;
						TX_tempPacket.disableNSS = nssDisableAfterTX;
						TXPacketBuffer.push_back(TX_tempPacket);

						if (slaveSelectControl == SS_AUTOMATIC_CONTROL)
							writeSS(LogicLevel::LOW);
							
						if (data_out == nullptr)
							error = HAL_SPI_Transmit_DMA(&spi_handle, data_in, length);
						else
						{
							/* In order for TransmitReceive to work, both subperipherals must be in the same mode. This will
							 * silently clobber whatever settings were previously there. */
							if (!txRxModesEqual(txMode))
								setMode(SubPeripheral::RX, txMode);
							
							error = HAL_SPI_TransmitReceive_DMA(&spi_handle, data_in, data_out, length);
						}
							
						return Status::PERIPH_TX_IN_PROGRESS;
					}
					else
					{
						/* Hardware is currently busy. Buffer the data packet. */
						TX_tempPacket.data_tx = data_in;
						TX_tempPacket.data_rx = data_out;
						TX_tempPacket.length = length;
						TX_tempPacket.disableNSS = nssDisableAfterTX;
						TXPacketBuffer.push_back(TX_tempPacket);

						return Status::PERIPH_NOT_READY;
					}
					break;

				default: 
					return Status::PERIPH_ERROR;
					break;
				}
				return Status::PERIPH_OK;
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
						txMode = mode;   // Must be set before the other functions 
						
						/* Make sure RX side isn't using interrupts before disabling */
						if(rxMode == Modes::BLOCKING)
							SPI_DisableInterrupts();

						SPI_DMA_DeInit(SubPeripheral::TX);
						break;

					case Modes::INTERRUPT:
						txMode = mode;   // Must be set before the other functions 
						
						SPI_EnableInterrupts();
						SPI_DMA_DeInit(SubPeripheral::TX);
						break;

					case Modes::DMA:
						txMode = mode;   // Must be set before the other functions 
						
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
						rxMode = mode;  // Must be set before the other functions 
						
						/* Make sure TX side isn't using interrupts before disabling */
						if(txMode == Modes::BLOCKING)
							SPI_DisableInterrupts();

						SPI_DMA_DeInit(SubPeripheral::RX);
						break;

					case Modes::INTERRUPT:
						rxMode = mode;  // Must be set before the other functions 
						
						SPI_EnableInterrupts();
						SPI_DMA_DeInit(SubPeripheral::RX);
						break;

					case Modes::DMA:
						rxMode = mode;  // Must be set before the other functions 
						
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
			
			void SPIClass::writeSS(LogicLevel state)
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

					spi_dma_manager.attachCallbackFunction_TXDMA(spi_channel, boost::bind(&SPIClass::SPI_IRQHandler_TXDMA, this));

					SPI_DMA_EnableInterrupts(periph);

					SPI_PeriphState.dma_enabled_tx = true;
				}
				else if (periph == SubPeripheral::RX)
				{
					SPI_DMA_EnableClock();

					if (HAL_DMA_Init(&hdma_spi_rx) != HAL_OK)
						BasicErrorHandler(logError("Failed RX Modes::DMA Init. Check Init settings."));

					__HAL_LINKDMA(&spi_handle, hdmarx, hdma_spi_rx);

					spi_dma_manager.attachCallbackFunction_RXDMA(spi_channel, boost::bind(&SPIClass::SPI_IRQHandler_RXDMA, this));

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
					spi_dma_manager.removeCallbackFunction_TXDMA(spi_channel);

					SPI_PeriphState.dma_enabled_tx = false;
				}
				else if (periph == SubPeripheral::RX)
				{
					if (!SPI_PeriphState.dma_enabled_rx)
						return;

					HAL_DMA_Abort(spi_handle.hdmarx);
					HAL_DMA_DeInit(spi_handle.hdmarx);
					SPI_DMA_DisableInterrupts(periph);
					spi_dma_manager.removeCallbackFunction_RXDMA(spi_channel);

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
		spi->_setTXComplete();

		if (!spi->_txBufferEmpty())
		{
			/*-------------------------------
			* Handle any operations on the completed TX
			*-------------------------------*/
			auto packet = spi->_txBufferNextPacket();

			if (packet.disableNSS && (spi->_getSSControlType() == SS_AUTOMATIC_CONTROL))
				spi->writeSS(LogicLevel::HIGH);

			spi->_txBufferRemoveFrontPacket();

			/*-------------------------------
			* If more data is left to send, queue it up.
			*-------------------------------*/
			if (!spi->_txBufferEmpty())
			{
				packet = spi->_txBufferNextPacket();

				if (packet.data_tx != nullptr)
					spi->write(packet.data_tx, packet.length, packet.disableNSS);

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
				spi->writeSS(LogicLevel::HIGH);

			spi->_txBufferRemoveFrontPacket();

			/*-------------------------------
			* If more data is left to send, queue it up.
			*-------------------------------*/
			if (!spi->_txBufferEmpty())
			{
				packet = spi->_txBufferNextPacket();

				if (packet.data_tx != nullptr && packet.data_rx != nullptr)
					spi->write(packet.data_tx, packet.data_rx, packet.length, packet.disableNSS);

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

