/* Boost Includes */
#include <boost/bind.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/static_vector.hpp>

/* Thor Includes */
#include <Thor/include/spi.hpp>
#include <Thor/include/exti.hpp>

using namespace Thor::Definitions;
using namespace Thor::Definitions::SPI;
using namespace Thor::Definitions::GPIO;
using namespace Thor::Defaults::SPI;

using namespace Thor::Peripheral::SPI;
using namespace Thor::Peripheral::GPIO;



/* Stores references to available SPIClass objects */
static boost::container::static_vector<SPIClass_sPtr, MAX_SPI_CHANNELS + 1> spiObjects(MAX_SPI_CHANNELS + 1);

/* Directly maps the HAL SPI Instance pointer to a possible SPIClass object */
static boost::container::flat_map<SPI_TypeDef*, SPIClass_sPtr> spiObjectLookup =
{
	#if defined(SPI1)
	{ SPI1, spiObjects[1] },	
	#endif 
	#if defined(SPI2)
	{ SPI2, spiObjects[2] },	
	#endif 
	#if defined(SPI3)
	{ SPI3, spiObjects[3] },	
	#endif 
	#if defined(SPI4)
	{ SPI4, spiObjects[4] },	
	#endif 
	#if defined(SPI5)
	{ SPI5, spiObjects[5] },	
	#endif 
	#if defined(SPI6)
	{ SPI6, spiObjects[6] }	
	#endif 
};

/* Directly maps the HAL SPI Instance pointer to the correct bit mask for enabling/disabling the peripheral clock  */
static boost::container::flat_map<SPI_TypeDef*, uint32_t> spiClockMask = 
{ 
	#if defined(TARGET_STM32F4) || defined(TARGET_STM32F7)
		#if defined (SPI1)
		{ SPI1, RCC_APB2ENR_SPI1EN },
		#endif
		
		#if defined (SPI2)
	    { SPI2, RCC_APB1ENR_SPI2EN },
		#endif
	
		#if defined (SPI3)
	    { SPI3, RCC_APB1ENR_SPI3EN },
		#endif
		
		#if defined (SPI4)
	    { SPI4, RCC_APB2ENR_SPI4EN },
		#endif
	#endif 
	
	
	#if defined(TARGET_STM32F7)
		#if defined (SPI5)
	    { SPI5, RCC_APB2ENR_SPI5EN },
		#endif
	
		#if defined (SPI6)
	    { SPI6, RCC_APB2ENR_SPI6EN },
		#endif
	#endif 
};

/* Directly maps the HAL SPI Instance pointer to the correct register for enabling/disabling the peripheral clock */
static boost::container::flat_map<SPI_TypeDef*, uint32_t> spiClockRegister =
{ 
	#if defined(TARGET_STM32F4) || defined(TARGET_STM32F7)
		#if defined (SPI1)
		{ SPI1, RCC->APB2ENR },
		#endif
		
		#if defined (SPI2)
		{ SPI2, RCC->APB1ENR },
		#endif
	
		#if defined (SPI3)
		{ SPI3, RCC->APB1ENR },
		#endif
		
		#if defined (SPI4)
		{ SPI4, RCC->APB2ENR },
		#endif
	#endif 
	
	
	#if defined(TARGET_STM32F7)
		#if defined (SPI5)
		{ SPI5, RCC->APB2ENR },
		#endif
	
		#if defined (SPI6)
		{ SPI6, RCC->APB2ENR },
		#endif
	#endif 
};


namespace Thor
{
	namespace Peripheral
	{
		namespace SPI
		{
			#ifdef USING_CHIMERA
			Chimera::SPI::Status SPIClass::init(int channel, const Chimera::SPI::Setup& setupStruct)
			{
				/* Inform the SPI shared_ptrs of what they actually point to. Requires that a
				 * reference to a shared_ptr<SPIClass> instance exists. This is created by 
				 * Chimera::SPI::SPIClass(int). 
				 */

				#ifdef ENABLE_SPI1
				if (channel == 1)
				{
					spi1 = getSharedPtrRef();
				}
				#endif 

				#ifdef ENABLE_SPI2
				if (channel == 2)
				{
					spi2 = getSharedPtrRef();
				}
				#endif 

				#ifdef ENABLE_SPI3
				if (channel == 3)
				{
					spi3 = getSharedPtrRef();
				}
				#endif 

				#ifdef ENABLE_SPI4
				if (channel == 4)
				{
					spi4 = getSharedPtrRef();
				}
				#endif 

				#ifdef ENABLE_SPI5
				if (channel == 5)
				{
					spi5 = getSharedPtrRef();
				}
				#endif 

				#ifdef ENABLE_SPI6
				if (channel == 6)
				{
					spi6 = getSharedPtrRef();
				}
				#endif 




				return Chimera::SPI::Status::SPI_OK;
			}

			Chimera::SPI::Status SPIClass::write(uint8_t* in, size_t length)
			{
				return Chimera::SPI::Status::SPI_OK;
			}

			Chimera::SPI::Status SPIClass::write(uint8_t* in, uint8_t* out, size_t length)
			{
				return Chimera::SPI::Status::SPI_OK;
			}

			Chimera::SPI::Status SPIClass::setTxMode(Chimera::SPI::TXRXMode mode)
			{
				return Chimera::SPI::Status::SPI_OK;
			}

			Chimera::SPI::Status SPIClass::setRxMode(Chimera::SPI::TXRXMode mode)
			{
				return Chimera::SPI::Status::SPI_OK;
			}

			#endif
			
			using namespace Thor::Definitions;
			
			
			using namespace Thor::Definitions::GPIO;

			using Status = Thor::Definitions::Status;
			using Modes = Thor::Definitions::Modes;
			using Options = Thor::Definitions::SPI::Options;
			

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

				if (ss_mode == SS_AUTOMATIC_CONTROL)
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
				SPI_DeInit();
				SPI_Init();
			}

			Status SPIClass::write(uint8_t* data_in, size_t length, Options options)
			{
				return this->write(data_in, nullptr, length, options);
			}

			Status SPIClass::write(uint8_t* data_in, uint8_t* data_out, size_t length, Options options)
			{
				if (!SPI_PeriphState.gpio_enabled || !SPI_PeriphState.spi_enabled)
					return Status::PERIPH_NOT_INITIALIZED;

				HAL_StatusTypeDef error;

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
								#if defined(USING_FREERTOS)
								error = HAL_SPI_Transmit(&spi_handle, data_in, length, pdMS_TO_TICKS(BLOCKING_TIMEOUT_MS));
								#else
								error = HAL_SPI_Transmit(&spi_handle, data_in, length, BLOCKING_TIMEOUT_MS);
								#endif
							else
								#if defined(USING_FREERTOS)
								error = HAL_SPI_TransmitReceive(&spi_handle, data_in, data_out, length, pdMS_TO_TICKS(BLOCKING_TIMEOUT_MS));
								#else
								error = HAL_SPI_TransmitReceive(&spi_handle, data_in, data_out, length, BLOCKING_TIMEOUT_MS);
								#endif
								
							if((options & SS_INACTIVE_AFTER_TX) == SS_INACTIVE_AFTER_TX)
								writeSS(LogicLevel::HIGH);
						}
						else
							error = HAL_SPI_TransmitReceive(&spi_handle, data_in, data_out, length, HAL_MAX_DELAY);

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

						/* Buffer only the slave select behavior for after packet has been transmitted */
						TX_tempPacket.data_tx = nullptr;
						TX_tempPacket.data_rx = nullptr;
						TX_tempPacket.length = 0;
						TX_tempPacket.options = options;
						TXPacketBuffer.push_back(TX_tempPacket);

						if (slaveSelectControl == SS_AUTOMATIC_CONTROL)
							writeSS(LogicLevel::LOW);
						
						if (data_out == nullptr)
							error = HAL_SPI_Transmit_IT(&spi_handle, data_in, length);
						else
							error = HAL_SPI_TransmitReceive_IT(&spi_handle, data_in, data_out, length);
						
						return Status::PERIPH_TX_IN_PROGRESS;
					}
					else
					{
						/* Busy. Buffer data */
						TX_tempPacket.data_tx = data_in;
						TX_tempPacket.data_rx = data_out;
						TX_tempPacket.length = length;
						TX_tempPacket.options = options;
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
						TX_tempPacket.options = options;
						TXPacketBuffer.push_back(TX_tempPacket);

						if (slaveSelectControl == SS_AUTOMATIC_CONTROL)
							writeSS(LogicLevel::LOW);
							
						if (data_out == nullptr)
							error = HAL_SPI_Transmit_DMA(&spi_handle, data_in, length);
						else
							error = HAL_SPI_TransmitReceive_DMA(&spi_handle, data_in, data_out, length);
							
						return Status::PERIPH_TX_IN_PROGRESS;
					}
					else
					{
						TX_tempPacket.data_tx = data_in;
						TX_tempPacket.data_rx = data_out;
						TX_tempPacket.length = length;
						TX_tempPacket.options = options;
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
				if (periph == SubPeripheral::TX)
				{
					switch (mode)
					{
					case Modes::BLOCKING:
						txMode = mode;   // Must be set before the other functions 
						
						/* Make sure RX side isn't using interrupts before disabling */
						if(rxMode == Modes::BLOCKING)
							SPI_DisableInterrupts();

						SPI_DMA_DeInit(periph);
						break;

					case Modes::INTERRUPT:
						txMode = mode;   // Must be set before the other functions 
						
						SPI_EnableInterrupts();
						SPI_DMA_DeInit(periph);
						break;

					case Modes::DMA:
						txMode = mode;   // Must be set before the other functions 
						
						SPI_EnableInterrupts();
						SPI_DMA_Init(periph);
						break;

					default:
						txMode = Modes::MODE_UNDEFINED;
						return Status::PERIPH_ERROR;
						break;
					}

					return Status::PERIPH_OK;
				}
				else
				{
					switch (mode)
					{
					case Modes::BLOCKING:
						rxMode = mode;   // Must be set before the other functions 
						
						/* Make sure TX side isn't using interrupts before disabling */
						if(txMode == Modes::BLOCKING)
							SPI_DisableInterrupts();

						SPI_DMA_DeInit(periph);
						
						break;

					case Modes::INTERRUPT:
						rxMode = mode;  	// Must be set before the other functions 
						
						SPI_EnableInterrupts();
						SPI_DMA_DeInit(periph);
						break;

					case Modes::DMA:
						rxMode = mode;   // Must be set before the other functions 
						
						SPI_EnableInterrupts();
						SPI_DMA_Init(periph);

						/* Set the idle line interrupt for asynchronously getting the end of transmission */
						//UART_EnableIT_IDLE(&uart_handle);
						
						/* Instruct the DMA hardware to start listening for transmissions */
						//HAL_UART_Receive_DMA(&uart_handle, RX_Queue[RXQueueIdx], UART_QUEUE_BUFFER_SIZE);
						break;

					default:
						rxMode = Modes::MODE_UNDEFINED;
						return Status::PERIPH_ERROR;
						break;
					}

					return Status::PERIPH_OK;
				}
			}
			
			void SPIClass::writeSS(LogicLevel state)
			{
				if (SlaveSelectType == EXTERNAL_SLAVE_SELECT && EXT_NSS_ATTACHED)
					EXT_NSS->write(state);
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
				spiClockRegister[spi_handle.Instance] |= spiClockMask[spi_handle.Instance];
			}

			void SPIClass::SPI_DisableClock()
			{
				spiClockRegister[spi_handle.Instance] &= ~spiClockMask[spi_handle.Instance];
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
				else
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
				else
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
				else
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
				else
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
	 * resides in an Modes::INTERRUPT, grab a constant reference so we don't take time instantiating a new shared_ptr. */
	const SPIClass_sPtr& spi = spiObjectLookup[hspi->Instance];

	if (spi && spi->_getInitStatus())
	{
		spi->_setTXComplete();

		if (!spi->_txBufferEmpty())
		{
			/*-------------------------------
			* Handle any operations on the completed TX
			*-------------------------------*/
			auto packet = spi->_txBufferNextPacket();

			if ((packet.options & SS_INACTIVE_AFTER_TX) == SS_INACTIVE_AFTER_TX &&
				(spi->_getSSControlType() == SS_AUTOMATIC_CONTROL))
				spi->writeSS(LogicLevel::HIGH);

			spi->_txBufferRemoveFrontPacket();

			/*-------------------------------
			* If more data is left to send, queue it up.
			*-------------------------------*/
			if (!spi->_txBufferEmpty())
			{
				packet = spi->_txBufferNextPacket();

				if (packet.data_tx != nullptr)
					spi->write(packet.data_tx, packet.length, packet.options);

				/* Don't pop_front() yet because the options are needed for when TX is complete */
			}
		}
	}
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	/* Determine at runtime which object actually triggered this callback. Because this function
	 * resides in an Modes::INTERRUPT, grab a constant reference so we don't take time instantiating a new shared_ptr. */
	const SPIClass_sPtr& spi = spiObjectLookup[hspi->Instance];

	if (spi && spi->_getInitStatus())
	{
		spi->_setTXComplete();

		if (!spi->_txBufferEmpty())
		{
			/*-------------------------------
			* Handle any operations on the completed TX
			*-------------------------------*/
			auto packet = spi->_txBufferNextPacket();

			if ((packet.options & SS_INACTIVE_AFTER_TX) == SS_INACTIVE_AFTER_TX &&
				(spi->_getSSControlType() == SS_AUTOMATIC_CONTROL))
				spi->writeSS(LogicLevel::HIGH);

			spi->_txBufferRemoveFrontPacket();

			/*-------------------------------
			* If more data is left to send, queue it up.
			*-------------------------------*/
			if (!spi->_txBufferEmpty())
			{
				packet = spi->_txBufferNextPacket();

				if (packet.data_tx != nullptr && packet.data_rx != nullptr)
					spi->write(packet.data_tx, packet.data_rx, packet.length, packet.options);

				/* Don't pop_front() yet because the options are needed for when TX is complete */
			}
		}

		/*------------------------------------
		* Signal Waiting Threads 
		*------------------------------------*/
		#if defined(USING_FREERTOS)
		
		/* Inform the semaphore task manager that RX is complete and data is 
		 * ready to be read out of the buffer. */
		//EXTI0_TaskMGR->logEventGenerator(SRC_SPI, spi_channel);
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