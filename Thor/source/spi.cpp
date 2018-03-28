#include "../include/spi.h"
#include "../include/exti.h"

using namespace ThorDef::SPI;
using namespace ThorDef::GPIO;
using namespace Defaults::SPI;
using namespace Interrupt;

/************************************************************************/
/*					  Local Object Instantiations                       */
/************************************************************************/
#ifdef ENABLE_SPI1
SPIClass_sPtr spi1 = boost::make_shared<SPIClass>(1);

void SPI1_IRQHandler()
{
	spi1->SPI_IRQHandler();
}
#endif
#ifdef ENABLE_SPI2
SPIClass_sPtr spi2 = boost::make_shared<SPIClass>(2);

void SPI2_IRQHandler()
{
	spi2->SPI_IRQHandler();
}
#endif

#ifdef ENABLE_SPI3
SPIClass_sPtr spi3 = boost::make_shared<SPIClass>(3);

void SPI3_IRQHandler()
{
	spi3->SPI_IRQHandler();
}
#endif

#ifdef ENABLE_SPI4
SPIClass_sPtr spi4 = boost::make_shared<SPIClass>(4);

void SPI4_IRQHandler()
{
	spi4->SPI_IRQHandler();
}
#endif

#ifdef ENABLE_SPI5
SPIClass_sPtr spi5 = boost::make_shared<SPIClass>(5);

void SPI5_IRQHandler()
{
	spi5->SPI_IRQHandler();
}
#endif

#ifdef ENABLE_SPI6
SPIClass_sPtr spi6 = boost::make_shared<SPIClass>(6);

void SPI6_IRQHandler()
{
	spi6->SPI_IRQHandler();
}
#endif

/************************************************************************/
/*						    Interrupt Handlers                          */
/************************************************************************/
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

/************************************************************************/
/*			           Constructors/Deconstructors                      */
/************************************************************************/
SPIClass::SPIClass(int channel)
{
	/* Initialize some variable states */
	spi_channel = channel;
	txMode = TX_MODE_NONE;
	rxMode = RX_MODE_NONE;

	isInitialized = &SPI_PeriphState.spi_enabled;

	SPI_PeriphState.gpio_enabled = false;
	SPI_PeriphState.spi_enabled = false;
	SPI_PeriphState.spi_interrupts_enabled = false;
	SPI_PeriphState.dma_enabled_tx = false;
	SPI_PeriphState.dma_enabled_rx = false;
	SPI_PeriphState.dma_interrupts_enabled_tx = false;
	SPI_PeriphState.dma_interrupts_enabled_rx = false;

	SlaveSelectControl = SS_AUTOMATIC_CONTROL;

	/* Assign the instance */
	spi_handle.Instance = spi_cfg[spi_channel].instance;

	/* Copy over interrupt info */
	ITSettingsHW.IRQn				= spi_cfg[spi_channel].IT_HW.IRQn;
	ITSettingsHW.groupPriority		= spi_cfg[spi_channel].IT_HW.groupPriority;
	ITSettingsHW.preemptPriority	= spi_cfg[spi_channel].IT_HW.preemptPriority;
	ITSettingsHW.subPriority		= spi_cfg[spi_channel].IT_HW.subPriority;

	ITSettings_DMA_TX.IRQn				= spi_cfg[spi_channel].dmaIT_TX.IRQn;
	ITSettings_DMA_TX.groupPriority		= spi_cfg[spi_channel].dmaIT_TX.groupPriority;
	ITSettings_DMA_TX.preemptPriority	= spi_cfg[spi_channel].dmaIT_TX.preemptPriority;
	ITSettings_DMA_TX.subPriority		= spi_cfg[spi_channel].dmaIT_TX.subPriority;

	ITSettings_DMA_RX.IRQn = spi_cfg[spi_channel].dmaIT_RX.IRQn;
	ITSettings_DMA_RX.groupPriority = spi_cfg[spi_channel].dmaIT_RX.groupPriority;
	ITSettings_DMA_RX.preemptPriority = spi_cfg[spi_channel].dmaIT_RX.preemptPriority;
	ITSettings_DMA_RX.subPriority = spi_cfg[spi_channel].dmaIT_RX.subPriority;

	/* Use the default HW configuration and modify a few values */
	spi_handle.Init = Defaults::SPI::dflt_SPI_Init;
	hdma_spi_tx.Init = Defaults::SPI::dflt_DMA_Init_TX;
	hdma_spi_rx.Init = Defaults::SPI::dflt_DMA_Init_RX;

	hdma_spi_tx.Init.Channel = spi_cfg[spi_channel].dmaTX.channel;
	hdma_spi_rx.Init.Channel = spi_cfg[spi_channel].dmaRX.channel;

	hdma_spi_tx.Instance = spi_cfg[spi_channel].dmaTX.Instance;
	hdma_spi_rx.Instance = spi_cfg[spi_channel].dmaRX.Instance;

	/* Create the GPIO pin objects */
	MOSI = boost::make_shared<GPIOClass>(
		spi_cfg[spi_channel].MOSI.GPIOx,
		spi_cfg[spi_channel].MOSI.PinNum,
		spi_cfg[spi_channel].MOSI.Speed,
		spi_cfg[spi_channel].MOSI.Alternate);

	MISO = boost::make_shared<GPIOClass>(
		spi_cfg[spi_channel].MISO.GPIOx,
		spi_cfg[spi_channel].MISO.PinNum,
		spi_cfg[spi_channel].MISO.Speed,
		spi_cfg[spi_channel].MISO.Alternate);

	SCK = boost::make_shared<GPIOClass>(
		spi_cfg[spi_channel].SCK.GPIOx,
		spi_cfg[spi_channel].SCK.PinNum,
		spi_cfg[spi_channel].SCK.Speed,
		spi_cfg[spi_channel].SCK.Alternate);

	NSS = boost::make_shared<GPIOClass>(
		spi_cfg[spi_channel].NSS.GPIOx,
		spi_cfg[spi_channel].NSS.PinNum,
		spi_cfg[spi_channel].NSS.Speed,
		spi_cfg[spi_channel].NSS.Alternate);

	/* Initialize the buffer memories */
	TXPacketBuffer.set_capacity(ThorDef::SPI::SPI_BUFFER_SIZE);

	rxBufferedPackets = new SmartBuffer::RingBuffer<uint16_t>(_rxbuffpckt, ThorDef::SPI::SPI_BUFFER_SIZE);
	rxBufferedPacketLengths = new SmartBuffer::RingBuffer<size_t>(_rxbuffpcktlen, ThorDef::SPI::SPI_BUFFER_SIZE);


	/* SUPER HACKY QUICK FIX FOR DRONE SD LOGGER BUG */
	internalTXBuffer = new uint8_t(512u);
}

SPIClass::~SPIClass()
{
}

/************************************************************************/
/*						     Public Functions                           */
/************************************************************************/
void SPIClass::begin(SPIOptions options)
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

	setTxModeBlock();
	setRxModeBlock();
}

void SPIClass::attachPin(boost::shared_ptr<GPIOClass> slave_select)
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

void SPIClass::setSSMode(ThorDef::SPI::SPIOptions ss_mode)
{
	if (ss_mode == SS_MANUAL_CONTROL)
		SlaveSelectControl = SS_MANUAL_CONTROL;

	if (ss_mode == SS_AUTOMATIC_CONTROL)
		SlaveSelectControl = SS_AUTOMATIC_CONTROL;
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

SPIStatus SPIClass::write(uint8_t* val, size_t length, SPIOptions options)
{
	if (!SPI_PeriphState.gpio_enabled || !SPI_PeriphState.spi_enabled)
		return SPI_NOT_INITIALIZED;

	HAL_StatusTypeDef errorCode = HAL_OK;

	switch (txMode)
	{
	case TX_MODE_BLOCKING:
		if (tx_complete)
		{
			tx_complete = false;

			if (SlaveSelectControl == SS_AUTOMATIC_CONTROL)
			{
				writeSS(LOW);
				errorCode = HAL_SPI_Transmit(&spi_handle, val, length, HAL_MAX_DELAY);

				if ((options & SS_INACTIVE_AFTER_TX) == SS_INACTIVE_AFTER_TX)
					writeSS(HIGH);
			}
			else
				HAL_SPI_Transmit(&spi_handle, val, length, HAL_MAX_DELAY);

			tx_complete = true;
			return SPI_READY;
		}
		else
			return SPI_TX_BUSY;
		break;

	case TX_MODE_INTERRUPT:
	case TXRX_MODE_INTERRUPT:
		if (SPI_PeriphState.spi_interrupts_enabled)
		{
			if (tx_complete)
			{
				/* TX hardware is free. Go ahead and send data.*/
				tx_complete = false;

				TX_tempPacket.data_tx = nullptr;
				TX_tempPacket.data_rx = nullptr;
				TX_tempPacket.length = 0;
				TX_tempPacket.options = options;
				TXPacketBuffer.push_back(TX_tempPacket);

				if (SlaveSelectControl == SS_AUTOMATIC_CONTROL)
					writeSS(LOW);

				errorCode = HAL_SPI_Transmit_IT(&spi_handle, val, length);
				return SPI_TX_BUSY;
			}
			else
			{
				TX_tempPacket.data_tx = val;
				TX_tempPacket.data_rx = nullptr;
				TX_tempPacket.length = length;
				TX_tempPacket.options = options;
				TXPacketBuffer.push_back(TX_tempPacket);

				return SPI_NOT_READY;
			}
		}
		else
			return SPI_ERROR;
		break;

	case TX_MODE_DMA:
	case TXRX_MODE_DMA:
		if (SPI_PeriphState.dma_enabled_tx)
		{
			if (tx_complete)
			{
				tx_complete = false;

				TX_tempPacket.data_tx = nullptr;
				TX_tempPacket.data_rx = nullptr;
				TX_tempPacket.length = 0;
				TX_tempPacket.options = options;
				TXPacketBuffer.push_back(TX_tempPacket);

				if (SlaveSelectControl == SS_AUTOMATIC_CONTROL)
					writeSS(LOW);

				errorCode = HAL_SPI_Transmit_DMA(&spi_handle, val, length);
				return SPI_TX_BUSY;
			}
			else
			{
				TX_tempPacket.data_tx = val;
				TX_tempPacket.data_rx = nullptr;
				TX_tempPacket.length = length;
				TX_tempPacket.options = options;
				TXPacketBuffer.push_back(TX_tempPacket);

				return SPI_NOT_READY;
			}
		}
		else
			return SPI_ERROR;
		break;

	default: break;
	}
}

SPIStatus SPIClass::write(uint8_t* val_in, uint8_t* val_out, size_t length, SPIOptions options)
{
	if (!SPI_PeriphState.gpio_enabled || !SPI_PeriphState.spi_enabled)
		return SPI_NOT_INITIALIZED;

	/* SUPER HACKY QUICK FIX FOR DRONE SD LOGGER BUG */
	//memcpy(internalTXBuffer, val_in, length);


	HAL_StatusTypeDef error;

	switch (txMode)
	{
	case TX_MODE_BLOCKING:
		if (tx_complete)
		{
			tx_complete = false;
			if (SlaveSelectControl == SS_AUTOMATIC_CONTROL)
			{
				writeSS(LOW);

				error = HAL_SPI_TransmitReceive(&spi_handle, val_in, val_out, length, HAL_MAX_DELAY);
				//error = HAL_SPI_TransmitReceive(&spi_handle, internalTXBuffer, val_out, length, HAL_MAX_DELAY);

				if ((options & SS_INACTIVE_AFTER_TX) == SS_INACTIVE_AFTER_TX)
					writeSS(HIGH);
			}
			else
				//error = HAL_SPI_TransmitReceive(&spi_handle, internalTXBuffer, val_out, length, HAL_MAX_DELAY);
				error = HAL_SPI_TransmitReceive(&spi_handle, val_in, val_out, length, HAL_MAX_DELAY);
				
			tx_complete = true;
			
			return SPI_READY;
		}
		else
			return SPI_TX_BUSY;
		break;

	case TX_MODE_INTERRUPT:
	case TXRX_MODE_INTERRUPT:
		if (SPI_PeriphState.spi_interrupts_enabled)
		{
			if (tx_complete)
			{
				/* TX hardware is free. Go ahead and send data.*/
				tx_complete = false;

				/* Buffer only the SS option for after packet has been transmitted */
				TX_tempPacket.data_tx = nullptr;
				TX_tempPacket.data_rx = nullptr;
				TX_tempPacket.length = 0;
				TX_tempPacket.options = options;
				TXPacketBuffer.push_back(TX_tempPacket);

				if (SlaveSelectControl == SS_AUTOMATIC_CONTROL)
					writeSS(LOW);

				HAL_SPI_TransmitReceive_IT(&spi_handle, val_in, val_out, length);
				return SPI_TX_BUSY;
			}
			else
			{
				/* Busy. Buffer data */
				TX_tempPacket.data_tx = val_in;
				TX_tempPacket.data_rx = val_out;
				TX_tempPacket.length = length;
				TX_tempPacket.options = options;
				TXPacketBuffer.push_back(TX_tempPacket);
				return SPI_NOT_READY;
			}
		}
		else
			return SPI_ERROR;
		break;

	case TX_MODE_DMA:
	case TXRX_MODE_DMA:
		if (SPI_PeriphState.dma_enabled_tx)
		{
			if (tx_complete)
			{
				tx_complete = false;

				TX_tempPacket.data_tx = nullptr;
				TX_tempPacket.length = 0;
				TX_tempPacket.options = options;
				TXPacketBuffer.push_back(TX_tempPacket);

				if (SlaveSelectControl == SS_AUTOMATIC_CONTROL)
					writeSS(LOW);

				HAL_SPI_TransmitReceive_DMA(&spi_handle, val_in, val_out, length);
				return SPI_TX_BUSY;
			}
			else
			{
				TX_tempPacket.data_tx = val_in;
				TX_tempPacket.data_rx = val_out;
				TX_tempPacket.length = length;
				TX_tempPacket.options = options;
				TXPacketBuffer.push_back(TX_tempPacket);

				return SPI_NOT_READY;
			}
		}
		else
			return SPI_ERROR;
		break;

	default: break;
	}
}

void SPIClass::end()
{
	SPI_GPIO_DeInit();
	SPI_DisableInterrupts();
	SPI_DMA_DeInit_TX();
	SPI_DMA_DeInit_RX();

	txMode = TX_MODE_NONE;
	rxMode = RX_MODE_NONE;
}

void SPIClass::setTxModeBlock()
{
	txMode = TX_MODE_BLOCKING;

	if (rxMode == TX_MODE_BLOCKING)
		SPI_DisableInterrupts();

	SPI_DMA_DeInit_TX();
}

void SPIClass::setTxModeInterrupt()
{
	txMode = TX_MODE_INTERRUPT;

	SPI_EnableInterrupts();
	SPI_DMA_DeInit_TX();
}

void SPIClass::setTxModeDMA()
{
	txMode = TX_MODE_DMA;

	SPI_EnableInterrupts();
	SPI_DMA_Init_TX();
}

void SPIClass::setRxModeBlock()
{
	rxMode = RX_MODE_BLOCKING;

	if (txMode == TX_MODE_BLOCKING)
		SPI_DisableInterrupts();

	SPI_DMA_DeInit_RX();
}

void SPIClass::setRxModeInterrupt()
{
	rxMode = RX_MODE_INTERRUPT;

	SPI_EnableInterrupts();
	SPI_DMA_DeInit_RX();
}

void SPIClass::setRxModeDMA()
{
	rxMode = RX_MODE_INTERRUPT;

	SPI_EnableInterrupts();
	SPI_DMA_Init_TX();
}

void SPIClass::setTxRxModeBlock()
{
	txMode = TXRX_MODE_BLOCKING;
	rxMode = TXRX_MODE_BLOCKING;

	SPI_DisableInterrupts();
	SPI_DMA_DeInit_TX();
	SPI_DMA_DeInit_RX();
}

void SPIClass::setTxRxModeInterrupt()
{
	txMode = TXRX_MODE_INTERRUPT;
	rxMode = TXRX_MODE_INTERRUPT;

	SPI_EnableInterrupts();
	SPI_DMA_DeInit_TX();
	SPI_DMA_DeInit_RX();
}

void SPIClass::setTxRxModeDMA()
{
	txMode = TXRX_MODE_DMA;
	rxMode = TXRX_MODE_DMA;

	SPI_EnableInterrupts();
	SPI_DMA_Init_TX();
	SPI_DMA_Init_RX();
}

void SPIClass::writeSS(LogicLevel state)
{
	if (SlaveSelectType == EXTERNAL_SLAVE_SELECT && EXT_NSS_ATTACHED)
		EXT_NSS->write(state);
}

void SPIClass::SPI_Init()
{
	SPI_EnableClock(spi_channel);

	if (HAL_SPI_Init(&spi_handle) != HAL_OK)
		BasicErrorHandler(logError("Failed SPI Init"));

	SPI_PeriphState.spi_enabled = true;
}

void SPIClass::SPI_DeInit()
{
	if (HAL_SPI_DeInit(&spi_handle) != HAL_OK)
		BasicErrorHandler(logError("Failed SPI DeInit"));

	SPI_DisableClock(spi_channel);

	SPI_PeriphState.spi_enabled = false;
}

void SPIClass::SPI_EnableClock(int channel)
{
	#ifdef ENABLE_SPI1
	if (channel == 1)
		__SPI1_CLK_ENABLE();
	#endif
	#ifdef ENABLE_SPI2
	if (channel == 2)
		__SPI2_CLK_ENABLE();
	#endif

	#ifdef ENABLE_SPI3
	if (channel == 3)
		__SPI3_CLK_ENABLE();
	#endif

	#ifdef ENABLE_SPI4
	if (channel == 4)
		__SPI4_CLK_ENABLE();
	#endif

	#ifdef ENABLE_SPI5
	if (channel == 5)
		__SPI5_CLK_ENABLE();
	#endif

	#ifdef ENABLE_SPI6
	if (channel == 6)
		__SPI6_CLK_ENABLE();
	#endif
}

void SPIClass::SPI_DisableClock(int channel)
{
	#ifdef ENABLE_SPI1
	if (channel == 1)
		__SPI1_CLK_DISABLE();
	#endif
	#ifdef ENABLE_SPI2
	if (channel == 2)
		__SPI2_CLK_DISABLE();
	#endif

	#ifdef ENABLE_SPI3
	if (channel == 3)
		__SPI3_CLK_DISABLE();
	#endif

	#ifdef ENABLE_SPI4
	if (channel == 4)
		__SPI4_CLK_DISABLE();
	#endif

	#ifdef ENABLE_SPI5
	if (channel == 5)
		__SPI5_CLK_DISABLE();
	#endif

	#ifdef ENABLE_SPI6
	if (channel == 6)
		__SPI6_CLK_DISABLE();
	#endif
}

void SPIClass::SPI_EnableInterrupts()
{
	HAL_NVIC_DisableIRQ(ITSettingsHW.IRQn);
	HAL_NVIC_SetPriorityGrouping(ITSettingsHW.groupPriority);
	HAL_NVIC_SetPriority(ITSettingsHW.IRQn, ITSettingsHW.preemptPriority, ITSettingsHW.subPriority);
	HAL_NVIC_EnableIRQ(ITSettingsHW.IRQn);

	/* Specific interrupts to enable */
	if (rxMode == RX_MODE_INTERRUPT)
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
			EXT_NSS->write(HIGH);
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
	//If there ever is a GPIOClass deinit function, use it.
}

void SPIClass::SPI_DMA_Init_TX()
{
	SPI_DMA_EnableClock();

	if (HAL_DMA_Init(&hdma_spi_tx) != HAL_OK)
		BasicErrorHandler(logError("Failed TX DMA Init"));

	__HAL_LINKDMA(&spi_handle, hdmatx, hdma_spi_tx);

	spi_dma_manager.attachCallbackFunction_TXDMA(spi_channel, boost::bind(&SPIClass::SPI_IRQHandler_TXDMA, this));

	SPI_DMA_EnableInterrupts_TX();

	SPI_PeriphState.dma_enabled_tx = true;
}

void SPIClass::SPI_DMA_Init_RX()
{
	SPI_DMA_EnableClock();

	if (HAL_DMA_Init(&hdma_spi_rx) != HAL_OK)
		BasicErrorHandler(logError("Failed RX DMA Init. Check Init settings."));

	__HAL_LINKDMA(&spi_handle, hdmarx, hdma_spi_rx);

	spi_dma_manager.attachCallbackFunction_RXDMA(spi_channel, boost::bind(&SPIClass::SPI_IRQHandler_RXDMA, this));

	SPI_DMA_EnableInterrupts_RX();

	SPI_PeriphState.dma_enabled_rx = true;
}

void SPIClass::SPI_DMA_DeInit_TX()
{
	if (!SPI_PeriphState.dma_enabled_tx)
		return;

	HAL_DMA_DeInit(spi_handle.hdmatx);
	SPI_DMA_DisableInterrupts_TX();
	spi_dma_manager.removeCallbackFunction_TXDMA(spi_channel);

	SPI_PeriphState.dma_enabled_tx = false;
}

void SPIClass::SPI_DMA_DeInit_RX()
{
	if (!SPI_PeriphState.dma_enabled_rx)
		return;

	HAL_DMA_DeInit(spi_handle.hdmarx);
	SPI_DMA_DisableInterrupts_RX();
	spi_dma_manager.removeCallbackFunction_RXDMA(spi_channel);

	SPI_PeriphState.dma_enabled_rx = false;
}

void SPIClass::SPI_DMA_EnableClock()
{
	/* Global DMA Clock options. Only turn on capability is
	provided due to other peripherals possibly using DMA. */
	if (__DMA1_IS_CLK_DISABLED())
		__DMA1_CLK_ENABLE();

	if (__DMA2_IS_CLK_DISABLED())
		__DMA2_CLK_ENABLE();
}

void SPIClass::SPI_DMA_EnableInterrupts_TX()
{
	HAL_NVIC_DisableIRQ(ITSettings_DMA_TX.IRQn);
	HAL_NVIC_ClearPendingIRQ(ITSettings_DMA_TX.IRQn);
	HAL_NVIC_SetPriorityGrouping(ITSettings_DMA_TX.groupPriority);
	HAL_NVIC_SetPriority(ITSettings_DMA_TX.IRQn, ITSettings_DMA_TX.preemptPriority, ITSettings_DMA_TX.subPriority);
	HAL_NVIC_EnableIRQ(ITSettings_DMA_TX.IRQn);

	SPI_PeriphState.dma_interrupts_enabled_tx = true;
}

void SPIClass::SPI_DMA_EnableInterrupts_RX()
{
	HAL_NVIC_DisableIRQ(ITSettings_DMA_RX.IRQn);
	HAL_NVIC_ClearPendingIRQ(ITSettings_DMA_RX.IRQn);
	HAL_NVIC_SetPriorityGrouping(ITSettings_DMA_RX.groupPriority);
	HAL_NVIC_SetPriority(ITSettings_DMA_RX.IRQn, ITSettings_DMA_RX.preemptPriority, ITSettings_DMA_RX.subPriority);
	HAL_NVIC_EnableIRQ(ITSettings_DMA_RX.IRQn);

	SPI_PeriphState.dma_interrupts_enabled_rx = true;
}

void SPIClass::SPI_DMA_DisableInterrupts_TX()
{
	HAL_NVIC_ClearPendingIRQ(ITSettings_DMA_TX.IRQn);
	HAL_NVIC_DisableIRQ(ITSettings_DMA_TX.IRQn);

	SPI_PeriphState.dma_interrupts_enabled_tx = false;
}

void SPIClass::SPI_DMA_DisableInterrupts_RX()
{
	HAL_NVIC_ClearPendingIRQ(ITSettings_DMA_RX.IRQn);
	HAL_NVIC_DisableIRQ(ITSettings_DMA_RX.IRQn);

	SPI_PeriphState.dma_interrupts_enabled_rx = false;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	/* This callback is used when only the Transmit() HAL function is used.
	 * TransmitReceive() is handled in the TxRxCpltCallback() */

	/*-------------------------------
	* Filter through the possible calling instances
	*-------------------------------*/
	SPIClass_sPtr spi;

	#ifdef ENABLE_SPI1
	if (hspi->Instance == SPI1)
		spi = spi1;
	#endif

	#ifdef ENABLE_SPI2
	if (hspi->Instance == SPI2)
		spi = spi2;
	#endif

	#ifdef ENABLE_SPI3
	if (hspi->Instance == SPI3)
		spi = spi3;
	#endif

	#ifdef ENABLE_SPI4
	if (hspi->Instance == SPI4)
		spi = spi4;
	#endif

	#ifdef ENABLE_SPI5
	if (hspi->Instance == SPI5)
		spi = spi5;
	#endif

	#ifdef ENABLE_SPI6
	if (hspi->Instance == SPI6)
		spi = spi6;
	#endif

	/* No SPI instance available */
	if (!spi)
		return;

	if (spi->isInitialized)
	{
		spi->tx_complete = true;

		if (!spi->TXPacketBuffer.empty())
		{
			/*-------------------------------
			* Handle any operations on the completed TX
			*-------------------------------*/
			SPIClass::SPIPacket packet = spi->TXPacketBuffer.front();

			if ((packet.options & SS_INACTIVE_AFTER_TX) == SS_INACTIVE_AFTER_TX &&
				(spi->SlaveSelectControl == SS_AUTOMATIC_CONTROL))
				spi->writeSS(HIGH);

			spi->TXPacketBuffer.pop_front();

			/*-------------------------------
			* If more data is left to send, queue it up.
			*-------------------------------*/
			if (!spi->TXPacketBuffer.empty())
			{
				packet = spi->TXPacketBuffer.front();

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
	/* This callback is used when the TransmitReceive() HAL function is used. */

	/*-------------------------------
	* Filter through the possible calling instances
	*-------------------------------*/
	SPIClass_sPtr spi;
	uint32_t spi_channel = 0;

	#ifdef ENABLE_SPI1
	if (hspi->Instance == SPI1)
	{
		spi = spi1;
		spi_channel = 1;
	}
	#endif

	#ifdef ENABLE_SPI2
	if (hspi->Instance == SPI2)
	{
		spi = spi2;
		spi_channel = 2;
	}
	#endif

	#ifdef ENABLE_SPI3
	if (hspi->Instance == SPI3)
	{
		spi = spi3;
		spi_channel = 3;
	}
	#endif

	#ifdef ENABLE_SPI4
	if (hspi->Instance == SPI4)
	{
		spi = spi4;
		spi_channel = 4;
	}
	#endif

	#ifdef ENABLE_SPI5
	if (hspi->Instance == SPI5)
	{
		spi = spi5;
		spi_channel = 5;
	}
	#endif

	#ifdef ENABLE_SPI6
	if (hspi->Instance == SPI6)
	{
		spi = spi6;
		spi_channel = 6;
	}
	#endif

	/* No SPI instance available */
	if (!spi)
		return;

	if (spi->isInitialized)
	{
		spi->tx_complete = true;

		if (!spi->TXPacketBuffer.empty())
		{
			/*-------------------------------
			* Handle any operations on the completed TX
			*-------------------------------*/
			SPIClass::SPIPacket packet = spi->TXPacketBuffer.front();

			if ((packet.options & SS_INACTIVE_AFTER_TX) == SS_INACTIVE_AFTER_TX &&
				(spi->SlaveSelectControl == SS_AUTOMATIC_CONTROL))
				spi->writeSS(HIGH);

			spi->TXPacketBuffer.pop_front();

			/*-------------------------------
			* If more data is left to send, queue it up.
			*-------------------------------*/
			if (!spi->TXPacketBuffer.empty())
			{
				packet = spi->TXPacketBuffer.front();

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
		EXTI0_TaskMGR->logEventGenerator(SRC_SPI, spi_channel);
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