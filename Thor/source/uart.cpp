#include "../include/uart.h"

using namespace ThorDef::UART;
using namespace Defaults::Timer;
using namespace Defaults::Serial;
using namespace Interrupt;

/************************************************************************/
/*					  Local Object Instantiations                       */
/************************************************************************/

#ifdef ENABLE_UART1
UARTClass_sPtr uart1 = boost::make_shared<UARTClass>(1);

void USART1_IRQHandler(void)
{
	uart1->UART_IRQHandler();
}
#endif

#ifdef ENABLE_UART2
UARTClass_sPtr uart2 = boost::make_shared<UARTClass>(2);

void USART2_IRQHandler(void)
{
	//uart2->UART_IRQHandler(); //TODO: Need to separate USART and UART handlers!
}
#endif

#ifdef ENABLE_UART3
UARTClass_sPtr uart3 = boost::make_shared<UARTClass>(3);

void USART3_IRQHandler(void)
{
	uart3->UART_IRQHandler();
}
#endif

#ifdef ENABLE_UART4
UARTClass_sPtr uart4 = boost::make_shared<UARTClass>(4);

void UART4_IRQHandler(void)
{
	uart4->UART_IRQHandler();
}
#endif

#ifdef ENABLE_UART5
UARTClass_sPtr uart5 = boost::make_shared<UARTClass>(5);

void UART5_IRQHandler(void)
{
	uart5->UART_IRQHandler();
}
#endif

#ifdef ENABLE_UART7
UARTClass_sPtr uart7 = boost::make_shared<UARTClass>(7);

void UART7_IRQHandler(void)
{
	uart7->UART_IRQHandler();
}
#endif

#ifdef ENABLE_UART8
UARTClass_sPtr uart8 = boost::make_shared<UARTClass>(8);

void UART8_IRQHandler(void)
{
	uart8->UART_IRQHandler();
}
#endif


/************************************************************************/
/*						    Interrupt Handlers                          */
/************************************************************************/
void UARTClass::UART_IRQHandler(void)
{
	#if defined(STM32F7)
	bool RX_DATA_READY = __HAL_UART_GET_FLAG(&uart_handle, UART_FLAG_RXNE);
	bool RX_LINE_IDLE = __HAL_UART_GET_FLAG(&uart_handle, UART_FLAG_IDLE);

	/*------------------------------------
	* Handle Async RX (Interrupt and DMA Mode)
	*------------------------------------*/
	/* RX In Process */
	if (RX_ASYNC && RX_DATA_READY && uart_handle.gState != HAL_UART_STATE_BUSY_TX)
	{
		uint32_t isrflags = READ_REG(uart_handle.Instance->ISR);
		uint32_t errorflags = (isrflags & (uint32_t)(USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE));


		if (errorflags == RESET)
		{
			/* Detected start of a new frame of unknown size. Enable the
			* IDLE interrupt bit to detect end of frame. */
			if (rxAsyncPacketSize == 0)
			{
				memset(packetQueue[currentQueuePacket], 0, ThorDef::UART::UART_BUFFER_SIZE);
				__HAL_UART_ENABLE_IT(&uart_handle, UART_IT_IDLE);
			}

			/* Buffer the new data */
			if (rxMode == RX_MODE_INTERRUPT && (rxAsyncPacketSize < ThorDef::UART::UART_BUFFER_SIZE))
			{
				packetQueue[currentQueuePacket][rxAsyncPacketSize] = (uint8_t)(uart_handle.Instance->RDR & (uint8_t)0xFF);
				rxAsyncPacketSize += 1u;
			}
			else
			{
				/* Forced to read data without being able to store it.
				* Really need to put some kind of error thing here...*/
				uart_handle.Instance->RDR;
			}
		}
		/* Error Handling */
		else
		{
			//Do this more elegantly later
			__HAL_UART_CLEAR_IT(&uart_handle, UART_CLEAR_PEF);
			__HAL_UART_CLEAR_IT(&uart_handle, UART_CLEAR_FEF);
			__HAL_UART_CLEAR_IT(&uart_handle, UART_CLEAR_NEF);
			__HAL_UART_CLEAR_IT(&uart_handle, UART_CLEAR_OREF);
		}
	}

	/* RX Complete */
	if (RX_ASYNC && RX_LINE_IDLE)
	{
		/* Disable Idle Line Interrupt */
		__HAL_UART_DISABLE_IT(&uart_handle, UART_IT_IDLE);
		__HAL_UART_CLEAR_IT(&uart_handle, UART_CLEAR_IDLEF);

		/* Copy packets received to the internal buffer */
		if (rxMode == RX_MODE_INTERRUPT)
		{
			/* Store the address of the the new data packet */
			RX_tempPacket.data = 0;
			RX_tempPacket.data_ptr = packetQueue[currentQueuePacket];
			RX_tempPacket.length = rxAsyncPacketSize;
			RXPacketBuffer.push_back(RX_tempPacket);

			rxAsyncPacketSize = 0;	/* Reset the internal packet counter so we know when a new frame starts */
			currentQueuePacket++;	/* Go to the next buffer location */

			if (currentQueuePacket == ThorDef::UART::UART_PACKET_QUEUE_SIZE)
				currentQueuePacket = 0;
		}

		else if (rxMode == RX_MODE_DMA)
		{
			/* Force a hard reset of the DMA to trigger the DMA RX Complete handler */
			if (uart_handle.hdmarx->Instance->NDTR != 0)
				__HAL_DMA_DISABLE(uart_handle.hdmarx);
		}

		rx_complete = true;
		totalWaitingPackets++;
	}

	/*------------------------------------
	* Handle Explicit User Requests:
	* Only run the normal IRQHandler if an explicit RX packet request
	* was generated or if TX-ing some data.
	*------------------------------------*/
	if (!RX_ASYNC || uart_handle.gState == HAL_UART_STATE_BUSY_TX)
		HAL_UART_IRQHandler(&uart_handle);
	#endif 

	#if defined(STM32F4)
	/* Reading these two in the order of SR then DR ends up 
	 * clearing all flags, so it's best to store the returned 
	 * contents for further processing. */
	volatile uint32_t isrflags = READ_REG(uart_handle.Instance->SR);
	volatile uint32_t data_reg = READ_REG(uart_handle.Instance->DR);

	/*------------------------------------
	* Handle Async RX of unknown length(Interrupt and DMA Mode)
	*------------------------------------*/
	if (RX_ASYNC)
	{
		volatile uint32_t cr1 = READ_REG(uart_handle.Instance->CR1);

		bool RX_DATA_READY = ((isrflags & UART_FLAG_RXNE) == UART_FLAG_RXNE);
		bool RX_DATA_READY_IE = ((cr1 & USART_CR1_RXNEIE) == USART_CR1_RXNEIE);
		bool RX_LINE_IDLE = ((isrflags & UART_FLAG_IDLE) == UART_FLAG_IDLE);
		bool RX_LINE_IDLE_IE = ((cr1 & USART_CR1_IDLEIE) == USART_CR1_IDLEIE);

		if (RX_DATA_READY && RX_DATA_READY_IE)
		{
			uint32_t errorflags = (isrflags & (uint32_t)(USART_SR_PE | USART_SR_FE | USART_SR_ORE | USART_SR_NE));

			/* No Errors Found */
			if (errorflags == 0u)
			{
				/* Detected start of a new frame of unknown size. Enable the
				 * IDLE interrupt bit to detect end of frame. */
				if (rxAsyncPacketSize == 0)
				{
					memset(packetQueue[currentQueuePacket], 0, ThorDef::UART::UART_BUFFER_SIZE);
					__HAL_UART_ENABLE_IT(&uart_handle, UART_IT_IDLE);
				}

				/* Buffer the new data and increase packet size count */
				if (rxMode == RX_MODE_INTERRUPT && (rxAsyncPacketSize < ThorDef::UART::UART_BUFFER_SIZE))
				{
					packetQueue[currentQueuePacket][rxAsyncPacketSize] = (uint8_t)(data_reg & (uint8_t)0xFF);
					rxAsyncPacketSize += 1u;
				}
				else
				{
					/* Somehow inform the user that data is being tossed out */
				}
			}
			
			/* Error Handling */
			else
			{
				// TODO
			}
		}

		/* RX Complete */
		if (RX_LINE_IDLE && RX_LINE_IDLE_IE)
		{
			/* Disable Idle Line Interrupt */
			__HAL_UART_DISABLE_IT(&uart_handle, UART_IT_IDLE);

			/* Copy packets received to the internal buffer */
			if (rxMode == RX_MODE_INTERRUPT)
			{
				/* Store the address of the the new data packet */
				RX_tempPacket.data = 0;
				RX_tempPacket.data_ptr = packetQueue[currentQueuePacket];
				RX_tempPacket.length = rxAsyncPacketSize;
				RXPacketBuffer.push_back(RX_tempPacket);

				rxAsyncPacketSize = 0;	/* Reset the internal packet counter so we know when a new frame starts */
				currentQueuePacket++;	/* Go to the next buffer location */

				if (currentQueuePacket == ThorDef::UART::UART_PACKET_QUEUE_SIZE)
					currentQueuePacket = 0;


				/*------------------------------------
				* Signal Waiting Threads 
				*------------------------------------*/
				#if defined(USING_FREERTOS)
				
				/* Inform the semaphore task manager that a particular event has occured 
				 * on the given source and periph instance. If a semaphore is tied to this,
				 * 1 semaphore will be given in the EXTI0 interrupt. */
				EXTI0_TaskMGR->logEventGenerator(SRC_UART, uart_channel);
				#endif 
			}

			else if (rxMode == RX_MODE_DMA)
			{
				/* Force a hard reset of the DMA to trigger the DMA RX Complete handler */
				if (uart_handle.hdmarx->Instance->NDTR != 0)
					__HAL_DMA_DISABLE(uart_handle.hdmarx);
			}

			rx_complete = true;
			totalWaitingPackets++;
		}
	}

	/*------------------------------------
	* Handle Explicit User Requests:
	* Only run the normal IRQHandler if an explicit RX packet request
	* was generated or if TX-ing some data.
	*------------------------------------*/

	//TODO: Better fix this later to support usart handler in uart mode
	if (!RX_ASYNC || uart_handle.gState == HAL_UART_STATE_BUSY_TX)
		HAL_UART_IRQHandler(&uart_handle);
	#endif
}

void UARTClass::UART_IRQHandler_TXDMA()
{
	HAL_DMA_IRQHandler(uart_handle.hdmatx);
}

void UARTClass::UART_IRQHandler_RXDMA()
{
	HAL_DMA_IRQHandler(uart_handle.hdmarx);
}

/************************************************************************/
/*			           Constructors/Deconstructors                      */
/************************************************************************/
UARTClass::UARTClass(int channel)
{
	uart_channel = channel;

	/* Reset all internal status flags */
	UART_PeriphState.gpio_enabled = false;
	UART_PeriphState.uart_enabled = false;
	UART_PeriphState.uart_interrupts_enabled = false;
	UART_PeriphState.dma_enabled_tx = false;
	UART_PeriphState.dma_enabled_rx = false;
	UART_PeriphState.dma_interrupts_enabled_tx = false;
	UART_PeriphState.dma_interrupts_enabled_rx = false;

	/* Assign external flag pointers */
	isInitialized = &UART_PeriphState.uart_enabled;

	txMode = TX_MODE_NONE;
	rxMode = RX_MODE_NONE;
	RX_ASYNC = true;

	/* Default handle settings for this instance */
	uart_handle.Init = Defaults::Serial::dflt_UART_Init;
	uart_handle.Instance = srl_cfg[uart_channel].instance;

	#if defined(STM32F7)
	uart_handle.AdvancedInit = Defaults::Serial::dflt_UART_AdvInit;
	#endif

	/* Copy over the interrupt settings information */
	ITSettings_HW = srl_cfg[uart_channel].IT_HW;
	ITSettings_DMA_TX = srl_cfg[uart_channel].dmaIT_TX;
	ITSettings_DMA_RX = srl_cfg[uart_channel].dmaIT_RX;

	/* Create the output gpio pin objects */
	tx_pin = boost::make_shared<GPIOClass>(
		srl_cfg[uart_channel].txPin.GPIOx,
		srl_cfg[uart_channel].txPin.PinNum,
		srl_cfg[uart_channel].txPin.Speed,
		srl_cfg[uart_channel].txPin.Alternate);

	rx_pin = boost::make_shared<GPIOClass>(
		srl_cfg[uart_channel].rxPin.GPIOx,
		srl_cfg[uart_channel].rxPin.PinNum,
		srl_cfg[uart_channel].rxPin.Speed,
		srl_cfg[uart_channel].rxPin.Alternate);

	/* Initialize the buffer memory */
	//memset(rxBufferInternal, 0, ThorDef::UART::UART_BUFFER_SIZE);
	rxAsyncPacketSize = 0;
	totalWaitingPackets = 0;
	currentQueuePacket = 0;
	
	TXPacketBuffer.set_capacity(ThorDef::UART::UART_BUFFER_SIZE);
	RXPacketBuffer.set_capacity(ThorDef::UART::UART_BUFFER_SIZE);


}

UARTClass::~UARTClass()
{
	end();
}

/************************************************************************/
/*						     Public Functions                           */
/************************************************************************/
UART_Status UARTClass::begin()
{
	return begin(115200, TX_MODE_BLOCKING, RX_MODE_BLOCKING);
}

UART_Status UARTClass::begin(uint32_t baud)
{
	return begin(baud, TX_MODE_BLOCKING, RX_MODE_BLOCKING);
}

UART_Status UARTClass::begin(uint32_t baud, uint32_t tx_mode, uint32_t rx_mode)
{
	UART_GPIO_Init();
	
	uart_handle.Init.BaudRate = baud;
	UART_Init();

	/* Configure the transmission mode */
	switch (tx_mode)
	{
	case TX_MODE_BLOCKING:
		setTxModeBlock();
		break;

	case TX_MODE_INTERRUPT:
		setTxModeIT();
		break;

	case TX_MODE_DMA:
		setTxModeDMA();
		break;

	default:
		txMode = TX_MODE_BLOCKING;
		setTxModeBlock();
		break;
	}

	/* Configure the reception mode */
	switch (rx_mode)
	{
	case RX_MODE_BLOCKING:
		setRxModeBlock();
		break;

	case RX_MODE_INTERRUPT:
		setRxModeIT();
		break;

	case RX_MODE_DMA:
		setRxModeDMA();
		break;

	default:
		rxMode = RX_MODE_BLOCKING;
		setRxModeBlock();
		break;
	}

	return UART_READY;
}

UART_Status UARTClass::write(const char* string)
{
	return write((uint8_t*)string, strlen(string));
}

UART_Status UARTClass::write(char* string, size_t length)
{
	return write((uint8_t*)string, length);
}

UART_Status UARTClass::write(uint8_t* val, size_t length)
{
	/*------------------------------------
	* NOTE:
	* This method does no real time checking of the TX length compared
	* to the length of data that is passed in. This is because the data
	* (val) decays to a pointer upon entry to the function and all information
	* regarding length is lost. As such, it is up to the programmer to ensure
	* that the length of the input data is not exceeded during TX or undefined
	* behavior will occur.
	*------------------------------------*/
	if (!UART_PeriphState.gpio_enabled || !UART_PeriphState.uart_enabled)
		return UART_NOT_INITIALIZED;

	/*------------------------------------
	* Transmit based on the user mode
	*------------------------------------*/
	switch (txMode)
	{
	case TX_MODE_BLOCKING:
		if (tx_complete)
		{
			tx_complete = false;
			HAL_UART_Transmit(&uart_handle, val, length, HAL_MAX_DELAY);
			tx_complete = true;
		}
		return UART_READY;
		break;

	case TX_MODE_INTERRUPT:
		if (UART_PeriphState.uart_interrupts_enabled)
		{
			if (tx_complete)
			{
				/* TX hardware is free. Go ahead and send data.*/
				tx_complete = false;
				HAL_UART_Transmit_IT(&uart_handle, val, length);
				return UART_TX_IN_PROGRESS;
			}
			else
			{
				/* TX hardware is tied up. Buffer the data. */
				TX_tempPacket.data = 0;
				TX_tempPacket.data_ptr = val;
				TX_tempPacket.length = length;

				TXPacketBuffer.push_back(TX_tempPacket);
				return UART_NOT_READY;
			}
		}
		else
			return UART_ERROR;
		break;

	case TX_MODE_DMA:
		if (UART_PeriphState.dma_enabled_tx && UART_PeriphState.uart_interrupts_enabled)
		{
			if (tx_complete)
			{
				tx_complete = false;
				HAL_UART_Transmit_DMA(&uart_handle, val, length);
				return UART_TX_IN_PROGRESS;
			}
			else
			{
				TX_tempPacket.data = 0;
				TX_tempPacket.data_ptr = val;
				TX_tempPacket.length = length;

				TXPacketBuffer.push_back(TX_tempPacket);
				return UART_NOT_READY;
			}
		}
		else
			return UART_ERROR;
		break;

	default: return UART_ERROR;
	}
}

UART_Status UARTClass::write(std::string string)
{
	return write((uint8_t*)string.data(), string.size());
}

int UARTClass::readPacket(uint8_t* buff, size_t buff_length)
{
	UARTPacket packet = RXPacketBuffer.front();

	size_t packetLength = packet.length;
	int error = 0;

	/* Check if the received packet is too large for the buffer */
	if (packetLength > buff_length)
	{
		packetLength = buff_length;
		error = -1;
	}

	memcpy(buff, packet.data_ptr, packetLength);

	RXPacketBuffer.pop_front();

	totalWaitingPackets--;
	return error;
}

int UARTClass::nextPacketSize()
{
	if (RXPacketBuffer.empty())
		return 0;
	else
		return RXPacketBuffer.front().length;
}

int UARTClass::availablePackets()
{
	return totalWaitingPackets;
}

void UARTClass::flush()
{
	
}

void UARTClass::end()
{
	UART_DeInit();
	UART_GPIO_DeInit();
	UART_DisableInterrupts();
	UART_DMA_DeInit_TX();
	UART_DMA_DeInit_RX();

	txMode = TX_MODE_NONE;
	rxMode = RX_MODE_NONE;
}

void UARTClass::attachSettings(UART_InitTypeDef config)
{
	uart_handle.Init = config;

	if(UART_PeriphState.uart_enabled)
		UART_DeInit();

	UART_Init();
}

void UARTClass::setTxModeBlock()
{
	txMode = TX_MODE_BLOCKING;
	
	/* Make sure RX side isn't using interrupts before disabling */
	if (rxMode = RX_MODE_BLOCKING)
		UART_DisableInterrupts();

	UART_DMA_DeInit_TX();
}

void UARTClass::setTxModeIT()
{
	txMode = TX_MODE_INTERRUPT;

	UART_EnableInterrupts();
	UART_DMA_DeInit_TX();
}

void UARTClass::setTxModeDMA()
{
	txMode = TX_MODE_DMA;

	UART_EnableInterrupts();
	UART_DMA_Init_TX();
}

void UARTClass::setRxModeBlock()
{
	rxMode = RX_MODE_BLOCKING;

	/* Make sure TX side isn't using interrupts before disabling */
	if (txMode = TX_MODE_BLOCKING)
		UART_DisableInterrupts();

	UART_DMA_DeInit_RX();
}

void UARTClass::setRxModeIT()
{
	rxMode = RX_MODE_INTERRUPT;

	UART_EnableInterrupts();
	UART_DMA_DeInit_RX();
}

void UARTClass::setRxModeDMA()
{
	rxMode = RX_MODE_DMA;

	UART_EnableInterrupts();
	UART_DMA_Init_RX();
	
	/* Instruct the DMA hardware to start listening for packets. Set the idle line bit 
	 * for triggering the end of packet interrupt.*/
	//HAL_UART_Receive_DMA(&uart_handle, rxBufferInternal, ThorDef::UART::UART_BUFFER_SIZE);
	HAL_UART_Receive_DMA(&uart_handle, packetQueue[currentQueuePacket], ThorDef::UART::UART_BUFFER_SIZE);
	__HAL_UART_ENABLE_IT(&uart_handle, UART_IT_IDLE);

	#if defined(STM32F7)
	__HAL_UART_CLEAR_IT(&uart_handle, UART_CLEAR_IDLEF);
	#endif

	#if defined(STM32F4)
	__HAL_UART_CLEAR_FLAG(&uart_handle, UART_FLAG_IDLE);
	#endif
	
}

/************************************************************************/
/*						    Private Functions                           */
/************************************************************************/
void UARTClass::UART_Init()
{
	UART_EnableClock();

	if (HAL_UART_Init(&uart_handle) != HAL_OK)
		BasicErrorHandler(logError("Failed UART Init. Check settings."));

	setTxModeBlock();
	setRxModeBlock();

	UART_PeriphState.uart_enabled = true;
}

void UARTClass::UART_DeInit()
{
	HAL_UART_DeInit(&uart_handle);
	UART_PeriphState.uart_enabled = false;
}

void UARTClass::UART_EnableClock()
{
	#ifdef ENABLE_UART1
	if (uart_channel == 1)
		__USART1_CLK_ENABLE();
	#endif

	#ifdef ENABLE_UART2
	if (uart_channel == 2)
		__USART2_CLK_ENABLE();
	#endif

	#ifdef ENABLE_UART3
	if (uart_channel == 3)
		__USART3_CLK_ENABLE();
	#endif

	#ifdef ENABLE_UART4
	if (uart_channel == 4)
		__UART4_CLK_ENABLE();
	#endif

	#ifdef ENABLE_UART5
	if (uart_channel == 5)
		__UART5_CLK_ENABLE();
	#endif

	#ifdef ENABLE_UART7
	if (uart_channel == 7)
		__UART7_CLK_ENABLE();
	#endif

	#ifdef ENABLE_UART8
	if (uart_channel == 8)
		__UART8_CLK_ENABLE();
	#endif
}

void UARTClass::UART_DisableClock()
{
	#ifdef ENABLE_UART1
	if (uart_channel == 1)
		__USART1_CLK_DISABLE();
	#endif

	#ifdef ENABLE_UART2
	if (uart_channel == 2)
		__USART2_CLK_DISABLE();
	#endif

	#ifdef ENABLE_UART3
	if (uart_channel == 3)
		__USART3_CLK_DISABLE();
	#endif

	#ifdef ENABLE_UART4
	if (uart_channel == 4)
		__UART4_CLK_DISABLE();
	#endif

	#ifdef ENABLE_UART5
	if (uart_channel == 5)
		__UART5_CLK_DISABLE();
	#endif

	#ifdef ENABLE_UART7
	if (uart_channel == 7)
		__UART7_CLK_DISABLE();
	#endif

	#ifdef ENABLE_UART8
	if (uart_channel == 8)
		__UART8_CLK_DISABLE();
	#endif
}

void UARTClass::UART_EnableInterrupts()
{
	HAL_NVIC_DisableIRQ(ITSettings_HW.IRQn);
	HAL_NVIC_SetPriorityGrouping(ITSettings_HW.groupPriority);
	HAL_NVIC_SetPriority(ITSettings_HW.IRQn, ITSettings_HW.preemptPriority, ITSettings_HW.subPriority);
	HAL_NVIC_EnableIRQ(ITSettings_HW.IRQn); //Triggers an interrupt immediately. Why?
	HAL_NVIC_ClearPendingIRQ(ITSettings_HW.IRQn);
	
	/* Specific interrupts to enable */
	if(rxMode == RX_MODE_INTERRUPT)
		__HAL_UART_ENABLE_IT(&uart_handle, UART_IT_RXNE);	//RX Data Register not Empty

	UART_PeriphState.uart_interrupts_enabled = true;
}

void UARTClass::UART_DisableInterrupts()
{
	__HAL_UART_DISABLE_IT(&uart_handle, UART_IT_IDLE);
	__HAL_UART_DISABLE_IT(&uart_handle, UART_IT_RXNE);
	
	HAL_NVIC_ClearPendingIRQ(ITSettings_HW.IRQn);
	HAL_NVIC_DisableIRQ(ITSettings_HW.IRQn);

	UART_PeriphState.uart_interrupts_enabled = false;
}

void UARTClass::UART_GPIO_Init()
{
	/* These should be configured as ALT_PP with PULLUPs */
	if(tx_pin != NULL)
		tx_pin->mode(srl_cfg[uart_channel].txPin.Mode, srl_cfg[uart_channel].rxPin.Pull);

	if(rx_pin != NULL)
		rx_pin->mode(srl_cfg[uart_channel].rxPin.Mode, srl_cfg[uart_channel].rxPin.Pull);

	UART_PeriphState.gpio_enabled = true;
}

void UARTClass::UART_GPIO_DeInit()
{
	//TODO: Implement GPIO DeInit
}

void UARTClass::UART_DMA_Init_TX()
{
	UART_DMA_EnableClock();
	hdma_uart_tx.Instance = srl_cfg[uart_channel].dmaTX.Instance;

	/* Grab the default init settings and modify for the specific hardware */
	hdma_uart_tx.Init = Defaults::Serial::dflt_DMA_Init_TX;
	hdma_uart_tx.Init.Channel = Defaults::Serial::srl_cfg[uart_channel].dmaTX.channel;
	hdma_uart_tx.Init.Direction = Defaults::Serial::srl_cfg[uart_channel].dmaTX.direction;

	/* Hard error if initialization fails. */
	if (HAL_DMA_Init(&hdma_uart_tx) != HAL_OK)
		BasicErrorHandler(logError("Failed UART DMA TX Init. Check handle settings."));

	__HAL_LINKDMA(&uart_handle, hdmatx, hdma_uart_tx);

	uart_dma_manager.attachCallbackFunction_TXDMA(uart_channel, boost::bind(&UARTClass::UART_IRQHandler_TXDMA, this));

	UART_DMA_EnableInterrupts_TX();

	UART_PeriphState.dma_enabled_tx = true;
}

void UARTClass::UART_DMA_Init_RX()
{
	UART_DMA_EnableClock();
	hdma_uart_rx.Instance = srl_cfg[uart_channel].dmaRX.Instance;

	/* Grab the default init settings and modify for the specific hardware */
	hdma_uart_rx.Init = Defaults::Serial::dflt_DMA_Init_RX;
	hdma_uart_rx.Init.Channel = Defaults::Serial::srl_cfg[uart_channel].dmaRX.channel;
	hdma_uart_rx.Init.Direction = Defaults::Serial::srl_cfg[uart_channel].dmaRX.direction;

	/* Hard error if initialization fails. */
	if (HAL_DMA_Init(&hdma_uart_rx) != HAL_OK)
		BasicErrorHandler(logError("Failed UART DMA TX Init. Check handle settings."));

	__HAL_LINKDMA(&uart_handle, hdmarx, hdma_uart_rx);

	uart_dma_manager.attachCallbackFunction_RXDMA(uart_channel, boost::bind(&UARTClass::UART_IRQHandler_RXDMA, this));

	UART_DMA_EnableInterrupts_RX();

	UART_PeriphState.dma_enabled_rx = true;
}

void UARTClass::UART_DMA_DeInit_TX()
{
	if (!UART_PeriphState.dma_enabled_tx)
		return;

	HAL_DMA_Abort(uart_handle.hdmatx);
	HAL_DMA_DeInit(uart_handle.hdmatx);
	UART_DMA_DisableInterrupts_TX();
	uart_dma_manager.removeCallbackFunction_TXDMA(uart_channel);

	UART_PeriphState.dma_enabled_tx = false;
}

void UARTClass::UART_DMA_DeInit_RX()
{
	if (!UART_PeriphState.dma_enabled_rx)
		return;

	HAL_DMA_Abort(uart_handle.hdmarx);
	HAL_DMA_DeInit(uart_handle.hdmarx);
	UART_DMA_DisableInterrupts_RX();
	uart_dma_manager.removeCallbackFunction_RXDMA(uart_channel);

	UART_PeriphState.dma_enabled_rx = false;
}

void UARTClass::UART_DMA_EnableClock()
{
	/* Global DMA Clock options. Only turn on capability is
	provided due to other peripherals possibly using DMA. */
	if (__DMA1_IS_CLK_DISABLED())
		__DMA1_CLK_ENABLE();


	if (__DMA2_IS_CLK_DISABLED())
		__DMA2_CLK_ENABLE();
}

void UARTClass::UART_DMA_EnableInterrupts_TX()
{
	HAL_NVIC_DisableIRQ(ITSettings_DMA_TX.IRQn);
	HAL_NVIC_ClearPendingIRQ(ITSettings_DMA_TX.IRQn);
	HAL_NVIC_SetPriorityGrouping(ITSettings_DMA_TX.groupPriority);
	HAL_NVIC_SetPriority(ITSettings_DMA_TX.IRQn, ITSettings_DMA_TX.preemptPriority, ITSettings_DMA_TX.subPriority);
	HAL_NVIC_EnableIRQ(ITSettings_DMA_TX.IRQn);

	UART_PeriphState.dma_interrupts_enabled_tx = true;
}

void UARTClass::UART_DMA_EnableInterrupts_RX()
{
	HAL_NVIC_DisableIRQ(ITSettings_DMA_RX.IRQn);
	HAL_NVIC_ClearPendingIRQ(ITSettings_DMA_RX.IRQn);
	HAL_NVIC_SetPriorityGrouping(ITSettings_DMA_RX.groupPriority);
	HAL_NVIC_SetPriority(ITSettings_DMA_RX.IRQn, ITSettings_DMA_RX.preemptPriority, ITSettings_DMA_RX.subPriority);
	HAL_NVIC_EnableIRQ(ITSettings_DMA_RX.IRQn);

	UART_PeriphState.dma_interrupts_enabled_rx = true;
}

void UARTClass::UART_DMA_DisableInterrupts_TX()
{
	HAL_NVIC_ClearPendingIRQ(ITSettings_DMA_TX.IRQn);
	HAL_NVIC_DisableIRQ(ITSettings_DMA_TX.IRQn);

	UART_PeriphState.dma_interrupts_enabled_tx = false;
}

void UARTClass::UART_DMA_DisableInterrupts_RX()
{
	HAL_NVIC_ClearPendingIRQ(ITSettings_DMA_RX.IRQn);
	HAL_NVIC_DisableIRQ(ITSettings_DMA_RX.IRQn);

	UART_PeriphState.dma_interrupts_enabled_rx = false;
}

/************************************************************************/
/*						   Callback Functions                           */
/************************************************************************/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	UARTClass_sPtr uart;

	/* Filter through the possible calling peripherals */
	#ifdef ENABLE_UART1
	if (UartHandle->Instance == USART1)
		uart = uart1;
	#endif

	#ifdef ENABLE_UART2
	if (UartHandle->Instance == USART2)
		uart = uart2;
	#endif

	#ifdef ENABLE_UART3
	if (UartHandle->Instance == USART3)
		uart = uart3;
	#endif

	#ifdef ENABLE_UART4
	if (UartHandle->Instance == UART4)
		uart = uart4;
	#endif

	#ifdef ENABLE_UART5
	if (UartHandle->Instance == UART5)
		uart = uart5;
	#endif

	#ifdef ENABLE_UART7
	if (UartHandle->Instance == UART7)
		uart = uart7;
	#endif

	#ifdef ENABLE_UART8
	if (UartHandle->Instance == UART8)
		uart = uart8;
	#endif

	/* If nothing matched, force a return. */
	if (uart == NULL)
		return;
	
	

	if (uart->isInitialized)
	{
		uart->tx_complete = true;

		/*-------------------------------
		* Check if we have more data to send out
		*-------------------------------*/
		if (!uart->TXPacketBuffer.empty())
		{
			UARTClass::UARTPacket packet = uart->TXPacketBuffer.front();

			uart->write(packet.data_ptr, packet.length);

			//Try popping off the packet to see if it causes write errors.
			uart->TXPacketBuffer.pop_front();
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	UARTClass_sPtr uart;
	volatile uint32_t uart_channel = 0;

	/* Filter through the possible calling peripherals */
	#ifdef ENABLE_UART1
	if (UartHandle->Instance == USART1)
	{
		uart = uart1;
		uart_channel = 1;
	}
	#endif

	#ifdef ENABLE_UART2
	if (UartHandle->Instance == USART2)
	{
		uart = uart2;
		uart_channel = 2;
	}
	#endif

	#ifdef ENABLE_UART3
	if (UartHandle->Instance == USART3)
	{
		uart = uart3;
		uart_channel = 3;
	}
	#endif

	#ifdef ENABLE_UART4
	if (UartHandle->Instance == UART4)
	{
		uart = uart4;
		uart_channel = 4;
	}
	#endif

	#ifdef ENABLE_UART5
	if (UartHandle->Instance == UART5)
	{
		uart = uart5;
		uart_channel = 5;
	}
	#endif

	#ifdef ENABLE_UART7
	if (UartHandle->Instance == UART7)
	{
		uart = uart7;
		uart_channel = 7;
	}
	#endif

	#ifdef ENABLE_UART8
	if (UartHandle->Instance == UART8)
	{
		uart = uart8;
		uart_channel = 8;
	}
	#endif

	/* If nothing matched, force a return. */
	if (uart == NULL)
		return;

	if (uart->rxMode == RX_MODE_DMA)
	{
		uart->rx_complete = true;

		uint16_t bufferMax = UartHandle->RxXferSize;
		uint16_t bufferRemaining = UartHandle->hdmarx->Instance->NDTR;

		size_t num_received = (size_t)(bufferMax - bufferRemaining);

		UARTClass::UARTPacket tempPacket;
		tempPacket.data = 0;
		tempPacket.data_ptr = uart->packetQueue[uart->currentQueuePacket];
		tempPacket.length = num_received;
		uart->RXPacketBuffer.push_back(tempPacket);

		/* Start the listening process again for a new packet */
		__HAL_UART_ENABLE_IT(UartHandle, UART_IT_IDLE);
		HAL_UART_Receive_DMA(UartHandle, uart->packetQueue[uart->currentQueuePacket], ThorDef::UART::UART_BUFFER_SIZE);

		/*------------------------------------
		* Signal Waiting Threads
		*------------------------------------*/
		#if defined(USING_FREERTOS)
		EXTI0_TaskMGR->logEventGenerator(SRC_UART, uart_channel);
		#endif
	}

	/* This only runs IF the user explicitly requests an RX/TX*/
	else if (!uart->rx_complete)
	{
		uart->rx_complete = true;
		uart->RX_ASYNC = true; //Revert back to asynchronous listening for RX
	}
}

void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *UartHandle)
{
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *UartHandle)
{
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
}

