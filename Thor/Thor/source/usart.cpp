/* Project Includes */
#include <Thor/include/config.hpp>
#include <Thor/include/usart.hpp>
#include <Thor/include/exceptions.hpp>
#include <Thor/include/interrupt.hpp>

#if defined(USING_FREERTOS)
#include <Thor/include/exti.hpp>
static SemaphoreHandle_t usartSemphrs[Thor::Definitions::Serial::MAX_SERIAL_CHANNELS + 1];
TaskTrigger usartTaskTrigger;
#endif


using namespace Thor::Definitions;
using namespace Thor::Definitions::Serial;
using namespace Thor::Definitions::USART;
using namespace Thor::Definitions::Interrupt;
using namespace Thor::Peripheral::USART;
using namespace Thor::Defaults::Serial;


static USARTClass_sPtr usartObjects[MAX_SERIAL_CHANNELS + 1];

static const USARTClass_sPtr& getUSARTClassRef(USART_TypeDef* instance)
{
	/* Simply converts the pointer into the raw numerical address value, which be compared against
	the peripheral base address. USARTx is simply (USART_TypeDef*)USARTx_Base. */
	auto i = reinterpret_cast<std::uintptr_t>(instance);
	switch (i)
	{
		#if defined(USART1)
	case USART1_BASE:
		return usartObjects[1];
		break;
		#endif
		#if defined(USART2)
	case USART2_BASE:
		return usartObjects[2];
		break;
		#endif
		#if defined(USART3)
	case USART3_BASE:
		return usartObjects[3];
		break;
		#endif
		#if defined(USART4)
	case USART4_BASE:
		return usartObjects[4];
		break;
		#endif
		#if defined(USART5)
	case USART5_BASE:
		return usartObjects[5];
		break;
		#endif
		#if defined(USART6)
	case USART6_BASE:
		return usartObjects[6];
		break;
		#endif
		#if defined(USART7)
	case USART7_BASE:
		return usartObjects[7];
		break;
		#endif
		#if defined(USART8)
	case USART8_BASE:
		return usartObjects[8];
		break;
		#endif

	default:
		return usartObjects[0];
		break;
	};
};


static volatile uint32_t* getUsartClockReg(USART_TypeDef* instance)
{
	auto i = reinterpret_cast<std::uintptr_t>(instance);
	switch (i)
	{
		#if defined(STM32F446xx) || defined(STM32F767xx)
		#if defined(USART1)
	case USART1_BASE:
		return &(RCC->APB2ENR);
		break;
		#endif
		#if defined(USART2)
	case USART2_BASE:
		return &(RCC->APB1ENR);
		break;
		#endif
		#if defined(USART3)
	case USART3_BASE:
		return &(RCC->APB1ENR);
		break;
		#endif
		#if defined(USART6)
	case USART6_BASE:
		return &(RCC->APB2ENR);
		break;
		#endif
		#endif /* !STM32F446xx  !STM32F767xx */

	default:
		return nullptr;
		break;
	};
}

static uint32_t usartClockMask(USART_TypeDef* instance)
{
	auto i = reinterpret_cast<std::uintptr_t>(instance);
	switch (i)
	{
		#if defined(STM32F446xx) || defined(STM32F767xx)
		#if defined(USART1)
	case USART1_BASE:
		return RCC_APB2ENR_USART1EN;
		break;
		#endif
		#if defined(USART2)
	case USART2_BASE:
		return RCC_APB1ENR_USART2EN;
		break;
		#endif
		#if defined(USART3)
	case USART3_BASE:
		return RCC_APB1ENR_USART3EN;
		break;
		#endif
		#if defined(USART6)
	case USART6_BASE:
		return RCC_APB2ENR_USART6EN;
		break;
		#endif
		#endif /* !STM32F446xx  !STM32F767xx */

	default:
		return 0u;
		break;
	};
};


namespace Thor
{
	namespace Peripheral
	{
		namespace USART
		{
            using namespace Thor::Definitions::GPIO;

            inline void USART_ClearIT_IDLE(USART_HandleTypeDef *UsartHandle)
			{
				#if defined(STM32F7)
				__HAL_USART_CLEAR_IT(UsartHandle, USART_CLEAR_IDLEF);
				#endif
			}

			inline void USART_EnableIT_IDLE(USART_HandleTypeDef *UsartHandle)
			{
				USART_ClearIT_IDLE(UsartHandle);
				__HAL_USART_ENABLE_IT(UsartHandle, USART_IT_IDLE);
			}

			inline void USART_DisableIT_IDLE(USART_HandleTypeDef *UsartHandle)
			{
				USART_ClearIT_IDLE(UsartHandle);
				__HAL_USART_DISABLE_IT(UsartHandle, USART_IT_IDLE);
			}

			Status USARTClass::begin(const BaudRate& baud, const Modes& tx_mode, const Modes& rx_mode)
			{
				USART_GPIO_Init();

				usartHandle.Init.BaudRate = static_cast<uint32_t>(baud);
				USART_Init();

				setMode(SubPeripheral::TX, tx_mode);
				setMode(SubPeripheral::RX, rx_mode);

				return Status::PERIPH_OK;
			}

			Status USARTClass::setMode(const SubPeripheral& periph, const Modes& mode)
			{
				if (periph == SubPeripheral::TX)
				{
					switch (mode)
					{
					case Modes::BLOCKING:
						txMode = mode; // Must be set before the other functions

									   /* Make sure RX side isn't using interrupts before disabling */
						if (rxMode == Modes::BLOCKING)
							USART_DisableInterrupts();

						USART_DMA_DeInit(periph);
						break;

					case Modes::INTERRUPT:
						txMode = mode; // Must be set before the other functions

						USART_EnableInterrupts();
						USART_DMA_DeInit(periph);
						break;

					case Modes::DMA:
						txMode = mode; // Must be set before the other functions

						USART_EnableInterrupts();
						USART_DMA_Init(periph);
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
						rxMode = mode; // Must be set before the other functions

									   /* Make sure TX side isn't using interrupts before disabling */
						if (txMode == Modes::BLOCKING)
							USART_DisableInterrupts();

						USART_DMA_DeInit(periph);

						break;

					case Modes::INTERRUPT:
						rxMode = mode;	// Must be set before the other functions

						USART_EnableInterrupts();
						USART_DMA_DeInit(periph);
						break;

					case Modes::DMA:
						rxMode = mode; // Must be set before the other functions

						USART_EnableInterrupts();
						USART_DMA_Init(periph);

						/* Set the idle line interrupt for asynchronously getting the end of transmission */
						USART_EnableIT_IDLE(&usartHandle);

						/* Instruct the DMA hardware to start listening for transmissions */
						HAL_USART_Receive_DMA(&usartHandle, RX_Queue[RXQueueIdx], USART_QUEUE_BUFFER_SIZE);
						break;

					default:
						rxMode = Modes::MODE_UNDEFINED;
						return Status::PERIPH_ERROR;
						break;
					}

					return Status::PERIPH_OK;
				}
			}

			Status USARTClass::setBaud(const uint32_t& baud)
			{
				/* Copy, modify, write */
				USART_InitTypeDef init = usartHandle.Init;
				init.BaudRate = baud;
				usartHandle.Init = init;

				/* Clear the hardware config and re-initialize with new settings */
				USART_DeInit();
				USART_Init();

				return Status::PERIPH_OK;
			}

			Status USARTClass::setBaud(const BaudRate& baud)
			{
				/* Copy, modify, write */
				USART_InitTypeDef init = usartHandle.Init;
				init.BaudRate = static_cast<uint32_t>(baud);
				usartHandle.Init = init;

				/* Clear the hardware config and re-initialize with new settings */
				USART_DeInit();
				USART_Init();

				return Status::PERIPH_OK;
			}

			Status USARTClass::write(uint8_t* val, size_t length)
			{
				if (!USARTPeriphState.gpio_enabled || !USARTPeriphState.usart_enabled)
					return Status::PERIPH_NOT_INITIALIZED;

				auto statusCode = Status::PERIPH_ERROR;
				auto halCode = HAL_OK;

				switch (txMode)
				{
				case Modes::BLOCKING:
					#if defined(USING_FREERTOS)
					halCode = HAL_USART_Transmit(&usartHandle, val, length, pdMS_TO_TICKS(BLOCKING_TIMEOUT_MS));
					#else
					halCode = HAL_USART_Transmit(&usartHandle, val, length, BLOCKING_TIMEOUT_MS);
					#endif

					if (halCode == HAL_TIMEOUT)
						statusCode = Status::PERIPH_TIMEOUT;
					else
						statusCode = Status::PERIPH_OK;
					break;

				case Modes::INTERRUPT:
					if (tx_complete)
					{
						/* Starting a brand new IT transmission */
						tx_complete = false;
						HAL_USART_Transmit_IT(&usartHandle, assignTXBuffer(val, length), length);
						statusCode = Status::PERIPH_TX_IN_PROGRESS;
					}
					else
					{
						statusCode = Status::PERIPH_BUSY;

						#if defined(USING_FREERTOS)
						if (xSemaphoreTakeFromISR(usartSemphrs[usartChannel], NULL) != pdPASS)
						{
							statusCode = Status::PERIPH_LOCKED;
							break;
						}
						#endif

						/* A previous IT transmission is still going. Queue the data */
						TX_tempPacket.data_ptr = assignTXBuffer(val, length);
						TX_tempPacket.length = length;
						TXPacketBuffer.push_back(TX_tempPacket);
					}
					break;

				case Modes::DMA:
					if (tx_complete)
					{
						/* Starting a brand new DMA transmission */
						tx_complete = false;
						HAL_USART_Transmit_DMA(&usartHandle, assignTXBuffer(val, length), length);
						statusCode = Status::PERIPH_TX_IN_PROGRESS;
					}
					else
					{
						statusCode = Status::PERIPH_BUSY;

						#if defined(USING_FREERTOS)
						if (xSemaphoreTakeFromISR(usartSemphrs[usartChannel], NULL) != pdPASS)
						{
							statusCode = Status::PERIPH_LOCKED;
							break;
						}
						#endif

						/* A previous DMA transmission is still going. Queue the data */
						TX_tempPacket.data_ptr = assignTXBuffer(val, length);
						TX_tempPacket.length = length;

						TXPacketBuffer.push_back(TX_tempPacket);
					}
					break;

				default:
					statusCode = Status::PERIPH_ERROR;
					break;
				}
				return statusCode;
			}

			Status USARTClass::write(char* string, size_t length)
			{
				return write((uint8_t*)string, length);
			}

			Status USARTClass::write(const char* string)
			{
				return write((uint8_t*)string, strlen(string));
			}

			Status USARTClass::write(const char* string, size_t length)
			{
				return write((uint8_t*)string, length);
			}

			Status USARTClass::readPacket(uint8_t* buff, size_t length)
			{
				Status error = Status::PERIPH_ERROR;
				USARTPacket packet = RXPacketBuffer.front();

				/* First make sure we have a valid address */
				if (packet.data_ptr)
				{
					//Note: Could return an error code here on (true), but that screws up the eRPC library
					size_t readLength = packet.length - packet.bytesRead;
					if (readLength > length)
						readLength = length;

					memcpy(buff, packet.data_ptr + packet.bytesRead, readLength);

					/* Check if this read operation will completely empty the
					 * stored packet data. If so, we are free to pop it off the buffer */
					if (length + packet.bytesRead >= packet.length)
					{
						RXPacketBuffer.pop_front();
						totalUnreadPackets--;
					}
					else /* Reinsert the modified packet since we aren't done yet */
					{
						packet.bytesRead += length;
						RXPacketBuffer.pop_front();
						RXPacketBuffer.push_front(packet);
					}

					error = Status::PERIPH_OK;
				}

				return error;
			}

			Status USARTClass::readSync(uint8_t* buff, size_t length)
			{
				if (!USARTPeriphState.gpio_enabled || !USARTPeriphState.usart_enabled)
					return Status::PERIPH_NOT_INITIALIZED;

				auto statusCode = Status::PERIPH_ERROR;
				switch (rxMode)
				{
				case Modes::BLOCKING:
					/* It's possible to get into the condition where ORE is set before trying to receive some
					* new data. In the current STM HAL library, all error interrupts for the blocking mode are
					* disabled by default so the overrun has to be handled manually. This restores normal
					* operation. A nearly exact condition of this bug is encountered here: https://goo.gl/bKi8Ps
					**/
					USART_OverrunHandler();

					#if defined(USING_FREERTOS)
					if (HAL_USART_Receive(&usartHandle, buff, length, pdMS_TO_TICKS(BLOCKING_TIMEOUT_MS)) == HAL_OK)
						statusCode = Status::PERIPH_OK;
					#else
					if (HAL_USART_Receive(&usartHandle, buff, length, BLOCKING_TIMEOUT_MS) == HAL_OK)
						statusCode = Status::PERIPH_OK;
					#endif
					break;

				case Modes::INTERRUPT:
					/* Redirects the data storage from internal buffers to the one specified for this function */
					RX_ASYNC = false;

					if (HAL_USART_Receive_IT(&usartHandle, buff, length) == HAL_OK)
						statusCode = Status::PERIPH_RX_IN_PROGRESS;
					break;

				case Modes::DMA:
					/* Redirects the data storage from internal buffers to the one specified for this function */
					RX_ASYNC = false;

					if (HAL_USART_Receive_DMA(&usartHandle, buff, length) == HAL_OK)
						statusCode = Status::PERIPH_RX_IN_PROGRESS;
					break;

				default: break;
				}

				return statusCode;
			}

			uint32_t USARTClass::availablePackets()
			{
				return totalUnreadPackets;
			}

			size_t USARTClass::nextPacketSize()
			{
				if (RXPacketBuffer.empty())
					return (size_t)0;
				else
					return RXPacketBuffer.front().length;
			}

			void USARTClass::end()
			{
				USART_DeInit();
				USART_GPIO_DeInit();
				USART_DisableInterrupts();
				USART_DMA_DeInit(SubPeripheral::TX);
				USART_DMA_DeInit(SubPeripheral::RX);

				txMode = Modes::MODE_UNDEFINED;
				rxMode = Modes::MODE_UNDEFINED;
			}

			void USARTClass::IRQHandler()
			{
				#if defined(STM32F7)

				#endif

				#if defined(STM32F4)
				/** Reading these two in the order of SR then DR ends up clearing all flags, so it's best to store the returned
				 *	contents for further processing. See uart clearing procedure in device datasheet in the Registers section.
				 **/
				volatile uint32_t isrflags = READ_REG(usartHandle.Instance->SR);
				volatile uint32_t data_reg = READ_REG(usartHandle.Instance->DR);

				/*------------------------------------
				 * Handle Asynchronous RX (Interrupt and DMA Mode)
				 *------------------------------------*/
				if (RX_ASYNC)
				{
					volatile uint32_t cr1 = READ_REG(usartHandle.Instance->CR1);

					/* Prepare necessary flags for ISR flow control */
					bool RX_DATA_READY = ((isrflags & USART_FLAG_RXNE) == USART_FLAG_RXNE);
					bool RX_LINE_IDLE = ((isrflags & USART_FLAG_IDLE) == USART_FLAG_IDLE);
					bool RX_DATA_READY_IE = ((cr1 & USART_CR1_RXNEIE) == USART_CR1_RXNEIE);
					bool RX_LINE_IDLE_IE = ((cr1 & USART_CR1_IDLEIE) == USART_CR1_IDLEIE);

					/* RX In Progress */
					if (RX_DATA_READY && RX_DATA_READY_IE && usartHandle.State != HAL_USART_STATE_BUSY_TX)
					{
						uint32_t errorflags = (isrflags & (uint32_t)(USART_SR_PE | USART_SR_FE | USART_SR_ORE | USART_SR_NE));

						/* No Errors Found */
						if (errorflags == 0u)
						{
							/* Detected start of a new frame of unknown size. Enable the
							* IDLE interrupt bit to detect end of frame. */
							if (asyncRXDataSize == 0)
							{
								memset(RX_Queue[RXQueueIdx], 0, USART_QUEUE_BUFFER_SIZE);
								USART_EnableIT_IDLE(&usartHandle);
							}

							/* Buffer the new data and increase packet size count */
							if (rxMode == Modes::INTERRUPT && (asyncRXDataSize < USART_QUEUE_BUFFER_SIZE))
							{
								RX_Queue[RXQueueIdx][asyncRXDataSize] = (uint8_t)(data_reg & (uint8_t)0xFF);
								asyncRXDataSize += 1u;
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
							// All flags are cleared at this point, so it's only handling specific events.
						}
					}

					/* RX Complete */
					if (RX_LINE_IDLE && RX_LINE_IDLE_IE)
					{
						if (rxMode == Modes::INTERRUPT)
						{
							USART_DisableIT_IDLE(&usartHandle);

							/* Copy packets received to the internal buffer */
							RX_tempPacket.data_ptr = RX_Queue[RXQueueIdx];
							RX_tempPacket.length = asyncRXDataSize;
							RXPacketBuffer.push_back(RX_tempPacket);

							/* Clean up the class variables to prepare for a new reception */
							rx_complete = true;
							asyncRXDataSize = 0;
							totalUnreadPackets++;
							_rxIncrQueueIdx();

							/* Finally, call this here because the normal HAL_USART_IRQHandler does not get called
							* due to the asynchronous nature of operation. */
							HAL_USART_RxCpltCallback(&usartHandle);
						}

						else if (rxMode == Modes::DMA)
						{
							auto num_received = (size_t)(usartHandle.RxXferSize - usartHandle.hdmarx->Instance->NDTR);

							if (num_received != 0)
							{
								USART_DisableIT_IDLE(&usartHandle);
								rx_complete = true;
								totalUnreadPackets++;

								/* Force DMA hard reset to trigger the DMA RX Complete handler */
								__HAL_DMA_DISABLE(usartHandle.hdmarx);
							}
						}
					}
				}

				/*------------------------------------
				* Handle Synchronous RX or Asynchronous TX (Interrupt Mode)
				*------------------------------------*/
				if (!RX_ASYNC || usartHandle.State == HAL_USART_STATE_BUSY_TX)
					HAL_USART_IRQHandler(&usartHandle);
				#endif
			}

			void USARTClass::IRQHandler_TXDMA()
			{
				HAL_DMA_IRQHandler(usartHandle.hdmatx);
			}

			void USARTClass::IRQHandler_RXDMA()
			{
				HAL_DMA_IRQHandler(usartHandle.hdmarx);
			}

			#if defined(USING_FREERTOS)
			void USARTClass::attachThreadTrigger(Trigger trig, SemaphoreHandle_t* semphr)
			{
				usartTaskTrigger.attachEventConsumer(trig, semphr);
			}

			void USARTClass::removeThreadTrigger(Trigger trig)
			{
				usartTaskTrigger.removeEventConsumer(trig);
			}
			#endif

			USARTClass::USARTClass(const int& channel, SerialPins* pinConfig)
			{
				usartChannel = channel;

				/* Get the default handle settings for this instance */
				usartHandle.Init = Defaults::Serial::dflt_USART_Init;
				usartHandle.Instance = srl_cfg[usartChannel].instance;

				#if defined(STM32F7)
				//usartHandle.AdvancedInit = Defaults::Serial::
				#endif

				/* Get the default Interrupt settings for this instance */
				ITSettings_HW = srl_cfg[usartChannel].IT_HW;
				ITSettings_DMA_TX = srl_cfg[usartChannel].dmaIT_TX;
				ITSettings_DMA_RX = srl_cfg[usartChannel].dmaIT_RX;

				/* Set up the GPIO pins */
				if (pinConfig)
				{
					tx_pin = boost::make_shared<Thor::Peripheral::GPIO::GPIOClass>();
                    tx_pin->initAdvanced(
						pinConfig->TX_GPIOx,
						pinConfig->TX_Pin,
						PinSpeed::ULTRA_SPD,
						pinConfig->TX_AltFuncCode);

					rx_pin = boost::make_shared<Thor::Peripheral::GPIO::GPIOClass>();
                    rx_pin->initAdvanced(
						pinConfig->RX_GPIOx,
						pinConfig->RX_Pin,
						PinSpeed::ULTRA_SPD,
						pinConfig->RX_AltFuncCode);
				}
				else
				{
					tx_pin = boost::make_shared<Thor::Peripheral::GPIO::GPIOClass>();
                    tx_pin->initAdvanced(
						srl_cfg[usartChannel].txPin.GPIOx,
						srl_cfg[usartChannel].txPin.PinNum,
						srl_cfg[usartChannel].txPin.Speed,
						srl_cfg[usartChannel].txPin.Alternate);

					rx_pin = boost::make_shared<Thor::Peripheral::GPIO::GPIOClass>();
                    rx_pin->initAdvanced(
						srl_cfg[usartChannel].rxPin.GPIOx,
						srl_cfg[usartChannel].rxPin.PinNum,
						srl_cfg[usartChannel].rxPin.Speed,
						srl_cfg[usartChannel].rxPin.Alternate);
				}

				/* Initialize the buffer memory */
				TXPacketBuffer.set_capacity(USART_QUEUE_SIZE);	TXPacketBuffer.clear();
				RXPacketBuffer.set_capacity(USART_QUEUE_SIZE);	RXPacketBuffer.clear();


				#if defined(USING_FREERTOS)
				usartSemphrs[usartChannel] = xSemaphoreCreateCounting(USART_QUEUE_SIZE, USART_QUEUE_SIZE);
				#endif
			}

			USARTClass::~USARTClass()
			{
				this->end();
			}

			boost::shared_ptr<USARTClass> USARTClass::create(const int channel, SerialPins* pinConfig)
			{
				//TODO: Put runtime assertion here and gracefully fail if channel outside bounds

				//Forced to use this instead of make_shared due to required private constructor
				boost::shared_ptr<USARTClass> newClass(new USARTClass(channel, pinConfig));

				//Register the object so interrupt handlers know the correct reference
				usartObjects[channel] = newClass;

				return newClass;
			}

			uint8_t* USARTClass::assignTXBuffer(const uint8_t* data, const size_t length)
			{
				/* Move to the next array */
				txIncrQueueIdx();

				/* Overwrite the old data */
				memcpy(txCurrentQueueAddr(), data, length);

				return txCurrentQueueAddr();
			}

			void USARTClass::USART_GPIO_Init()
			{
				/* These should be configured as Thor::Definitions::GPIO::PinMode::ALT_PP with Thor::Definitions::GPIO::PinPull::PULLUP in order to work properly. Ignore srl_cfg settings. */
				if (tx_pin && rx_pin)
				{
					tx_pin->mode(Thor::Definitions::GPIO::PinMode::ALT_PP, Thor::Definitions::GPIO::PinPull::PULLUP);
					rx_pin->mode(Thor::Definitions::GPIO::PinMode::ALT_PP, Thor::Definitions::GPIO::PinPull::PULLUP);
					USARTPeriphState.gpio_enabled = true;
				}
				else
					USARTPeriphState.gpio_enabled = false;
			}

			void USARTClass::USART_GPIO_DeInit()
			{
				//TODO: Implement GPIO DeInit
			}

			void USARTClass::USART_Init()
			{
				USART_EnableClock();

				if (HAL_USART_Init(&usartHandle) != HAL_OK)
					BasicErrorHandler(logError("Failed USART init. Check settings."));

				setMode(SubPeripheral::TX, Modes::BLOCKING);
				setMode(SubPeripheral::RX, Modes::BLOCKING);

				USARTPeriphState.usart_enabled = true;
			}

			void USARTClass::USART_DeInit()
			{
				HAL_USART_DeInit(&usartHandle);
				USARTPeriphState.usart_enabled = false;
			}

			void USARTClass::USART_EnableClock()
			{
				*getUsartClockReg(usartHandle.Instance) |= (usartClockMask(usartHandle.Instance));
			}

			void USARTClass::USART_DisableClock()
			{
				*getUsartClockReg(usartHandle.Instance) &= ~(usartClockMask(usartHandle.Instance));
			}

			void USARTClass::USART_DMA_EnableClock()
			{
				/* Global DMA Clock options. Only turn on capability is
				provided due to other peripherals possibly using DMA. */
				if (__DMA1_IS_CLK_DISABLED())
					__DMA1_CLK_ENABLE();

				if (__DMA2_IS_CLK_DISABLED())
					__DMA2_CLK_ENABLE();
			}

			void USARTClass::USART_EnableInterrupts()
			{
				HAL_NVIC_DisableIRQ(ITSettings_HW.IRQn);
				HAL_NVIC_ClearPendingIRQ(ITSettings_HW.IRQn);
				HAL_NVIC_SetPriority(ITSettings_HW.IRQn, ITSettings_HW.preemptPriority, ITSettings_HW.subPriority);
				HAL_NVIC_EnableIRQ(ITSettings_HW.IRQn);

				/* Specific interrupts to enable */
				if (rxMode == Modes::INTERRUPT)
				{
					RX_ASYNC = true;
					__HAL_USART_ENABLE_IT(&usartHandle, USART_IT_RXNE); 	//RX Data Register not Empty
				}

				USARTPeriphState.usart_interrupts_enabled = true;
			}

			void USARTClass::USART_DisableInterrupts()
			{
				__HAL_USART_DISABLE_IT(&usartHandle, USART_IT_IDLE);
				__HAL_USART_DISABLE_IT(&usartHandle, USART_IT_RXNE);

				HAL_NVIC_DisableIRQ(ITSettings_HW.IRQn);
				HAL_NVIC_ClearPendingIRQ(ITSettings_HW.IRQn);

				USARTPeriphState.usart_interrupts_enabled = false;
			}

			void USARTClass::USART_DMA_Init(const SubPeripheral& periph)
			{
				if (periph == SubPeripheral::TX)
				{
					USART_DMA_EnableClock();
					hdma_usart_tx.Instance = srl_cfg[usartChannel].dmaTX.Instance;

					/* Grab the default init settings and then modify them for the specific hardware */
					hdma_usart_tx.Init = Defaults::Serial::dflt_DMA_Init_TX;
					hdma_usart_tx.Init.Channel = Defaults::Serial::srl_cfg[usartChannel].dmaTX.channel;
					hdma_usart_tx.Init.Direction = Defaults::Serial::srl_cfg[usartChannel].dmaTX.direction;

					/* Hard error if initialization fails */
					if (HAL_DMA_Init(&hdma_usart_tx) != HAL_OK)
						BasicErrorHandler(logError("Failed USART DMA TX Init. Check handle settings."));

					__HAL_LINKDMA(&usartHandle, hdmatx, hdma_usart_tx);

					serialDMAManager.attachCallback_TXDMA(usartChannel, boost::bind(&USARTClass::IRQHandler_TXDMA, this));

					USART_DMA_EnableIT(periph);

					USARTPeriphState.dma_enabled_tx = true;
				}
				else
				{
					USART_DMA_EnableClock();
					hdma_usart_rx.Instance = srl_cfg[usartChannel].dmaRX.Instance;

					/* Grab the default init settings and then modify them for the specific hardware */
					hdma_usart_rx.Init = Defaults::Serial::dflt_DMA_Init_RX;
					hdma_usart_rx.Init.Channel = Defaults::Serial::srl_cfg[usartChannel].dmaRX.channel;
					hdma_usart_rx.Init.Direction = Defaults::Serial::srl_cfg[usartChannel].dmaRX.direction;

					/* Hard error if initialization fails */
					if (HAL_DMA_Init(&hdma_usart_rx) != HAL_OK)
						BasicErrorHandler(logError("Failed USART DMA RX Init. Check handle settings."));

					__HAL_LINKDMA(&usartHandle, hdmarx, hdma_usart_rx);

					serialDMAManager.attachCallback_RXDMA(usartChannel, boost::bind(&USARTClass::IRQHandler_RXDMA, this));

					USART_DMA_EnableIT(periph);

					USARTPeriphState.dma_enabled_rx = true;
				}
			}

			void USARTClass::USART_DMA_DeInit(const SubPeripheral& periph)
			{
				if (periph == SubPeripheral::TX)
				{
					if (!USARTPeriphState.dma_enabled_tx)
						return;

					HAL_DMA_Abort(usartHandle.hdmatx);
					HAL_DMA_DeInit(usartHandle.hdmatx);
					USART_DMA_DisableIT(periph);
					serialDMAManager.removeCallback_TXDMA(usartChannel);

					USARTPeriphState.dma_enabled_tx = false;
				}
				else
				{
					if (!USARTPeriphState.dma_enabled_rx)
						return;

					HAL_DMA_Abort(usartHandle.hdmarx);
					HAL_DMA_DeInit(usartHandle.hdmarx);
					USART_DMA_DisableIT(periph);
					serialDMAManager.removeCallback_RXDMA(usartChannel);

					USARTPeriphState.dma_enabled_rx = false;
				}
			}

			void USARTClass::USART_DMA_EnableIT(const SubPeripheral& periph)
			{
				if (periph == SubPeripheral::TX)
				{
					HAL_NVIC_DisableIRQ(ITSettings_DMA_TX.IRQn);
					HAL_NVIC_ClearPendingIRQ(ITSettings_DMA_TX.IRQn);
					HAL_NVIC_SetPriority(ITSettings_DMA_TX.IRQn, ITSettings_DMA_TX.preemptPriority, ITSettings_DMA_TX.subPriority);
					HAL_NVIC_EnableIRQ(ITSettings_DMA_TX.IRQn);

					USARTPeriphState.dma_interrupts_enabled_tx = true;
				}
				else
				{
					HAL_NVIC_DisableIRQ(ITSettings_DMA_RX.IRQn);
					HAL_NVIC_ClearPendingIRQ(ITSettings_DMA_RX.IRQn);
					HAL_NVIC_SetPriority(ITSettings_DMA_RX.IRQn, ITSettings_DMA_RX.preemptPriority, ITSettings_DMA_RX.subPriority);
					HAL_NVIC_EnableIRQ(ITSettings_DMA_RX.IRQn);

					USARTPeriphState.dma_interrupts_enabled_rx = true;
				}
			}

			void USARTClass::USART_DMA_DisableIT(const SubPeripheral& periph)
			{
				if (periph == SubPeripheral::TX)
				{
					HAL_NVIC_ClearPendingIRQ(ITSettings_DMA_TX.IRQn);
					HAL_NVIC_DisableIRQ(ITSettings_DMA_TX.IRQn);

					USARTPeriphState.dma_interrupts_enabled_tx = false;
				}
				else
				{
					HAL_NVIC_ClearPendingIRQ(ITSettings_DMA_RX.IRQn);
					HAL_NVIC_DisableIRQ(ITSettings_DMA_RX.IRQn);

					USARTPeriphState.dma_interrupts_enabled_rx = false;
				}
			}

			void USARTClass::USART_OverrunHandler()
			{
				#if defined(STM32F7)
				__HAL_USART_CLEAR_IT(&usartHandle, USART_CLEAR_OREF);
				usartHandle.Instance->RDR;
				#endif
			}
		}
	}
}

void HAL_USART_TxCpltCallback(USART_HandleTypeDef *UsartHandle)
{
	/* Deduce at runtime which class object triggered this interrupt */
	auto usart = getUSARTClassRef(UsartHandle->Instance);

	if (usart && usart->_getInitStatus())
	{
		usart->_setTxComplete();

		/* Check if we have more data to send out */
		if (!usart->_txBufferEmpty())
		{
			auto packet = usart->_txBufferNextPacket();
			usart->write(packet.data_ptr, packet.length);

			/* Release the resource used for buffering */
			usart->_txBufferRemoveFrontPacket();
			#if defined(USING_FREERTOS)
			xSemaphoreGiveFromISR(usartSemphrs[usart->_getChannel()], NULL);
			#endif
		}
	}

	/* Signal any waiting threads */
	#if defined(USING_FREERTOS)
	usartTaskTrigger.logEvent(TX_COMPLETE, &usartTaskTrigger);
	#endif
}

void HAL_USART_RxCpltCallback(USART_HandleTypeDef *UsartHandle)
{
	/* Deduce at runtime which class object triggered this interrupt */
	auto usart = getUSARTClassRef(UsartHandle->Instance);

	if (usart && (usart->_getRxMode() == Modes::DMA))
	{
		usart->_setRxComplete();

		/* Calculate how many bytes were received by looking at remaining RX buffer space
		* num_received = bufferMaxSize - bufferSizeRemaining */
		size_t num_received = (size_t)(UsartHandle->RxXferSize - UsartHandle->hdmarx->Instance->NDTR);

		/* Construct the received packet and push into the receive queue */
		Thor::Peripheral::USART::USARTClass::USARTPacket tempPacket;
		tempPacket.data_ptr = usart->_rxCurrentQueueAddr();
		tempPacket.length = num_received;
		usart->_rxBufferPushBack(tempPacket);

		/* Increment the queue address pointer so we don't overwrite data */
		usart->_rxIncrQueueIdx();

		/* Start the listening process again for a new packet */
		USART_EnableIT_IDLE(UsartHandle);
		HAL_USART_Receive_DMA(UsartHandle, usart->_rxCurrentQueueAddr(), USART_QUEUE_BUFFER_SIZE);
	}

	/* Only runs if the user explicitly requests RX in blocking or interrupt mode */
	else if (usart && !usart->_getRxComplete())
	{
		usart->_setRxComplete();

		if (usart->_getRxMode() == Modes::INTERRUPT)
			usart->_setRxAsync();
	}

	/* Signal any waiting threads  */
	#if defined(USING_FREERTOS)
	usartTaskTrigger.logEvent(RX_COMPLETE, &usartTaskTrigger);
	#endif
}

void HAL_USART_TxHalfCpltCallback(USART_HandleTypeDef *UsartHandle)
{
}

void HAL_USART_RxHalfCpltCallback(USART_HandleTypeDef *UsartHandle)
{
}

void HAL_USART_ErrorCallback(USART_HandleTypeDef *UsartHandle)
{
}

void USART1_IRQHandler(void)
{
	if (usartObjects[1])
	{
		usartObjects[1]->IRQHandler();
	}
}

void USART2_IRQHandler(void)
{
	if (usartObjects[2])
	{
		usartObjects[2]->IRQHandler();
	}
}

void USART3_IRQHandler(void)
{
	if (usartObjects[3])
	{
		usartObjects[3]->IRQHandler();
	}
}

void USART4_IRQHandler(void)
{
	if (usartObjects[4])
	{
		usartObjects[4]->IRQHandler();
	}
}

void USART5_IRQHandler(void)
{
	if (usartObjects[5])
	{
		usartObjects[5]->IRQHandler();
	}
}

void USART6_IRQHandler(void)
{
	if (usartObjects[6])
	{
		usartObjects[6]->IRQHandler();
	}
}

void USART7_IRQHandler(void)
{
	if (usartObjects[7])
	{
		usartObjects[7]->IRQHandler();
	}
}

void USART8_IRQHandler(void)
{
	if (usartObjects[8])
	{
		usartObjects[8]->IRQHandler();
	}
}
