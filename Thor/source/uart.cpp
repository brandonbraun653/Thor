/* Boost Includes */
#include <boost/bind.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/static_vector.hpp>

/* Project Includes */
#include <Thor/include/uart.h>
#include <Thor/include/exceptions.h>
#include <Thor/include/interrupt.h>

using namespace Thor::Definitions::Serial;
using namespace Thor::Definitions::UART;
using namespace Thor::Definitions::Interrupt;
using namespace Thor::Peripheral::UART;
using namespace Thor::Peripheral::GPIO;
using namespace Thor::Defaults::Serial;


#if defined(USING_FREERTOS)
static boost::container::static_vector<SemaphoreHandle_t, MAX_SERIAL_CHANNELS + 1> uart_semphrs(MAX_SERIAL_CHANNELS + 1);

TaskTrigger uartTaskTrigger;

#endif

static boost::container::static_vector<UARTClass_sPtr, MAX_SERIAL_CHANNELS + 1> uartObjects(MAX_SERIAL_CHANNELS + 1);

static boost::container::flat_map<USART_TypeDef*, uint32_t> uartObjectIndex =
{
	#if defined(UART1)
	{ UART1, 1 },
	#endif
	#if defined(UART2)
	{ UART2, 2 },
	#endif
	#if defined(UART3)
	{ UART3, 3 },
	#endif
	#if defined(UART4)
	{ UART4, 4 },
	#endif
	#if defined(UART5)
	{ UART5, 5 },
	#endif
	#if defined(UART6)
	{ UART6, 6 },
	#endif
	#if defined(UART7)
	{ UART7, 7 },
	#endif
	#if defined(UART8)
	{ UART8, 8 },
	#endif
};

static boost::container::flat_map<USART_TypeDef*, uint32_t> uartClockMask =
{
	#if defined(STM32F446xx) || defined(STM32F767xx)
		#if defined(UART4)
	{ UART4, RCC_APB1ENR_UART4EN },
	#endif
	#if defined(UART5)
	{ UART5, RCC_APB1ENR_UART5EN },
	#endif
	#if defined(UART7)
	{ UART7, RCC_APB1ENR_UART7EN },
	#endif
	#if defined(UART8)
	{ UART8, RCC_APB1ENR_UART8EN },
	#endif
#endif
};


namespace Thor
{
	namespace Peripheral
	{
		namespace UART
		{
			void UART_EnableIT_IDLE(UART_HandleTypeDef *UartHandle)
			{
				#if defined(STM32F7) || defined(STM32F4)
				__HAL_UART_CLEAR_IT(UartHandle, UART_CLEAR_IDLEF);
				__HAL_UART_ENABLE_IT(UartHandle, UART_IT_IDLE);
				#endif
			}
			
			void UART_DisableIT_IDLE(UART_HandleTypeDef *UartHandle)
			{
				#if defined(STM32F7) || defined(STM32F4)
				__HAL_UART_DISABLE_IT(UartHandle, UART_IT_IDLE);
				__HAL_UART_CLEAR_IT(UartHandle, UART_CLEAR_IDLEF);
				#endif
			}
			
			void UART_ClearIT_IDLE(UART_HandleTypeDef *UartHandle)
			{
				#if defined(STM32F7) || defined(STM32F4)
				__HAL_UART_CLEAR_IT(UartHandle, UART_CLEAR_IDLEF);
				#endif
			}
			
			
			UARTClass::UARTClass(const int& channel)
			{
				uart_channel = channel;

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
				tx_pin = boost::make_shared<Thor::Peripheral::GPIO::GPIOClass>(
					srl_cfg[uart_channel].txPin.GPIOx,
					srl_cfg[uart_channel].txPin.PinNum,
					srl_cfg[uart_channel].txPin.Speed,
					srl_cfg[uart_channel].txPin.Alternate);

				rx_pin = boost::make_shared<Thor::Peripheral::GPIO::GPIOClass>(
					srl_cfg[uart_channel].rxPin.GPIOx,
					srl_cfg[uart_channel].rxPin.PinNum,
					srl_cfg[uart_channel].rxPin.Speed,
					srl_cfg[uart_channel].rxPin.Alternate);

				/* Initialize the buffer memory */
				TXPacketBuffer.set_capacity(UART_QUEUE_SIZE);	TXPacketBuffer.clear();
				RXPacketBuffer.set_capacity(UART_QUEUE_SIZE);	RXPacketBuffer.clear();
				
				
				//Sometimes the buffer initializes with random data, which is bad...
				//TODO: Check what causes this error ^^^
				UARTPacket tmp;
				tmp.data_ptr = nullptr;
				tmp.length = 0;
				tmp.bytesRead = 0;
				
				for (int i = 0; i < RXPacketBuffer.capacity(); i++)
					RXPacketBuffer.push_back(tmp);
				
				//Should reset the begin/end pointers? 
				RXPacketBuffer.clear();

				#if defined(USING_FREERTOS)
				uart_semphrs[uart_channel] = xSemaphoreCreateCounting(UART_QUEUE_SIZE, UART_QUEUE_SIZE);
				#endif
			}

			UARTClass::~UARTClass()
			{
				end();
			}

			boost::shared_ptr<UARTClass> UARTClass::create(const int channel)
			{
				//TODO: Put runtime assertion here and gracefully fail if channel outside bounds

				//Forced to use this instead of make_shared due to required private constructor
				boost::shared_ptr<UARTClass> newClass(new UARTClass(channel));

				//Register the object so interrupt handlers know the correct reference
				uartObjects[channel] = newClass;

				return newClass;
			}

			Status UARTClass::begin(const BaudRate& baud, const Modes& tx_mode, const Modes& rx_mode)
			{
				UART_GPIO_Init();

				uart_handle.Init.BaudRate = baud;
				UART_Init();

				setMode(TX, tx_mode);
				setMode(RX, rx_mode);

				return PERIPH_OK;
			}

			Status UARTClass::setMode(const SubPeripheral& periph, const Modes& mode)
			{
				if (periph == TX)
				{
					switch (mode)
					{
					case BLOCKING:
						txMode = mode; // Must be set before the other functions 
						
						/* Make sure RX side isn't using interrupts before disabling */
						if (rxMode == BLOCKING)
							UART_DisableInterrupts();

						UART_DMA_DeInit(periph);
						break;

					case INTERRUPT:
						txMode = mode; // Must be set before the other functions 
						
						UART_EnableInterrupts();
						UART_DMA_DeInit(periph);
						break;

					case DMA:
						txMode = mode; // Must be set before the other functions 
						
						UART_EnableInterrupts();
						UART_DMA_Init(periph);
						break;

					default:
						txMode = MODE_UNDEFINED;
						return PERIPH_ERROR;
						break;
					}

					return PERIPH_OK;
				}
				else
				{
					switch (mode)
					{
					case BLOCKING:
						rxMode = mode; // Must be set before the other functions 
						
						/* Make sure TX side isn't using interrupts before disabling */
						if (txMode == BLOCKING)
							UART_DisableInterrupts();

						UART_DMA_DeInit(periph);
						
						break;

					case INTERRUPT:
						rxMode = mode;	// Must be set before the other functions 
						
						UART_EnableInterrupts();
						UART_DMA_DeInit(periph);
						break;

					case DMA:
						rxMode = mode; // Must be set before the other functions 
						
						UART_EnableInterrupts();
						UART_DMA_Init(periph);

						/* Set the idle line interrupt for asynchronously getting the end of transmission */
						UART_EnableIT_IDLE(&uart_handle);
						
						/* Instruct the DMA hardware to start listening for transmissions */
						HAL_UART_Receive_DMA(&uart_handle, RX_Queue[RXQueueIdx], UART_QUEUE_BUFFER_SIZE);
						break;

					default:
						rxMode = MODE_UNDEFINED;
						return PERIPH_ERROR;
						break;
					}

					return PERIPH_OK;
				}
			}

			Status UARTClass::write(char* string, size_t length)
			{
				return write((uint8_t*)string, length);
			}

			Status UARTClass::write(const char* string)
			{
				return write((uint8_t*)string, strlen(string));
			}

			Status UARTClass::write(const char* string, size_t length)
			{
				return write((uint8_t*)string, length);
			}

			Status UARTClass::write(uint8_t* val, size_t length)
			{
				if (!UART_PeriphState.gpio_enabled || !UART_PeriphState.uart_enabled)
					return PERIPH_NOT_INITIALIZED;

				auto statusCode = PERIPH_ERROR;
				auto halCode = HAL_OK;

				switch (txMode)
				{
				case BLOCKING:
					#if defined(USING_FREERTOS)
						halCode = HAL_UART_Transmit(&uart_handle, val, length, pdMS_TO_TICKS(BLOCKING_TIMEOUT_MS));
					#else 
						halCode = HAL_UART_Transmit(&uart_handle, val, length, BLOCKING_TIMEOUT_MS));
					#endif

					if (halCode == HAL_TIMEOUT)
						statusCode = PERIPH_TIMEOUT;
					else
						statusCode = PERIPH_OK;
					break;

				case INTERRUPT:
					if (tx_complete)
					{
						/* Starting a brand new IT transmission */
						tx_complete = false;
						HAL_UART_Transmit_IT(&uart_handle, assignTXBuffer(val, length), length);
						statusCode = PERIPH_TX_IN_PROGRESS;
					}
					else
					{
						statusCode = PERIPH_NOT_READY;
						
						#if defined(USING_FREERTOS)
						if (xSemaphoreTakeFromISR(uart_semphrs[uart_channel], NULL) != pdPASS)
						{
							statusCode = PERIPH_LOCKED;
							break;
						}
						#endif

						/* A previous IT transmission is still going. Queue the data */
						TX_tempPacket.data_ptr = assignTXBuffer(val, length);
						TX_tempPacket.length = length;
						TXPacketBuffer.push_back(TX_tempPacket);
					}
					break;

				case DMA:
					if (tx_complete)
					{
						/* Starting a brand new DMA transmission */
						tx_complete = false;
						HAL_UART_Transmit_DMA(&uart_handle, assignTXBuffer(val, length), length);
						statusCode = PERIPH_TX_IN_PROGRESS;
					}
					else
					{
						statusCode = PERIPH_NOT_READY;
						
						#if defined(USING_FREERTOS)
						if (xSemaphoreTakeFromISR(uart_semphrs[uart_channel], NULL) != pdPASS)
						{
							statusCode = PERIPH_LOCKED;
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
					statusCode = PERIPH_ERROR; 
					break;
				}
				return statusCode;
			}

			Status UARTClass::readSync(uint8_t* buff, size_t length)
			{
				if (!UART_PeriphState.gpio_enabled || !UART_PeriphState.uart_enabled)
					return PERIPH_NOT_INITIALIZED;

				auto statusCode = PERIPH_ERROR;
				switch (rxMode)
				{
				case BLOCKING:
					/* It's possible to get into the condition where ORE is set before trying to receive some
					 * new data. In the current STM HAL library, all error interrupts for the blocking mode are 
					 * disabled by default so the overrun has to be handled manually. This restores normal
					 * operation. A nearly exact condition of this bug is encountered here: https://goo.gl/bKi8Ps 
					 **/
					UART_OverrunHandler();
					
					
					#if defined(USING_FREERTOS)
					if (HAL_UART_Receive(&uart_handle, buff, length, pdMS_TO_TICKS(BLOCKING_TIMEOUT_MS)) == HAL_OK)
						statusCode = PERIPH_OK;
					#else
					if (HAL_UART_Receive(&uart_handle, buff, length, BLOCKING_TIMEOUT_MS) == HAL_OK)
						statusCode = PERIPH_OK;
					#endif
					break;

				case INTERRUPT:
					/* Redirects the data storage from internal buffers to the one specified for this function */
					RX_ASYNC = false;
					
					if (HAL_UART_Receive_IT(&uart_handle, buff, length) == HAL_OK)
						statusCode = PERIPH_RX_IN_PROGRESS;
					break;

				case DMA:
					/* Redirects the data storage from internal buffers to the one specified for this function */
					RX_ASYNC = false;
					
					if (HAL_UART_Receive_DMA(&uart_handle, buff, length) == HAL_OK)
						statusCode = PERIPH_RX_IN_PROGRESS;
					break;

				default: break;
				}

				return statusCode;
			}

			Status UARTClass::readPacket(uint8_t* buff, size_t length)
			{
				Status error = PERIPH_OK;
				UARTPacket packet = RXPacketBuffer.front();
				
				/* First make sure we have a valid address */
				if (packet.data_ptr)
				{
					size_t readLength = packet.length;
					if (readLength > length)
					{
						readLength = length;
						//error = PERIPH_PACKET_TOO_LARGE_FOR_BUFFER;
					}
					
					if (packet.bytesRead == 0)
					{
						memcpy(buff, packet.data_ptr, readLength);
					}
					else
					{
						memcpy(buff, packet.data_ptr + packet.bytesRead, readLength);
					}
					
					/* Check if this read operation will completely empty the 
					 * stored packet data. If so, we are free to pop it off the buffer */
					if (length + packet.bytesRead >= packet.length)
					{
						RXPacketBuffer.pop_front();
						totalWaitingPackets--;
					}
					else /* Reinsert the modified packet since we aren't done yet */
					{
						packet.bytesRead += length;
						RXPacketBuffer.insert(RXPacketBuffer.begin(), packet);
						
						auto tmp = RXPacketBuffer.front();
						auto newLen = tmp.bytesRead;
					}
					
				}
				else
				{
					error = PERIPH_ERROR; //TODO: More meaningful message here
				}
				
				return error;
			}

			size_t UARTClass::nextPacketSize()
			{
				if (RXPacketBuffer.empty())
					return (size_t)0;
				else
					return RXPacketBuffer.front().length;
			}

			int UARTClass::availablePackets()
			{
				return totalWaitingPackets;
			}

			void UARTClass::end()
			{
				UART_DeInit();
				UART_GPIO_DeInit();
				UART_DisableInterrupts();
				UART_DMA_DeInit(SubPeripheral::TX);
				UART_DMA_DeInit(SubPeripheral::RX);

				txMode = MODE_UNDEFINED;
				rxMode = MODE_UNDEFINED;
			}

			void UARTClass::attachSettings(UART_InitTypeDef config)
			{
				uart_handle.Init = config;

				if (UART_PeriphState.uart_enabled)
					UART_DeInit();

				UART_Init();
			}

			void UARTClass::IRQHandler()
			{
				/** NOTE: The way some of these interrupts are handled may seem weird, but it's to protect against
				 *	instances where the ISR is randomly triggered and there really shouldn't be any processing going on.
				 *	While a few sources of these errant events have been tracked down, not all are found. Thus, it is 
				 *	easier to structure the code in such a way that it only runs exactly when certain conditions are met.
				 **/
				
				#if defined(STM32F7)
				bool RX_DATA_READY		= __HAL_UART_GET_FLAG(&uart_handle, UART_FLAG_RXNE);
				bool RX_LINE_IDLE		= __HAL_UART_GET_FLAG(&uart_handle, UART_FLAG_IDLE);
				bool RX_LINE_IDLE_EN	= __HAL_UART_GET_IT_SOURCE(&uart_handle, UART_IT_IDLE);

				/*------------------------------------
				* Handle Asynchronous RX (Interrupt and DMA Mode)
				*------------------------------------*/
				/* RX In Progress */
				if (RX_ASYNC && RX_DATA_READY && uart_handle.gState != HAL_UART_STATE_BUSY_TX)
				{
					uint32_t isrflags = READ_REG(uart_handle.Instance->ISR);
					uint32_t errorflags = (isrflags & (uint32_t)(USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE));

					if (errorflags == RESET)
					{
						/* Detected new RX of unknown size */
						if (asyncRXDataSize == 0)
						{
							memset(RX_Queue[RXQueueIdx], 0, UART_QUEUE_BUFFER_SIZE);
							
							/* Enable UART_IT_IDLE to detect transmission end.
							 * Sometimes IDLEF is set before interrupt enable, which will immediately trigger this ISR,
							 * and cause reception of only 1 character. Clear first to ensure accurate idle line trigger. */
							__HAL_UART_CLEAR_IT(&uart_handle, UART_CLEAR_IDLEF);
							__HAL_UART_ENABLE_IT(&uart_handle, UART_IT_IDLE);
						}

						/* Buffer the new data */
						if (rxMode == INTERRUPT && (asyncRXDataSize < UART_QUEUE_BUFFER_SIZE))
						{
							RX_Queue[RXQueueIdx][asyncRXDataSize] = (uint8_t)(uart_handle.Instance->RDR & (uint8_t)0xFF);
							asyncRXDataSize += 1u;
						}
						else
						{
							/* Forced to read data without being able to store it.
							 * Really need to put some kind of error thing here...*/
							uart_handle.Instance->RDR;
						}
					}
					else
					{
						//Do something more useful later
						__HAL_UART_CLEAR_IT(&uart_handle, UART_CLEAR_PEF);
						__HAL_UART_CLEAR_IT(&uart_handle, UART_CLEAR_FEF);
						__HAL_UART_CLEAR_IT(&uart_handle, UART_CLEAR_NEF);
						__HAL_UART_CLEAR_IT(&uart_handle, UART_CLEAR_OREF);
					}
				}

				/* RX Complete */
				if (RX_ASYNC && RX_LINE_IDLE_EN && RX_LINE_IDLE)
				{
					if (rxMode == INTERRUPT)
					{
						UART_DisableIT_IDLE(&uart_handle);
						
						/* Copy data received to the internal buffer */
						RX_tempPacket.data_ptr  = RX_Queue[RXQueueIdx];
						RX_tempPacket.length	= asyncRXDataSize;
						RXPacketBuffer.push_back(RX_tempPacket);
						
						/* Clean up the class variables to prepare for a new reception */
						rx_complete = true;
						asyncRXDataSize = 0;	
						totalWaitingPackets++;
						_rxIncrQueueIdx();
					}
					else if (rxMode == DMA)
					{
						auto num_received = (size_t)(uart_handle.RxXferSize - uart_handle.hdmarx->Instance->NDTR);
						
						if (num_received != 0)
						{
							UART_DisableIT_IDLE(&uart_handle);
							rx_complete = true;
							totalWaitingPackets++;
						
							/* Force DMA hard reset to trigger the DMA RX Complete handler */
							__HAL_DMA_DISABLE(uart_handle.hdmarx);
						}
						else
						{
							/* ISR was randomly triggered. Clear any errant flags that might be enabled.*/
							UART_ClearIT_IDLE(&uart_handle);
						}
							
					}
				}

				/*------------------------------------
				* Handle Synchronous RX or Asynchronous TX (Interrupt Mode)
				*------------------------------------*/
				if (!RX_ASYNC || uart_handle.gState == HAL_UART_STATE_BUSY_TX)
				{
					HAL_UART_IRQHandler(&uart_handle);
				}
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
								memset(packetQueue[currentQueuePacket], 0, Thor::Definitions::Serial::UART_BUFFER_SIZE);
								__HAL_UART_ENABLE_IT(&uart_handle, UART_IT_IDLE);
							}

							/* Buffer the new data and increase packet size count */
							if (rxMode == RX_MODE_INTERRUPT && (rxAsyncPacketSize < Thor::Definitions::Serial::UART_BUFFER_SIZE))
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

							rxAsyncPacketSize = 0; /* Reset the internal packet counter so we know when a new frame starts */
							currentQueuePacket++; /* Go to the next buffer location */

							if (currentQueuePacket == Thor::Definitions::Serial::UART_PACKET_QUEUE_SIZE)
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
				if(!RX_ASYNC || uart_handle.gState == HAL_UART_STATE_BUSY_TX)
					HAL_UART_IRQHandler(&uart_handle);
				#endif
			}

			void UARTClass::IRQHandler_TXDMA()
			{
				HAL_DMA_IRQHandler(uart_handle.hdmatx);
			}

			void UARTClass::IRQHandler_RXDMA()
			{
				HAL_DMA_IRQHandler(uart_handle.hdmarx);
			}

			#if defined(USING_FREERTOS)
			void UARTClass::attachThreadTrigger(Trigger trig, SemaphoreHandle_t* semphr)
			{
				uartTaskTrigger.attachEventConsumer(trig, semphr);
			}
			
			void UARTClass::removeThreadTrigger(Trigger trig)
			{
				uartTaskTrigger.removeEventConsumer(trig);
			}
			
			#endif 
			
			void UARTClass::UART_Init()
			{
				UART_EnableClock();

				if (HAL_UART_Init(&uart_handle) != HAL_OK)
					BasicErrorHandler(logError("Failed UART Init. Check settings."));

				setMode(TX, BLOCKING);
				setMode(RX, BLOCKING);

				UART_PeriphState.uart_enabled = true;
			}

			void UARTClass::UART_DeInit()
			{
				HAL_UART_DeInit(&uart_handle);
				UART_PeriphState.uart_enabled = false;
			}

			void UARTClass::UART_EnableClock()
			{
				using namespace Thor::Definitions::Serial;

				#if defined(TARGET_STM32F7) || defined(TARGET_STM32F4)
				RCC->APB1ENR |= (uartClockMask[uart_handle.Instance]);
				#endif
			}

			void UARTClass::UART_DisableClock()
			{
				using namespace Thor::Definitions::Serial;

				#if defined(TARGET_STM32F7) || defined(TARGET_STM32F4)
				RCC->APB1ENR &= ~(uartClockMask[uart_handle.Instance]);
				#endif
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

			void UARTClass::UART_EnableInterrupts()
			{
				HAL_NVIC_DisableIRQ(ITSettings_HW.IRQn);
				HAL_NVIC_ClearPendingIRQ(ITSettings_HW.IRQn);
				HAL_NVIC_SetPriority(ITSettings_HW.IRQn, ITSettings_HW.preemptPriority, ITSettings_HW.subPriority);
				HAL_NVIC_EnableIRQ(ITSettings_HW.IRQn);

				/* Specific interrupts to enable */
				if (rxMode == INTERRUPT)
				{
					RX_ASYNC = true;
					__HAL_UART_ENABLE_IT(&uart_handle, UART_IT_RXNE); 	//RX Data Register not Empty
				}
					

				UART_PeriphState.uart_interrupts_enabled = true;
			}

			void UARTClass::UART_DisableInterrupts()
			{
				__HAL_UART_DISABLE_IT(&uart_handle, UART_IT_IDLE);
				__HAL_UART_DISABLE_IT(&uart_handle, UART_IT_RXNE);

				HAL_NVIC_DisableIRQ(ITSettings_HW.IRQn);
				HAL_NVIC_ClearPendingIRQ(ITSettings_HW.IRQn);

				UART_PeriphState.uart_interrupts_enabled = false;
			}

			void UARTClass::UART_GPIO_Init()
			{
				/* These should be configured as ALT_PP with PULLUPs */
				if (tx_pin)
					tx_pin->mode(srl_cfg[uart_channel].txPin.Mode, srl_cfg[uart_channel].rxPin.Pull);

				if (rx_pin)
					rx_pin->mode(srl_cfg[uart_channel].rxPin.Mode, srl_cfg[uart_channel].rxPin.Pull);

				UART_PeriphState.gpio_enabled = true;
			}

			void UARTClass::UART_GPIO_DeInit()
			{
				//TODO: Implement GPIO DeInit
			}

			void UARTClass::UART_DMA_Init(const SubPeripheral& periph)
			{
				if (periph == SubPeripheral::TX)
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

					uart_dma_manager.attachCallbackFunction_TXDMA(uart_channel, boost::bind(&UARTClass::IRQHandler_TXDMA, this));

					UART_DMA_EnableIT(periph);

					UART_PeriphState.dma_enabled_tx = true;
				}
				else
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

					uart_dma_manager.attachCallbackFunction_RXDMA(uart_channel, boost::bind(&UARTClass::IRQHandler_RXDMA, this));

					UART_DMA_EnableIT(periph);

					UART_PeriphState.dma_enabled_rx = true;
				}
			}

			void UARTClass::UART_DMA_DeInit(const SubPeripheral& periph)
			{
				if (periph == SubPeripheral::TX)
				{
					if (!UART_PeriphState.dma_enabled_tx)
						return;

					HAL_DMA_Abort(uart_handle.hdmatx);
					HAL_DMA_DeInit(uart_handle.hdmatx);
					UART_DMA_DisableIT(periph);
					uart_dma_manager.removeCallbackFunction_TXDMA(uart_channel);

					UART_PeriphState.dma_enabled_tx = false;
				}
				else
				{
					if (!UART_PeriphState.dma_enabled_rx)
						return;

					HAL_DMA_Abort(uart_handle.hdmarx);
					HAL_DMA_DeInit(uart_handle.hdmarx);
					UART_DMA_DisableIT(periph);
					uart_dma_manager.removeCallbackFunction_RXDMA(uart_channel);

					UART_PeriphState.dma_enabled_rx = false;
				}
			}

			void UARTClass::UART_DMA_EnableIT(const SubPeripheral& periph)
			{
				if (periph == SubPeripheral::TX)
				{
					HAL_NVIC_DisableIRQ(ITSettings_DMA_TX.IRQn);
					HAL_NVIC_ClearPendingIRQ(ITSettings_DMA_TX.IRQn);
					HAL_NVIC_SetPriority(ITSettings_DMA_TX.IRQn, ITSettings_DMA_TX.preemptPriority, ITSettings_DMA_TX.subPriority);
					HAL_NVIC_EnableIRQ(ITSettings_DMA_TX.IRQn);

					UART_PeriphState.dma_interrupts_enabled_tx = true;
				}
				else
				{
					HAL_NVIC_DisableIRQ(ITSettings_DMA_RX.IRQn);
					HAL_NVIC_ClearPendingIRQ(ITSettings_DMA_RX.IRQn);
					HAL_NVIC_SetPriority(ITSettings_DMA_RX.IRQn, ITSettings_DMA_RX.preemptPriority, ITSettings_DMA_RX.subPriority);
					HAL_NVIC_EnableIRQ(ITSettings_DMA_RX.IRQn);

					UART_PeriphState.dma_interrupts_enabled_rx = true;
				}
			}

			void UARTClass::UART_DMA_DisableIT(const SubPeripheral& periph)
			{
				if (periph == SubPeripheral::TX)
				{
					HAL_NVIC_ClearPendingIRQ(ITSettings_DMA_TX.IRQn);
					HAL_NVIC_DisableIRQ(ITSettings_DMA_TX.IRQn);

					UART_PeriphState.dma_interrupts_enabled_tx = false;
				}
				else
				{
					HAL_NVIC_ClearPendingIRQ(ITSettings_DMA_RX.IRQn);
					HAL_NVIC_DisableIRQ(ITSettings_DMA_RX.IRQn);

					UART_PeriphState.dma_interrupts_enabled_rx = false;
				}
			}

			void UARTClass::UART_OverrunHandler()
			{
				#if defined(STM32F7)
				__HAL_UART_CLEAR_IT(&uart_handle, UART_CLEAR_OREF);
				uart_handle.Instance->RDR;
				#endif 
			}
			
			uint8_t* UARTClass::assignTXBuffer(const uint8_t* data, const size_t length)
			{
				/* Move to the next array */
				txIncrQueueIdx();
				
				/* Overwrite the old data */
				//memset(_txCurrentQueueAddr(), 0, UART_QUEUE_BUFFER_SIZE); //Likely not needed 
				memcpy(txCurrentQueueAddr(), data, length);
				
				return txCurrentQueueAddr();
			}
		}
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	/* Deduce at runtime which class object triggered this interrupt */
	uint32_t uartIdx = uartObjectIndex[UartHandle->Instance];
	auto uart = uartObjects[uartIdx];

	if (uart && uart->_getInitStatus())
	{
		uart->_setTxComplete();

		/* Check if we have more data to send out */
		if (!uart->_txBufferEmpty())
		{
			auto packet = uart->_txBufferNextPacket();
			uart->write(packet.data_ptr, packet.length);

			/* Release the resource used for buffering */
			uart->_txBufferRemoveFrontPacket();
			#if defined(USING_FREERTOS)
			xSemaphoreGiveFromISR(uart_semphrs[uart->_getChannel()], NULL);
			#endif
		}
	}
	
	/* Signal any waiting threads  */
	#if defined(USING_FREERTOS)
	uartTaskTrigger.logEvent(TX_COMPLETE, &uartTaskTrigger);
	#endif
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	/* Deduce at runtime which class object triggered this interrupt */
	uint32_t uartIdx = uartObjectIndex[UartHandle->Instance];
	auto uart = uartObjects[uartIdx];

	if (uart->_getRxMode() == DMA)
	{
		uart->_setRxComplete();

		/* Calculate how many bytes were received by looking at remaining RX buffer space
		 * num_received = bufferMaxSize - bufferSizeRemaining */
		size_t num_received = (size_t)(UartHandle->RxXferSize - UartHandle->hdmarx->Instance->NDTR);

		/* Construct the received packet and push into the receive queue */
		Thor::Peripheral::UART::UARTClass::UARTPacket tempPacket;
		tempPacket.data_ptr = uart->_rxCurrentQueueAddr();
		tempPacket.length	= num_received;
		uart->_rxBufferPushBack(tempPacket);
		
		/* Increment the queue address pointer so we don't overwrite data */
		uart->_rxIncrQueueIdx();
		
		/* Start the listening process again for a new packet */
		UART_EnableIT_IDLE(UartHandle);
		HAL_UART_Receive_DMA(UartHandle, uart->_rxCurrentQueueAddr(), UART_QUEUE_BUFFER_SIZE);
	}

	/* Only runs if the user explicitly requests RX in blocking or interrupt mode */
	else if(!uart->_getRxComplete())
	{
		uart->_setRxComplete();

		if (uart->_getRxMode() == INTERRUPT)
			uart->_setRxAsync();
	}
	
	/* Signal any waiting threads  */
	#if defined(USING_FREERTOS)
	uartTaskTrigger.logEvent(RX_COMPLETE, &uartTaskTrigger);
	#endif
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

void UART1_IRQHandler(void)
{
	if (uartObjects[1])
	{
		uartObjects[1]->IRQHandler();
	}
}

void UART2_IRQHandler(void)
{
	if (uartObjects[2])
	{
		uartObjects[2]->IRQHandler();
	}
}

void UART3_IRQHandler(void)
{
	if (uartObjects[3])
	{
		uartObjects[3]->IRQHandler();
	}
}

void UART4_IRQHandler(void)
{
	if (uartObjects[4])
	{
		uartObjects[4]->IRQHandler();
	}
}

void UART5_IRQHandler(void)
{
	if (uartObjects[5])
	{
		uartObjects[5]->IRQHandler();
	}
}

void UART6_IRQHandler(void)
{
	if (uartObjects[6])
	{
		uartObjects[6]->IRQHandler();
	}
}

void UART7_IRQHandler(void)
{
	if (uartObjects[7])
	{
		uartObjects[7]->IRQHandler();
	}
}

void UART8_IRQHandler(void)
{
	if (uartObjects[8])
	{
		uartObjects[8]->IRQHandler();
	}
}