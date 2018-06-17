#include <Thor/include/interrupt.hpp>

/* This macro becomes a problem if a chip doesn't have some of these SPI channels.... */
#if defined(STM32F767xx)
#define IS_SPI_INSTANCE(__INSTANCE__) (((__INSTANCE__) == SPI1)		|| \
										((__INSTANCE__) == SPI2)	|| \
                                        ((__INSTANCE__) == SPI3)	|| \
                                        ((__INSTANCE__) == SPI4)	|| \
                                        ((__INSTANCE__) == SPI5)	|| \
										((__INSTANCE__) == SPI6))
#endif

#if defined(STM32F446xx)
#define IS_SPI_INSTANCE(__INSTANCE__) (((__INSTANCE__) == SPI1)		|| \
										((__INSTANCE__) == SPI2)	|| \
                                        ((__INSTANCE__) == SPI3)	|| \
                                        ((__INSTANCE__) == SPI4))
#endif
namespace Thor
{
	namespace Interrupt
	{
		namespace DMA
		{
			DMAHandler::DMAHandler() {}
			DMAHandler::~DMAHandler() {}

			void DMAHandler::updateDMASource(uint32_t periph, uint32_t stream)
			{
				DMA_Periph = periph;
				DMA_Stream = stream;
			}

			void DMAHandler::copyRegisters(uint32_t REG_LISR, uint32_t REG_HISR, uint32_t REG_SxCR, uint32_t REG_SxPAR)
			{
				REG_DMA_LISR = REG_LISR;
				REG_DMA_HISR = REG_HISR;
				REG_DMA_SxCR = REG_SxCR;
				REG_DMA_SxPAR = REG_SxPAR;
			}

			bool DMAHandler::calculatePeriphSource(PeriphConfig& output)
			{
				bool source_found = true;

				/*-------------------------------
				* Get the Transfer Direction
				*-------------------------------*/
				switch (((REG_DMA_SxCR & DMA_SxCR_DIR_Msk) >> DMA_SxCR_DIR_Pos))
				{
				case 0u:
					output.direction = Thor::Definitions::DMA::PERIPH_TO_MEM;
					break;

				case 1u:
					output.direction = Thor::Definitions::DMA::MEM_TO_PERIPH;
					break;

				case 2u:
					output.direction = Thor::Definitions::DMA::MEM_TO_MEM;
					break;

				default:
					output.direction = Thor::Definitions::DMA::TRANSFER_DIRECTION_UNDEFINED;
					source_found = false;
					break;
				}

				/*-------------------------------
				* Get the Channel Selection
				* TODO: Will I really even need this?
				*-------------------------------*/
				output.channel_selection = ((REG_DMA_SxCR & DMA_SxCR_CHSEL_Msk) >> DMA_SxCR_CHSEL_Pos);

				/*-------------------------------
				* Get the Peripheral Type/Instance
				*-------------------------------*/
				/* All of the peripherals are derived off a base address with some offset. Checking
				* the base address with a few ST provided macros shortens the time required to
				* reverse search for the exact peripheral used.
				*
				* Because it is not certain whether all peripheral addressing follows the same
				* structure, each peripheral is handled on a case by case basis.
				* */
				
				//TODO: This looks like a great candidate for that map lookup system...

				void* uart_candidate = (void*)(REG_DMA_SxPAR & 0xFFFFFF00);
				void* spi_candidate = (void*)(REG_DMA_SxPAR & 0xFFFFFF00);
				//add others as needed

				/* UART/USART PERIPHERALS */
				if (IS_UART_INSTANCE(uart_candidate))
				{
					if (IS_USART_INSTANCE(uart_candidate))
					{
						output.peripheral_type = Thor::Interrupt::SRC_USART;

						#if defined(USART1)
						if (uart_candidate == USART1)
							output.peripheral_instance = Thor::Interrupt::SRC_USART1;
						#endif

						#if defined(USART2)
						if (uart_candidate == USART2)
							output.peripheral_instance = Thor::Interrupt::SRC_USART2;
						#endif

						#if defined(USART3)
						if (uart_candidate == USART3)
							output.peripheral_instance = Thor::Interrupt::SRC_USART3;
						#endif
						#if defined(USART6)
						if (uart_candidate == USART6)
							output.peripheral_instance = Thor::Interrupt::SRC_USART6;
						#endif
					}
					else
					{
						output.peripheral_type = Thor::Interrupt::SRC_UART;

						#if defined(UART4)
						if (uart_candidate == UART4)
							output.peripheral_instance = Thor::Interrupt::SRC_UART4;
						#endif
						#if defined(UART5)
						if (uart_candidate == UART5)
							output.peripheral_instance = Thor::Interrupt::SRC_UART5;
						#endif
						#if defined(UART7)
						if (uart_candidate == UART7)
							output.peripheral_instance = Thor::Interrupt::SRC_UART7;
						#endif
						#if defined(UART8)
						if (uart_candidate == UART8)
							output.peripheral_instance = Thor::Interrupt::SRC_UART8;
						#endif
					}
				}

				/* SPI PERIPHERALS */
				else if (IS_SPI_INSTANCE(spi_candidate))
				{
					output.peripheral_type = Thor::Interrupt::SRC_SPI;

					#if defined(SPI1)
					if (uart_candidate == SPI1)
						output.peripheral_instance = Thor::Interrupt::SRC_SPI1;
					#endif

					#if defined(SPI2)
					if (uart_candidate == SPI2)
						output.peripheral_instance = Thor::Interrupt::SRC_SPI2;
					#endif
					#if defined(SPI3)
					if (uart_candidate == SPI3)
						output.peripheral_instance = Thor::Interrupt::SRC_SPI3;
					#endif
					#if defined(SPI4)
					if (uart_candidate == SPI4)
						output.peripheral_instance = Thor::Interrupt::SRC_SPI4;
					#endif
					#if defined(SPI5)
					if (uart_candidate == SPI5)
						output.peripheral_instance = Thor::Interrupt::SRC_SPI5;
					#endif
					#if defined(SPI6)
					if (uart_candidate == SPI6)
						output.peripheral_instance = Thor::Interrupt::SRC_SPI6;
					#endif
				}

				else
				{
					source_found = false;
				}

				return source_found;
			}
		};

		namespace UART
		{
			UART_DMAHandlerManager::UART_DMAHandlerManager()
			{
				txdma_callbacks.resize(this->MAX_UART_CHANNELS);
				rxdma_callbacks.resize(this->MAX_UART_CHANNELS);
			}

			UART_DMAHandlerManager::~UART_DMAHandlerManager()
			{
			}

			void UART_DMAHandlerManager::requestCallback(Thor::Interrupt::DMA::PeriphConfig pConfig)
			{
				switch (pConfig.peripheral_instance)
				{
					#if defined(USART1)
				case Thor::Interrupt::SRC_USART1:
					if (pConfig.direction == Thor::Definitions::DMA::MEM_TO_PERIPH)
						executeCallbackFunction_TXDMA(1);

					else if (pConfig.direction == Thor::Definitions::DMA::PERIPH_TO_MEM)
						executeCallbackFunction_RXDMA(1);
					break;
					#endif

					#if defined(USART2)
				case Thor::Interrupt::SRC_USART2:
					if (pConfig.direction == Thor::Definitions::DMA::MEM_TO_PERIPH)
						executeCallbackFunction_TXDMA(2);

					else if (pConfig.direction == Thor::Definitions::DMA::PERIPH_TO_MEM)
						executeCallbackFunction_RXDMA(2);
					break;
					#endif

					#if defined(USART3)
				case Thor::Interrupt::SRC_USART3:
					if (pConfig.direction == Thor::Definitions::DMA::MEM_TO_PERIPH)
						executeCallbackFunction_TXDMA(3);

					else if (pConfig.direction == Thor::Definitions::DMA::PERIPH_TO_MEM)
						executeCallbackFunction_RXDMA(3);
					break;
					#endif

					#if defined(UART4)
				case Thor::Interrupt::SRC_UART4:
					if (pConfig.direction == Thor::Definitions::DMA::MEM_TO_PERIPH)
						executeCallbackFunction_TXDMA(4);

					else if (pConfig.direction == Thor::Definitions::DMA::PERIPH_TO_MEM)
						executeCallbackFunction_RXDMA(4);
					break;
					#endif

					#if defined(UART5)
				case Thor::Interrupt::SRC_UART5:
					if (pConfig.direction == Thor::Definitions::DMA::MEM_TO_PERIPH)
						executeCallbackFunction_TXDMA(5);

					else if (pConfig.direction == Thor::Definitions::DMA::PERIPH_TO_MEM)
						executeCallbackFunction_RXDMA(5);
					break;
					#endif

					#if defined(USART6)
				case Thor::Interrupt::SRC_USART6:
					if (pConfig.direction == Thor::Definitions::DMA::MEM_TO_PERIPH)
						executeCallbackFunction_TXDMA(6);

					else if (pConfig.direction == Thor::Definitions::DMA::PERIPH_TO_MEM)
						executeCallbackFunction_RXDMA(6);
					break;
					#endif

					#if defined(UART7)
				case Thor::Interrupt::SRC_UART7:
					if (pConfig.direction == Thor::Definitions::DMA::MEM_TO_PERIPH)
						executeCallbackFunction_TXDMA(7);

					else if (pConfig.direction == Thor::Definitions::DMA::PERIPH_TO_MEM)
						executeCallbackFunction_RXDMA(7);
					break;
					#endif

					#if defined(UART8)
				case Thor::Interrupt::SRC_UART8:
					if (pConfig.direction == Thor::Definitions::DMA::MEM_TO_PERIPH)
						executeCallbackFunction_TXDMA(8);

					else if (pConfig.direction == Thor::Definitions::DMA::PERIPH_TO_MEM)
						executeCallbackFunction_RXDMA(8);
					break;
					#endif

				default: break;
				}
			}

			void UART_DMAHandlerManager::attachCallbackFunction_TXDMA(int uart_num, func_void func)
			{
				txdma_callbacks[uart_num - 1] = func;
			}

			void UART_DMAHandlerManager::attachCallbackFunction_RXDMA(int uart_num, func_void func)
			{
				rxdma_callbacks[uart_num - 1] = func;
			}

			void UART_DMAHandlerManager::removeCallbackFunction_TXDMA(int uart_num)
			{
				txdma_callbacks[uart_num - 1].clear();
			}

			void UART_DMAHandlerManager::removeCallbackFunction_RXDMA(int uart_num)
			{
				rxdma_callbacks[uart_num - 1].clear();
			}

			void UART_DMAHandlerManager::executeCallbackFunction_TXDMA(int uart_num)
			{
				if (!txdma_callbacks[uart_num - 1].empty())
					txdma_callbacks[uart_num - 1]();
			}

			void UART_DMAHandlerManager::executeCallbackFunction_RXDMA(int uart_num)
			{
				if (!rxdma_callbacks[uart_num - 1].empty())
					rxdma_callbacks[uart_num - 1]();
			}
		};

		namespace SPI
		{
			SPI_DMAHandlerManager::SPI_DMAHandlerManager()
			{
				txdma_callbacks.resize(Thor::Definitions::SPI::MAX_SPI_CHANNELS);
				rxdma_callbacks.resize(Thor::Definitions::SPI::MAX_SPI_CHANNELS);
			}

			SPI_DMAHandlerManager::~SPI_DMAHandlerManager()
			{
			}

			void SPI_DMAHandlerManager::requestCallback(Thor::Interrupt::DMA::PeriphConfig pConfig)
			{
				switch (pConfig.peripheral_instance)
				{
					#if defined(SPI1)
				case Thor::Interrupt::SRC_SPI1:
					if (pConfig.direction == Thor::Definitions::DMA::MEM_TO_PERIPH)
						executeCallbackFunction_TXDMA(1);

					else if (pConfig.direction == Thor::Definitions::DMA::PERIPH_TO_MEM)
						executeCallbackFunction_RXDMA(1);
					break;
					#endif

					#if defined(SPI2)
				case Thor::Interrupt::SRC_SPI2:
					if (pConfig.direction == Thor::Definitions::DMA::MEM_TO_PERIPH)
						executeCallbackFunction_TXDMA(2);

					else if (pConfig.direction == Thor::Definitions::DMA::PERIPH_TO_MEM)
						executeCallbackFunction_RXDMA(2);
					break;
					#endif

					#if defined(SPI3)
				case Thor::Interrupt::SRC_SPI3:
					if (pConfig.direction == Thor::Definitions::DMA::MEM_TO_PERIPH)
						executeCallbackFunction_TXDMA(3);

					else if (pConfig.direction == Thor::Definitions::DMA::PERIPH_TO_MEM)
						executeCallbackFunction_RXDMA(3);

					break;
					#endif

					#if defined(SPI4)
				case Thor::Interrupt::SRC_SPI4:
					if (pConfig.direction == Thor::Definitions::DMA::MEM_TO_PERIPH)
						executeCallbackFunction_TXDMA(4);

					else if (pConfig.direction == Thor::Definitions::DMA::PERIPH_TO_MEM)
						executeCallbackFunction_RXDMA(4);
					break;
					#endif

					#if defined(SPI5)
				case Thor::Interrupt::SRC_SPI5:
					if (pConfig.direction == Thor::Definitions::DMA::MEM_TO_PERIPH)
						executeCallbackFunction_TXDMA(5);

					else if (pConfig.direction == Thor::Definitions::DMA::PERIPH_TO_MEM)
						executeCallbackFunction_RXDMA(5);
					break;
					#endif

					#if defined(SPI6)
				case Thor::Interrupt::SRC_SPI6:
					if (pConfig.direction == Thor::Definitions::DMA::MEM_TO_PERIPH)
						executeCallbackFunction_TXDMA(6);

					else if (pConfig.direction == Thor::Definitions::DMA::PERIPH_TO_MEM)
						executeCallbackFunction_RXDMA(6);
					break;
					#endif

				default: break;
				}
			}

			void SPI_DMAHandlerManager::attachCallbackFunction_TXDMA(int spi_num, func_void func)
			{
				txdma_callbacks[spi_num - 1] = func;
			}

			void SPI_DMAHandlerManager::attachCallbackFunction_RXDMA(int spi_num, func_void func)
			{
				rxdma_callbacks[spi_num - 1] = func;
			}

			void SPI_DMAHandlerManager::removeCallbackFunction_TXDMA(int spi_num)
			{
				txdma_callbacks[spi_num - 1].clear();
			}

			void SPI_DMAHandlerManager::removeCallbackFunction_RXDMA(int spi_num)
			{
				rxdma_callbacks[spi_num - 1].clear();
			}

			void SPI_DMAHandlerManager::executeCallbackFunction_TXDMA(int spi_num)
			{
				if (!txdma_callbacks[spi_num - 1].empty())
					txdma_callbacks[spi_num - 1]();
			}

			void SPI_DMAHandlerManager::executeCallbackFunction_RXDMA(int spi_num)
			{
				if (!rxdma_callbacks[spi_num - 1].empty())
					rxdma_callbacks[spi_num - 1]();
			}
		};
	};
}

Thor::Interrupt::DMA::DMAHandler dma_handler;
Thor::Interrupt::UART::UART_DMAHandlerManager uart_dma_manager, usart_dma_manager;
Thor::Interrupt::SPI::SPI_DMAHandlerManager spi_dma_manager;

#if defined(DMA1)
void DMA1_Stream0_IRQHandler(void)
{
	dma_handler.updateDMASource(1u, 0u);
	dma_handler.copyRegisters(DMA1_LISR, DMA1_HISR, DMA1_S0CR, DMA1_S0PAR);

	Thor::Interrupt::DMA::PeriphConfig config;
	if (dma_handler.calculatePeriphSource(config))
	{
		switch (config.peripheral_type)
		{
		case Thor::Interrupt::SRC_UART:
			uart_dma_manager.requestCallback(config);
			break;

		case Thor::Interrupt::SRC_USART:
			usart_dma_manager.requestCallback(config);

		case Thor::Interrupt::SRC_SPI:
			spi_dma_manager.requestCallback(config);

		default: break;
		};
	}
	else
		;
}

void DMA1_Stream1_IRQHandler(void)
{
	dma_handler.updateDMASource(1u, 1u);
	dma_handler.copyRegisters(DMA1_LISR, DMA1_HISR, DMA1_S1CR, DMA1_S1PAR);

	Thor::Interrupt::DMA::PeriphConfig config;
	if (dma_handler.calculatePeriphSource(config))
	{
		switch (config.peripheral_type)
		{
		case Thor::Interrupt::SRC_UART:
			uart_dma_manager.requestCallback(config);
			break;

		case Thor::Interrupt::SRC_USART:
			usart_dma_manager.requestCallback(config);

		case Thor::Interrupt::SRC_SPI:
			spi_dma_manager.requestCallback(config);

		default: break;
		};
	}
	else
		;
}

void DMA1_Stream2_IRQHandler(void)
{
	dma_handler.updateDMASource(1u, 2u);
	dma_handler.copyRegisters(DMA1_LISR, DMA1_HISR, DMA1_S2CR, DMA1_S2PAR);

	Thor::Interrupt::DMA::PeriphConfig config;
	if (dma_handler.calculatePeriphSource(config))
	{
		switch (config.peripheral_type)
		{
		case Thor::Interrupt::SRC_UART:
			uart_dma_manager.requestCallback(config);
			break;

		case Thor::Interrupt::SRC_USART:
			usart_dma_manager.requestCallback(config);

		case Thor::Interrupt::SRC_SPI:
			spi_dma_manager.requestCallback(config);

		default: break;
		};
	}
	else
		;
}

void DMA1_Stream3_IRQHandler(void)
{
	dma_handler.updateDMASource(1u, 3u);
	dma_handler.copyRegisters(DMA1_LISR, DMA1_HISR, DMA1_S3CR, DMA1_S3PAR);

	Thor::Interrupt::DMA::PeriphConfig config;
	if (dma_handler.calculatePeriphSource(config))
	{
		switch (config.peripheral_type)
		{
		case Thor::Interrupt::SRC_UART:
			uart_dma_manager.requestCallback(config);
			break;

		case Thor::Interrupt::SRC_USART:
			usart_dma_manager.requestCallback(config);

		case Thor::Interrupt::SRC_SPI:
			spi_dma_manager.requestCallback(config);

		default: break;
		};
	}
	else
		;
}

void DMA1_Stream4_IRQHandler(void)
{
	dma_handler.updateDMASource(1u, 4u);
	dma_handler.copyRegisters(DMA1_LISR, DMA1_HISR, DMA1_S4CR, DMA1_S4PAR);

	Thor::Interrupt::DMA::PeriphConfig config;
	if (dma_handler.calculatePeriphSource(config))
	{
		switch (config.peripheral_type)
		{
		case Thor::Interrupt::SRC_UART:
			uart_dma_manager.requestCallback(config);
			break;

		case Thor::Interrupt::SRC_USART:
			usart_dma_manager.requestCallback(config);

		case Thor::Interrupt::SRC_SPI:
			spi_dma_manager.requestCallback(config);

		default: break;
		};
	}
	else
		;
}

void DMA1_Stream5_IRQHandler(void)
{
	dma_handler.updateDMASource(1u, 5u);
	dma_handler.copyRegisters(DMA1_LISR, DMA1_HISR, DMA1_S5CR, DMA1_S5PAR);

	Thor::Interrupt::DMA::PeriphConfig config;
	if (dma_handler.calculatePeriphSource(config))
	{
		switch (config.peripheral_type)
		{
		case Thor::Interrupt::SRC_UART:
			uart_dma_manager.requestCallback(config);
			break;

		case Thor::Interrupt::SRC_USART:
			usart_dma_manager.requestCallback(config);

		case Thor::Interrupt::SRC_SPI:
			spi_dma_manager.requestCallback(config);

		default: break;
		};
	}
	else
		;
}

void DMA1_Stream6_IRQHandler(void)
{
	dma_handler.updateDMASource(1u, 6u);
	dma_handler.copyRegisters(DMA1_LISR, DMA1_HISR, DMA1_S6CR, DMA1_S6PAR);

	Thor::Interrupt::DMA::PeriphConfig config;
	if (dma_handler.calculatePeriphSource(config))
	{
		switch (config.peripheral_type)
		{
		case Thor::Interrupt::SRC_UART:
			uart_dma_manager.requestCallback(config);
			break;

		case Thor::Interrupt::SRC_USART:
			usart_dma_manager.requestCallback(config);

		case Thor::Interrupt::SRC_SPI:
			spi_dma_manager.requestCallback(config);

		default: break;
		};
	}
	else
		;
}

void DMA1_Stream7_IRQHandler(void)
{
	dma_handler.updateDMASource(1u, 7u);
	dma_handler.copyRegisters(DMA1_LISR, DMA1_HISR, DMA1_S7CR, DMA1_S7PAR);

	Thor::Interrupt::DMA::PeriphConfig config;
	if (dma_handler.calculatePeriphSource(config))
	{
		switch (config.peripheral_type)
		{
		case Thor::Interrupt::SRC_UART:
			uart_dma_manager.requestCallback(config);
			break;

		case Thor::Interrupt::SRC_USART:
			usart_dma_manager.requestCallback(config);

		case Thor::Interrupt::SRC_SPI:
			spi_dma_manager.requestCallback(config);

		default: break;
		};
	}
	else
		;
}
#endif /* DMA1 */

#if defined(DMA2)
void DMA2_Stream0_IRQHandler(void)
{
	/* Inform the DMAHandler of a few pieces of information */
	dma_handler.updateDMASource(2u, 0u);
	dma_handler.copyRegisters(DMA2_LISR, DMA2_HISR, DMA2_S0CR, DMA2_S0PAR);

	/* Check to see if the code can deduce where the call came from */
	Thor::Interrupt::DMA::PeriphConfig config;
	if (dma_handler.calculatePeriphSource(config))
	{
		switch (config.peripheral_type)
		{
		case Thor::Interrupt::SRC_UART:
			uart_dma_manager.requestCallback(config);
			break;

		case Thor::Interrupt::SRC_USART:
			usart_dma_manager.requestCallback(config);

		case Thor::Interrupt::SRC_SPI:
			spi_dma_manager.requestCallback(config);

		default: break;
		};
	}
	else
		;
}

void DMA2_Stream1_IRQHandler(void)
{
	/* Inform the DMAHandler of a few pieces of information */
	dma_handler.updateDMASource(2u, 1u);
	dma_handler.copyRegisters(DMA2_LISR, DMA2_HISR, DMA2_S1CR, DMA2_S1PAR);

	/* Check to see if the code can deduce where the call came from */
	Thor::Interrupt::DMA::PeriphConfig config;
	if (dma_handler.calculatePeriphSource(config))
	{
		switch (config.peripheral_type)
		{
		case Thor::Interrupt::SRC_UART:
			uart_dma_manager.requestCallback(config);
			break;

		case Thor::Interrupt::SRC_USART:
			usart_dma_manager.requestCallback(config);

		case Thor::Interrupt::SRC_SPI:
			spi_dma_manager.requestCallback(config);

		default: break;
		};
	}
	else
		;
}

void DMA2_Stream2_IRQHandler(void)
{
	dma_handler.updateDMASource(2u, 2u);
	dma_handler.copyRegisters(DMA2_LISR, DMA2_HISR, DMA2_S2CR, DMA2_S2PAR);

	Thor::Interrupt::DMA::PeriphConfig config;
	if (dma_handler.calculatePeriphSource(config))
	{
		switch (config.peripheral_type)
		{
		case Thor::Interrupt::SRC_UART:
			uart_dma_manager.requestCallback(config);
			break;

		case Thor::Interrupt::SRC_USART:
			usart_dma_manager.requestCallback(config);

		case Thor::Interrupt::SRC_SPI:
			spi_dma_manager.requestCallback(config);

		default: break;
		};
	}
	else
		;
}

void DMA2_Stream3_IRQHandler(void)
{
	dma_handler.updateDMASource(2u, 3u);
	dma_handler.copyRegisters(DMA2_LISR, DMA2_HISR, DMA2_S3CR, DMA2_S3PAR);

	Thor::Interrupt::DMA::PeriphConfig config;
	if (dma_handler.calculatePeriphSource(config))
	{
		switch (config.peripheral_type)
		{
		case Thor::Interrupt::SRC_UART:
			uart_dma_manager.requestCallback(config);
			break;

		case Thor::Interrupt::SRC_USART:
			usart_dma_manager.requestCallback(config);

		case Thor::Interrupt::SRC_SPI:
			spi_dma_manager.requestCallback(config);

		default: break;
		};
	}
	else
		;
}

void DMA2_Stream4_IRQHandler(void)
{
	dma_handler.updateDMASource(2u, 4u);
	dma_handler.copyRegisters(DMA2_LISR, DMA2_HISR, DMA2_S4CR, DMA2_S4PAR);

	Thor::Interrupt::DMA::PeriphConfig config;
	if (dma_handler.calculatePeriphSource(config))
	{
		switch (config.peripheral_type)
		{
		case Thor::Interrupt::SRC_UART:
			uart_dma_manager.requestCallback(config);
			break;

		case Thor::Interrupt::SRC_USART:
			usart_dma_manager.requestCallback(config);

		case Thor::Interrupt::SRC_SPI:
			spi_dma_manager.requestCallback(config);

		default: break;
		};
	}
	else
		;
}

void DMA2_Stream5_IRQHandler(void)
{
	dma_handler.updateDMASource(2u, 5u);
	dma_handler.copyRegisters(DMA2_LISR, DMA2_HISR, DMA2_S5CR, DMA2_S5PAR);

	Thor::Interrupt::DMA::PeriphConfig config;
	if (dma_handler.calculatePeriphSource(config))
	{
		switch (config.peripheral_type)
		{
		case Thor::Interrupt::SRC_UART:
			uart_dma_manager.requestCallback(config);
			break;

		case Thor::Interrupt::SRC_USART:
			usart_dma_manager.requestCallback(config);

		case Thor::Interrupt::SRC_SPI:
			spi_dma_manager.requestCallback(config);

		default: break;
		};
	}
	else
		;
}

void DMA2_Stream6_IRQHandler(void)
{
	dma_handler.updateDMASource(2u, 6u);
	dma_handler.copyRegisters(DMA2_LISR, DMA2_HISR, DMA2_S6CR, DMA2_S6PAR);

	Thor::Interrupt::DMA::PeriphConfig config;
	if (dma_handler.calculatePeriphSource(config))
	{
		switch (config.peripheral_type)
		{
		case Thor::Interrupt::SRC_UART:
			uart_dma_manager.requestCallback(config);
			break;

		case Thor::Interrupt::SRC_USART:
			usart_dma_manager.requestCallback(config);

		case Thor::Interrupt::SRC_SPI:
			spi_dma_manager.requestCallback(config);

		default: break;
		};
	}
	else
		;
}

void DMA2_Stream7_IRQHandler(void)
{
	dma_handler.updateDMASource(2u, 7u);
	dma_handler.copyRegisters(DMA2_LISR, DMA2_HISR, DMA2_S7CR, DMA2_S7PAR);

	Thor::Interrupt::DMA::PeriphConfig config;
	if (dma_handler.calculatePeriphSource(config))
	{
		switch (config.peripheral_type)
		{
		case Thor::Interrupt::SRC_UART:
			uart_dma_manager.requestCallback(config);
			break;

		case Thor::Interrupt::SRC_USART:
			usart_dma_manager.requestCallback(config);

		case Thor::Interrupt::SRC_SPI:
			spi_dma_manager.requestCallback(config);

		default: break;
		};
	}
	else
		;
}
#endif /* DMA2 */