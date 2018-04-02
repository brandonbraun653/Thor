#include "../include/interrupt.h"

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
				output.direction = ThorDef::DMA::PERIPH_TO_MEM;
				break;

			case 1u:
				output.direction = ThorDef::DMA::MEM_TO_PERIPH;
				break;

			case 2u:
				output.direction = ThorDef::DMA::MEM_TO_MEM;
				break;

			default:
				output.direction = ThorDef::DMA::TRANSFER_DIRECTION_UNDEFINED;
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
			 * the base address with a few STMicro provided macros shortens the time required to
			 * reverse search for the exact peripheral used.
			 *
			 * Because it is not certain whether all peripheral addressing follows the same
			 * structure, each peripheral is handled on a case by case basis.
			 * */

			void* uart_candidate = (void*)(REG_DMA_SxPAR & 0xFFFFFF00);
			void* spi_candidate = (void*)(REG_DMA_SxPAR & 0xFFFFFF00);
			//add others as needed

			/* UART/USART PERIPHERALS */
			if (IS_UART_INSTANCE(uart_candidate))
			{
				if (IS_USART_INSTANCE(uart_candidate))
				{
					output.peripheral_type = Interrupt::SRC_USART;
					
					#ifdef ENABLE_USART1
					if (uart_candidate == USART1)
						output.peripheral_instance = Interrupt::SRC_USART1; 
				    #endif

					#ifdef ENABLE_USART2
					if (uart_candidate == USART2)
						output.peripheral_instance = Interrupt::SRC_USART2;
					#endif
					
					#ifdef ENABLE_USART3
					if (uart_candidate == USART3)
						output.peripheral_instance = Interrupt::SRC_USART3;
					#endif
					#ifdef ENABLE_USART6
					if (uart_candidate == USART6)
						output.peripheral_instance = Interrupt::SRC_USART6;
					#endif
				}
				else
				{
					output.peripheral_type = Interrupt::SRC_UART;
					
					#ifdef ENABLE_UART4
					if (uart_candidate == UART4)
						output.peripheral_instance = Interrupt::SRC_UART4;
					#endif				
					#ifdef ENABLE_UART5
					if (uart_candidate == UART5)
						output.peripheral_instance = Interrupt::SRC_UART5;
					#endif
					#ifdef ENABLE_UART7
					if (uart_candidate == UART7)
						output.peripheral_instance = Interrupt::SRC_UART7;
					#endif
					#ifdef ENABLE_UART8
					if (uart_candidate == UART8)
						output.peripheral_instance = Interrupt::SRC_UART8;
					#endif
				}
			}

			/* SPI PERIPHERALS */
			else if (IS_SPI_INSTANCE(spi_candidate))
			{
				output.peripheral_type = Interrupt::SRC_SPI;
					
				#ifdef ENABLE_SPI1
				if (uart_candidate == SPI1)
					output.peripheral_instance = Interrupt::SRC_SPI1;
				#endif
				
				#ifdef ENABLE_SPI2
				if (uart_candidate == SPI2)
					output.peripheral_instance = Interrupt::SRC_SPI2;
				#endif
				#ifdef ENABLE_SPI3
				if (uart_candidate == SPI3)
					output.peripheral_instance = Interrupt::SRC_SPI3;
				#endif
				#ifdef ENABLE_SPI4
				if (uart_candidate == SPI4)
					output.peripheral_instance = Interrupt::SRC_SPI4;
				#endif
				#ifdef ENABLE_SPI5
				if (uart_candidate == SPI5)
					output.peripheral_instance = Interrupt::SRC_SPI5;
				#endif
				#ifdef ENABLE_SPI6
				if (uart_candidate == SPI6)
					output.peripheral_instance = Interrupt::SRC_SPI6;
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

		void UART_DMAHandlerManager::requestCallback(Interrupt::DMA::PeriphConfig pConfig)
		{
			switch (pConfig.peripheral_instance)
			{
			#ifdef ENABLE_UART1
			case Interrupt::SRC_UART1:
				if (pConfig.direction == ThorDef::DMA::MEM_TO_PERIPH)
					executeCallbackFunction_TXDMA(1);

				else if (pConfig.direction == ThorDef::DMA::PERIPH_TO_MEM)
					executeCallbackFunction_RXDMA(1);
				break;
			#endif

			#ifdef ENABLE_UART2
			case Interrupt::SRC_UART2:
				if (pConfig.direction == ThorDef::DMA::MEM_TO_PERIPH)
					executeCallbackFunction_TXDMA(2);

				else if (pConfig.direction == ThorDef::DMA::PERIPH_TO_MEM)
					executeCallbackFunction_RXDMA(2);
				break;
			#endif

			#ifdef ENABLE_UART3
			case Interrupt::SRC_UART3:
				if (pConfig.direction == ThorDef::DMA::MEM_TO_PERIPH)
					executeCallbackFunction_TXDMA(3);

				else if (pConfig.direction == ThorDef::DMA::PERIPH_TO_MEM)
					executeCallbackFunction_RXDMA(3);
				break;
			#endif

			#ifdef ENABLE_UART4
			case Interrupt::SRC_UART4:
				if (pConfig.direction == ThorDef::DMA::MEM_TO_PERIPH)
					executeCallbackFunction_TXDMA(4);

				else if (pConfig.direction == ThorDef::DMA::PERIPH_TO_MEM)
					executeCallbackFunction_RXDMA(4);
				break;
			#endif

			#ifdef ENABLE_UART5
			case Interrupt::SRC_UART5:
				if (pConfig.direction == ThorDef::DMA::MEM_TO_PERIPH)
					executeCallbackFunction_TXDMA(5);

				else if (pConfig.direction == ThorDef::DMA::PERIPH_TO_MEM)
					executeCallbackFunction_RXDMA(5);
				break;
			#endif

			#ifdef ENABLE_UART7
			case Interrupt::SRC_UART7:
				if (pConfig.direction == ThorDef::DMA::MEM_TO_PERIPH)
					executeCallbackFunction_TXDMA(7);

				else if (pConfig.direction == ThorDef::DMA::PERIPH_TO_MEM)
					executeCallbackFunction_RXDMA(7);
				break;
			#endif

			#ifdef ENABLE_UART8
			case Interrupt::SRC_UART8:
				if (pConfig.direction == ThorDef::DMA::MEM_TO_PERIPH)
					executeCallbackFunction_TXDMA(8);

				else if (pConfig.direction == ThorDef::DMA::PERIPH_TO_MEM)
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
			txdma_callbacks.resize(ThorDef::SPI::MAX_SPI_CHANNELS);
			rxdma_callbacks.resize(ThorDef::SPI::MAX_SPI_CHANNELS);
		}

		SPI_DMAHandlerManager::~SPI_DMAHandlerManager()
		{
		}

		void SPI_DMAHandlerManager::requestCallback(Interrupt::DMA::PeriphConfig pConfig)
		{
			switch (pConfig.peripheral_instance)
			{
			#ifdef ENABLE_SPI1
			case Interrupt::SRC_SPI1:
				if (pConfig.direction == ThorDef::DMA::MEM_TO_PERIPH)
					executeCallbackFunction_TXDMA(1);

				else if (pConfig.direction == ThorDef::DMA::PERIPH_TO_MEM)
					executeCallbackFunction_RXDMA(1);
				break;
			#endif
				
			#ifdef ENABLE_SPI2			
			case Interrupt::SRC_SPI2:
				if (pConfig.direction == ThorDef::DMA::MEM_TO_PERIPH)
					executeCallbackFunction_TXDMA(2);

				else if (pConfig.direction == ThorDef::DMA::PERIPH_TO_MEM)
					executeCallbackFunction_RXDMA(2);
				break;
			#endif
				
			#ifdef ENABLE_SPI3
			case Interrupt::SRC_SPI3:
				if (pConfig.direction == ThorDef::DMA::MEM_TO_PERIPH)
					executeCallbackFunction_TXDMA(3);

				else if (pConfig.direction == ThorDef::DMA::PERIPH_TO_MEM)
					executeCallbackFunction_RXDMA(3);

				break;
			#endif
			
			#ifdef ENABLE_SPI4
			case Interrupt::SRC_SPI4:
				if (pConfig.direction == ThorDef::DMA::MEM_TO_PERIPH)
					executeCallbackFunction_TXDMA(4);

				else if (pConfig.direction == ThorDef::DMA::PERIPH_TO_MEM)
					executeCallbackFunction_RXDMA(4);
				break;
			#endif
				
			#ifdef ENABLE_SPI5
			case Interrupt::SRC_SPI5:
				if (pConfig.direction == ThorDef::DMA::MEM_TO_PERIPH)
					executeCallbackFunction_TXDMA(5);

				else if (pConfig.direction == ThorDef::DMA::PERIPH_TO_MEM)
					executeCallbackFunction_RXDMA(5);
				break;
			#endif
			
			#ifdef ENABLE_SPI6
			case Interrupt::SRC_SPI6:
				if (pConfig.direction == ThorDef::DMA::MEM_TO_PERIPH)
					executeCallbackFunction_TXDMA(6);

				else if (pConfig.direction == ThorDef::DMA::PERIPH_TO_MEM)
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

Interrupt::DMA::DMAHandler dma_handler;
Interrupt::UART::UART_DMAHandlerManager uart_dma_manager;
Interrupt::SPI::SPI_DMAHandlerManager spi_dma_manager;

#ifdef DMA1
void DMA1_Stream0_IRQHandler(void)
{
	dma_handler.updateDMASource(1u, 0u);
	dma_handler.copyRegisters(DMA1_LISR, DMA1_HISR, DMA1_S0CR, DMA1_S0PAR);

	Interrupt::DMA::PeriphConfig config;
	if (dma_handler.calculatePeriphSource(config))
	{
		switch (config.peripheral_type)
		{
		case Interrupt::SRC_UART:
			uart_dma_manager.requestCallback(config);
			break;

		case Interrupt::SRC_SPI:
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

	Interrupt::DMA::PeriphConfig config;
	if (dma_handler.calculatePeriphSource(config))
	{
		switch (config.peripheral_type)
		{
		case Interrupt::SRC_UART:
			uart_dma_manager.requestCallback(config);
			break;

		case Interrupt::SRC_SPI:
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

	Interrupt::DMA::PeriphConfig config;
	if (dma_handler.calculatePeriphSource(config))
	{
		switch (config.peripheral_type)
		{
		case Interrupt::SRC_UART:
			uart_dma_manager.requestCallback(config);
			break;

		case Interrupt::SRC_SPI:
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

	Interrupt::DMA::PeriphConfig config;
	if (dma_handler.calculatePeriphSource(config))
	{
		switch (config.peripheral_type)
		{
		case Interrupt::SRC_UART:
			uart_dma_manager.requestCallback(config);
			break;

		case Interrupt::SRC_SPI:
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

	Interrupt::DMA::PeriphConfig config;
	if (dma_handler.calculatePeriphSource(config))
	{
		switch (config.peripheral_type)
		{
		case Interrupt::SRC_UART:
			uart_dma_manager.requestCallback(config);
			break;

		case Interrupt::SRC_SPI:
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

	Interrupt::DMA::PeriphConfig config;
	if (dma_handler.calculatePeriphSource(config))
	{
		switch (config.peripheral_type)
		{
		case Interrupt::SRC_UART:
			uart_dma_manager.requestCallback(config);
			break;

		case Interrupt::SRC_SPI:
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

	Interrupt::DMA::PeriphConfig config;
	if (dma_handler.calculatePeriphSource(config))
	{
		switch (config.peripheral_type)
		{
		case Interrupt::SRC_UART:
			uart_dma_manager.requestCallback(config);
			break;

		case Interrupt::SRC_SPI:
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

	Interrupt::DMA::PeriphConfig config;
	if (dma_handler.calculatePeriphSource(config))
	{
		switch (config.peripheral_type)
		{
		case Interrupt::SRC_UART:
			uart_dma_manager.requestCallback(config);
			break;

		case Interrupt::SRC_SPI:
			spi_dma_manager.requestCallback(config);

		default: break;
		};
	}
	else
		;
}
#endif /* DMA1 */

#ifdef DMA2
void DMA2_Stream0_IRQHandler(void)
{
	/* Inform the DMAHandler of a few pieces of information */
	dma_handler.updateDMASource(2u, 0u);
	dma_handler.copyRegisters(DMA2_LISR, DMA2_HISR, DMA2_S0CR, DMA2_S0PAR);

	/* Check to see if the code can deduce where the call came from */
	Interrupt::DMA::PeriphConfig config;
	if (dma_handler.calculatePeriphSource(config))
	{
		switch (config.peripheral_type)
		{
		case Interrupt::SRC_UART:
			uart_dma_manager.requestCallback(config);
			break;

		case Interrupt::SRC_SPI:
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
	Interrupt::DMA::PeriphConfig config;
	if (dma_handler.calculatePeriphSource(config))
	{
		switch (config.peripheral_type)
		{
		case Interrupt::SRC_UART:
			uart_dma_manager.requestCallback(config);
			break;

		case Interrupt::SRC_SPI:
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

	Interrupt::DMA::PeriphConfig config;
	if (dma_handler.calculatePeriphSource(config))
	{
		switch (config.peripheral_type)
		{
		case Interrupt::SRC_UART:
			uart_dma_manager.requestCallback(config);
			break;

		case Interrupt::SRC_SPI:
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

	Interrupt::DMA::PeriphConfig config;
	if (dma_handler.calculatePeriphSource(config))
	{
		switch (config.peripheral_type)
		{
		case Interrupt::SRC_UART:
			uart_dma_manager.requestCallback(config);
			break;

		case Interrupt::SRC_SPI:
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

	Interrupt::DMA::PeriphConfig config;
	if (dma_handler.calculatePeriphSource(config))
	{
		switch (config.peripheral_type)
		{
		case Interrupt::SRC_UART:
			uart_dma_manager.requestCallback(config);
			break;

		case Interrupt::SRC_SPI:
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

	Interrupt::DMA::PeriphConfig config;
	if (dma_handler.calculatePeriphSource(config))
	{
		switch (config.peripheral_type)
		{
		case Interrupt::SRC_UART:
			uart_dma_manager.requestCallback(config);
			break;

		case Interrupt::SRC_SPI:
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

	Interrupt::DMA::PeriphConfig config;
	if (dma_handler.calculatePeriphSource(config))
	{
		switch (config.peripheral_type)
		{
		case Interrupt::SRC_UART:
			uart_dma_manager.requestCallback(config);
			break;

		case Interrupt::SRC_SPI:
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

	Interrupt::DMA::PeriphConfig config;
	if (dma_handler.calculatePeriphSource(config))
	{
		switch (config.peripheral_type)
		{
		case Interrupt::SRC_UART:
			uart_dma_manager.requestCallback(config);
			break;

		case Interrupt::SRC_SPI:
			spi_dma_manager.requestCallback(config);

		default: break;
		};
	}
	else
		;
}
#endif /* DMA2 */