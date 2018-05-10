#pragma once
#ifndef INTERRUPT_H_
#define INTERRUPT_H_

/* Boost Includes */
#include <boost/function.hpp>
#include <boost/container/vector.hpp>

/* Thor Includes */
#include <Thor/include/config.h>
#include <Thor/include/definitions.h>
#include <Thor/include/types.h>

#ifdef USING_FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#endif

namespace Thor
{
	namespace Interrupt
	{
		//DO NOT MOVE THIS. DMA handlers currently depend on it...
		enum TriggerSource
		{
			SRC_UART,
			SRC_UART1,
			SRC_UART1_TX,
			SRC_UART1_RX,
			SRC_UART2,
			SRC_UART2_TX,
			SRC_UART2_RX,
			SRC_UART3,
			SRC_UART3_TX,
			SRC_UART3_RX,
			SRC_UART4,
			SRC_UART4_TX,
			SRC_UART4_RX,
			SRC_UART5,
			SRC_UART5_TX,
			SRC_UART5_RX,
			SRC_UART7,
			SRC_UART7_TX,
			SRC_UART7_RX,
			SRC_UART8,
			SRC_UART8_TX,
			SRC_UART8_RX,
			SRC_USART,
			SRC_USART1,
			SRC_USART1_TX,
			SRC_USART1_RX,
			SRC_USART2,
			SRC_USART2_TX,
			SRC_USART2_RX,
			SRC_USART3,
			SRC_USART3_TX,
			SRC_USART3_RX,
			SRC_USART6,
			SRC_USART6_TX,
			SRC_USART6_RX,
				
			SRC_SPI,
			SRC_SPI1,
			SRC_SPI2,
			SRC_SPI3,
			SRC_SPI4,
			SRC_SPI5,
			SRC_SPI6,
				
		};

		namespace DMA
		{
			struct PeriphConfig
			{
				TriggerSource peripheral_type;
				TriggerSource peripheral_instance;
				Thor::Definitions::DMA::TransferDirection direction;
				uint32_t channel_selection;
			};

			class DMAHandler
			{
			public:
				bool calculatePeriphSource(PeriphConfig& output);
				void updateDMASource(uint32_t periph, uint32_t stream);
				void copyRegisters(uint32_t REG_LISR, uint32_t REG_HISR, uint32_t REG_SxCR, uint32_t REG_SxPAR);

				DMAHandler();
				~DMAHandler();
			private:
				uint32_t DMA_Periph, DMA_Stream, DMA_Channel;
				uint32_t REG_DMA_LISR, REG_DMA_HISR;
				uint32_t REG_DMA_SxCR, REG_DMA_SxPAR;
			};
		}

		namespace UART
		{
			class UART_DMAHandlerManager
			{
			public:
				void requestCallback(Interrupt::DMA::PeriphConfig pConfig);

				void attachCallbackFunction_TXDMA(int uart_num, func_void func);
				void removeCallbackFunction_TXDMA(int uart_num);
				void attachCallbackFunction_RXDMA(int uart_num, func_void func);
				void removeCallbackFunction_RXDMA(int uart_num);

				UART_DMAHandlerManager();
				~UART_DMAHandlerManager();

			private:
				/* Assumed maximum number of channels to ever be encountered on any device.
				* If needed, the number can always be updated. It is only used for memory
				* allocation sizes. */
				const int MAX_UART_CHANNELS = 10;
				boost::container::vector<func_void> txdma_callbacks;
				boost::container::vector<func_void> rxdma_callbacks;

				void executeCallbackFunction_TXDMA(int uart_num);
				void executeCallbackFunction_RXDMA(int uart_num);
			};
		}

		namespace SPI
		{
			class SPI_DMAHandlerManager
			{
			public:
				void requestCallback(Interrupt::DMA::PeriphConfig pConfig);

				void attachCallbackFunction_TXDMA(int spi_num, func_void func);
				void removeCallbackFunction_TXDMA(int spi_num);
				void attachCallbackFunction_RXDMA(int spi_num, func_void func);
				void removeCallbackFunction_RXDMA(int spi_num);

				SPI_DMAHandlerManager();
				~SPI_DMAHandlerManager();

			private:
				boost::container::vector<func_void> txdma_callbacks;
				boost::container::vector<func_void> rxdma_callbacks;

				void executeCallbackFunction_TXDMA(int spi_num);
				void executeCallbackFunction_RXDMA(int spi_num);
			};
		}
	}
}

extern Thor::Interrupt::DMA::DMAHandler dma_handler;
extern Thor::Interrupt::UART::UART_DMAHandlerManager uart_dma_manager;
extern Thor::Interrupt::SPI::SPI_DMAHandlerManager spi_dma_manager;

#ifdef __cplusplus
extern "C" {
#endif
	#if defined(DMA1)
	void DMA1_Stream0_IRQHandler();
	void DMA1_Stream1_IRQHandler();
	void DMA1_Stream2_IRQHandler();
	void DMA1_Stream3_IRQHandler();
	void DMA1_Stream4_IRQHandler();
	void DMA1_Stream5_IRQHandler();
	void DMA1_Stream6_IRQHandler();
	void DMA1_Stream7_IRQHandler();
	#endif

	#if defined(DMA2)
	void DMA2_Stream0_IRQHandler();
	void DMA2_Stream1_IRQHandler();
	void DMA2_Stream2_IRQHandler();
	void DMA2_Stream3_IRQHandler();
	void DMA2_Stream4_IRQHandler();
	void DMA2_Stream5_IRQHandler();
	void DMA2_Stream6_IRQHandler();
	void DMA2_Stream7_IRQHandler();
	#endif
	

	/* Possible USART Handlers: Note that not all of these will actually exist in hardware */
	void USART1_IRQHandler();
	void USART2_IRQHandler();
	void USART3_IRQHandler();
	void USART6_IRQHandler();
	
	/* Possible UART Handlers: Note that not all of these will actually exist in hardware */
	void UART1_IRQHandler();
	void UART2_IRQHandler();
	void UART3_IRQHandler();
	void UART4_IRQHandler();
	void UART5_IRQHandler();
	void UART6_IRQHandler();
	void UART7_IRQHandler();
	void UART8_IRQHandler();
	
	
#ifdef __cplusplus
}
#endif

#endif