#pragma once
#ifndef DEFAULTS_H_
#define DEFAULTS_H_

#include <Thor/include/config.hpp>
#include <Thor/include/definitions.hpp>
#include <Thor/include/types.hpp>

#if defined(USING_FREERTOS)
#include "FreeRTOSConfig.h"

/** Checks an interrupt's priority value for compatibility with FreeRTOS. If triggered, the priority level
 *	needs to be lowered (lower priorities are higher numeric values for ARM-M). This macro should ONLY be used
 *	to check peripherals that call a FreeRTOS API function from an ISR. Otherwise, this check is pointless.
 *	See for more detail: http://www.freertos.org/RTOS-Cortex-M3-M4.html
 **/
#define CHECK_IT_PRIO(X) static_assert(X >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, "Invalid interrupt priority for " #X)
#endif

/** @namespace Thor */
namespace Thor
{
	/** @namespace Thor::Defaults */
	namespace Defaults
	{
		/** @namespace Thor::Defaults::GPIO */
		namespace GPIO
		{

		}

		/** @namespace Thor::Defaults::Serial */
		namespace Serial
		{
			/** @struct Serial Config */
			struct SerialConfig
			{
				/* IO Config: */
				GPIO_Initializer txPin;
				GPIO_Initializer rxPin;

				/* Peripheral Instance */
				USART_TypeDef* instance;

				/* Interrupt Settings */
				IT_Initializer IT_HW;
				IT_Initializer dmaIT_TX;
				IT_Initializer dmaIT_RX;

				/* DMA Settings */
				DMA_Initializer dmaTX;
				DMA_Initializer dmaRX;
			};
			extern const SerialConfig srl_cfg[];

			#if defined(STM32F7) || defined(STM32F4)
			extern const USART_InitTypeDef dflt_USART_Init;
			extern const UART_InitTypeDef dflt_UART_Init;
			extern const DMA_InitTypeDef dflt_DMA_Init_TX;
			extern const DMA_InitTypeDef dflt_DMA_Init_RX;
			#endif

			#ifdef STM32F7
			extern const UART_AdvFeatureInitTypeDef dflt_UART_AdvInit;
			#endif
		}

		/** @namespace Thor::Defaults::Timer */
		namespace Timer
		{

		}

		/** @namespace Thor::Defaults::SPI */
		namespace SPI
		{
			struct SPIConfig
			{
				/* IO Config */
				GPIO_Initializer MOSI;
				GPIO_Initializer MISO;
				GPIO_Initializer SCK;
				GPIO_Initializer NSS;

				/* Peripheral Instance */
				SPI_TypeDef* instance;

				/* Interrupt Settings */
				IT_Initializer IT_HW;
				IT_Initializer dmaIT_TX;
				IT_Initializer dmaIT_RX;

				/* DMA Settings */
				DMA_Initializer dmaTX;
				DMA_Initializer dmaRX;

				/* Clock Bus */
				Thor::Definitions::ClockBus clockBus;
			};
			extern const SPIConfig spi_cfg[];

			#if defined(STM32F7) || defined(STM32F4)
			extern const SPI_InitTypeDef dflt_SPI_Init;
			extern const DMA_InitTypeDef dflt_DMA_Init_TX;
			extern const DMA_InitTypeDef dflt_DMA_Init_RX;
			#endif
		}

		/** @namespace Thor::Defaults::Interrupt */
		namespace Interrupt
		{
			const uint32_t SYSTEM_NVIC_PRIORITY_GROUPING = NVIC_PRIORITYGROUP_4;	/**< DO NOT CHANGE: Sets the priority grouping to use all 4 preempt bits with no subpriority bits. */


			#if defined(USING_FREERTOS)
			/* These values can utilize the full range of priority grouping bits EXCEPT for peripherals
			 * that us the FreeRTOS ISR API calls. Their priority cannot be higher than configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY.*/
			const uint32_t UART_IT_PREEMPT_PRIORITY = 5; CHECK_IT_PRIO(UART_IT_PREEMPT_PRIORITY);
			const uint32_t UART_DMA_PREEMPT_PRIORITY = 5; CHECK_IT_PRIO(UART_DMA_PREEMPT_PRIORITY);
			const uint32_t USART_IT_PREEMPT_PRIORITY = UART_IT_PREEMPT_PRIORITY;
			const uint32_t USART_DMA_PREEMPT_PRIORITY = UART_DMA_PREEMPT_PRIORITY;

			#else
			/* These values can safely take on the full range of the priority grouping bits (0-15) with 0 as the highest priority. */
			const uint32_t UART_IT_PREEMPT_PRIORITY = 2;
			const uint32_t UART_DMA_PREEMPT_PRIORITY = 2;

			#endif
		}
	}
}


#endif // !DEFAULTS_H_
