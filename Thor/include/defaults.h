#pragma once
#ifndef DEFAULTS_H_
#define DEFAULTS_H_

#include "thor_config.h"

#include <stdlib.h>
#ifdef TARGET_STM32F7
#include <stm32f7xx_hal.h>
#endif

#ifdef TARGET_STM32F4
#include <stm32f4xx_hal.h>
#endif

#include "thor_peripherals.hpp"
#include "thor_definitions.h"
#include "types.h"


namespace Defaults
{
	namespace Serial
	{
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
	};
	
	namespace Timer
	{
	
	};
	
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
		};
		extern const SPIConfig spi_cfg[];

		#if defined(STM32F7) || defined(STM32F4)
		extern const SPI_InitTypeDef dflt_SPI_Init;
		extern const DMA_InitTypeDef dflt_DMA_Init_TX;
		extern const DMA_InitTypeDef dflt_DMA_Init_RX;
		#endif
	};
};

#endif // !DEFAULTS_H_
