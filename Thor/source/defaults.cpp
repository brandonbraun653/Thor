#include "../include/defaults.h"

using namespace Thor::Definitions::DMA;
using namespace Thor::Definitions::GPIO;
using namespace Thor::Definitions::SPI;
using namespace Thor::Definitions::TIMER;
using namespace Thor::Definitions::UART;

using namespace Thor::Libraries::Serial;	//TODO: Deprecate in future

using namespace Thor::Peripheral::GPIO;

namespace Thor
{
	namespace Defaults
	{
		namespace Serial
		{
			const SerialConfig srl_cfg[] =
			{
				/* SERIAL 0: Doesn't exist. Only used for easy index alignment. */
				{
					//TX Pin
					{},

					//RX pin
			{},

			//UART/USART Instance Ptr
			nullptr,

			//IT_HW
			{},

			//DMA_IT_TX
			{},

			//DMA_IT_RX
			{},

			//DMA TX settings
			{},

			//DMA RX settings
			{}
				},

				/* SERIAL 1: */
		{
			#if defined(ENABLE_USART1)
			#if defined(STM32F767xx) || defined(STM32F446xx)
			//TX pin
			{ GPIOB, PIN_6, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF7_USART1 },

			//RX Pin
			{ GPIOB, PIN_7, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF7_USART1 },

			//Instance Ptr
			USART1,

			//IT_HW
			{},

			//DMA_IT_TX
			{},

			//DMA_IT_RX
			{},

			//DMA TX settings
			{},

			//DMA RX settings
			{}
			#endif

			/* WHEN YOU SUPORT USART, COMPARE DMA AND IT SETTINGS */

				#endif
		},

				/* SERIAL 2: */
		{
			#if defined(ENABLE_USART2) && !defined(ENABLE_UART2)
			#if defined(STM32F767xx) || defined(STM32F446xx)
			//TX pin
			{ GPIOD, PIN_5, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF7_USART2 },

			//RX Pin
			{ GPIOD, PIN_6, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF7_USART2 },

			//Instance Ptr
			USART2,

			//IT_HW
			{},

			//DMA_IT_TX
			{},

			//DMA_IT_RX
			{},

			//DMA TX settings
			{},

			//DMA RX settings
			{}
			#endif

			/* WHEN YOU SUPORT USART, COMPARE DMA AND IT SETTINGS */
				#endif

				#if defined(ENABLE_UART2) && !defined(ENABLE_USART2)
				#if defined(STM32F446xx)
			//TX pin
			{ GPIOA, PIN_2, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF7_USART2 },

				//RX Pin
			{ GPIOA, PIN_3, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF7_USART2 },

				//Instance Ptr
				USART2,

				//IT_HW
			{ USART2_IRQn, NVIC_PRIORITYGROUP_4, 6u, 0u },

				//DMA_IT_TX
			{},

				//DMA_IT_RX
			{},

				//DMA TX settings
			{},

				//DMA RX settings
			{}
				#endif

				#endif
		},

				/* SERIAL 3: */
		{
			#if defined(ENABLE_USART3) 
			#if defined(STM32F767xx) || defined(STM32F446xx)
			//TX Pin
			{ GPIOB, PIN_10, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF7_USART3 },

			//RX Pin
			{ GPIOB, PIN_11, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF7_USART3 },

			//Instance Ptr
			USART3,

			//IT_HW
			{},

			//DMA_IT_TX
			{},

			//DMA_IT_RX
			{},

			//DMA TX settings
			{},

			//DMA RX settings
			{}
			#endif
			/* WHEN YOU SUPORT USART, COMPARE DMA AND IT SETTINGS */
				#endif
		},

				/* SERIAL 4: */
		{
			#if defined(ENABLE_UART4)
			#if defined(STM32F767xx) || defined(STM32F446xx)

			//TX Pin
			{ GPIOA, PIN_0, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF8_UART4 }, //Normally C10, changed for VFCS

																		 //RX Pin
			{ GPIOA, PIN_1, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF8_UART4 }, //Normally C11, changed for VFCS

																		 //Instance Ptr
																		 UART4,

																		 //IT_HW
			{ UART4_IRQn, NVIC_PRIORITYGROUP_4, 6u, 0u },

			//DMA_IT_TX
			{ DMA1_Stream4_IRQn, NVIC_PRIORITYGROUP_4, 0u, 0u },

			//DMA_IT_RX
			{ DMA1_Stream2_IRQn, NVIC_PRIORITYGROUP_4, 0u, 0u },

			//DMA TX settings
			{ DMA1_Stream4, DMA_CHANNEL_4, DMA_MEMORY_TO_PERIPH },

			//DMA RX settings
			{ DMA1_Stream2, DMA_CHANNEL_4, DMA_PERIPH_TO_MEMORY }
			#endif
			#endif
		},

				/* SERIAL 5: */
		{
			#if defined(ENABLE_UART5)
			#if defined(STM32F767xx) || defined(STM32F446xx)

			//TX Pin
			{ GPIOC, PIN_12, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF8_UART5 },

			//RX Pin
			{ GPIOD, PIN_2, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF8_UART5 },

			//Instance Ptr
			UART5,

			//IT_HW
			{ UART5_IRQn, NVIC_PRIORITYGROUP_4, 0u, 0u },

			//DMA_IT_TX
			{ DMA1_Stream7_IRQn, NVIC_PRIORITYGROUP_4, 0u, 0u },

			//DMA_IT_RX
			{ DMA1_Stream0_IRQn, NVIC_PRIORITYGROUP_4, 0u, 0u },

			//DMA TX settings
			{ DMA1_Stream7, DMA_CHANNEL_4, DMA_MEMORY_TO_PERIPH },

			//DMA RX settings
			{ DMA1_Stream0, DMA_CHANNEL_4, DMA_PERIPH_TO_MEMORY }
			#endif
			#endif
		},

				/* SERIAL 6: */
		{
			#if defined(ENABLE_USART6)
			#if defined(STM32F767xx) || defined(STM32F446xx)
			//TX Pin
			{ GPIOG, PIN_14, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF8_USART6 },

			//RX Pin
			{ GPIOG, PIN_9, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF8_USART6 },

			//Instance Ptr
			USART6,

			//IT_HW
			{},

			//DMA_IT_TX
			{},

			//DMA_IT_RX
			{},

			//DMA TX settings
			{},

			//DMA RX settings
			{}
			#endif
			/* WHEN YOU SUPORT USART, COMPARE DMA AND IT SETTINGS */
				#endif
		},

				/* SERIAL 7: */
		{
			#if defined(ENABLE_UART7)
			#if defined(STM32F767xx)
			//TX Pin
			{ GPIOE, PIN_8, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF8_UART7 },

			//RX pin
			{ GPIOE, PIN_7, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF8_UART7 },

			//IT_HW
			{ UART7_IRQn, NVIC_PRIORITYGROUP_4, 0u, 0u },

			//Instance Ptr
			UART7,

			//DMA_IT_TX
			{ DMA1_Stream1_IRQn, NVIC_PRIORITYGROUP_4, 0u, 0u },

			//DMA_IT_RX
			{ DMA1_Stream3_IRQn, NVIC_PRIORITYGROUP_4, 0u, 0u },

			//DMA TX settings
			{ DMA1_Stream1, DMA_CHANNEL_5, DMA_MEMORY_TO_PERIPH },

			//DMA RX settings
			{ DMA1_Stream3, DMA_CHANNEL_5, DMA_PERIPH_TO_MEMORY }
			#endif
			#endif
		},

				/* SERIAL 8: */
		{
			#if defined(ENABLE_UART8)
			#if defined(STM32F767xx)
			//TX Pin
			{ GPIOE, PIN_1, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF8_UART8 },

			//RX pin
			{ GPIOE, PIN_0, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF8_UART8 },

			//Instance Ptr
			UART8,

			//IT_HW
			{ UART8_IRQn, NVIC_PRIORITYGROUP_4, 0u, 0u },

			//DMA_IT_TX
			{ DMA1_Stream0_IRQn, NVIC_PRIORITYGROUP_4, 0u, 0u },

			//DMA_IT_RX
			{ DMA1_Stream6_IRQn, NVIC_PRIORITYGROUP_4, 0u, 0u },

			//DMA TX settings
			{ DMA1_Stream0, DMA_CHANNEL_5, DMA_MEMORY_TO_PERIPH },

			//DMA RX settings
			{ DMA1_Stream6, DMA_CHANNEL_5, DMA_PERIPH_TO_MEMORY }
			#endif
			#endif
		}
			};

			#if defined(STM32F7) || defined(STM32F4)
			const DMA_InitTypeDef dflt_DMA_Init_TX = {
				DMA_CHANNEL_0,				/* Channel */
				DMA_PERIPH_TO_MEMORY,		/* Direction */
				DMA_PINC_DISABLE,			/* PeriphInc */
				DMA_MINC_ENABLE,			/* MemInc */
				DMA_PDATAALIGN_BYTE,		/* PeriphDataAlignment */
				DMA_MDATAALIGN_BYTE,		/* MemDataAlignment */
				DMA_NORMAL,					/* Mode */
				DMA_PRIORITY_LOW,			/* Priority */
				DMA_FIFOMODE_DISABLE,		/* FIFOMode */
				DMA_FIFO_THRESHOLD_FULL,	/* FIFOThreshold */
				DMA_MBURST_SINGLE,			/* MemBurst */
				DMA_PBURST_SINGLE			/* PeriphBurst */
			};

			const DMA_InitTypeDef dflt_DMA_Init_RX = {
				DMA_CHANNEL_0,				/* Channel */
				DMA_PERIPH_TO_MEMORY,		/* Direction */
				DMA_PINC_DISABLE,			/* PeriphInc */
				DMA_MINC_ENABLE,			/* MemInc */
				DMA_PDATAALIGN_BYTE,		/* PeriphDataAlignment */
				DMA_MDATAALIGN_BYTE,		/* MemDataAlignment */
				DMA_NORMAL,					/* Mode */
				DMA_PRIORITY_LOW,			/* Priority */
				DMA_FIFOMODE_DISABLE,		/* FIFOMode */
				DMA_FIFO_THRESHOLD_FULL,	/* FIFOThreshold */
				DMA_MBURST_SINGLE,			/* MemBurst */
				DMA_PBURST_SINGLE			/* PeriphBurst */
			};
			#endif

			#if defined(STM32F4)
			const USART_InitTypeDef dflt_USART_Init = {
				SERIAL_BAUD_115200,				/* BaudRate */
				USART_WORDLENGTH_8B,			/* WordLength */
				USART_STOPBITS_1,				/* StopBits */
				USART_PARITY_NONE,				/* Parity */
				USART_MODE_TX_RX,				/* Mode */
				USART_POLARITY_LOW,				/* CLKPolarity */
				USART_PHASE_1EDGE,				/* CLKPhase */
				USART_LASTBIT_DISABLE			/* CLKLastBit */
			};

			const UART_InitTypeDef dflt_UART_Init = {
				SERIAL_BAUD_115200,				/* BaudRate */
				UART_WORDLENGTH_8B,				/* WordLength */
				UART_STOPBITS_1,				/* StopBits */
				UART_PARITY_NONE,				/* Parity */
				UART_MODE_TX_RX,				/* Mode */
				UART_HWCONTROL_NONE,			/* HwFlowCtl */
				UART_OVERSAMPLING_16,			/* OverSampling */
			};
			#endif

			#if defined(STM32F7)
			const USART_InitTypeDef dflt_USART_Init = {
				SERIAL_BAUD_115200,				/* BaudRate */
				USART_WORDLENGTH_8B,			/* WordLength */
				USART_STOPBITS_1,				/* StopBits */
				USART_PARITY_NONE,				/* Parity */
				USART_MODE_TX_RX,				/* Mode */
				USART_OVERSAMPLING_16,			/* OverSampling */
				USART_POLARITY_LOW,				/* CLKPolarity */
				USART_PHASE_1EDGE,				/* CLKPhase */
				USART_LASTBIT_DISABLE			/* CLKLastBit */
			};

			const UART_InitTypeDef dflt_UART_Init = {
				SERIAL_BAUD_115200,				/* BaudRate */
				UART_WORDLENGTH_8B,				/* WordLength */
				UART_STOPBITS_1,				/* StopBits */
				UART_PARITY_NONE,				/* Parity */
				UART_MODE_TX_RX,				/* Mode */
				UART_HWCONTROL_NONE,			/* HwFlowCtl */
				UART_OVERSAMPLING_16,			/* OverSampling */
				UART_ONE_BIT_SAMPLE_DISABLE		/* OneBitSampling */
			};

			const UART_AdvFeatureInitTypeDef dflt_UART_AdvInit = {
				UART_ADVFEATURE_NO_INIT,					/* AdvFeatureInit */
				UART_ADVFEATURE_TXINV_DISABLE,				/* TxPinLevelInvert */
				UART_ADVFEATURE_RXINV_DISABLE,				/* RxPinLevelInvert */
				UART_ADVFEATURE_DATAINV_DISABLE,			/* DataInvert */
				UART_ADVFEATURE_SWAP_DISABLE,				/* Swap */
				UART_ADVFEATURE_OVERRUN_DISABLE,			/* OverrunDisable */
				UART_ADVFEATURE_DMA_DISABLEONRXERROR,		/* DMADisableonRxError */
				UART_ADVFEATURE_AUTOBAUDRATE_DISABLE,		/* AutoBaudRateEnable */
				UART_ADVFEATURE_AUTOBAUDRATE_ONSTARTBIT,	/* AutoBaudRateMode	 */
				UART_ADVFEATURE_MSBFIRST_DISABLE			/* MSBFirst */
			};
			#endif
		};

		namespace Timer
		{



		};

		namespace SPI
		{
			/*-------------------------------------
			* Handles all the default IO/IT/DMA configuration settings for
			* various chips. Usually this just involves swapping out a few
			* GPIO definitions.
			*------------------------------------*/
			const SPIConfig spi_cfg[] =
			{
				/* SPI 0: Doesn't exist. Only used for easy index alignment to peripheral names */
				{
					//MOSI Pin
					{},

					//MISO Pin
			{},

			//SCK Pin
			{},

			//NSS Pin
			{},

			//SPI Instance Ptr
			nullptr,

			//IT_HW
			{},

			//DMA_IT_TX
			{},

			//DMA_IT_RX
			{},

			//DMA TX settings
			{},

			//DMA RX settings
			{}
				},

				/* SPI 1: */
		{
			#if defined(ENABLE_SPI1)

			#if (defined(STM32F446xx) || defined(STM32F767xx))
			//MOSI Pin
			{ GPIOA, PIN_7, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF5_SPI1 },

			//MISO Pin
			{ GPIOA, PIN_6, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF5_SPI1 },

			//SCK Pin
			{ GPIOA, PIN_5, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF5_SPI1 },

			//NSS Pin
			{ GPIOA, PIN_4, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF5_SPI1 },

			//SPI Instance Ptr
			SPI1,

			//IT_HW
			{ SPI1_IRQn, NVIC_PRIORITYGROUP_4, 0u, 0u },

			//DMA_IT_TX
			{ DMA2_Stream5_IRQn, NVIC_PRIORITYGROUP_4, 0u, 0u },

			//DMA_IT_RX
			{ DMA2_Stream0_IRQn, NVIC_PRIORITYGROUP_4, 0u, 0u },

			//DMA TX settings
			{ DMA2_Stream5, DMA_CHANNEL_3, DMA_MEMORY_TO_PERIPH },

			//DMA RX settings
			{ DMA2_Stream0, DMA_CHANNEL_3, DMA_PERIPH_TO_MEMORY }
			#endif

			#endif /* ENABLE_SPI1 */
		},

				/* SPI 2: */
		{
			#if defined(ENABLE_SPI2)

			#if defined(STM32F767xx)
			//MOSI Pin
			{ GPIOC, PIN_3, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF5_SPI2 },

			//MISO Pin
			{ GPIOC, PIN_2, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF5_SPI2 },

			//SCK Pin
			{ GPIOB, PIN_10, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF5_SPI2 },

			//NSS Pin
			{ GPIOB, PIN_12, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF5_SPI2 },
			#endif

			#if defined(STM32F446xx)
			//MOSI Pin
			{ GPIOC, PIN_1, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF7_SPI2 },	//Nucleo was B15 AF5

																		//MISO Pin
			{ GPIOC, PIN_2, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF5_SPI2 },	//Nucleo was B14 AF5

																		//SCK Pin
			{ GPIOB, PIN_10, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF5_SPI2 },//Nucleo was B3 AF5

																		//NSS Pin
			{ GPIOB, PIN_12, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF5_SPI2 },
			#endif

			#if (defined(STM32F446xx) || defined(STM32F767xx))
			//SPI Instance Ptr
			SPI2,

			//IT_HW
			{ SPI2_IRQn, NVIC_PRIORITYGROUP_4, 0u, 0u },

			//DMA_IT_TX
			{ DMA1_Stream4_IRQn, NVIC_PRIORITYGROUP_4, 0u, 0u },

			//DMA_IT_RX
			{ DMA1_Stream3_IRQn, NVIC_PRIORITYGROUP_4, 0u, 0u },

			//DMA TX settings
			{ DMA1_Stream4, DMA_CHANNEL_0, DMA_MEMORY_TO_PERIPH },

			//DMA RX settings
			{ DMA1_Stream3, DMA_CHANNEL_0, DMA_PERIPH_TO_MEMORY }
			#endif

			#endif /* ENABLE_SPI2 */
		},

				/* SPI 3: */
		{
			#if defined(ENABLE_SPI3)
			#if defined(STM32F767xx)
			//MOSI Pin
			{ GPIOC, PIN_12, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF6_SPI3 },

			//MISO Pin
			{ GPIOC, PIN_11, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF6_SPI3 },

			//SCK Pin
			{ GPIOC, PIN_10, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF6_SPI3 },

			//NSS Pin
			{ GPIOA, PIN_15, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF6_SPI3 },
			#endif

			#if defined(STM32F446xx)
			//MOSI Pin
			{ GPIOB, PIN_0, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF7_SPI3 },	//nucleo was B5 AF6

																		//MISO Pin
			{ GPIOC, PIN_11, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF6_SPI3 }, //nucleo was B4

																		 //SCK Pin
			{ GPIOC, PIN_10, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF6_SPI3 }, //nucleo was B3

																		 //NSS Pin
			{ GPIOA, PIN_15, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF6_SPI3 },
			#endif

			#if defined(STM32F767xx) || defined(STM32F446xx)
			//SPI Instance Ptr
			SPI3,

			//IT_HW
			{ SPI3_IRQn, NVIC_PRIORITYGROUP_4, 0u, 0u },

			//DMA_IT_TX
			{ DMA1_Stream5_IRQn, NVIC_PRIORITYGROUP_4, 0u, 0u },

			//DMA_IT_RX
			{ DMA1_Stream2_IRQn, NVIC_PRIORITYGROUP_4, 0u, 0u },

			//DMA TX settings
			{ DMA1_Stream5, DMA_CHANNEL_0, DMA_MEMORY_TO_PERIPH },

			//DMA RX settings
			{ DMA1_Stream2, DMA_CHANNEL_0, DMA_PERIPH_TO_MEMORY }
			#endif 
			#endif /* ENABLE_SPI3 */
		},

				/* SPI 4: */
		{
			#if defined(ENABLE_SPI4)
			#if defined(STM32F767xx) || defined(STM32F446xx)
			//MOSI Pin
			{ GPIOE, PIN_6, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF5_SPI4 },

			//MISO Pin
			{ GPIOE, PIN_5, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF5_SPI4 },

			//SCK Pin
			{ GPIOE, PIN_2, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF5_SPI4 },

			//NSS Pin
			{ GPIOE, PIN_4, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF5_SPI4 },

			//SPI Instance Ptr
			SPI4,

			//IT_HW
			{ SPI4_IRQn, NVIC_PRIORITYGROUP_4, 0u, 0u },

			//DMA_IT_TX
			{ DMA2_Stream1_IRQn, NVIC_PRIORITYGROUP_4, 0u, 0u },

			//DMA_IT_RX
			{ DMA2_Stream0_IRQn, NVIC_PRIORITYGROUP_4, 0u, 0u },

			//DMA TX settings
			{ DMA2_Stream1, DMA_CHANNEL_4, DMA_MEMORY_TO_PERIPH },

			//DMA RX settings
			{ DMA2_Stream0, DMA_CHANNEL_4, DMA_PERIPH_TO_MEMORY }
			#endif
			#endif /* ENABLE_SPI4 */
		},

				/* SPI 5: */
		{
			#if defined(ENABLE_SPI5)
			#if defined(STM32F767xx)
			//MOSI Pin
			{ GPIOF, PIN_9, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF5_SPI5 },

			//MISO Pin
			{ GPIOF, PIN_8, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF5_SPI5 },

			//SCK Pin
			{ GPIOF, PIN_7, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF5_SPI5 },

			//NSS Pin
			{ GPIOF, PIN_6, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF5_SPI5 },

			//SPI Instance Ptr
			SPI5,

			//IT_HW
			{ SPI5_IRQn, NVIC_PRIORITYGROUP_4, 0u, 0u },

			//DMA_IT_TX
			{ DMA2_Stream4_IRQn, NVIC_PRIORITYGROUP_4, 0u, 0u },

			//DMA_IT_RX
			{ DMA2_Stream3_IRQn, NVIC_PRIORITYGROUP_4, 0u, 0u },

			//DMA TX settings
			{ DMA2_Stream4, DMA_CHANNEL_2, DMA_MEMORY_TO_PERIPH },

			//DMA RX settings
			{ DMA2_Stream3, DMA_CHANNEL_2, DMA_PERIPH_TO_MEMORY }
			#endif
			#endif /* ENABLE_SPI5 */


		},

				/* SPI 6: */
		{
			#if defined(ENABLE_SPI6)
			#if defined(STM32F767xx)
			//MOSI Pin
			{ GPIOG, PIN_14, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF5_SPI6 },

			//MISO Pin
			{ GPIOG, PIN_12, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF5_SPI6 },

			//SCK Pin
			{ GPIOG, PIN_13, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF5_SPI6 },

			//NSS Pin
			{ GPIOG, PIN_8, ALT_PP, ULTRA_SPD, PULLUP, GPIO_AF5_SPI6 },

			//SPI Instance Ptr
			SPI6,

			//IT_HW
			{ SPI6_IRQn, NVIC_PRIORITYGROUP_4, 0u, 0u },

			//DMA_IT_TX
			{ DMA2_Stream5_IRQn, NVIC_PRIORITYGROUP_4, 0u, 0u },

			//DMA_IT_RX
			{ DMA2_Stream6_IRQn, NVIC_PRIORITYGROUP_4, 0u, 0u },

			//DMA TX settings
			{ DMA2_Stream5, DMA_CHANNEL_1, DMA_MEMORY_TO_PERIPH },

			//DMA RX settings
			{ DMA2_Stream6, DMA_CHANNEL_1, DMA_PERIPH_TO_MEMORY }
			#endif
			#endif /* ENABLE_SPI6 */
		}
			};


			/*-------------------------------------
			* Rest of the default initializer structs. Some of the lower end
			* chips don't have all the HAL options so separate defaults sometimes
			* need to be created for a particular family of devices.
			*------------------------------------*/
			#if defined(STM32F7) || defined(STM32F4)
			const DMA_InitTypeDef dflt_DMA_Init_TX = {
				DMA_CHANNEL_0,				/* Channel */
				DMA_MEMORY_TO_PERIPH,		/* Direction */
				DMA_PINC_DISABLE,			/* PeriphInc */
				DMA_MINC_ENABLE,			/* MemInc */
				DMA_PDATAALIGN_BYTE,		/* PeriphDataAlignment */
				DMA_MDATAALIGN_BYTE,		/* MemDataAlignment */
				DMA_NORMAL,					/* Mode */
				DMA_PRIORITY_LOW,			/* Priority */
				DMA_FIFOMODE_DISABLE,		/* FIFOMode */
				DMA_FIFO_THRESHOLD_FULL,	/* FIFOThreshold */
				DMA_MBURST_SINGLE,			/* MemBurst */
				DMA_PBURST_SINGLE			/* PeriphBurst */
			};

			const DMA_InitTypeDef dflt_DMA_Init_RX = {
				DMA_CHANNEL_0,				/* Channel */
				DMA_PERIPH_TO_MEMORY,		/* Direction */
				DMA_PINC_DISABLE,			/* PeriphInc */
				DMA_MINC_ENABLE,			/* MemInc */
				DMA_PDATAALIGN_BYTE,		/* PeriphDataAlignment */
				DMA_MDATAALIGN_BYTE,		/* MemDataAlignment */
				DMA_NORMAL,					/* Mode */
				DMA_PRIORITY_LOW,			/* Priority */
				DMA_FIFOMODE_DISABLE,		/* FIFOMode */
				DMA_FIFO_THRESHOLD_FULL,	/* FIFOThreshold */
				DMA_MBURST_SINGLE,			/* MemBurst */
				DMA_PBURST_SINGLE			/* PeriphBurst */
			};
			#endif

			#if defined (STM32F4)
			const SPI_InitTypeDef dflt_SPI_Init = {
				SPI_MODE_MASTER,				/* Mode */
				SPI_DIRECTION_2LINES,			/* Direction */
				SPI_DATASIZE_8BIT,				/* DataSize */
				SPI_POLARITY_LOW,				/* CLKPolarity */
				SPI_PHASE_1EDGE,				/* CLKPhase */
				SPI_NSS_HARD_OUTPUT,			/* NSS */
				SPI_BAUDRATEPRESCALER_8,		/* BaudRatePrescaler */
				SPI_FIRSTBIT_MSB,				/* FirstBit */
				SPI_TIMODE_DISABLED,			/* TIMode */
				SPI_CRCCALCULATION_DISABLED,	/* CRCCalculation */
				65535,							/* CRCPolynomial */
			};
			#endif

			#if defined (STM32F7)
			const SPI_InitTypeDef dflt_SPI_Init = {
				SPI_MODE_MASTER,				/* Mode */
				SPI_DIRECTION_2LINES,			/* Direction */
				SPI_DATASIZE_8BIT,				/* DataSize */
				SPI_POLARITY_LOW,				/* CLKPolarity */
				SPI_PHASE_1EDGE,				/* CLKPhase */
				SPI_NSS_HARD_OUTPUT,			/* NSS */
				SPI_BAUDRATEPRESCALER_8,		/* BaudRatePrescaler */
				SPI_FIRSTBIT_MSB,				/* FirstBit */
				SPI_TIMODE_DISABLED,			/* TIMode */
				SPI_CRCCALCULATION_DISABLED,	/* CRCCalculation */
				65535,							/* CRCPolynomial */
				SPI_CRC_LENGTH_DATASIZE,		/* CRCLength */
				SPI_NSS_PULSE_ENABLE			/* NSSPMode */
			};
			#endif




		};
	}
}


