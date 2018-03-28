#pragma once
#ifndef THOR_CONFIG_H_
#define THOR_CONFIG_H_
/*-------------------------------------------
* Check for supported compiler options:
* https://clang.llvm.org/docs/LanguageExtensions.html 
*-------------------------------------------*/
#ifndef __has_include
#error FATAL ERROR: Please use a compiler that supports __has_include(), such as Clang or MSVC 2015 Update 2 or higher
#endif

/*-------------------------------------------
* Compile time checking for the correct files 
*-------------------------------------------*/

/* STM32F7 SERIES DEVICES */
#if __has_include("stm32f7xx.h")
	#define TARGET_STM32F7

	#if __has_include("stm32f7xx_hal.h")
		#define HAL_AVAILABLE
	#else
		#error Please include the HAL driver for STM32F7
	#endif

/* STM32F4 SERIES DEVICES */
#elif __has_include("stm32f4xx.h")
	#define TARGET_STM32F4

	#if __has_include("stm32f4xx_hal.h")
		#define HAL_AVAILABLE
	#else	
		#error Please include the HAL driver for STM32F4
	#endif

#else
	#error Target Device Not Supported Yet
#endif

/* RTOS */
#if __has_include("FreeRTOS.h")
	#define USING_FREERTOS		/* Thor Flag */
#endif


/*-------------------------------------------
* UART/USART:
* Choose which serial peripherals to be available
*-------------------------------------------*/
#ifdef TARGET_STM32F7
	#if defined(STM32F767xx)
		//#define ENABLE_USART1	
		//#define ENABLE_USART2	
		//#define ENABLE_USART3	
		//#define ENABLE_UART4	
		//#define ENABLE_UART5	
		//#define ENABLE_USART6	
		//#define ENABLE_UART7	
		//#define ENABLE_UART8	
	#endif
#endif

#ifdef TARGET_STM32F4
	#if defined(STM32F446xx)
		//#define ENABLE_USART1
		//#define ENABLE_USART2	
		//#define ENABLE_USART3	
		#define ENABLE_UART4	
		#define ENABLE_UART5	
		//#define ENABLE_USART6		
	#endif
#endif
/*-------------------------------------------
* SPI:
* Choose which SPI peripherals to be available
*-------------------------------------------*/
#ifdef TARGET_STM32F7
	#if defined(STM32F767xx)
		#define ENABLE_SPI1
		//#define ENABLE_SPI2
		//#define ENABLE_SPI3
		//#define ENABLE_SPI4
		//#define ENABLE_SPI5
		//#define ENABLE_SPI6
	#endif
#endif

#ifdef TARGET_STM32F4
	#if defined (STM32F446xx)
		//#define ENABLE_SPI1
		#define ENABLE_SPI2
		#define ENABLE_SPI3
		//#define ENABLE_SPI4
	#endif
#endif


/*-------------------------------------------
* ??
*-------------------------------------------*/

#endif