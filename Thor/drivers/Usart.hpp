/********************************************************************************
 *  File Name:
 *    Usart.hpp
 *  
 *  Description:
 *    Common header for Thor USART that configures the driver based on which
 *    chip family is being compiled against. 
 *  
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_USART_CONFIG_HPP
#define THOR_USART_CONFIG_HPP

#include <Thor/preprocessor.hpp>
#include <Thor/drivers/common/interrupts/usart_interrupt_vectors.hpp>

/*-------------------------------------------------
Using the custom STM32 driver
-------------------------------------------------*/
#if defined( THOR_CUSTOM_DRIVERS ) && ( THOR_CUSTOM_DRIVERS == 1)

#include <Thor/drivers/common/types/serial_types.hpp>

#if defined( TARGET_STM32F4 )
#include <Thor/drivers/f4/usart/hw_usart_driver.hpp>
#include <Thor/drivers/f4/usart/hw_usart_mapping.hpp>
#endif

#if defined( TARGET_STM32F7 )
#include <Thor/drivers/f7/usart/hw_usart_driver.hpp>
#endif

/*-------------------------------------------------
Using the STM32HAL
-------------------------------------------------*/
#elif defined( THOR_STM32HAL_DRIVERS ) && ( THOR_STM32HAL_DRIVERS == 1 )
#if defined( TARGET_STM32F4 )
#include <stm32f4/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_usart.h>
#endif

#if defined( TARGET_STM32F7 )
#include <stm32f7/STM32F7xx_HAL_Driver/Inc/stm32f7xx_hal_usart.h>
#endif

/*-------------------------------------------------
Using an unsupported driver?
-------------------------------------------------*/
#else
#error Unknown Thor USART driver implementation
#endif /* THOR_CUSTOM_DRIVERS */

#endif /* !THOR_USART_CONFIG_HPP */