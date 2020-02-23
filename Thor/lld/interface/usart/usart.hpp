/********************************************************************************
 *  File Name:
 *    usart.hpp
 *  
 *  Description:
 *    Common header for Thor USART that configures the driver based on which
 *    chip family is being compiled against. 
 *  
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_USART_CONFIG_HPP
#define THOR_USART_CONFIG_HPP

#if defined( TARGET_STM32F4 )
#include <Thor/lld/stm32f4x/usart/hw_usart_driver.hpp>
#include <Thor/lld/stm32f4x/usart/hw_usart_mapping.hpp>
#endif

#endif /* !THOR_USART_CONFIG_HPP */