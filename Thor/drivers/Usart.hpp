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

#if defined( STM32F4 )
#include <Thor/drivers/f4/usart/hw_usart_driver.hpp>
#endif /* STM32F4 */

#if defined( STM32F7 )
#include <Thor/drivers/f7/usart/hw_usart_driver.hpp>
#endif /* STM32F7 */

#endif /* !THOR_USART_CONFIG_HPP */