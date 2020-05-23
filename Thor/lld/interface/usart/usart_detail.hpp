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
#ifndef THOR_USART_DETAIL_HPP
#define THOR_USART_DETAIL_HPP

#if defined( TARGET_LLD_MOCK )
#include <Thor/lld/interface/usart/mock/usart_mock.hpp>
#elif defined( TARGET_STM32F4 )
#include <Thor/lld/stm32f4x/usart/hw_usart_driver.hpp>
#include <Thor/lld/stm32f4x/usart/hw_usart_prj.hpp>
#include <Thor/lld/stm32f4x/usart/hw_usart_mapping.hpp>
#elif defined( TARGET_STM32L4 )
#include <Thor/lld/stm32l4x/usart/hw_usart_driver.hpp>
#include <Thor/lld/stm32l4x/usart/hw_usart_prj.hpp>
#include <Thor/lld/stm32l4x/usart/hw_usart_mapping.hpp>
#else
#pragma message( "Unknown target device for USART low level driver" )
#endif

#endif  /* !THOR_USART_DETAIL_HPP */
