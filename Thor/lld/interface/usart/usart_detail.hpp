/********************************************************************************
 *  File Name:
 *    usart_detail.hpp
 *
 *  Description:
 *    Includes the LLD specific headers for chip implementation details
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_USART_DETAIL_HPP
#define THOR_USART_DETAIL_HPP

#if defined( TARGET_LLD_MOCK )
#include <Thor/lld/interface/usart/mock/usart_mock.hpp>
#include <Thor/lld/interface/usart/mock/usart_mock_variant.hpp>
#elif defined( TARGET_LLD_TEST )
#include <Thor/lld/interface/usart/sim/usart_sim_variant.hpp>
#include <Thor/lld/interface/usart/sim/usart_sim_types.hpp>
#elif defined( TARGET_STM32F4 )
#include <Thor/lld/stm32f4x/usart/hw_usart_driver.hpp>
#include <Thor/lld/stm32f4x/usart/hw_usart_prj.hpp>
#include <Thor/lld/stm32f4x/usart/hw_usart_mapping.hpp>
#elif defined( TARGET_STM32L4 )
#include <Thor/lld/stm32l4x/usart/hw_usart_types.hpp>
#include <Thor/lld/stm32l4x/usart/hw_usart_prj.hpp>
#else
#pragma message( "usart_detail.hpp: Unknown target for LLD" )
#endif

#endif /* !THOR_USART_DETAIL_HPP */
