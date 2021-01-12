/********************************************************************************
 *  File Name:
 *    rcc_detail.hpp
 *
 *  Description:
 *    Includes the LLD specific headers for chip implementation details as well
 *    as defines various functions that are available on all chip types.
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_RCC_DETAIL_HPP
#define THOR_LLD_RCC_DETAIL_HPP

#if defined( TARGET_LLD_MOCK )
#include <Thor/lld/interface/rcc/mock/rcc_mock.hpp>
#elif defined( TARGET_STM32F4 )
#include <Thor/lld/stm32f4x/rcc/hw_rcc_prj.hpp>
#include <Thor/lld/stm32f4x/rcc/hw_rcc_types.hpp>
#elif defined( TARGET_STM32L4 )
#include <Thor/lld/stm32l4x/rcc/hw_rcc_prj.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_types.hpp>
#else
#pragma message( "rcc_detail.hpp: Unknown target for LLD" )
#endif

#endif /* !THOR_LLD_RCC_DETAIL_HPP */
