/******************************************************************************
 *  File Name:
 *    exti_detail.hpp
 *
 *  Description:
 *    Includes the LLD specific headers for chip implementation details
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_EXTI_INTF_DETAIL_HPP
#define THOR_LLD_EXTI_INTF_DETAIL_HPP

#if defined( TARGET_LLD_MOCK )
#include <Thor/lld/interface/exti/mock/exti_mock.hpp>
#include <Thor/lld/interface/exti/mock/exti_mock_variant.hpp>
#elif defined( TARGET_LLD_TEST )
#include <Thor/lld/interface/exti/sim/exti_sim_variant.hpp>
#include <Thor/lld/interface/exti/sim/exti_sim_types.hpp>
#elif defined( TARGET_STM32F4 )
#include <Thor/lld/stm32f4x/exti/hw_exti_prj.hpp>
#include <Thor/lld/stm32f4x/exti/hw_exti_types.hpp>
#elif defined( TARGET_STM32L4 )
#include <Thor/lld/stm32l4x/exti/hw_exti_prj.hpp>
#include <Thor/lld/stm32l4x/exti/hw_exti_types.hpp>
#else
#pragma message( "exti_detail.hpp: Unknown target for LLD" )
#endif

#endif  /* !THOR_LLD_EXTI_INTF_DETAIL_HPP */
