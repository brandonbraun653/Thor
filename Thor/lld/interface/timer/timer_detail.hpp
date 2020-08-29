/********************************************************************************
 *  File Name:
 *    timer_detail.hpp
 *
 *  Description:
 *    Common header for Thor TIMER that configures the driver based on which
 *    chip family is being compiled against.
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_LLD_TIMER_DETAIL_HPP
#define THOR_LLD_TIMER_DETAIL_HPP

#if defined( TARGET_LLD_MOCK )
#include <Thor/lld/interface/timer/mock/timer_mock.hpp>
#include <Thor/lld/interface/timer/mock/timer_mock_variant.hpp>
#elif defined( TARGET_STM32F4 )
#include <Thor/lld/stm32f4x/timer/hw_timer_driver.hpp>
#elif defined( TARGET_STM32L4 )
#include <Thor/lld/stm32l4x/timer/hw_timer_driver.hpp>
#else
#pragma message( "timer_detail.hpp: Unknown target for LLD" )
#endif

#endif /* !THOR_LLD_TIMER_DETAIL_HPP */
