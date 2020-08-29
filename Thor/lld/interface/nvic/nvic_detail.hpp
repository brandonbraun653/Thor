/********************************************************************************
 *  File Name:
 *    nvic.hpp
 *
 *  Description:
 *    Common header for Thor NVIC that configures the driver based on which
 *    chip family is being compiled against.
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_NVIC_DETAIL_HPP
#define THOR_NVIC_DETAIL_HPP

#if defined( TARGET_LLD_MOCK )
#include <Thor/lld/interface/nvic/mock/nvic_mock.hpp>
#elif defined( TARGET_STM32F4 )
#include <Thor/lld/stm32f4x/nvic/hw_nvic_driver.hpp>
#else
#pragma message( "nvic_detail.hpp: Unknown target for LLD" )
#endif

#endif /* !THOR_NVIC_DETAIL_HPP */
