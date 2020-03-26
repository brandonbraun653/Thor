/********************************************************************************
 *  File Name:
 *    config.hpp
 *
 *  Description:
 *    Sources configuration settings from the appropriate file. Try not to modify
 *    this too often as it will trigger a complete rebuild.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_CONFIG_HPP
#define THOR_CONFIG_HPP

/* Thor Includes */
#include <Thor/hld/common/preprocessor.hpp>

#if defined( _SIM ) && !defined( TARGET_LLD_MOCK )
#include <Thor/config/simulator/thor_sim_config.hpp>
#elif defined( TARGET_STM32F4 )
#include <Thor/config/hardware/thor_stm32f4x_config.hpp>
#elif defined( TARGET_STM32L4 )
#include <Thor/config/hardware/thor_stm32l4x_config.hpp>
#elif defined( TARGET_LLD_MOCK )
#include <Thor/config/simulator/thor_gtest_config.hpp>
#else
#pragma message("Unknown configuration options")
#endif 

#endif /* !THOR_CONFIG_HPP */
