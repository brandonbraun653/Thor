/******************************************************************************
 *  File Name:
 *    config.hpp
 *
 *  Description:
 *    Provides system configuration settings by routing to the correct
 *    header file. Usually these files enable/disable peripheral modules.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_CONFIG_HPP
#define THOR_CONFIG_HPP

/* Thor Includes */
#include <Thor/hld/common/preprocessor.hpp>

/*-------------------------------------------------------------------------------
Select the appropriate driver configuration file
-------------------------------------------------------------------------------*/
#if defined( THOR_USER_CONFIG )
#include "thor_user_config.hpp"
#else /* Default Project Configuration */
#if !defined( TARGET_LLD_MOCK ) && ( defined( TARGET_LLD_TEST ) || defined( SIM ) )
#include <Thor/config/simulator/thor_sim_config.hpp>
#elif defined( TARGET_STM32F4 )
#include <Thor/config/hardware/thor_stm32f4x_config.hpp>
#elif defined( TARGET_STM32L4 )
#include <Thor/config/hardware/thor_stm32l4x_config.hpp>
#elif defined( TARGET_LLD_MOCK )
#include <Thor/config/simulator/thor_gtest_config.hpp>
#else
#pragma message( "Unknown configuration options" )
#endif
#endif /* THOR_USER_CONFIG */

#endif /* !THOR_CONFIG_HPP */
