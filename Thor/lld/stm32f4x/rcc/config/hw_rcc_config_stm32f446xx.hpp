/********************************************************************************
 *  File Name:
 *    hw_rcc_config_stm32f446xx.hpp
 *
 *  Description:
 *    Contains reset and clock configuration options for the STM32F446xx
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_DRIVER_RCC_CONFIG_STM32F446XX_HPP
#define THOR_DRIVER_RCC_CONFIG_STM32F446XX_HPP

/* C++ Includes */
#include <cstdint>

/* Driver Includes */
#include <Thor/cfg>

#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_RCC ) && defined( STM32F446xx )

#ifdef __cplusplus
extern "C"
{
#endif

  /**
   *  Variable that holds the current value of the SYSCLK configuration. This value
   *  is updated anytime the clock configuration is updated. C-linkage is required
   *  for integration with FreeRTOS and some CMSIS functionality.
   */
  extern uint32_t SystemCoreClock;

  extern const uint8_t AHBPrescTable[ 16 ];
  extern const uint8_t APBPrescTable[ 8 ];

#ifdef __cplusplus
}
#endif

#endif /* TARGET_STM32F4 */
#endif /* !THOR_DRIVER_RCC_CONFIG_STM32F446XX_HPP */