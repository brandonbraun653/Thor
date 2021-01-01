/********************************************************************************
 *  File Name:
 *    hw_rcc_config_stm32l432xx.hpp
 *
 *  Description:
 *    Contains reset and clock configuration options
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_RCC_CONFIG_STM32L432XX_HPP
#define THOR_LLD_RCC_CONFIG_STM32L432XX_HPP

/* C++ Includes */
#include <cstdint>

/* Driver Includes */
#include <Thor/cfg>

#if defined( TARGET_STM32L4 ) && defined( THOR_LLD_RCC ) && defined( STM32L432xx )

#ifdef __cplusplus
extern "C"
{
#endif

  extern const uint8_t AHBPrescTable[ 16 ];
  extern const uint8_t APBPrescTable[ 8 ];
  extern const uint32_t MSIRangeTable[ 12 ];

#ifdef __cplusplus
}
#endif

#endif /* TARGET_STM32L4 */

#endif  /* !THOR_LLD_RCC_CONFIG_STM32L432XX_HPP */
