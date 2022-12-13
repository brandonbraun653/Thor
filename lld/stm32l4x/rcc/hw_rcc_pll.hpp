/******************************************************************************
 *  File Name:
 *    hw_rcc_pll.hpp
 *
 *  Description:
 *    Utilities for helping configure the PLLs on an STM32L4 chip
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_HW_RCC_PLL_HPP
#define THOR_HW_RCC_PLL_HPP

#include <Thor/lld/stm32l4x/rcc/hw_rcc_types.hpp>

namespace Thor::LLD::RCC
{
  /**
   *  Configures the P oscillator output in the PLL
   *
   *  @note Assumes interrupts are disabled and the PLL is not configured
   *        as the source clock for the system or peripheral
   *
   *  @param[in]  config    The current configuration settings
   *  @return bool
   */
  bool PLLConfigureP( OscillatorSettings &config );

  /**
   *  Configures the Q oscillator output in the PLL
   *
   *  @note Assumes interrupts are disabled and the PLL is not configured
   *        as the source clock for the system or peripheral
   *
   *  @param[in]  config    The current configuration settings
   *  @return bool
   */
  bool PLLConfigureQ( OscillatorSettings &config );

  /**
   *  Configures the R oscillator output in the PLL
   *
   *  @note Assumes interrupts are disabled and the PLL is not configured
   *        as the source clock for the system or peripheral
   *
   *  @param[in]  config    The current configuration settings
   *  @return bool
   */
  bool PLLConfigureR( OscillatorSettings &config );
}

#endif  /* !THOR_HW_RCC_PLL_HPP */
