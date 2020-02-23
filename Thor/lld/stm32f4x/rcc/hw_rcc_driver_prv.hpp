/********************************************************************************
 *  File Name:
 *    hw_rcc_driver_prv.hpp
 *
 *  Description:
 *    Private declarations for internal use with the RCC driver
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_DRIVER_RCC_PRIVATE_HPP
#define THOR_DRIVER_RCC_PRIVATE_HPP

/* C++ Includes */
#include <cstdint>

namespace Thor::LLD::RCC
{
  static constexpr uint32_t PLL_TIMEOUT_VALUE_MS          = 2u;
  static constexpr uint32_t DBP_TIMEOUT_VALUE_MS          = 2u;
  static constexpr uint32_t LSE_TIMEOUT_VALUE_MS          = 5000u;
  static constexpr uint32_t HSE_TIMEOUT_VALUE_MS          = 100u;
  static constexpr uint32_t HSI_TIMEOUT_VALUE_MS          = 2u; 
  static constexpr uint32_t LSI_TIMEOUT_VALUE_MS          = 2u; 
  static constexpr uint32_t CLOCKSWITCH_TIMEOUT_VALUE_MS  = 5000u;
}

#endif /* !THOR_DRIVER_RCC_PRIVATE_HPP */