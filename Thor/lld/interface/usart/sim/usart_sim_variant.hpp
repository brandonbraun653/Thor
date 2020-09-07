/********************************************************************************
 *  File Name:
 *    hw_usart_register_stm32l432kc.hpp
 *
 *  Description:
 *    USART register definitions for the STM32L432KC series chips.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_USART_REGISTER_SIM_VARIANT_HPP
#define THOR_HW_USART_REGISTER_SIM_VARIANT_HPP

/* C++ Includes */
#include <array>
#include <cstdint>
#include <cstddef>

/* Chimera Includes */
#include <Chimera/serial>

/* Driver Includes */
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/system/sim/system_sim_memory_map.hpp>

/*-------------------------------------------------
Peripheral Availability
-------------------------------------------------*/
// Available on every STM32L4 device
#define STM32_USART1_PERIPH_AVAILABLE
#define STM32_USART2_PERIPH_AVAILABLE

#if defined( STM32L432xx )
// Device doesn't have USART3
#else
#define STM32_USART3_PERIPH_AVAILABLE
#endif

namespace Thor::LLD::USART
{
  /**
   *  Initializes the LLD register resources and memory
   *
   *  @return void
   */
  void initializeRegisters();

  /*-------------------------------------------------
  Peripheral Instance Memory Map Base
  -------------------------------------------------*/
  static constexpr RIndex_t NUM_USART_PERIPHS = 3;

#if defined( STM32_USART1_PERIPH_AVAILABLE )
  static constexpr uint32_t USART1_BASE_ADDR = Thor::System::MemoryMap::USART1_PERIPH_START_ADDRESS;
#else
  static constexpr uint32_t USART1_BASE_ADDR = std::numeric_limits<uint32_t>::max();
#endif

#if defined( STM32_USART2_PERIPH_AVAILABLE )
  static constexpr uint32_t USART2_BASE_ADDR = Thor::System::MemoryMap::USART2_PERIPH_START_ADDRESS;
#else
  static constexpr uint32_t USART2_BASE_ADDR = std::numeric_limits<uint32_t>::max();
#endif

#if defined( STM32_USART3_PERIPH_AVAILABLE )
  static constexpr uint32_t USART3_BASE_ADDR = Thor::System::MemoryMap::USART3_PERIPH_START_ADDRESS;
#else
  static constexpr uint32_t USART3_BASE_ADDR = std::numeric_limits<uint32_t>::max();
#endif

  /*-------------------------------------------------
  Lookup addresses
  -------------------------------------------------*/
  static constexpr std::array<uint32_t, NUM_USART_PERIPHS> periphAddressList = { USART1_BASE_ADDR, USART2_BASE_ADDR,
                                                                                 USART3_BASE_ADDR };

}    // namespace Thor::LLD::USART

#endif /* !THOR_HW_USART_REGISTER_STM32L432KC_HPP */