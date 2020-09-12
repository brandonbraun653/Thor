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
#define STM32_USART1_PERIPH_AVAILABLE
#define STM32_USART2_PERIPH_AVAILABLE


namespace Thor::LLD::USART
{
  /**
   *  Initializes the LLD register resources and memory
   *
   *  @return void
   */
  void initializeRegisters();

  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  static constexpr size_t NUM_USART_PERIPHS = 2;

  static constexpr RIndex_t USART1_RESOURCE_INDEX = 0u;
  static constexpr RIndex_t USART2_RESOURCE_INDEX = 1u;
  static constexpr RIndex_t USART3_RESOURCE_INDEX = INVALID_RESOURCE_INDEX;

  static constexpr uint32_t USART1_BASE_ADDR = Thor::System::MemoryMap::USART1_PERIPH_START_ADDRESS;
  static constexpr uint32_t USART2_BASE_ADDR = Thor::System::MemoryMap::USART2_PERIPH_START_ADDRESS;
  static constexpr uint32_t USART3_BASE_ADDR = Thor::System::MemoryMap::USART3_PERIPH_START_ADDRESS;

}    // namespace Thor::LLD::USART

#endif /* !THOR_HW_USART_REGISTER_STM32L432KC_HPP */