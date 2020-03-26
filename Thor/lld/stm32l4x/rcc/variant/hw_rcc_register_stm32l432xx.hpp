/********************************************************************************
 *  File Name:
 *    hw_rcc_register_stm32l432xx.hpp
 *
 *  Description:
 *    RCC register definitions for the STM32L432xx series chips
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_RCC_REGISTER_HPP
#define THOR_LLD_RCC_REGISTER_HPP

/* C++ Includes */
#include <cstdint>

/* Chimera Includes */
#include <Chimera/common>

/* Driver Includes */
#include <Thor/lld/stm32l4x/system/sys_memory_map_prj.hpp>
//
//#define STM32_RCC1_PERIPH_AVAILABLE
//
//namespace Thor::LLD::RCC
//{
//  void initializeRegisters();
//
//
//  static constexpr Reg32_t NUM_RCC_PERIPHS     = 1;
//  static constexpr uint8_t RCC1_RESOURCE_INDEX = 0u;
//  static constexpr Reg32_t RCC1_BASE_ADDR      = Thor::System::MemoryMap::AHB1PERIPH_BASE_ADDR + 0x3800U;
//
//  static constexpr std::array<Reg32_t, NUM_RCC_PERIPHS> periphAddressList = { RCC1_BASE_ADDR };
//
//}    // namespace Thor::LLD::RCC

#endif /* !THOR_LLD_RCC_REGISTER_HPP */
