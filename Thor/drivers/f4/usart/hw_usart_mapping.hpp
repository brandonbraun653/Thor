/********************************************************************************
 *   File Name:
 *    hw_usart_mapping.hpp
 *
 *   Description:
 *    Provides structures for conversion and mapping between data types for fast
 *    runtime performance of driver code.
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_USART_MAPPING_HPP
#define THOR_HW_USART_MAPPING_HPP

/* C++ Includes */
#include <unordered_map>

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/f4/interrupt/hw_it_prj.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_types.hpp>
#include <Thor/drivers/f4/usart/hw_usart_types.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_USART == 1 )

namespace Thor::Driver::USART
{
  /**
   *  Maps a USART peripheral into the corresponding resource index
   */
  extern const Thor::Driver::RCC::ResourceMap_t InstanceToResourceIndex;

  extern const std::unordered_map<size_t, RegisterMap *const> ChanneltoInstance;

  /**
   *  Gets the interrupt request number tied to a USART instance.
   *
   *  @note Must match the mapping in InstanceToResourceIndex
   */
  extern const IRQn_Type USART_IRQn[ NUM_USART_PERIPHS ];


  /**
   *  Conversion arrays to "map" Chimera::Serial enum configuration options
   *  into the relevant register configuration values for the MCU.
   */
  extern const std::array<uint32_t, NUM_USART_PERIPHS> CharWidToRegConfig;
  extern const std::array<uint32_t, NUM_USART_PERIPHS> ParityToRegConfig;
  extern const std::array<uint32_t, NUM_USART_PERIPHS> StopBitsToRegConfig;

}    // namespace Thor::Driver::USART

#endif /* TARGET_STM32F4 && THOR_DRIVER_USART */
#endif /* !THOR_HW_USART_MAPPING_HPP */