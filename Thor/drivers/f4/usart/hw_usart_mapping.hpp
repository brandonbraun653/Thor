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
#include <Thor/drivers/f4/usart/hw_usart_types.hpp>

namespace Thor::Driver::USART
{
  /* clang-format off */

  /**
   *  Maps a USART peripheral into the corresponding resource index 
   */
  static const std::unordered_map<std::uintptr_t, size_t> InstanceToResourceIndex{
    { reinterpret_cast<std::uintptr_t>( USART1_PERIPH ), 0 },
    { reinterpret_cast<std::uintptr_t>( USART2_PERIPH ), 1 },
    { reinterpret_cast<std::uintptr_t>( USART3_PERIPH ), 2 },
    { reinterpret_cast<std::uintptr_t>( USART6_PERIPH ), 3 }
  };

  /* clang-format on */
}    // namespace Thor::Driver::USART

#endif /* !THOR_HW_USART_MAPPING_HPP */