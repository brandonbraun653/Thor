/********************************************************************************
 *   File Name:
 *    hw_uart_mapping.hpp
 *
 *   Description:
 *    Provides structures for conversion and mapping between data types for fast
 *    runtime performance of driver code.
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_UART_MAPPING_HPP
#define THOR_HW_UART_MAPPING_HPP

/* C++ Includes */
#include <unordered_map>

/* Driver Includes */
#include <Thor/drivers/f4/uart/hw_uart_types.hpp>

namespace Thor::Driver::UART
{
  /* clang-format off */

  /**
   *  Maps a UART peripheral into the corresponding resource index 
   */
  static const std::unordered_map<std::uintptr_t, size_t> InstanceToResourceIndex{
    { reinterpret_cast<std::uintptr_t>( UART4_PERIPH ), 0 },
    { reinterpret_cast<std::uintptr_t>( UART5_PERIPH ), 1 }
  };

  /* clang-format on */
}    // namespace Thor::Driver::UART

#endif /* !THOR_HW_UART_MAPPING_HPP */
