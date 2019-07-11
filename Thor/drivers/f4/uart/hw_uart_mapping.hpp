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
#include <Thor/headers.hpp>
#include <Thor/drivers/f4/uart/hw_uart_types.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_UART == 1 )

namespace Thor::Driver::UART
{
  /**
   *  Maps a UART peripheral into the corresponding resource index
   */
  extern const std::unordered_map<std::uintptr_t, size_t> InstanceToResourceIndex;

}    // namespace Thor::Driver::UART

#endif /* TARGET_STM32F4 && THOR_DRIVER_UART */
#endif /* !THOR_HW_UART_MAPPING_HPP */
