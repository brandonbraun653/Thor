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
#include <Thor/drivers/f4/usart/hw_usart_types.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_USART == 1 )

namespace Thor::Driver::USART
{
  /**
   *  Maps a USART peripheral into the corresponding resource index
   */
  extern const std::unordered_map<std::uintptr_t, size_t> InstanceToResourceIndex;

}    // namespace Thor::Driver::USART

#endif /* TARGET_STM32F4 && THOR_DRIVER_USART */
#endif /* !THOR_HW_USART_MAPPING_HPP */