/********************************************************************************
 *   File Name:
 *    peripheral_mapping.hpp
 *
 *   Description:
 *    Mappings for Chimera based peripheral types onto various data structures
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_DRIVER_COMMON_MAPPING_HPP
#define THOR_DRIVER_COMMON_MAPPING_HPP

/* C++ Includes */
#include <unordered_map>

/* Chimera Includes */
#include <Chimera/types/peripheral_types.hpp>

namespace Thor::Driver::Mapping
{
  /* clang-format off */

  /**
   *  Maps a peripheral type into an index used to access peripheral specific resources.
   */
  static const std::unordered_map<Chimera::Peripheral::Type, uint8_t> PeriphTypeToIterator{
    { Chimera::Peripheral::Type::GPIO,  0u },
    { Chimera::Peripheral::Type::UART,  1u },
    { Chimera::Peripheral::Type::USART, 2u },
  };

  /* clang-format on */
}

#endif /* !THOR_DRIVER_COMMON_MAPPING_HPP */