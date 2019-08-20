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

/* Chimera Includes */
#include <Chimera/types/peripheral_types.hpp>
#include <Chimera/container.hpp>

namespace Thor::Driver::Mapping
{
  /**
   *  Maps a peripheral type into an index used to access peripheral specific resources.
   */
  extern const Chimera::Container::LightFlatMap<Chimera::Peripheral::Type, uint8_t> PeriphTypeToIterator;
}

#endif /* !THOR_DRIVER_COMMON_MAPPING_HPP */