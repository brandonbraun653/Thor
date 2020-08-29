/********************************************************************************
 *  File Name:
 *    gpio_types.hpp
 *
 *  Description:
 *    Common LLD GPIO Types
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_LLD_GPIO_DRIVER_TYPES_HPP
#define THOR_LLD_GPIO_DRIVER_TYPES_HPP

/* STL Includes */
#include <cstdint>

/* Chimera Includes */
#include <Chimera/container>
#include <Chimera/gpio>

namespace Thor::LLD::GPIO
{
  /*-------------------------------------------------------------------------------
  Foward Declarations
  -------------------------------------------------------------------------------*/
  class IDriver;
  struct RegisterMap;

  /*-------------------------------------------------------------------------------
  Aliases
  -------------------------------------------------------------------------------*/
  using AFToReg      = Chimera::Container::LightFlatMap<Chimera::GPIO::Alternate, Reg32_t>;
  using AlternateMap = Chimera::Container::LightFlatMap<RegisterMap *, const void *>;
  using IDriver_rPtr = IDriver *;
  using IndexMap     = Chimera::Container::LightFlatMap<std::uintptr_t, size_t>;
  using InstanceMap  = Chimera::Container::LightFlatMap<Chimera::GPIO::Port, RegisterMap *>;
  using PinToAFMap   = Chimera::Container::LightFlatMap<uint8_t, const AFToReg *>;
  using PortMap      = Chimera::Container::LightFlatMap<RegisterMap *, Chimera::GPIO::Port>;

  /*-------------------------------------------------------------------------------
  Enumerations
  -------------------------------------------------------------------------------*/
  /**
   *  Effectively defines the drive strength of the GPIO output. Actual
   *  strength depends on VDD and the connected load.
   *
   *  @note Do not change these enum values as they are used to index arrays
   */
  enum class Speed : uint8_t
  {
    LOW = 0,
    MEDIUM,
    FAST,
    HIGH,
    NUM_OPTIONS,
    UNKNOWN_SPEED
  };
}    // namespace Thor::LLD::GPIO

#endif /* !THOR_LLD_GPIO_DRIVER_TYPES_HPP */
