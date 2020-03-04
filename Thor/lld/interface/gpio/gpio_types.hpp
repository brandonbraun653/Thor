/********************************************************************************
 *   File Name:
 *    gpio_types.hpp
 *
 *   Description:
 *    Common GPIO types used in Thor drivers
 *
 *   2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once 
#ifndef THOR_DRIVER_GPIO_COMMON_TYPES_HPP
#define THOR_DRIVER_GPIO_COMMON_TYPES_HPP

/* C++ Includes */
#include <cstdint>

namespace Thor::LLD::GPIO
{
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
    NUM_OPTIONS
  };

  /**
   *  Forward declaration to ease compilation
   */
  struct RegisterMap;
}

#endif /* !THOR_DRIVER_GPIO_COMMON_TYPES_HPP */