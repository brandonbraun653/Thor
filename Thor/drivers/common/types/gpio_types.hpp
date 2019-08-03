/********************************************************************************
 *   File Name:
 *    gpio_types.hpp
 *
 *   Description:
 *    Common GPIO types used in Thor drivers
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once 
#ifndef THOR_DRIVER_GPIO_COMMON_TYPES_HPP
#define THOR_DRIVER_GPIO_COMMON_TYPES_HPP

/* C++ Includes */
#include <cstdint>

namespace Thor::Driver::GPIO
{
  /**
   *  Effectively defines the drive strength of the GPIO output. Actual
   *  strength depends on VDD and the connected load.
   */
  enum class Speed : uint8_t
  {
    LOW,
    MEDIUM,
    FAST,
    HIGH,
    MAX_SPEED = HIGH
  };

  /**
   *  Forward declaration to ease compilation
   */
  struct RegisterMap;
}

#endif /* !THOR_DRIVER_GPIO_COMMON_TYPES_HPP */