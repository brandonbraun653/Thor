/********************************************************************************
 *   File Name:
 *    clock_types.hpp
 *
 *   Description:
 *    Thor Clock types
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_CLOCK_TYPES_HPP
#define THOR_CLOCK_TYPES_HPP

/* C++ Includes */
#include <cstdint>

namespace Thor::Clock
{
  /**
   *  Describes the fundamental clock sources which can be used as inputs to
   *  generate other internal system clocks.
   *
   *  @note Not every device has all these options. This is a master list.
   */
  enum class Source : uint8_t
  {
    LSI,  /**< Low speed internal clock */
    LSE,  /**< Low speed external clock */
    HSI,  /**< High speed external clock */
    HSE,  /**< High speed internal clock */
    CSI,  /**< ? */
    PLL,  /**< Phase locked loop clock */

    NUM_CLOCK_SOURCES
  };

  /**
   *  Various types of internal clock busses supported by various STM32 devices.
   *
   *  @note Not every device has all these options. This is a master list.
   */
  enum class Bus : uint8_t
  {
    APB1_PERIPH,
    APB2_PERIPH,
    APB1_TIMER,
    APB2_TIMER
  };
}
#endif /* !THOR_CLOCK_TYPES_HPP */