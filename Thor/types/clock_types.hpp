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
#ifndef THOR_CLK_TYPES_HPP
#define THOR_CLK_TYPES_HPP

/* C++ Includes */
#include <cstdint>

namespace Thor::CLK
{
  enum class ClockBus : uint8_t
  {
    APB1_PERIPH,
    APB2_PERIPH,
    APB1_TIMER,
    APB2_TIMER
  };
}
#endif /* !THOR_CLK_TYPES_HPP */