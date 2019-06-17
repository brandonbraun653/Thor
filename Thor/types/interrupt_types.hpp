/********************************************************************************
 *   File Name:
 *    interrupt_types.hpp
 *
 *   Description:
 *    Thor Interrupt types
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_IT_TYPES_HPP
#define THOR_IT_TYPES_HPP

/* C++ Includes */
#include <cstdint>

/* Thor Includes */
#include <Thor/headers.hpp>

#if defined( THOR_STM32HAL_DRIVERS ) && ( THOR_STM32HAL_DRIVERS == 1 )
namespace Thor::Interrupt
{
  struct Initializer
  {
    IRQn_Type IRQn;
    uint32_t preemptPriority;
    uint32_t subPriority;
  };
}    // namespace Thor::Interrupt
#endif 

#endif /* !THOR_IT_TYPES_HPP */