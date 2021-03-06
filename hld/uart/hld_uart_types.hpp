/********************************************************************************
 *   File Name:
 *    uart_types.hpp
 *
 *   Description:
 *    Types associated with the Thor UART driver
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_UART_TYPES_HPP
#define THOR_UART_TYPES_HPP

/* C++ Includes */
#include <memory>

namespace Thor::UART
{
  /*-------------------------------------------------------------------------------
  Forward Declarations
  -------------------------------------------------------------------------------*/
  class Driver;


  /*-------------------------------------------------------------------------------
  Aliases
  -------------------------------------------------------------------------------*/
  using Driver_rPtr = Driver *;
}    // namespace Thor::UART

#endif /* !THOR_UART_TYPES_HPP */
