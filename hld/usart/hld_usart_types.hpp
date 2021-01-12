/********************************************************************************
 *  File Name:
 *    usart_types.hpp
 *
 *  Description:
 *    Types associated with the Thor USART driver
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_USART_TYPES_HPP
#define THOR_USART_TYPES_HPP

/* C++ Includes */
#include <memory>

namespace Thor::USART
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

#endif /* !THOR_USART_TYPES_HPP */
