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
  class USARTClass;
  using USARTClass_sPtr = std::shared_ptr<USARTClass>;
  using USARTClass_uPtr = std::unique_ptr<USARTClass>;
}    // namespace Thor::UART

#endif /* !THOR_USART_TYPES_HPP */