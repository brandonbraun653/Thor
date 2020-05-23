/********************************************************************************
 *  File Name:
 *    usart_types.hpp
 *
 *  Description:
 *    Common USART types used in Thor Drivers
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_DRIVER_USART_COMMON_TYPES_HPP
#define THOR_DRIVER_USART_COMMON_TYPES_HPP

/* STL Includes */
#include <cstdint>
#include <memory>

/* Chimera Includes */
#include <Chimera/common>

namespace Thor::LLD::USART
{
  struct RegisterMap;

  class Driver;
  using Driver_sPtr = std::shared_ptr<Driver>;
  using Driver_uPtr = std::unique_ptr<Driver>;
}    // namespace Thor::LLD::USART

#endif /* !THOR_DRIVER_USART_COMMON_TYPES_HPP */
