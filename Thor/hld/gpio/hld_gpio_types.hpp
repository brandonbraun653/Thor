/********************************************************************************
 *  File Name:
 *    gpio_types.hpp
 *
 *  Description:
 *    Thor GPIO types
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_GPIO_TYPES_HPP
#define THOR_GPIO_TYPES_HPP

/* C++ Includes */
#include <cstdint>
#include <memory>

namespace Thor::GPIO
{
  class Driver;
  using Driver_sPtr = std::shared_ptr<Driver>;
  using Driver_uPtr = std::unique_ptr<Driver>;

  static constexpr size_t ACCESS_TIMEOUT = 10;  /**< Default timeout (mS) for accessing GPIO hardware */

}    // namespace Thor::GPIO

#endif /* !THOR_GPIO_TYPES_HPP */