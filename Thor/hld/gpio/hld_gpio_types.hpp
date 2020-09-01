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
#ifndef THOR_HLD_GPIO_TYPES_HPP
#define THOR_HLD_GPIO_TYPES_HPP

/* C++ Includes */
#include <memory>

namespace Thor::GPIO
{
  /*-------------------------------------------------------------------------------
  Forward Declarations
  -------------------------------------------------------------------------------*/
  class Driver;


  /*-------------------------------------------------------------------------------
  Aliases
  -------------------------------------------------------------------------------*/
  using Driver_rPtr = Driver *;
  using Driver_sPtr = std::shared_ptr<Driver>;
}    // namespace Thor::GPIO

#endif /* !THOR_HLD_GPIO_TYPES_HPP */
