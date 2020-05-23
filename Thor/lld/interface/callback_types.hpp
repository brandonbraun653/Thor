/********************************************************************************
 *   File Name:
 *    callback_types.hpp
 *
 *   Description:
 *    STM32 Callback Types
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_DRIVER_TYPES_CALLBACK_HPP
#define THOR_DRIVER_TYPES_CALLBACK_HPP

/* C++ Includes */
#include <cstdint>
#include <cstdlib>

/* Boost Includes */
#include <boost/function.hpp>

namespace Thor::Driver
{
  enum class CallbackEvent : uint8_t
  {
    TRANSMIT_COMPLETE,
    RECEIVE_COMPLETE,
    TRANSMIT_HALF_COMPLETE,
    RECEIVE_HALF_COMPLETE,
    ERROR
  };

  using VoidCallback = boost::function<void( void )>;
  using ConstVoidPtrCallback = boost::function<void( const void *const )>;

}    // namespace Thor::LLD::Serial


#endif /* !THOR_DRIVER_TYPES_CALLBACK_HPP */


