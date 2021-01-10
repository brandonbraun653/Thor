/********************************************************************************
 *  File Name:
 *    can_types.hpp
 *
 *  Description:
 *    Thor CAN types
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HLD_CAN_TYPES_HPP
#define THOR_HLD_CAN_TYPES_HPP

/* C++ Includes */
#include <memory>

namespace Thor::CAN
{
  /*-------------------------------------------------------------------------------
  Forward Declarations
  -------------------------------------------------------------------------------*/
  class Driver;


  /*-------------------------------------------------------------------------------
  Aliases
  -------------------------------------------------------------------------------*/
  using Driver_rPtr = Driver *;
  /*-------------------------------------------------------------------------------
  Enumerations
  -------------------------------------------------------------------------------*/
  enum class FilterType : uint8_t
  {
    MODE_16BIT_LIST,
    MODE_16BIT_MASK,
    MODE_32BIT_LIST,
    MODE_32BIT_MASK,

    NUM_OPTIONS,
    UNKNOWN
  };

}

#endif /* !THOR_HLD_CAN_TYPES_HPP */
