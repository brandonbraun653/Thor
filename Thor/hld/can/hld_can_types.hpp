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
  using Driver_sPtr = std::shared_ptr<Driver>;
}

#endif /* !THOR_HLD_CAN_TYPES_HPP */
