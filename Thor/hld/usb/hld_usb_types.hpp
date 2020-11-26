/********************************************************************************
 *  File Name:
 *    hld_usb_types.hpp
 *
 *  Description:
 *    Thor USB types
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HLD_USB_TYPES_HPP
#define THOR_HLD_USB_TYPES_HPP

/* C++ Includes */
#include <memory>

namespace Thor::USB
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
}    // namespace Thor::USB

#endif /* !THOR_HLD_USB_TYPES_HPP */
