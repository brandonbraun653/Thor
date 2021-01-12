/********************************************************************************
 *  File Name:
 *    usb_types.hpp
 *
 *  Description:
 *    Common LLD USB types
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_LLD_USB_COMMON_TYPES_HPP
#define THOR_LLD_USB_COMMON_TYPES_HPP

/* C++ Includes */
#include <cstdint>

/* Chimera Includes */
#include <Chimera/common>

namespace Thor::LLD::USB
{
  /*-------------------------------------------------------------------------------
  Forward Declarations
  -------------------------------------------------------------------------------*/
  class Driver;
  struct RegisterMap;


  /*-------------------------------------------------------------------------------
  Aliases
  -------------------------------------------------------------------------------*/
  using Driver_rPtr = Driver *;


  /*-------------------------------------------------------------------------------
  Structures
  -------------------------------------------------------------------------------*/

}    // namespace Thor::LLD::USB

#endif /* !THOR_LLD_USB_COMMON_TYPES_HPP */
