/******************************************************************************
 *  File Name:
 *    des_intf.hpp
 *
 *  Description:
 *    Types for the Device Electronic Signature module
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_DES_INTERFACE_TYPES_HPP
#define THOR_LLD_DES_INTERFACE_TYPES_HPP

/* STL Includes */
#include <array>

namespace Thor::LLD::DES
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  /**
   *  Unique identifiers for STM32 chips are 96 bits each
   */
  using UniqueID = std::array<uint8_t, 12>;
}    // namespace Thor::LLD::DES

#endif /* !THOR_LLD_DES_INTERFACE_TYPES_HPP */
