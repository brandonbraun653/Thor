/********************************************************************************
 *  File Name:
 *    types.hpp
 *
 *  Description:
 *    Common types used across the Thor LLD
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_COMMON_TYPES_HPP
#define THOR_LLD_COMMON_TYPES_HPP

/* STL Includes */
#include <limits>

/* Chimera Includes */
#include <Chimera/container>

/* Type Safe Includes */
//#include <type_safe/strong_typedef.hpp>

namespace Thor::LLD
{
  /*-------------------------------------------------------------------------------
  Aliases
  -------------------------------------------------------------------------------*/
  using RIndex_t = uint8_t;

  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  static constexpr uint8_t INVALID_RESOURCE_INDEX = std::numeric_limits<uint8_t>::max();

}    // namespace Thor::LLD

#endif /* !THOR_LLD_COMMON_TYPES_HPP */
