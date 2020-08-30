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
#include <type_safe/strong_typedef.hpp>

namespace Thor::LLD
{
  /*-------------------------------------------------------------------------------
  Aliases
  -------------------------------------------------------------------------------*/
  using RIndexType = uint8_t;
  using RIndexMap  = Chimera::Container::LightFlatMap<std::uintptr_t, RIndexType>;

  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  static constexpr uint8_t INVALID_RESOURCE_INDEX = std::numeric_limits<uint8_t>::max();

  /*-------------------------------------------------------------------------------
  Structures
  -------------------------------------------------------------------------------*/
  /**
   *  Index type used to access various low level driver resources.
   */
  struct RIndex : type_safe::strong_typedef<RIndex, RIndexType>,
                  type_safe::strong_typedef_op::equality_comparison<RIndex>,
                  type_safe::strong_typedef_op::relational_comparison<RIndex>
  {
    using strong_typedef::strong_typedef;

    /**
     *  Gets the underlying value back in the orginal type
     *
     *  @return size_t
     */
    constexpr RIndexType value() const
    {
      return type_safe::get( *this );
    }

    /**
     *  Implicit conversion when assinging to a variable of the underlying type
     *
     *  @return size_t
     */
    constexpr operator RIndexType() const
    {
      return value();
    }
  };
}    // namespace Thor::LLD

#endif /* !THOR_LLD_COMMON_TYPES_HPP */
