/********************************************************************************
 *  File Name:
 *    types.hpp
 *
 *  Description:
 *    Common types used across the Thor HLD
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_HLD_COMMON_TYPES_HPP
#define THOR_HLD_COMMON_TYPES_HPP

/* Chimera Includes */
#include <Chimera/container>

/* Type Safe Includes */
#include <type_safe/strong_typedef.hpp>

namespace Thor::HLD
{
  using RIndex_t = uint8_t;
  using RIndexMap  = Chimera::Container::LightFlatMap<std::uintptr_t, RIndex_t>;
  struct RIndex : type_safe::strong_typedef<RIndex, RIndex_t>,
                  type_safe::strong_typedef_op::equality_comparison<RIndex>,
                  type_safe::strong_typedef_op::relational_comparison<RIndex>
  {
    using strong_typedef::strong_typedef;

    /**
     *  Gets the underlying value back in the orginal type
     *
     *  @return size_t
     */
    constexpr RIndex_t value() const
    {
      return type_safe::get( *this );
    }

    /**
     *  Implicit conversion when assinging to a variable of the underlying type
     *
     *  @return size_t
     */
    constexpr operator RIndex_t() const
    {
      return value();
    }
  };
}    // namespace Thor::HLD

#endif /* !THOR_HLD_COMMON_TYPES_HPP */
