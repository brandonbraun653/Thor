/********************************************************************************
 *  File Name:
 *    field_accessor.hpp
 *
 *  Description:
 *    Helper macros/functions/classes for generating register level field access
 *    data structures in processor specific definition files.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_REGISTER_FIELD_ACCESSOR_HPP
#define THOR_LLD_REGISTER_FIELD_ACCESSOR_HPP

/* STL Includes */
#include <cstddef>

namespace Thor::LLD
{
  static constexpr size_t REG_READ  = ( 1u << 0 );
  static constexpr size_t REG_WRITE = ( 1u << 1 );
  static constexpr size_t REG_RW    = REG_READ | REG_WRITE;
}    // namespace Thor::LLD

#define REG_ACCESSOR( NAME, FIELD, MASK, ACCESS, MMAP )             \
  class NAME                                                        \
  {                                                                 \
  public:                                                           \
    static inline Reg32_t get( const MMAP *const periph )           \
    {                                                               \
      if constexpr ( ( ACCESS )&Thor::LLD::REG_READ )               \
      {                                                             \
        return periph->FIELD & ( MASK );                            \
      }                                                             \
      else                                                          \
      {                                                             \
        return 0;                                                   \
      }                                                             \
    }                                                               \
                                                                    \
    static inline void set( MMAP *const periph, const Reg32_t val ) \
    {                                                               \
      if constexpr ( ( ACCESS )&Thor::LLD::REG_WRITE )              \
      {                                                             \
        Reg32_t tmp = periph->FIELD;                                \
        tmp &= ~( MASK );                                           \
        tmp |= val & ( MASK );                                      \
        periph->FIELD = tmp;                                        \
      }                                                             \
    }                                                               \
  };


#endif /* !THOR_LLD_REGISTER_FIELD_ACCESSOR_HPP */
