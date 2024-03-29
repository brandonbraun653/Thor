/******************************************************************************
 *  File Name:
 *    field_accessor.hpp
 *
 *  Description:
 *    Helper macros/functions/classes for generating register level field access
 *    data structures in processor specific definition files.
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_REGISTER_FIELD_ACCESSOR_HPP
#define THOR_LLD_REGISTER_FIELD_ACCESSOR_HPP

/* STL Includes */
#include <cstddef>

/* Chimera Includes */
#include <Chimera/common>

namespace Thor::LLD
{
  /*------------------------------------------------
  Bit Access Levels: RM0394 Section 1.2
  ------------------------------------------------*/
  /**
   *  Software can only read this bit
   */
  static constexpr size_t BIT_ACCESS_R  = ( 1u << 0 );
  static constexpr size_t BIT_ACCESS_RO = BIT_ACCESS_R;

  /**
   *  Software can only write this bit
   */
  static constexpr size_t BIT_ACCESS_W = ( 1u << 1 );
  static constexpr size_t BIT_ACCESS_WO = BIT_ACCESS_W;

  /**
   *  Software can read and write this bit
   */
  static constexpr size_t BIT_ACCESS_RW = ( 1u << 2 );

  /**
   *  Software can read as well as clear this bit by writing 0. Writing 1 has no
   *  effect on the bit value.
   */
  static constexpr size_t BIT_ACCESS_RCW0 = ( 1u << 3 );

  /**
   *  Software can read as well as clear this bit by writing 1. Writing 0 has no
   *  effect on the bit value.
   */
  static constexpr size_t BIT_ACCESS_RCW1 = ( 1u << 4 );

  /**
   *  Software can read as well as clear this bit by writing to the register. The
   *  value written to this bit is not important.
   */
  static constexpr size_t BIT_ACCESS_RCW = ( 1u << 5 );

  /**
   *  Software can read this bit. Reading this bit automatically clears it to 0.
   *  Writing this bit has no effect on the bit value.
   */
  static constexpr size_t BIT_ACCESS_RCR = ( 1u << 6 );

  /**
   *  Software can read this bit. Reading this bit automatically sets it to 1.
   *  Writing this bit has no effect on the bit value.
   */
  static constexpr size_t BIT_ACCESS_RSR = ( 1u << 7 );

  /**
   *  Software can read as well as set this bit. Writing 0 has no effect on the
   *  bit value.
   */
  static constexpr size_t BIT_ACCESS_RS = ( 1u << 8 );

  /**
   *  Software can only write once to this bit and can also read it at any time.
   *  Only a reset can return the bit to its reset value.
   */
  static constexpr size_t BIT_ACCESS_RW_ONCE = ( 1u << 9 );

  /**
   *  The software can toggle this bit by writing 1. Writing 0 has no effect.
   */
  static constexpr size_t BIT_ACCESS_T = ( 1u << 10 );

  /**
   *  Software can read this bit. Writing 1 triggers an event but has no effect on
   *  the bit value.
   */
  static constexpr size_t BIT_ACCESS_RTW1 = ( 1u << 11 );

  /**
   *  All bits that allow some kind of read functionality
   */
  static constexpr size_t BIT_ACCESS_ALL_READ =
      ( BIT_ACCESS_R | BIT_ACCESS_RO | BIT_ACCESS_RW | BIT_ACCESS_RCR | BIT_ACCESS_RSR | BIT_ACCESS_RCW0 | BIT_ACCESS_RCW1 |
        BIT_ACCESS_RS | BIT_ACCESS_RW_ONCE | BIT_ACCESS_RTW1 );

  /**
   *  All bits that allow some kind of write functionality
   */
  static constexpr size_t BIT_ACCESS_ALL_WRITE =
      ( BIT_ACCESS_W | BIT_ACCESS_RS | BIT_ACCESS_RW | BIT_ACCESS_RCW0 | BIT_ACCESS_RCW1 | BIT_ACCESS_RCW | BIT_ACCESS_RW_ONCE |
        BIT_ACCESS_T | BIT_ACCESS_RTW1 );
}    // namespace Thor::LLD

/**
 * Helper macro to declare a class that directly interacts with bits inside
 * a hardware peripheral register. This is to help clean up some of the code
 * readability as well as prevent read/write access mistakes.
 *
 *
 * @param MEM_MAP_TYPE    Memory mapped struct type that defines the peripheral
 * @param REGISTER        The actual register name to be interacted with
 * @param MASK            Bit mask to ensure the proper bits are accessed
 * @param NAME            User friendly name of the class
 * @param ACCESS          Access level of the bits (r/w/rw)
 */
#define REG_ACCESSOR( MEM_MAP_TYPE, REGISTER, MASK, NAME, ACCESS )             \
  class NAME                                                                   \
  {                                                                            \
  public:                                                                      \
    static inline Reg32_t get( const MEM_MAP_TYPE *const periph )              \
    {                                                                          \
      using namespace Thor::LLD;                                               \
      if constexpr ( ACCESS == ( ( ACCESS )&BIT_ACCESS_ALL_READ ) )     \
      {                                                                        \
        const Reg32_t tmp = periph->REGISTER;                                  \
        return tmp & ( MASK );                                                 \
      }                                                                        \
      else                                                                     \
      {                                                                        \
        return 0;                                                              \
      }                                                                        \
    }                                                                          \
                                                                               \
    static inline void set( MEM_MAP_TYPE *const periph, const Reg32_t val )    \
    {                                                                          \
      if constexpr ( ACCESS == ( ( ACCESS )&BIT_ACCESS_ALL_WRITE ) )    \
      {                                                                        \
        Reg32_t tmp = periph->REGISTER;                                        \
        tmp &= ~( MASK );                                                      \
        tmp |= val & ( MASK );                                                 \
        periph->REGISTER = tmp;                                                \
      }                                                                        \
    }                                                                          \
                                                                               \
    static inline void setbit( MEM_MAP_TYPE *const periph, const Reg32_t val ) \
    {                                                                          \
      if constexpr ( ACCESS == ( ( ACCESS )&BIT_ACCESS_ALL_WRITE ) )    \
      {                                                                        \
        Reg32_t tmp = periph->REGISTER;                                        \
        tmp |= val & ( MASK );                                                 \
        periph->REGISTER = tmp;                                                \
      }                                                                        \
    }                                                                          \
                                                                               \
    static inline void clear( MEM_MAP_TYPE *const periph, const Reg32_t val )  \
    {                                                                          \
      if constexpr ( ACCESS == ( ( ACCESS )&BIT_ACCESS_ALL_WRITE ) )    \
      {                                                                        \
        Reg32_t tmp = periph->REGISTER;                                        \
        tmp &= ~( val & MASK );                                                \
        periph->REGISTER = tmp;                                                \
      }                                                                        \
    }                                                                          \
  };

/**
 *  Helper macro that repeatedly attempts to set a particular bit or set of bits
 *  in a register. Intented to be used with the class that is defined by the
 *  REG_ACCESSOR macro.
 *
 *  @param[in]  NAME      Name used to identify a register bit in REG_ACCESSOR
 *  @param[in]  PERIPH    The peripheral instance (memory mapped struct)
 *  @param[in]  BITS      Which bits should be set
 */
#define REG_FORCE_SET( NAME, PERIPH, BITS ) \
  do                                        \
  {                                         \
    NAME::set( PERIPH, BITS );              \
  } while ( ( NAME::get( PERIPH ) & BITS ) != BITS )

/**
 *  Helper macro that repeatedly attempts to clear a particular bit or set of bits
 *  in a register. Intented to be used with the class that is defined by the
 *  REG_ACCESSOR macro.
 *
 *  @param[in]  NAME      Name used to identify a register bit in REG_ACCESSOR
 *  @param[in]  PERIPH    The peripheral instance (memory mapped struct)
 *  @param[in]  BITS      Which bits should be cleared
 */
#define REG_FORCE_CLR( NAME, PERIPH, BITS ) \
  do                                        \
  {                                         \
    NAME::clear( PERIPH, BITS );            \
  } while ( NAME::get( PERIPH ) != 0 )

#endif /* !THOR_LLD_REGISTER_FIELD_ACCESSOR_HPP */
