/********************************************************************************
 *  File Name:
 *    hw_rcc_types.hpp
 *
 *  Description:
 *    RCC Type Declarations for STM32L4xxx
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_RCC_TYPES_HPP
#define THOR_LLD_RCC_TYPES_HPP

/* C++ Includes */
#include <cstdint>

/* Chimera/Includes */
#include <Chimera/container>
#include <Chimera/common>

/* Driver Includes */
#include <Thor/lld/common/registers/field_accessor.hpp>
#include <Thor/lld/interface/rcc/rcc_types.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_prj.hpp>

namespace Thor::LLD::RCC
{
  struct RegisterMap
  {
    volatile Reg32_t CR;                                                                                                    
    volatile Reg32_t ICSCR;                   
    volatile Reg32_t CFGR;
    volatile Reg32_t PLLCFGR;
    volatile Reg32_t PLLSAI1CFGR;
    Reg32_t RESERVED0;
    volatile Reg32_t CIER;
    volatile Reg32_t CIFR;
    volatile Reg32_t CICR;
    Reg32_t RESERVED1;
    volatile Reg32_t AHB1RSTR;
    volatile Reg32_t AHB2RSTR;
    volatile Reg32_t AHB3RSTR;
    Reg32_t RESERVED2;
    volatile Reg32_t APB1RSTR1;
    volatile Reg32_t APB1RSTR2;
    volatile Reg32_t APB2RSTR;
    Reg32_t RESERVED3;
    volatile Reg32_t AHB1ENR;
    volatile Reg32_t AHB2ENR;
    volatile Reg32_t AHB3ENR;
    Reg32_t RESERVED4;
    volatile Reg32_t APB1ENR1;
    volatile Reg32_t APB1ENR2;
    volatile Reg32_t APB2ENR;
    Reg32_t RESERVED5;
    volatile Reg32_t AHB1SMENR;
    volatile Reg32_t AHB2SMENR;
    volatile Reg32_t AHB3SMENR;
    Reg32_t RESERVED6;
    volatile Reg32_t APB1SMENR1;
    volatile Reg32_t APB1SMENR2;
    volatile Reg32_t APB2SMENR;
    Reg32_t RESERVED7;
    volatile Reg32_t CCIPR;
    Reg32_t RESERVED8;
    volatile Reg32_t BDCR;
    volatile Reg32_t CSR;
    volatile Reg32_t CRRCR;
    volatile Reg32_t CCIPR2;
  };

  /*------------------------------------------------
  Ensure that the memory alignment is correct
  ------------------------------------------------*/
  static_assert( offsetof( RegisterMap, CR ) == 0x00 );
  static_assert( offsetof( RegisterMap, ICSCR ) == 0x04 );
  static_assert( offsetof( RegisterMap, CFGR ) == 0x08 );
  static_assert( offsetof( RegisterMap, PLLCFGR ) == 0x0C );
  static_assert( offsetof( RegisterMap, PLLSAI1CFGR ) == 0x10 );
  static_assert( offsetof( RegisterMap, CIER ) == 0x18 );
  static_assert( offsetof( RegisterMap, CIFR ) == 0x1C );
  static_assert( offsetof( RegisterMap, CICR ) == 0x20 );
  static_assert( offsetof( RegisterMap, AHB1RSTR ) == 0x28 );
  static_assert( offsetof( RegisterMap, AHB2RSTR ) == 0x2C );
  static_assert( offsetof( RegisterMap, AHB3RSTR ) == 0x30 );
  static_assert( offsetof( RegisterMap, APB1RSTR1 ) == 0x38 );
  static_assert( offsetof( RegisterMap, APB1RSTR2 ) == 0x3C );
  static_assert( offsetof( RegisterMap, APB2RSTR ) == 0x40 );
  static_assert( offsetof( RegisterMap, AHB1ENR ) == 0x48 );
  static_assert( offsetof( RegisterMap, AHB2ENR ) == 0x4C );
  static_assert( offsetof( RegisterMap, AHB3ENR ) == 0x50 );
  static_assert( offsetof( RegisterMap, APB1ENR1 ) == 0x58 );
  static_assert( offsetof( RegisterMap, APB1ENR2 ) == 0x5C );
  static_assert( offsetof( RegisterMap, APB2ENR ) == 0x60 );
  static_assert( offsetof( RegisterMap, AHB1SMENR ) == 0x68 );
  static_assert( offsetof( RegisterMap, AHB2SMENR ) == 0x6C );
  static_assert( offsetof( RegisterMap, AHB3SMENR ) == 0x70 );
  static_assert( offsetof( RegisterMap, APB1SMENR1 ) == 0x78 );
  static_assert( offsetof( RegisterMap, APB1SMENR2 ) == 0x7C );
  static_assert( offsetof( RegisterMap, APB2SMENR ) == 0x80 );
  static_assert( offsetof( RegisterMap, CCIPR ) == 0x88 );
  static_assert( offsetof( RegisterMap, BDCR ) == 0x90 );
  static_assert( offsetof( RegisterMap, CSR ) == 0x94 );
  static_assert( offsetof( RegisterMap, CRRCR ) == 0x98 );
  static_assert( offsetof( RegisterMap, CCIPR2 ) == 0x9C );

  using PeriphRegisterList = std::array<RegisterMap *, NUM_RCC_PERIPHS>;

  
  /**
   *  Configuration struct for the clock enable register
   */
  struct RegisterConfig
  {
    volatile Reg32_t *reg; /**< Clock enable register */
    Reg32_t mask;          /**< Bit mask that will enable/disable the peripheral's clock */
  };

  /**
   *  Peripheral Control & Config (PCC)
   *  Describes a generic set of registers and configurations for a
   *  peripheral type that allows the RCC driver to generically configure
   *  a large number of peripherals by referencing these lookup tables.
   *
   *  @note All pointers here reference a lookup
   */
  struct PCC
  {
    const RegisterConfig *clock;    /**< Standard clock configuration registers */
    const RegisterConfig *clockLP;  /**< Low power clock configuration registers */
    const RegisterConfig *reset;    /**< Peripheral reset registers */
    const ClockType_t *clockSource; /**< Which system clock is used on the peripheral */
    Chimera::Container::LightFlatMap<std::uintptr_t, size_t>
        *resourceIndexMap; /**< Converts a peripheral address into a resource index */
    size_t elements;       /**< Number of elements in the tables */
  };

  /*------------------------------------------------
  Configuration Options
  ------------------------------------------------*/
  namespace Configuration
  {
    /**
     *  High level structure describing what kinds of clocks are available
     *  to be used as a source for the System Clock.
     */
    using OscillatorType_t = Reg32_t;
    namespace OscillatorType
    {
      static constexpr OscillatorType_t HSE     = 1u;
      static constexpr OscillatorType_t HSI     = 2u;
      static constexpr OscillatorType_t PLLCLK  = 4u;
      static constexpr OscillatorType_t PLLRCLK = 8u;
    }    // namespace OscillatorType

    /**
     *  High level structure describing what kinds of clocks are available
     *  to be configured by the code.
     */
    namespace ClockType
    {
      static constexpr ClockType_t SYSCLK = 1u;
      static constexpr ClockType_t HCLK   = 2u;
      static constexpr ClockType_t PCLK1  = 4u;
      static constexpr ClockType_t PCLK2  = 8u;
    }    // namespace ClockType
  }      // namespace Configuration

  /*------------------------------------------------
  Clock Control Register
  ------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CR, CR_MSION_Msk, MSION, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR, CR_MSIRDY_Msk, MSIRDY, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, CR, CR_MSIPLLEN_Msk, MSIPLLEN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR, CR_MSIRGSEL_Msk, MSIRGSEL, BIT_ACCESS_RS );


  /*------------------------------------------------
  Control Status Register
  ------------------------------------------------*/
  enum ResetFlags : Reg32_t
  {
    CLEARED     = 0,
    LOW_POWER   = CSR_LPWRRSTF,
    WWDG        = CSR_WWDGRSTF,
    IWDG        = CSR_IWDGRSTF,
    SOFTWARE    = CSR_SFTRSTF,
    BROWN_OUT   = CSR_BORRSTF,
    PIN_RESET   = CSR_PINRSTF,
    OPTION_BYTE = CSR_OBLRSTF,
    FIREWALL    = CSR_FWRSTF
  };

  REG_ACCESSOR( RegisterMap, CSR, CSR_LPWRRSTF, LPWRRSTF, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, CSR, CSR_WWDGRSTF, WWDGRSTF, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, CSR, CSR_IWDGRSTF, IWDGRSTF, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, CSR, CSR_SFTRSTF, SFTRSTF, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, CSR, CSR_BORRSTF, BORRSTF, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, CSR, CSR_PINRSTF, PINRSTF, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, CSR, CSR_OBLRSTF, OBLRSTF, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, CSR, CSR_FWRSTF, FWRSTF, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, CSR, CSR_RMVF, RMVFRSTF, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CSR, CSR_MSISRANGE, MSIRANGE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CSR, CSR_LSIRDY, LSIRDY, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, CSR, CSR_LSION, LSION, BIT_ACCESS_RW );


}    // namespace Thor::LLD::RCC

#endif  /* !THOR_LLD_RCC_TYPES_HPP */
