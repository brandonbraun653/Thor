/******************************************************************************
 *  File Name:
 *    hw_power_types.hpp
 *
 *  Description:
 *    STM32L4 Types for the POWER Peripheral
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_HW_POWER_TYPES_HPP
#define THOR_HW_POWER_TYPES_HPP

/* C++ Includes */
#include <cstdint>
#include <array>

/* Driver Includes */
#include <Thor/lld/common/registers/field_accessor.hpp>
#include <Thor/lld/stm32l4x/power/hw_power_prj.hpp>

namespace Thor::LLD::PWR
{
  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/
  enum class VoltageScale : uint8_t
  {
    SCALE_1,  /**< High power/freq */
    SCALE_2,  /**< Med power/freq */
    SCALE_3,  /**< Low power/freq */

    NUM_OPTIONS,
    INVALID
  };


  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct RegisterMap
  {
    volatile Reg32_t CR1;   /*!< PWR power control register 1,        Address offset: 0x00 */
    volatile Reg32_t CR2;   /*!< PWR power control register 2,        Address offset: 0x04 */
    volatile Reg32_t CR3;   /*!< PWR power control register 3,        Address offset: 0x08 */
    volatile Reg32_t CR4;   /*!< PWR power control register 4,        Address offset: 0x0C */
    volatile Reg32_t SR1;   /*!< PWR power status register 1,         Address offset: 0x10 */
    volatile Reg32_t SR2;   /*!< PWR power status register 2,         Address offset: 0x14 */
    volatile Reg32_t SCR;   /*!< PWR power status reset register,     Address offset: 0x18 */
    Reg32_t RESERVED;       /*!< Reserved,                            Address offset: 0x1C */
    volatile Reg32_t PUCRA; /*!< Pull_up control register of portA,   Address offset: 0x20 */
    volatile Reg32_t PDCRA; /*!< Pull_Down control register of portA, Address offset: 0x24 */
    volatile Reg32_t PUCRB; /*!< Pull_up control register of portB,   Address offset: 0x28 */
    volatile Reg32_t PDCRB; /*!< Pull_Down control register of portB, Address offset: 0x2C */
    volatile Reg32_t PUCRC; /*!< Pull_up control register of portC,   Address offset: 0x30 */
    volatile Reg32_t PDCRC; /*!< Pull_Down control register of portC, Address offset: 0x34 */
    volatile Reg32_t PUCRD; /*!< Pull_up control register of portD,   Address offset: 0x38 */
    volatile Reg32_t PDCRD; /*!< Pull_Down control register of portD, Address offset: 0x3C */
    volatile Reg32_t PUCRE; /*!< Pull_up control register of portE,   Address offset: 0x40 */
    volatile Reg32_t PDCRE; /*!< Pull_Down control register of portE, Address offset: 0x44 */
    Reg32_t RESERVED1;      /*!< Reserved,                            Address offset: 0x48 */
    Reg32_t RESERVED2;      /*!< Reserved,                            Address offset: 0x4C */
    Reg32_t RESERVED3;      /*!< Reserved,                            Address offset: 0x50 */
    Reg32_t RESERVED4;      /*!< Reserved,                            Address offset: 0x54 */
    volatile Reg32_t PUCRH; /*!< Pull_up control register of portH,   Address offset: 0x58 */
    volatile Reg32_t PDCRH; /*!< Pull_Down control register of portH, Address offset: 0x5C */
  };

  using PeriphRegisterList = std::array<RegisterMap *, 1>;

  /*------------------------------------------------
  Control Register 1
  ------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CR1, CR1_VOS_Msk, VOS, BIT_ACCESS_RW );


  namespace CR1
  {
    static constexpr Reg32_t VOS_SCALE_1 = CR1_VOS_0;
    static constexpr Reg32_t VOS_SCALE_2 = CR1_VOS_1;
  }

}    // namespace Thor::LLD::POWER

#endif /* !THOR_HW_POWER_TYPES_HPP*/
