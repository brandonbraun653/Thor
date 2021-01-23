/********************************************************************************
 *  File Name:
 *    hw_power_types.hpp
 *
 *  Description:
 *    Declares types specific to the PWR peripheral
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_POWER_TYPES_HPP
#define THOR_HW_POWER_TYPES_HPP

/* C++ Includes */
#include <cstdint>

/* Driver Includes */
#include <Thor/lld/common/registers/field_accessor.hpp>
#include <Thor/lld/stm32f4x/power/hw_power_prj.hpp>

namespace Thor::LLD::PWR
{
  /*-------------------------------------------------------------------------------
  Enumerations
  -------------------------------------------------------------------------------*/
  enum class VoltageScale : uint8_t
  {
    SCALE_1,
    SCALE_2,
    SCALE_3,

    NUM_OPTIONS,
    INVALID
  };

  /*-------------------------------------------------------------------------------
  Structures
  -------------------------------------------------------------------------------*/
  struct RegisterMap
  {
    volatile uint32_t CR;  /**< PWR control register,         Address offset: 0x00 */
    volatile uint32_t CSR; /**< PWR control status register,  Address offset: 0x04 */
  };

  /*------------------------------------------------
  Control Register
  ------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CR, CR_ODSWEN_Msk, ODSWEN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR, CR_ODEN_Msk, ODEN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR, CR_VOS_Msk, VOS, BIT_ACCESS_RW );

  /*-------------------------------------------------
  Control Status Register
  -------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CSR, CSR_ODSWRDY_Msk, ODSWRDY, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, CSR, CSR_ODRDY_Msk, ODRDY, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, CSR, CSR_VOSRDY_Msk, VOSRDY, BIT_ACCESS_R );

}    // namespace Thor::LLD::PWR

#endif /* !THOR_HW_POWER_TYPES_HPP */