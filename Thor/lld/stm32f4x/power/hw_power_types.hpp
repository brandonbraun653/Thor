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
  struct RegisterMap
  {
    volatile uint32_t CR;  /**< PWR control register,         Address offset: 0x00 */
    volatile uint32_t CSR; /**< PWR control status register,  Address offset: 0x04 */
  };

  /*------------------------------------------------
  Control Register 1
  ------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CR, CR_VOS_Msk, VOS, BIT_ACCESS_RW );


  namespace CR1
  {
    static constexpr Reg32_t VOS_SCALE_1 = CR_VOS_0;
    static constexpr Reg32_t VOS_SCALE_2 = CR_VOS_1;
  }    // namespace CR1

}    // namespace Thor::LLD::PWR

#endif /* !THOR_HW_POWER_TYPES_HPP */