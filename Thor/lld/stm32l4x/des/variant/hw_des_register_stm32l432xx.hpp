/********************************************************************************
 *  File Name:
 *    hw_des_register_stm32l432xx.hpp
 *
 *  Description:
 *    Register definitions for the DES hardware on the STM32L432 chips
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_DES_REGISTER_HPP
#define THOR_LLD_DES_REGISTER_HPP

/* Chimera Includes */
#include <Chimera/common>

#define STM32_UDID_AVAILABLE
#define STM32_FS_AVAILABLE
#define STM32_PD_AVAILABLE

namespace Thor::LLD::DES
{
  void initializeRegisters();

  static constexpr Reg32_t UDIDR_BASE_ADDRESS = 0x1FFF7590;
  static constexpr Reg32_t FSDR_BASE_ADDRESS  = 0x1FFF75E0;
  static constexpr Reg32_t PDR_BASE_ADDRESS   = 0x1FFF7500;

  /********************  Bit definition for UDID register  ********************/
  static constexpr Reg32_t UDIDR_Msk = 0xFFFFFFFF;

  /********************  Bit definition for FSD register  *********************/
  static constexpr Reg32_t FSDR_Msk = 0x0000FFFF;

  /********************  Bit definition for PD register  **********************/
  static constexpr Reg32_t PDR_Msk = 0x0000001F;

  static constexpr Reg32_t PDR_LQFP64   = 0x00;
  static constexpr Reg32_t PDR_WLCSP64  = 0x01;
  static constexpr Reg32_t PDR_LQFP100  = 0x02;
  static constexpr Reg32_t PDR_WLCSP36  = 0x05;
  static constexpr Reg32_t PDR_UFQFPN32 = 0x08;
  static constexpr Reg32_t PDR_LQFP32   = 0x09;
  static constexpr Reg32_t PDR_UFQFPN48 = 0x0A;
  static constexpr Reg32_t PDR_LQFP48   = 0x0B;
  static constexpr Reg32_t PDR_WLCSP49  = 0x0C;
  static constexpr Reg32_t PDR_UFBGA64  = 0x0D;
  static constexpr Reg32_t PDR_UFBGA100 = 0x0E;

}    // namespace Thor::LLD::DES

#endif /* !THOR_LLD_DES_REGISTER_STM32L432_HPP */
