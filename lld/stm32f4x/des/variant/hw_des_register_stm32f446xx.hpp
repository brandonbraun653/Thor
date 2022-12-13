/******************************************************************************
 *  File Name:
 *    hw_des_register_stm32f446xx.hpp
 *
 *  Description:
 *    Register definitions for the DES hardware on the STM32F446xx chips
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

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
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr Reg32_t UDIDR_BASE_ADDRESS = 0x1FFF7A10;
  static constexpr Reg32_t FSDR_BASE_ADDRESS  = 0x1FFF7A22;
  static constexpr Reg32_t PDR_BASE_ADDRESS   = 0x1FFF7BF0;

  /********************  Bit definition for UDID register  ********************/
  static constexpr Reg32_t UDIDR_Msk = 0xFFFFFFFF;

  /********************  Bit definition for FSD register  *********************/
  static constexpr Reg32_t FSDR_Msk = 0x0000FFFF;

  /********************  Bit definition for PD register  **********************/
  static constexpr Reg32_t PDR_Msk = 0x000000700;
  static constexpr Reg32_t PDR_Pos = 8;

  static constexpr Reg32_t PDR_LQFP_UFBGA_144 = 0x011 << PDR_Pos;
  static constexpr Reg32_t PDR_WLCSP81        = 0x010 << PDR_Pos;
  static constexpr Reg32_t PDR_LQFP100        = 0x001 << PDR_Pos;
  static constexpr Reg32_t PDR_LQFP64         = 0x000 << PDR_Pos;

}    // namespace Thor::LLD::DES

#endif /* !THOR_LLD_DES_REGISTER_STM32F432_HPP */
