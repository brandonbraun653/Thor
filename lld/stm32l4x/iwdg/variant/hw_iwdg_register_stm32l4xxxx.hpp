/******************************************************************************
 *  File Name:
 *    hw_iwdg_register_stm32l4xxxx.hpp
 *
 *  Description:
 *    IWDG register definitions for the STM32L4xxxx series chips.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_HW_IWDG_REGISTER_STM32L4XXXX_HPP
#define THOR_HW_IWDG_REGISTER_STM32L4XXXX_HPP

/* C++ Includes */
#include <cstdint>

/* Chimera Includes */
#include <Chimera/common>

/* Driver Includes */
#include <Thor/lld/stm32l4x/system/variant/sys_memory_map_stm32l432kc.hpp>


namespace Thor::LLD::IWDG
{
  /*-------------------------------------------------
  Peripheral Instance Memory Map Base
  -------------------------------------------------*/
  static constexpr uint32_t IWDG1_BASE_ADDR      = Thor::System::MemoryMap::IWDG1_PERIPH_START_ADDRESS;
  static constexpr uint32_t PERIPH_CLOCK_FREQ_HZ = 32000u;
  static constexpr uint32_t COUNTER_MIN          = 0;
  static constexpr uint32_t COUNTER_MAX          = 0xFFF;

  /*-------------------------------------------------
  Peripheral Register Definitions
  -------------------------------------------------*/
  /*******************  Bit definition for KR register  ********************/
  static constexpr uint32_t KR_KEY_Pos = ( 0U );
  static constexpr uint32_t KR_KEY_Msk = ( 0xFFFFUL << KR_KEY_Pos );
  static constexpr uint32_t KR_KEY     = KR_KEY_Msk;

  static constexpr Reg32_t KR_REFRESH = 0xAAAA;
  static constexpr Reg32_t KR_START   = 0xCCCC;
  static constexpr Reg32_t KR_UNLOCK  = 0x5555;
  static constexpr Reg32_t KR_LOCK    = 0x0000;

  /*******************  Bit definition for PR register  ********************/
  static constexpr uint32_t PR_PR_Pos = ( 0U );
  static constexpr uint32_t PR_PR_Msk = ( 0x7UL << PR_PR_Pos );
  static constexpr uint32_t PR_PR     = PR_PR_Msk;
  static constexpr uint32_t PR_PR_0   = ( 0x1UL << PR_PR_Pos );
  static constexpr uint32_t PR_PR_1   = ( 0x2UL << PR_PR_Pos );
  static constexpr uint32_t PR_PR_2   = ( 0x4UL << PR_PR_Pos );

  static constexpr Reg32_t PR_PRESCALE_4   = ( 0 << PR_PR_Pos ) & PR_PR_Msk;
  static constexpr Reg32_t PR_PRESCALE_8   = ( 1 << PR_PR_Pos ) & PR_PR_Msk;
  static constexpr Reg32_t PR_PRESCALE_16  = ( 2 << PR_PR_Pos ) & PR_PR_Msk;
  static constexpr Reg32_t PR_PRESCALE_32  = ( 3 << PR_PR_Pos ) & PR_PR_Msk;
  static constexpr Reg32_t PR_PRESCALE_64  = ( 4 << PR_PR_Pos ) & PR_PR_Msk;
  static constexpr Reg32_t PR_PRESCALE_128 = ( 5 << PR_PR_Pos ) & PR_PR_Msk;
  static constexpr Reg32_t PR_PRESCALE_256 = ( 6 << PR_PR_Pos ) & PR_PR_Msk;
  static constexpr Reg32_t PR_MIN_PRESCALE = PR_PRESCALE_4;
  static constexpr Reg32_t PR_MAX_PRESCALE = PR_PRESCALE_256;

  static const uint32_t NumPrescalers                       = 7;
  static const uint32_t DecimalPrescalers[ NumPrescalers ]  = { 4, 8, 16, 32, 64, 128, 256 };
  static const Reg32_t RegisterPrescalers[ NumPrescalers ] = { PR_PRESCALE_4,  PR_PRESCALE_8,   PR_PRESCALE_16, PR_PRESCALE_32,
                                                               PR_PRESCALE_64, PR_PRESCALE_128, PR_PRESCALE_256 };

  /*******************  Bit definition for RLR register  *******************/
  static constexpr uint32_t RLR_RL_Pos = ( 0U );
  static constexpr uint32_t RLR_RL_Msk = ( 0xFFFUL << RLR_RL_Pos );
  static constexpr uint32_t RLR_RL     = RLR_RL_Msk;
  static constexpr Reg32_t RLR_MAX     = 0x0FFF;
  static constexpr Reg32_t RLR_MIN     = 0x0000;

  /*******************  Bit definition for SR register  ********************/
  static constexpr uint32_t SR_PVU_Pos = ( 0U );
  static constexpr uint32_t SR_PVU_Msk = ( 0x1UL << SR_PVU_Pos );
  static constexpr uint32_t SR_PVU     = SR_PVU_Msk;
  static constexpr uint32_t SR_RVU_Pos = ( 1U );
  static constexpr uint32_t SR_RVU_Msk = ( 0x1UL << SR_RVU_Pos );
  static constexpr uint32_t SR_RVU     = SR_RVU_Msk;
  static constexpr uint32_t SR_WVU_Pos = ( 2U );
  static constexpr uint32_t SR_WVU_Msk = ( 0x1UL << SR_WVU_Pos );
  static constexpr uint32_t SR_WVU     = SR_WVU_Msk;

  /*******************  Bit definition for KR register  ********************/
  static constexpr uint32_t WINR_WIN_Pos = ( 0U );
  static constexpr uint32_t WINR_WIN_Msk = ( 0xFFFUL << WINR_WIN_Pos );
  static constexpr uint32_t WINR_WIN     = WINR_WIN_Msk;

}    // namespace Thor::LLD::IWDG

#endif /* !THOR_HW_IWDG_REGISTER_STM32L4XXXX_HPP */
