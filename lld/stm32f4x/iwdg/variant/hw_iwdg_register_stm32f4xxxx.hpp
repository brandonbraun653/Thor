/******************************************************************************
 *  File Name:
 *    hw_iwdg_register_stm32f4xxxx.hpp
 *
 *  Description:
 *    Explicit hardware register definitions for the STM32F446xx Watchdog
 *    peripherals.
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#ifndef THOR_HW_IWDG_REGISTER_HPP
#define THOR_HW_IWDG_REGISTER_HPP

/* C++ Includes */
#include <array>
#include <cstdint>

/* Chimera Includes */
#include <Chimera/common>

/* Driver Includes */
#include <Thor/lld/stm32f4x/system/variant/sys_memory_map_stm32f446xx.hpp>


namespace Thor::LLD::IWDG
{
  static constexpr Reg32_t IWDG1_BASE_ADDR      = Thor::System::MemoryMap::IWDG_PERIPH_START_ADDRESS;
  static constexpr Reg32_t PERIPH_CLOCK_FREQ_HZ = 32000u;
  static constexpr uint32_t COUNTER_MIN         = 0;
  static constexpr uint32_t COUNTER_MAX         = 0xFFF;

  /*-------------------------------------------------
  IWDG Key Register
  -------------------------------------------------*/
  static constexpr Reg32_t KR_Msk     = 0xFFFF;
  static constexpr Reg32_t KR_Rst     = 0x00;
  static constexpr Reg32_t KR_KEY_Pos = ( 0U );
  static constexpr Reg32_t KR_KEY_Msk = ( 0xFFFFU << KR_KEY_Pos );
  static constexpr Reg32_t KR_KEY     = KR_KEY_Msk;

  static constexpr Reg32_t KR_REFRESH = 0xAAAA;
  static constexpr Reg32_t KR_START   = 0xCCCC;
  static constexpr Reg32_t KR_UNLOCK  = 0x5555;
  static constexpr Reg32_t KR_LOCK    = 0x0000;

  /*-------------------------------------------------
  IWDG Pre-scale Register
  -------------------------------------------------*/
  static constexpr Reg32_t PR_Msk    = 0x07;
  static constexpr Reg32_t PR_Rst    = 0x00;
  static constexpr Reg32_t PR_PR_Pos = ( 0U );
  static constexpr Reg32_t PR_PR_Msk = ( 0x7U << PR_PR_Pos );
  static constexpr Reg32_t PR_PR     = PR_PR_Msk;
  static constexpr Reg32_t PR_PR_0   = ( 0x1U << PR_PR_Pos );
  static constexpr Reg32_t PR_PR_1   = ( 0x2U << PR_PR_Pos );
  static constexpr Reg32_t PR_PR_2   = ( 0x4U << PR_PR_Pos );

  static constexpr Reg32_t PR_PRESCALE_4   = 0;
  static constexpr Reg32_t PR_PRESCALE_8   = 1;
  static constexpr Reg32_t PR_PRESCALE_16  = 2;
  static constexpr Reg32_t PR_PRESCALE_32  = 3;
  static constexpr Reg32_t PR_PRESCALE_64  = 4;
  static constexpr Reg32_t PR_PRESCALE_128 = 5;
  static constexpr Reg32_t PR_PRESCALE_256 = 6;
  static constexpr Reg32_t PR_MIN_PRESCALE = PR_PRESCALE_4;
  static constexpr Reg32_t PR_MAX_PRESCALE = PR_PRESCALE_256;

  static const uint32_t NumPrescalers                      = 7;
  static const uint32_t DecimalPrescalers[ NumPrescalers ] = { 4, 8, 16, 32, 64, 128, 256 };
  static const Reg32_t RegisterPrescalers[ NumPrescalers ] = { PR_PRESCALE_4,  PR_PRESCALE_8,   PR_PRESCALE_16, PR_PRESCALE_32,
                                                               PR_PRESCALE_64, PR_PRESCALE_128, PR_PRESCALE_256 };

  /*-------------------------------------------------
  IWDG Reload Register
  -------------------------------------------------*/
  static constexpr Reg32_t RLR_Msk    = 0x0FFF;
  static constexpr Reg32_t RLR_Rst    = 0x0FFF;
  static constexpr Reg32_t RLR_RL_Pos = ( 0U );
  static constexpr Reg32_t RLR_RL_Msk = ( 0xFFFU << RLR_RL_Pos );
  static constexpr Reg32_t RLR_RL     = RLR_RL_Msk;
  static constexpr Reg32_t RLR_MAX    = 0x0FFF;
  static constexpr Reg32_t RLR_MIN    = 0x0000;

  /*-------------------------------------------------
  IWDG Status Register
  -------------------------------------------------*/
  static constexpr Reg32_t SR_Msk     = 0x03;
  static constexpr Reg32_t SR_Rst     = 0x00;
  static constexpr Reg32_t SR_PVU_Pos = ( 0U );
  static constexpr Reg32_t SR_PVU_Msk = ( 0x1U << SR_PVU_Pos );
  static constexpr Reg32_t SR_PVU     = SR_PVU_Msk;
  static constexpr Reg32_t SR_RVU_Pos = ( 1U );
  static constexpr Reg32_t SR_RVU_Msk = ( 0x1U << SR_RVU_Pos );
  static constexpr Reg32_t SR_RVU     = SR_RVU_Msk;

}    // namespace Thor::LLD::IWDG

#endif /* !THOR_HW_IWDG_REGISTER_HPP */