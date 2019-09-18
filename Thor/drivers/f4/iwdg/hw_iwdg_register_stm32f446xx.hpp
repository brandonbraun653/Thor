/********************************************************************************
 *   File Name:
 *    hw_iwdg_register_stm32f446xx.hpp
 *
 *   Description:
 *    Explicit hardware register definitions for the STM32F446xx Watchdog
 *    peripherals.
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#ifndef THOR_HW_IWDG_REGISTER_HPP
#define THOR_HW_IWDG_REGISTER_HPP

/* C++ Includes */
#include <array>
#include <cstdint>

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/f4/system/sys_memory_map_stm32f446xx.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_WATCHDOG == 1 )
namespace Thor::Driver::IWDG
{
  static constexpr uint32_t IWDG_BASE_ADDR = Thor::System::MemoryMap::APB1PERIPH_BASE_ADDR + 0x2C00U;
  static constexpr uint32_t NUM_IWDG_PERIPHS = 1u;

  static constexpr std::array<uint32_t, NUM_IWDG_PERIPHS> periphAddressList = { IWDG_BASE_ADDR };

  /*-------------------------------------------------
  IWDG Key Register
  -------------------------------------------------*/
  static constexpr uint32_t KR_Msk     = 0xFFFF;
  static constexpr uint32_t KR_Rst     = 0x00;
  static constexpr uint32_t KR_KEY_Pos = ( 0U );
  static constexpr uint32_t KR_KEY_Msk = ( 0xFFFFU << KR_KEY_Pos );
  static constexpr uint32_t KR_KEY     = KR_KEY_Msk;

  /*-------------------------------------------------
  IWDG Prescale Register
  -------------------------------------------------*/
  static constexpr uint32_t PR_Msk    = 0x07;
  static constexpr uint32_t PR_Rst    = 0x00;
  static constexpr uint32_t PR_PR_Pos = ( 0U );
  static constexpr uint32_t PR_PR_Msk = ( 0x7U << PR_PR_Pos );
  static constexpr uint32_t PR_PR     = PR_PR_Msk;
  static constexpr uint32_t PR_PR_0   = ( 0x1U << PR_PR_Pos );
  static constexpr uint32_t PR_PR_1   = ( 0x2U << PR_PR_Pos );
  static constexpr uint32_t PR_PR_2   = ( 0x4U << PR_PR_Pos );

  /*-------------------------------------------------
  IWDG Reload Register
  -------------------------------------------------*/
  static constexpr uint32_t RLR_Msk    = 0x0FFF;
  static constexpr uint32_t RLR_Rst    = 0x0FFF;
  static constexpr uint32_t RLR_RL_Pos = ( 0U );
  static constexpr uint32_t RLR_RL_Msk = ( 0xFFFU << RLR_RL_Pos );
  static constexpr uint32_t RLR_RL     = RLR_RL_Msk;

  /*-------------------------------------------------
  IWDG Status Register
  -------------------------------------------------*/
  static constexpr uint32_t SR_Msk     = 0x03;
  static constexpr uint32_t SR_Rst     = 0x00;
  static constexpr uint32_t SR_PVU_Pos = ( 0U );
  static constexpr uint32_t SR_PVU_Msk = ( 0x1U << SR_PVU_Pos );
  static constexpr uint32_t SR_PVU     = SR_PVU_Msk;
  static constexpr uint32_t SR_RVU_Pos = ( 1U );
  static constexpr uint32_t SR_RVU_Msk = ( 0x1U << SR_RVU_Pos );
  static constexpr uint32_t SR_RVU     = SR_RVU_Msk;

}    // namespace Thor::Driver::IWDG

#endif /* TARGET_STM32F4 && THOR_DRIVER_WATCHDOG */
#endif /* !THOR_HW_IWDG_REGISTER_HPP */