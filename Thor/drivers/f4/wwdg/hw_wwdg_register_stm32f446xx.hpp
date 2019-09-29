/********************************************************************************
 *   File Name:
 *    hw_wwdg_register_stm32f446xx.hpp
 *
 *   Description:
 *    Explicit hardware register definitions for the STM32F446xx Watchdog
 *    peripherals.
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#ifndef THOR_HW_WWDG_REGISTER_HPP
#define THOR_HW_WWDG_REGISTER_HPP

/* C++ Includes */
#include <array>
#include <cstdint>

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/f4/system/sys_memory_map_stm32f446xx.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_WWDG == 1 )
namespace Thor::Driver::WWDG
{
  static constexpr uint32_t WWDG_BASE_ADDR = Thor::System::MemoryMap::APB1PERIPH_BASE_ADDR + 0x2C00U;
  static constexpr uint32_t NUM_WWDG_PERIPHS = 1u;

  static constexpr std::array<uint32_t, NUM_WWDG_PERIPHS> periphAddressList = { WWDG_BASE_ADDR };

  /*-------------------------------------------------
  WWDG Control Register
  -------------------------------------------------*/
  static constexpr uint32_t CR_Msk      = 0xFF;
  static constexpr uint32_t CR_Rst      = 0x7F;
  static constexpr uint32_t CR_T_Pos    = ( 0U );
  static constexpr uint32_t CR_T_Msk    = ( 0x7FU << CR_T_Pos );
  static constexpr uint32_t CR_T        = CR_T_Msk;
  static constexpr uint32_t CR_T_0      = ( 0x01U << CR_T_Pos );
  static constexpr uint32_t CR_T_1      = ( 0x02U << CR_T_Pos );
  static constexpr uint32_t CR_T_2      = ( 0x04U << CR_T_Pos );
  static constexpr uint32_t CR_T_3      = ( 0x08U << CR_T_Pos );
  static constexpr uint32_t CR_T_4      = ( 0x10U << CR_T_Pos );
  static constexpr uint32_t CR_T_5      = ( 0x20U << CR_T_Pos );
  static constexpr uint32_t CR_T_6      = ( 0x40U << CR_T_Pos );
  static constexpr uint32_t CR_WDGA_Pos = ( 7U );
  static constexpr uint32_t CR_WDGA_Msk = ( 0x1U << CR_WDGA_Pos );
  static constexpr uint32_t CR_WDGA     = CR_WDGA_Msk;

  static constexpr uint32_t CR_T_MAX = 0x7F;
  static constexpr uint32_t CR_T_MIN = 0x40;
  static constexpr uint32_t CR_T_RNG = CR_T_MAX - CR_T_MIN;
  static constexpr uint32_t CR_T_ACT_LOW_MANUAL_RESET = CR_T_6;

  /*-------------------------------------------------
  WWDG Configuration Register
  -------------------------------------------------*/
  static constexpr uint32_t CFR_Msk        = 0x3FF;
  static constexpr uint32_t CFR_Rst        = 0x7F;
  static constexpr uint32_t CFR_W_Pos     = ( 0U );
  static constexpr uint32_t CFR_W_Msk     = ( 0x7FU << CFR_W_Pos );
  static constexpr uint32_t CFR_W         = CFR_W_Msk;
  static constexpr uint32_t CFR_W_0       = ( 0x01U << CFR_W_Pos );
  static constexpr uint32_t CFR_W_1       = ( 0x02U << CFR_W_Pos );
  static constexpr uint32_t CFR_W_2       = ( 0x04U << CFR_W_Pos );
  static constexpr uint32_t CFR_W_3       = ( 0x08U << CFR_W_Pos );
  static constexpr uint32_t CFR_W_4       = ( 0x10U << CFR_W_Pos );
  static constexpr uint32_t CFR_W_5       = ( 0x20U << CFR_W_Pos );
  static constexpr uint32_t CFR_W_6       = ( 0x40U << CFR_W_Pos );
  static constexpr uint32_t CFR_WDGTB_Pos = ( 7U );
  static constexpr uint32_t CFR_WDGTB_Msk = ( 0x3U << CFR_WDGTB_Pos );
  static constexpr uint32_t CFR_WDGTB     = CFR_WDGTB_Msk;
  static constexpr uint32_t CFR_WDGTB_0   = ( 0x1U << CFR_WDGTB_Pos );
  static constexpr uint32_t CFR_WDGTB_1   = ( 0x2U << CFR_WDGTB_Pos );
  static constexpr uint32_t CFR_EWI_Pos   = ( 9U );
  static constexpr uint32_t CFR_EWI_Msk   = ( 0x1U << CFR_EWI_Pos );
  static constexpr uint32_t CFR_EWI       = CFR_EWI_Msk;

  static constexpr uint32_t CFR_CLK_DIV_1  = 0u;
  static constexpr uint32_t CFR_CLK_DIV_2  = 1u;
  static constexpr uint32_t CFR_CLK_DIV_4  = 2u;
  static constexpr uint32_t CFR_CLK_DIV_8  = 3u;
  static constexpr uint32_t CFR_PCLK_1_DIV = 4096u;

  /*-------------------------------------------------
  WWDG Status Register
  -------------------------------------------------*/
  static constexpr uint32_t SR_Msk      = 0x01;
  static constexpr uint32_t SR_Rst      = 0x00;
  static constexpr uint32_t SR_EWIF_Pos = ( 0U );
  static constexpr uint32_t SR_EWIF_Msk = ( 0x1U << SR_EWIF_Pos );
  static constexpr uint32_t SR_EWIF     = SR_EWIF_Msk;

}    // namespace Thor::Driver::WWDG

#endif /* TARGET_STM32F4 && THOR_DRIVER_WATCHDOG */
#endif /* !THOR_HW_WWDG_REGISTER_HPP */