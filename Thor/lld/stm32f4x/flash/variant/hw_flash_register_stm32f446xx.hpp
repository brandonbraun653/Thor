/********************************************************************************
 *  File Name:
 *    hw_flash_register_stm32f446xx.hpp
 *
 *  Description:
 *    Implements Flash register definitions for the STM32F446xx chips
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_DRIVER_FLASH_REGISTER_STM32F446XX_HPP
#define THOR_HW_DRIVER_FLASH_REGISTER_STM32F446XX_HPP

/* C++ Includes */
#include <cstdint>

/* Driver Includes */
#include <Thor/lld/stm32f4x/system/sys_memory_map_prj.hpp>

#define STM32_FLASH1_AVAILABLE

namespace Thor::LLD::FLASH
{
  static constexpr uint32_t FLASH_BASE_ADDR = Thor::System::MemoryMap::AHB1PERIPH_BASE_ADDR + 0x3C00U;

  /*------------------------------------------------
  Access Control Register (ACR)
  ------------------------------------------------*/
  static constexpr uint32_t ACR_LATENCY_Pos       = ( 0U );
  static constexpr uint32_t ACR_LATENCY_Msk       = ( 0xFU << ACR_LATENCY_Pos );
  static constexpr uint32_t ACR_LATENCY           = ACR_LATENCY_Msk;
  static constexpr uint32_t ACR_LATENCY_0WS       = 0x00000000U;
  static constexpr uint32_t ACR_LATENCY_1WS       = 0x00000001U;
  static constexpr uint32_t ACR_LATENCY_2WS       = 0x00000002U;
  static constexpr uint32_t ACR_LATENCY_3WS       = 0x00000003U;
  static constexpr uint32_t ACR_LATENCY_4WS       = 0x00000004U;
  static constexpr uint32_t ACR_LATENCY_5WS       = 0x00000005U;
  static constexpr uint32_t ACR_LATENCY_6WS       = 0x00000006U;
  static constexpr uint32_t ACR_LATENCY_7WS       = 0x00000007U;
  static constexpr uint32_t ACR_LATENCY_8WS       = 0x00000008U;
  static constexpr uint32_t ACR_LATENCY_9WS       = 0x00000009U;
  static constexpr uint32_t ACR_LATENCY_10WS      = 0x0000000AU;
  static constexpr uint32_t ACR_LATENCY_11WS      = 0x0000000BU;
  static constexpr uint32_t ACR_LATENCY_12WS      = 0x0000000CU;
  static constexpr uint32_t ACR_LATENCY_13WS      = 0x0000000DU;
  static constexpr uint32_t ACR_LATENCY_14WS      = 0x0000000EU;
  static constexpr uint32_t ACR_LATENCY_15WS      = 0x0000000FU;
  static constexpr uint32_t ACR_PRFTEN_Pos        = ( 8U );
  static constexpr uint32_t ACR_PRFTEN_Msk        = ( 0x1U << ACR_PRFTEN_Pos );
  static constexpr uint32_t ACR_PRFTEN            = ACR_PRFTEN_Msk;
  static constexpr uint32_t ACR_ICEN_Pos          = ( 9U );
  static constexpr uint32_t ACR_ICEN_Msk          = ( 0x1U << ACR_ICEN_Pos );
  static constexpr uint32_t ACR_ICEN              = ACR_ICEN_Msk;
  static constexpr uint32_t ACR_DCEN_Pos          = ( 10U );
  static constexpr uint32_t ACR_DCEN_Msk          = ( 0x1U << ACR_DCEN_Pos );
  static constexpr uint32_t ACR_DCEN              = ACR_DCEN_Msk;
  static constexpr uint32_t ACR_ICRST_Pos         = ( 11U );
  static constexpr uint32_t ACR_ICRST_Msk         = ( 0x1U << ACR_ICRST_Pos );
  static constexpr uint32_t ACR_ICRST             = ACR_ICRST_Msk;
  static constexpr uint32_t ACR_DCRST_Pos         = ( 12U );
  static constexpr uint32_t ACR_DCRST_Msk         = ( 0x1U << ACR_DCRST_Pos );
  static constexpr uint32_t ACR_DCRST             = ACR_DCRST_Msk;
  static constexpr uint32_t ACR_BYTE0_ADDRESS_Pos = ( 10U );
  static constexpr uint32_t ACR_BYTE0_ADDRESS_Msk = ( 0x10008FU << ACR_BYTE0_ADDRESS_Pos );
  static constexpr uint32_t ACR_BYTE0_ADDRESS     = ACR_BYTE0_ADDRESS_Msk;
  static constexpr uint32_t ACR_BYTE2_ADDRESS_Pos = ( 0U );
  static constexpr uint32_t ACR_BYTE2_ADDRESS_Msk = ( 0x40023C03U << ACR_BYTE2_ADDRESS_Pos );
  static constexpr uint32_t ACR_BYTE2_ADDRESS     = ACR_BYTE2_ADDRESS_Msk;

  /*------------------------------------------------
  Status Register (SR)
  ------------------------------------------------*/
  static constexpr uint32_t SR_EOP_Pos    = ( 0U );
  static constexpr uint32_t SR_EOP_Msk    = ( 0x1U << SR_EOP_Pos );
  static constexpr uint32_t SR_EOP        = SR_EOP_Msk;
  static constexpr uint32_t SR_SOP_Pos    = ( 1U );
  static constexpr uint32_t SR_SOP_Msk    = ( 0x1U << SR_SOP_Pos );
  static constexpr uint32_t SR_SOP        = SR_SOP_Msk;
  static constexpr uint32_t SR_WRPERR_Pos = ( 4U );
  static constexpr uint32_t SR_WRPERR_Msk = ( 0x1U << SR_WRPERR_Pos );
  static constexpr uint32_t SR_WRPERR     = SR_WRPERR_Msk;
  static constexpr uint32_t SR_PGAERR_Pos = ( 5U );
  static constexpr uint32_t SR_PGAERR_Msk = ( 0x1U << SR_PGAERR_Pos );
  static constexpr uint32_t SR_PGAERR     = SR_PGAERR_Msk;
  static constexpr uint32_t SR_PGPERR_Pos = ( 6U );
  static constexpr uint32_t SR_PGPERR_Msk = ( 0x1U << SR_PGPERR_Pos );
  static constexpr uint32_t SR_PGPERR     = SR_PGPERR_Msk;
  static constexpr uint32_t SR_PGSERR_Pos = ( 7U );
  static constexpr uint32_t SR_PGSERR_Msk = ( 0x1U << SR_PGSERR_Pos );
  static constexpr uint32_t SR_PGSERR     = SR_PGSERR_Msk;
  static constexpr uint32_t SR_RDERR_Pos  = ( 8U );
  static constexpr uint32_t SR_RDERR_Msk  = ( 0x1U << SR_RDERR_Pos );
  static constexpr uint32_t SR_RDERR      = SR_RDERR_Msk;
  static constexpr uint32_t SR_BSY_Pos    = ( 16U );
  static constexpr uint32_t SR_BSY_Msk    = ( 0x1U << SR_BSY_Pos );
  static constexpr uint32_t SR_BSY        = SR_BSY_Msk;

  /*------------------------------------------------
  Control Register (CR)
  ------------------------------------------------*/
  static constexpr uint32_t CR_PG_Pos    = ( 0U );
  static constexpr uint32_t CR_PG_Msk    = ( 0x1U << CR_PG_Pos );
  static constexpr uint32_t CR_PG        = CR_PG_Msk;
  static constexpr uint32_t CR_SER_Pos   = ( 1U );
  static constexpr uint32_t CR_SER_Msk   = ( 0x1U << CR_SER_Pos );
  static constexpr uint32_t CR_SER       = CR_SER_Msk;
  static constexpr uint32_t CR_MER_Pos   = ( 2U );
  static constexpr uint32_t CR_MER_Msk   = ( 0x1U << CR_MER_Pos );
  static constexpr uint32_t CR_MER       = CR_MER_Msk;
  static constexpr uint32_t CR_MER1      = CR_MER;
  static constexpr uint32_t CR_SNB_Pos   = ( 3U );
  static constexpr uint32_t CR_SNB_Msk   = ( 0x1FU << CR_SNB_Pos );
  static constexpr uint32_t CR_SNB       = CR_SNB_Msk;
  static constexpr uint32_t CR_SNB_0     = ( 0x01U << CR_SNB_Pos );
  static constexpr uint32_t CR_SNB_1     = ( 0x02U << CR_SNB_Pos );
  static constexpr uint32_t CR_SNB_2     = ( 0x04U << CR_SNB_Pos );
  static constexpr uint32_t CR_SNB_3     = ( 0x08U << CR_SNB_Pos );
  static constexpr uint32_t CR_SNB_4     = ( 0x10U << CR_SNB_Pos );
  static constexpr uint32_t CR_PSIZE_Pos = ( 8U );
  static constexpr uint32_t CR_PSIZE_Msk = ( 0x3U << CR_PSIZE_Pos );
  static constexpr uint32_t CR_PSIZE     = CR_PSIZE_Msk;
  static constexpr uint32_t CR_PSIZE_0   = ( 0x1U << CR_PSIZE_Pos );
  static constexpr uint32_t CR_PSIZE_1   = ( 0x2U << CR_PSIZE_Pos );
  static constexpr uint32_t CR_MER2_Pos  = ( 15U );
  static constexpr uint32_t CR_MER2_Msk  = ( 0x1U << CR_MER2_Pos );
  static constexpr uint32_t CR_MER2      = CR_MER2_Msk;
  static constexpr uint32_t CR_STRT_Pos  = ( 16U );
  static constexpr uint32_t CR_STRT_Msk  = ( 0x1U << CR_STRT_Pos );
  static constexpr uint32_t CR_STRT      = CR_STRT_Msk;
  static constexpr uint32_t CR_EOPIE_Pos = ( 24U );
  static constexpr uint32_t CR_EOPIE_Msk = ( 0x1U << CR_EOPIE_Pos );
  static constexpr uint32_t CR_EOPIE     = CR_EOPIE_Msk;
  static constexpr uint32_t CR_LOCK_Pos  = ( 31U );
  static constexpr uint32_t CR_LOCK_Msk  = ( 0x1U << CR_LOCK_Pos );
  static constexpr uint32_t CR_LOCK      = CR_LOCK_Msk;

  /*------------------------------------------------
  Option Control Register (OPTCR)
  ------------------------------------------------*/
  static constexpr uint32_t OPTCR_OPTLOCK_Pos    = ( 0U );
  static constexpr uint32_t OPTCR_OPTLOCK_Msk    = ( 0x1U << OPTCR_OPTLOCK_Pos );
  static constexpr uint32_t OPTCR_OPTLOCK        = OPTCR_OPTLOCK_Msk;
  static constexpr uint32_t OPTCR_OPTSTRT_Pos    = ( 1U );
  static constexpr uint32_t OPTCR_OPTSTRT_Msk    = ( 0x1U << OPTCR_OPTSTRT_Pos );
  static constexpr uint32_t OPTCR_OPTSTRT        = OPTCR_OPTSTRT_Msk;
  static constexpr uint32_t OPTCR_BOR_LEV_0      = 0x00000004U;
  static constexpr uint32_t OPTCR_BOR_LEV_1      = 0x00000008U;
  static constexpr uint32_t OPTCR_BOR_LEV_Pos    = ( 2U );
  static constexpr uint32_t OPTCR_BOR_LEV_Msk    = ( 0x3U << OPTCR_BOR_LEV_Pos );
  static constexpr uint32_t OPTCR_BOR_LEV        = OPTCR_BOR_LEV_Msk;
  static constexpr uint32_t OPTCR_BFB2_Pos       = ( 4U );
  static constexpr uint32_t OPTCR_BFB2_Msk       = ( 0x1U << OPTCR_BFB2_Pos );
  static constexpr uint32_t OPTCR_BFB2           = OPTCR_BFB2_Msk;
  static constexpr uint32_t OPTCR_WDG_SW_Pos     = ( 5U );
  static constexpr uint32_t OPTCR_WDG_SW_Msk     = ( 0x1U << OPTCR_WDG_SW_Pos );
  static constexpr uint32_t OPTCR_WDG_SW         = OPTCR_WDG_SW_Msk;
  static constexpr uint32_t OPTCR_nRST_STOP_Pos  = ( 6U );
  static constexpr uint32_t OPTCR_nRST_STOP_Msk  = ( 0x1U << OPTCR_nRST_STOP_Pos );
  static constexpr uint32_t OPTCR_nRST_STOP      = OPTCR_nRST_STOP_Msk;
  static constexpr uint32_t OPTCR_nRST_STDBY_Pos = ( 7U );
  static constexpr uint32_t OPTCR_nRST_STDBY_Msk = ( 0x1U << OPTCR_nRST_STDBY_Pos );
  static constexpr uint32_t OPTCR_nRST_STDBY     = OPTCR_nRST_STDBY_Msk;
  static constexpr uint32_t OPTCR_RDP_Pos        = ( 8U );
  static constexpr uint32_t OPTCR_RDP_Msk        = ( 0xFFU << OPTCR_RDP_Pos );
  static constexpr uint32_t OPTCR_RDP            = OPTCR_RDP_Msk;
  static constexpr uint32_t OPTCR_RDP_0          = ( 0x01U << OPTCR_RDP_Pos );
  static constexpr uint32_t OPTCR_RDP_1          = ( 0x02U << OPTCR_RDP_Pos );
  static constexpr uint32_t OPTCR_RDP_2          = ( 0x04U << OPTCR_RDP_Pos );
  static constexpr uint32_t OPTCR_RDP_3          = ( 0x08U << OPTCR_RDP_Pos );
  static constexpr uint32_t OPTCR_RDP_4          = ( 0x10U << OPTCR_RDP_Pos );
  static constexpr uint32_t OPTCR_RDP_5          = ( 0x20U << OPTCR_RDP_Pos );
  static constexpr uint32_t OPTCR_RDP_6          = ( 0x40U << OPTCR_RDP_Pos );
  static constexpr uint32_t OPTCR_RDP_7          = ( 0x80U << OPTCR_RDP_Pos );
  static constexpr uint32_t OPTCR_nWRP_Pos       = ( 16U );
  static constexpr uint32_t OPTCR_nWRP_Msk       = ( 0xFFFU << OPTCR_nWRP_Pos );
  static constexpr uint32_t OPTCR_nWRP           = OPTCR_nWRP_Msk;
  static constexpr uint32_t OPTCR_nWRP_0         = 0x00010000U;
  static constexpr uint32_t OPTCR_nWRP_1         = 0x00020000U;
  static constexpr uint32_t OPTCR_nWRP_2         = 0x00040000U;
  static constexpr uint32_t OPTCR_nWRP_3         = 0x00080000U;
  static constexpr uint32_t OPTCR_nWRP_4         = 0x00100000U;
  static constexpr uint32_t OPTCR_nWRP_5         = 0x00200000U;
  static constexpr uint32_t OPTCR_nWRP_6         = 0x00400000U;
  static constexpr uint32_t OPTCR_nWRP_7         = 0x00800000U;
  static constexpr uint32_t OPTCR_nWRP_8         = 0x01000000U;
  static constexpr uint32_t OPTCR_nWRP_9         = 0x02000000U;
  static constexpr uint32_t OPTCR_nWRP_10        = 0x04000000U;
  static constexpr uint32_t OPTCR_nWRP_11        = 0x08000000U;
  static constexpr uint32_t OPTCR_DB1M_Pos       = ( 30U );
  static constexpr uint32_t OPTCR_DB1M_Msk       = ( 0x1U << OPTCR_DB1M_Pos );
  static constexpr uint32_t OPTCR_DB1M           = OPTCR_DB1M_Msk;
  static constexpr uint32_t OPTCR_SPRMOD_Pos     = ( 31U );
  static constexpr uint32_t OPTCR_SPRMOD_Msk     = ( 0x1U << OPTCR_SPRMOD_Pos );
  static constexpr uint32_t OPTCR_SPRMOD         = OPTCR_SPRMOD_Msk;

  /*------------------------------------------------
  Option Control Register 1 (OPTCR1)
  ------------------------------------------------*/
  static constexpr uint32_t OPTCR1_nWRP_Pos = ( 16U );
  static constexpr uint32_t OPTCR1_nWRP_Msk = ( 0xFFFU << OPTCR1_nWRP_Pos );
  static constexpr uint32_t OPTCR1_nWRP     = OPTCR1_nWRP_Msk;
  static constexpr uint32_t OPTCR1_nWRP_0   = ( 0x001U << OPTCR1_nWRP_Pos );
  static constexpr uint32_t OPTCR1_nWRP_1   = ( 0x002U << OPTCR1_nWRP_Pos );
  static constexpr uint32_t OPTCR1_nWRP_2   = ( 0x004U << OPTCR1_nWRP_Pos );
  static constexpr uint32_t OPTCR1_nWRP_3   = ( 0x008U << OPTCR1_nWRP_Pos );
  static constexpr uint32_t OPTCR1_nWRP_4   = ( 0x010U << OPTCR1_nWRP_Pos );
  static constexpr uint32_t OPTCR1_nWRP_5   = ( 0x020U << OPTCR1_nWRP_Pos );
  static constexpr uint32_t OPTCR1_nWRP_6   = ( 0x040U << OPTCR1_nWRP_Pos );
  static constexpr uint32_t OPTCR1_nWRP_7   = ( 0x080U << OPTCR1_nWRP_Pos );
  static constexpr uint32_t OPTCR1_nWRP_8   = ( 0x100U << OPTCR1_nWRP_Pos );
  static constexpr uint32_t OPTCR1_nWRP_9   = ( 0x200U << OPTCR1_nWRP_Pos );
  static constexpr uint32_t OPTCR1_nWRP_10  = ( 0x400U << OPTCR1_nWRP_Pos );
  static constexpr uint32_t OPTCR1_nWRP_11  = ( 0x800U << OPTCR1_nWRP_Pos );
}    // namespace Thor::LLD::FLASH

#endif /* !THOR_HW_DRIVER_FLASH_REGISTER_STM32F446XX_HPP */