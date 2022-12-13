/******************************************************************************
 *  File Name:
 *    hw_flash_register_stm32l432kc.hpp
 *
 *  Description:
 *    FLASH register definitions for the STM32L432KC series chips.
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_HW_FLASH_REGISTER_STM32L432KC_HPP
#define THOR_HW_FLASH_REGISTER_STM32L432KC_HPP

/* C++ Includes */
#include <cstdint>

/* Chimera Includes */
#include <Chimera/common>

/* Driver Includes */
#include <Thor/lld/stm32l4x/system/variant/sys_memory_map_stm32l432kc.hpp>

/*-------------------------------------------------
Peripheral Availability
-------------------------------------------------*/
#define STM32_FLASH_PERIPH_AVAILABLE

namespace Thor::LLD::FLASH
{
  /*-------------------------------------------------
  Peripheral Instance Memory Map Base
  -------------------------------------------------*/
  static constexpr Reg32_t FLASH_BASE_ADDR = Thor::System::MemoryMap::FLASH_PERIPH_START_ADDRESS;

  /*-------------------------------------------------
  Peripheral Resource Lookup Indices
  -------------------------------------------------*/
  static constexpr uint32_t FLASH_RESOURCE_INDEX = 0u;

  /*-------------------------------------------------
  Lookup addresses
  -------------------------------------------------*/
  static constexpr size_t NUM_FLASH_PERIPHS = 1;

  /*-------------------------------------------------
  Peripheral Register Definitions
  -------------------------------------------------*/
  /*******************  Bits definition for ACR register  *****************/
  static constexpr Reg32_t ACR_LATENCY_Pos  = ( 0U );
  static constexpr Reg32_t ACR_LATENCY_Msk  = ( 0x7UL << ACR_LATENCY_Pos );
  static constexpr Reg32_t ACR_LATENCY      = ACR_LATENCY_Msk;
  static constexpr Reg32_t ACR_LATENCY_0WS  = ( 0x00000000UL );
  static constexpr Reg32_t ACR_LATENCY_1WS  = ( 0x00000001UL );
  static constexpr Reg32_t ACR_LATENCY_2WS  = ( 0x00000002UL );
  static constexpr Reg32_t ACR_LATENCY_3WS  = ( 0x00000003UL );
  static constexpr Reg32_t ACR_LATENCY_4WS  = ( 0x00000004UL );
  static constexpr Reg32_t ACR_PRFTEN_Pos   = ( 8U );
  static constexpr Reg32_t ACR_PRFTEN_Msk   = ( 0x1UL << ACR_PRFTEN_Pos );
  static constexpr Reg32_t ACR_PRFTEN       = ACR_PRFTEN_Msk;
  static constexpr Reg32_t ACR_ICEN_Pos     = ( 9U );
  static constexpr Reg32_t ACR_ICEN_Msk     = ( 0x1UL << ACR_ICEN_Pos );
  static constexpr Reg32_t ACR_ICEN         = ACR_ICEN_Msk;
  static constexpr Reg32_t ACR_DCEN_Pos     = ( 10U );
  static constexpr Reg32_t ACR_DCEN_Msk     = ( 0x1UL << ACR_DCEN_Pos );
  static constexpr Reg32_t ACR_DCEN         = ACR_DCEN_Msk;
  static constexpr Reg32_t ACR_ICRST_Pos    = ( 11U );
  static constexpr Reg32_t ACR_ICRST_Msk    = ( 0x1UL << ACR_ICRST_Pos );
  static constexpr Reg32_t ACR_ICRST        = ACR_ICRST_Msk;
  static constexpr Reg32_t ACR_DCRST_Pos    = ( 12U );
  static constexpr Reg32_t ACR_DCRST_Msk    = ( 0x1UL << ACR_DCRST_Pos );
  static constexpr Reg32_t ACR_DCRST        = ACR_DCRST_Msk;
  static constexpr Reg32_t ACR_RUN_PD_Pos   = ( 13U );
  static constexpr Reg32_t ACR_RUN_PD_Msk   = ( 0x1UL << ACR_RUN_PD_Pos );
  static constexpr Reg32_t ACR_RUN_PD       = ACR_RUN_PD_Msk;
  static constexpr Reg32_t ACR_SLEEP_PD_Pos = ( 14U );
  static constexpr Reg32_t ACR_SLEEP_PD_Msk = ( 0x1UL << ACR_SLEEP_PD_Pos );
  static constexpr Reg32_t ACR_SLEEP_PD     = ACR_SLEEP_PD_Msk;

  /*******************  Bits definition for SR register  ******************/
  static constexpr Reg32_t SR_EOP_Pos     = ( 0U );
  static constexpr Reg32_t SR_EOP_Msk     = ( 0x1UL << SR_EOP_Pos );
  static constexpr Reg32_t SR_EOP         = SR_EOP_Msk;
  static constexpr Reg32_t SR_OPERR_Pos   = ( 1U );
  static constexpr Reg32_t SR_OPERR_Msk   = ( 0x1UL << SR_OPERR_Pos );
  static constexpr Reg32_t SR_OPERR       = SR_OPERR_Msk;
  static constexpr Reg32_t SR_PROGERR_Pos = ( 3U );
  static constexpr Reg32_t SR_PROGERR_Msk = ( 0x1UL << SR_PROGERR_Pos );
  static constexpr Reg32_t SR_PROGERR     = SR_PROGERR_Msk;
  static constexpr Reg32_t SR_WRPERR_Pos  = ( 4U );
  static constexpr Reg32_t SR_WRPERR_Msk  = ( 0x1UL << SR_WRPERR_Pos );
  static constexpr Reg32_t SR_WRPERR      = SR_WRPERR_Msk;
  static constexpr Reg32_t SR_PGAERR_Pos  = ( 5U );
  static constexpr Reg32_t SR_PGAERR_Msk  = ( 0x1UL << SR_PGAERR_Pos );
  static constexpr Reg32_t SR_PGAERR      = SR_PGAERR_Msk;
  static constexpr Reg32_t SR_SIZERR_Pos  = ( 6U );
  static constexpr Reg32_t SR_SIZERR_Msk  = ( 0x1UL << SR_SIZERR_Pos );
  static constexpr Reg32_t SR_SIZERR      = SR_SIZERR_Msk;
  static constexpr Reg32_t SR_PGSERR_Pos  = ( 7U );
  static constexpr Reg32_t SR_PGSERR_Msk  = ( 0x1UL << SR_PGSERR_Pos );
  static constexpr Reg32_t SR_PGSERR      = SR_PGSERR_Msk;
  static constexpr Reg32_t SR_MISERR_Pos  = ( 8U );
  static constexpr Reg32_t SR_MISERR_Msk  = ( 0x1UL << SR_MISERR_Pos );
  static constexpr Reg32_t SR_MISERR      = SR_MISERR_Msk;
  static constexpr Reg32_t SR_FASTERR_Pos = ( 9U );
  static constexpr Reg32_t SR_FASTERR_Msk = ( 0x1UL << SR_FASTERR_Pos );
  static constexpr Reg32_t SR_FASTERR     = SR_FASTERR_Msk;
  static constexpr Reg32_t SR_RDERR_Pos   = ( 14U );
  static constexpr Reg32_t SR_RDERR_Msk   = ( 0x1UL << SR_RDERR_Pos );
  static constexpr Reg32_t SR_RDERR       = SR_RDERR_Msk;
  static constexpr Reg32_t SR_OPTVERR_Pos = ( 15U );
  static constexpr Reg32_t SR_OPTVERR_Msk = ( 0x1UL << SR_OPTVERR_Pos );
  static constexpr Reg32_t SR_OPTVERR     = SR_OPTVERR_Msk;
  static constexpr Reg32_t SR_BSY_Pos     = ( 16U );
  static constexpr Reg32_t SR_BSY_Msk     = ( 0x1UL << SR_BSY_Pos );
  static constexpr Reg32_t SR_BSY         = SR_BSY_Msk;
  static constexpr Reg32_t SR_PEMPTY_Pos  = ( 17U );
  static constexpr Reg32_t SR_PEMPTY_Msk  = ( 0x1UL << SR_PEMPTY_Pos );
  static constexpr Reg32_t SR_PEMPTY      = SR_PEMPTY_Msk;

  /*******************  Bits definition for CR register  ******************/
  static constexpr Reg32_t CR_PG_Pos         = ( 0U );
  static constexpr Reg32_t CR_PG_Msk         = ( 0x1UL << CR_PG_Pos );
  static constexpr Reg32_t CR_PG             = CR_PG_Msk;
  static constexpr Reg32_t CR_PER_Pos        = ( 1U );
  static constexpr Reg32_t CR_PER_Msk        = ( 0x1UL << CR_PER_Pos );
  static constexpr Reg32_t CR_PER            = CR_PER_Msk;
  static constexpr Reg32_t CR_MER1_Pos       = ( 2U );
  static constexpr Reg32_t CR_MER1_Msk       = ( 0x1UL << CR_MER1_Pos );
  static constexpr Reg32_t CR_MER1           = CR_MER1_Msk;
  static constexpr Reg32_t CR_PNB_Pos        = ( 3U );
  static constexpr Reg32_t CR_PNB_Msk        = ( 0x7FUL << CR_PNB_Pos );
  static constexpr Reg32_t CR_PNB            = CR_PNB_Msk;
  static constexpr Reg32_t CR_STRT_Pos       = ( 16U );
  static constexpr Reg32_t CR_STRT_Msk       = ( 0x1UL << CR_STRT_Pos );
  static constexpr Reg32_t CR_STRT           = CR_STRT_Msk;
  static constexpr Reg32_t CR_OPTSTRT_Pos    = ( 17U );
  static constexpr Reg32_t CR_OPTSTRT_Msk    = ( 0x1UL << CR_OPTSTRT_Pos );
  static constexpr Reg32_t CR_OPTSTRT        = CR_OPTSTRT_Msk;
  static constexpr Reg32_t CR_FSTPG_Pos      = ( 18U );
  static constexpr Reg32_t CR_FSTPG_Msk      = ( 0x1UL << CR_FSTPG_Pos );
  static constexpr Reg32_t CR_FSTPG          = CR_FSTPG_Msk;
  static constexpr Reg32_t CR_EOPIE_Pos      = ( 24U );
  static constexpr Reg32_t CR_EOPIE_Msk      = ( 0x1UL << CR_EOPIE_Pos );
  static constexpr Reg32_t CR_EOPIE          = CR_EOPIE_Msk;
  static constexpr Reg32_t CR_ERRIE_Pos      = ( 25U );
  static constexpr Reg32_t CR_ERRIE_Msk      = ( 0x1UL << CR_ERRIE_Pos );
  static constexpr Reg32_t CR_ERRIE          = CR_ERRIE_Msk;
  static constexpr Reg32_t CR_RDERRIE_Pos    = ( 26U );
  static constexpr Reg32_t CR_RDERRIE_Msk    = ( 0x1UL << CR_RDERRIE_Pos );
  static constexpr Reg32_t CR_RDERRIE        = CR_RDERRIE_Msk;
  static constexpr Reg32_t CR_OBL_LAUNCH_Pos = ( 27U );
  static constexpr Reg32_t CR_OBL_LAUNCH_Msk = ( 0x1UL << CR_OBL_LAUNCH_Pos );
  static constexpr Reg32_t CR_OBL_LAUNCH     = CR_OBL_LAUNCH_Msk;
  static constexpr Reg32_t CR_OPTLOCK_Pos    = ( 30U );
  static constexpr Reg32_t CR_OPTLOCK_Msk    = ( 0x1UL << CR_OPTLOCK_Pos );
  static constexpr Reg32_t CR_OPTLOCK        = CR_OPTLOCK_Msk;
  static constexpr Reg32_t CR_LOCK_Pos       = ( 31U );
  static constexpr Reg32_t CR_LOCK_Msk       = ( 0x1UL << CR_LOCK_Pos );
  static constexpr Reg32_t CR_LOCK           = CR_LOCK_Msk;

  /*******************  Bits definition for ECCR register  ***************/
  static constexpr Reg32_t ECCR_ADDR_ECC_Pos = ( 0U );
  static constexpr Reg32_t ECCR_ADDR_ECC_Msk = ( 0x7FFFFUL << ECCR_ADDR_ECC_Pos );
  static constexpr Reg32_t ECCR_ADDR_ECC     = ECCR_ADDR_ECC_Msk;
  static constexpr Reg32_t ECCR_SYSF_ECC_Pos = ( 20U );
  static constexpr Reg32_t ECCR_SYSF_ECC_Msk = ( 0x1UL << ECCR_SYSF_ECC_Pos );
  static constexpr Reg32_t ECCR_SYSF_ECC     = ECCR_SYSF_ECC_Msk;
  static constexpr Reg32_t ECCR_ECCIE_Pos    = ( 24U );
  static constexpr Reg32_t ECCR_ECCIE_Msk    = ( 0x1UL << ECCR_ECCIE_Pos );
  static constexpr Reg32_t ECCR_ECCIE        = ECCR_ECCIE_Msk;
  static constexpr Reg32_t ECCR_ECCC_Pos     = ( 30U );
  static constexpr Reg32_t ECCR_ECCC_Msk     = ( 0x1UL << ECCR_ECCC_Pos );
  static constexpr Reg32_t ECCR_ECCC         = ECCR_ECCC_Msk;
  static constexpr Reg32_t ECCR_ECCD_Pos     = ( 31U );
  static constexpr Reg32_t ECCR_ECCD_Msk     = ( 0x1UL << ECCR_ECCD_Pos );
  static constexpr Reg32_t ECCR_ECCD         = ECCR_ECCD_Msk;

  /*******************  Bits definition for OPTR register  ***************/
  static constexpr Reg32_t OPTR_RDP_Pos        = ( 0U );
  static constexpr Reg32_t OPTR_RDP_Msk        = ( 0xFFUL << OPTR_RDP_Pos );
  static constexpr Reg32_t OPTR_RDP            = OPTR_RDP_Msk;
  static constexpr Reg32_t OPTR_BOR_LEV_Pos    = ( 8U );
  static constexpr Reg32_t OPTR_BOR_LEV_Msk    = ( 0x7UL << OPTR_BOR_LEV_Pos );
  static constexpr Reg32_t OPTR_BOR_LEV        = OPTR_BOR_LEV_Msk;
  static constexpr Reg32_t OPTR_BOR_LEV_0      = ( 0x0UL << OPTR_BOR_LEV_Pos );
  static constexpr Reg32_t OPTR_BOR_LEV_1      = ( 0x1UL << OPTR_BOR_LEV_Pos );
  static constexpr Reg32_t OPTR_BOR_LEV_2      = ( 0x2UL << OPTR_BOR_LEV_Pos );
  static constexpr Reg32_t OPTR_BOR_LEV_3      = ( 0x3UL << OPTR_BOR_LEV_Pos );
  static constexpr Reg32_t OPTR_BOR_LEV_4      = ( 0x4UL << OPTR_BOR_LEV_Pos );
  static constexpr Reg32_t OPTR_nRST_STOP_Pos  = ( 12U );
  static constexpr Reg32_t OPTR_nRST_STOP_Msk  = ( 0x1UL << OPTR_nRST_STOP_Pos );
  static constexpr Reg32_t OPTR_nRST_STOP      = OPTR_nRST_STOP_Msk;
  static constexpr Reg32_t OPTR_nRST_STDBY_Pos = ( 13U );
  static constexpr Reg32_t OPTR_nRST_STDBY_Msk = ( 0x1UL << OPTR_nRST_STDBY_Pos );
  static constexpr Reg32_t OPTR_nRST_STDBY     = OPTR_nRST_STDBY_Msk;
  static constexpr Reg32_t OPTR_nRST_SHDW_Pos  = ( 14U );
  static constexpr Reg32_t OPTR_nRST_SHDW_Msk  = ( 0x1UL << OPTR_nRST_SHDW_Pos );
  static constexpr Reg32_t OPTR_nRST_SHDW      = OPTR_nRST_SHDW_Msk;
  static constexpr Reg32_t OPTR_IWDG_SW_Pos    = ( 16U );
  static constexpr Reg32_t OPTR_IWDG_SW_Msk    = ( 0x1UL << OPTR_IWDG_SW_Pos );
  static constexpr Reg32_t OPTR_IWDG_SW        = OPTR_IWDG_SW_Msk;
  static constexpr Reg32_t OPTR_IWDG_STOP_Pos  = ( 17U );
  static constexpr Reg32_t OPTR_IWDG_STOP_Msk  = ( 0x1UL << OPTR_IWDG_STOP_Pos );
  static constexpr Reg32_t OPTR_IWDG_STOP      = OPTR_IWDG_STOP_Msk;
  static constexpr Reg32_t OPTR_IWDG_STDBY_Pos = ( 18U );
  static constexpr Reg32_t OPTR_IWDG_STDBY_Msk = ( 0x1UL << OPTR_IWDG_STDBY_Pos );
  static constexpr Reg32_t OPTR_IWDG_STDBY     = OPTR_IWDG_STDBY_Msk;
  static constexpr Reg32_t OPTR_WWDG_SW_Pos    = ( 19U );
  static constexpr Reg32_t OPTR_WWDG_SW_Msk    = ( 0x1UL << OPTR_WWDG_SW_Pos );
  static constexpr Reg32_t OPTR_WWDG_SW        = OPTR_WWDG_SW_Msk;
  static constexpr Reg32_t OPTR_nBOOT1_Pos     = ( 23U );
  static constexpr Reg32_t OPTR_nBOOT1_Msk     = ( 0x1UL << OPTR_nBOOT1_Pos );
  static constexpr Reg32_t OPTR_nBOOT1         = OPTR_nBOOT1_Msk;
  static constexpr Reg32_t OPTR_SRAM2_PE_Pos   = ( 24U );
  static constexpr Reg32_t OPTR_SRAM2_PE_Msk   = ( 0x1UL << OPTR_SRAM2_PE_Pos );
  static constexpr Reg32_t OPTR_SRAM2_PE       = OPTR_SRAM2_PE_Msk;
  static constexpr Reg32_t OPTR_SRAM2_RST_Pos  = ( 25U );
  static constexpr Reg32_t OPTR_SRAM2_RST_Msk  = ( 0x1UL << OPTR_SRAM2_RST_Pos );
  static constexpr Reg32_t OPTR_SRAM2_RST      = OPTR_SRAM2_RST_Msk;
  static constexpr Reg32_t OPTR_nSWBOOT0_Pos   = ( 26U );
  static constexpr Reg32_t OPTR_nSWBOOT0_Msk   = ( 0x1UL << OPTR_nSWBOOT0_Pos );
  static constexpr Reg32_t OPTR_nSWBOOT0       = OPTR_nSWBOOT0_Msk;
  static constexpr Reg32_t OPTR_nBOOT0_Pos     = ( 27U );
  static constexpr Reg32_t OPTR_nBOOT0_Msk     = ( 0x1UL << OPTR_nBOOT0_Pos );
  static constexpr Reg32_t OPTR_nBOOT0         = OPTR_nBOOT0_Msk;

  /******************  Bits definition for PCROP1SR register  **********/
  static constexpr Reg32_t PCROP1SR_PCROP1_STRT_Pos = ( 0U );
  static constexpr Reg32_t PCROP1SR_PCROP1_STRT_Msk = ( 0x7FFFUL << PCROP1SR_PCROP1_STRT_Pos );
  static constexpr Reg32_t PCROP1SR_PCROP1_STRT     = PCROP1SR_PCROP1_STRT_Msk;

  /******************  Bits definition for PCROP1ER register  ***********/
  static constexpr Reg32_t PCROP1ER_PCROP1_END_Pos = ( 0U );
  static constexpr Reg32_t PCROP1ER_PCROP1_END_Msk = ( 0x7FFFUL << PCROP1ER_PCROP1_END_Pos );
  static constexpr Reg32_t PCROP1ER_PCROP1_END     = PCROP1ER_PCROP1_END_Msk;
  static constexpr Reg32_t PCROP1ER_PCROP_RDP_Pos  = ( 31U );
  static constexpr Reg32_t PCROP1ER_PCROP_RDP_Msk  = ( 0x1UL << PCROP1ER_PCROP_RDP_Pos );
  static constexpr Reg32_t PCROP1ER_PCROP_RDP      = PCROP1ER_PCROP_RDP_Msk;

  /******************  Bits definition for WRP1AR register  ***************/
  static constexpr Reg32_t WRP1AR_WRP1A_STRT_Pos = ( 0U );
  static constexpr Reg32_t WRP1AR_WRP1A_STRT_Msk = ( 0x7FUL << WRP1AR_WRP1A_STRT_Pos );
  static constexpr Reg32_t WRP1AR_WRP1A_STRT     = WRP1AR_WRP1A_STRT_Msk;
  static constexpr Reg32_t WRP1AR_WRP1A_END_Pos  = ( 16U );
  static constexpr Reg32_t WRP1AR_WRP1A_END_Msk  = ( 0x7FUL << WRP1AR_WRP1A_END_Pos );
  static constexpr Reg32_t WRP1AR_WRP1A_END      = WRP1AR_WRP1A_END_Msk;

  /******************  Bits definition for WRPB1R register  ***************/
  static constexpr Reg32_t WRP1BR_WRP1B_STRT_Pos = ( 0U );
  static constexpr Reg32_t WRP1BR_WRP1B_STRT_Msk = ( 0x7FUL << WRP1BR_WRP1B_STRT_Pos );
  static constexpr Reg32_t WRP1BR_WRP1B_STRT     = WRP1BR_WRP1B_STRT_Msk;
  static constexpr Reg32_t WRP1BR_WRP1B_END_Pos  = ( 16U );
  static constexpr Reg32_t WRP1BR_WRP1B_END_Msk  = ( 0x7FUL << WRP1BR_WRP1B_END_Pos );
  static constexpr Reg32_t WRP1BR_WRP1B_END      = WRP1BR_WRP1B_END_Msk;

}    // namespace Thor::LLD::FLASH

#endif /* !THOR_HW_FLASH_REGISTER_STM32L432KC_HPP */