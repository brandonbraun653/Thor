/******************************************************************************
 *  File Name:
 *    hw_sdio_register_stm32f4xxxx.hpp
 *
 *  Description:
 *    SDIO register definitions for the STM32F4xxxx series chips.
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_HW_REGISTER_STM32F4XXXX_HPP
#define THOR_HW_REGISTER_STM32F4XXXX_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Thor/lld/stm32f4x/system/variant/sys_memory_map_stm32f446xx.hpp>

namespace Thor::LLD::SDIO
{
  /*---------------------------------------------------------------------------
  Peripheral Memory Map Addresses
  ---------------------------------------------------------------------------*/
  static constexpr uint32_t SDIO1_BASE_ADDR = Thor::System::MemoryMap::SDMMC_PERIPH_START_ADDRESS;

	/*---------------------------------------------------------------------------
	Register Bit Definitions
	---------------------------------------------------------------------------*/
  /******************  Bit definition for POWER register  ******************/
  static constexpr uint32_t POWER_PWRCTRL_Pos = ( 0U );
  static constexpr uint32_t POWER_PWRCTRL_Msk = ( 0x3UL << POWER_PWRCTRL_Pos );
  static constexpr uint32_t POWER_PWRCTRL     = POWER_PWRCTRL_Msk;
  static constexpr uint32_t POWER_PWRCTRL_0   = ( 0x1UL << POWER_PWRCTRL_Pos );
  static constexpr uint32_t POWER_PWRCTRL_1   = ( 0x2UL << POWER_PWRCTRL_Pos );

  /******************  Bit definition for CLKCR register  ******************/
  static constexpr uint32_t CLKCR_CLKDIV_Pos = ( 0U );
  static constexpr uint32_t CLKCR_CLKDIV_Msk = ( 0xFFUL << CLKCR_CLKDIV_Pos );
  static constexpr uint32_t CLKCR_CLKDIV     = CLKCR_CLKDIV_Msk;
  static constexpr uint32_t CLKCR_CLKEN_Pos  = ( 8U );
  static constexpr uint32_t CLKCR_CLKEN_Msk  = ( 0x1UL << CLKCR_CLKEN_Pos );
  static constexpr uint32_t CLKCR_CLKEN      = CLKCR_CLKEN_Msk;
  static constexpr uint32_t CLKCR_PWRSAV_Pos = ( 9U );
  static constexpr uint32_t CLKCR_PWRSAV_Msk = ( 0x1UL << CLKCR_PWRSAV_Pos );
  static constexpr uint32_t CLKCR_PWRSAV     = CLKCR_PWRSAV_Msk;
  static constexpr uint32_t CLKCR_BYPASS_Pos = ( 10U );
  static constexpr uint32_t CLKCR_BYPASS_Msk = ( 0x1UL << CLKCR_BYPASS_Pos );
  static constexpr uint32_t CLKCR_BYPASS     = CLKCR_BYPASS_Msk;

  static constexpr uint32_t CLKCR_WIDBUS_Pos = ( 11U );
  static constexpr uint32_t CLKCR_WIDBUS_Msk = ( 0x3UL << CLKCR_WIDBUS_Pos );
  static constexpr uint32_t CLKCR_WIDBUS     = CLKCR_WIDBUS_Msk;
  static constexpr uint32_t CLKCR_WIDBUS_0   = ( 0x1UL << CLKCR_WIDBUS_Pos );
  static constexpr uint32_t CLKCR_WIDBUS_1   = ( 0x2UL << CLKCR_WIDBUS_Pos );

  static constexpr uint32_t CLKCR_NEGEDGE_Pos = ( 13U );
  static constexpr uint32_t CLKCR_NEGEDGE_Msk = ( 0x1UL << CLKCR_NEGEDGE_Pos );
  static constexpr uint32_t CLKCR_NEGEDGE     = CLKCR_NEGEDGE_Msk;
  static constexpr uint32_t CLKCR_HWFC_EN_Pos = ( 14U );
  static constexpr uint32_t CLKCR_HWFC_EN_Msk = ( 0x1UL << CLKCR_HWFC_EN_Pos );
  static constexpr uint32_t CLKCR_HWFC_EN     = CLKCR_HWFC_EN_Msk;

  /*******************  Bit definition for ARG register  *******************/
  static constexpr uint32_t ARG_CMDARG_Pos = ( 0U );
  static constexpr uint32_t ARG_CMDARG_Msk = ( 0xFFFFFFFFUL << ARG_CMDARG_Pos );
  static constexpr uint32_t ARG_CMDARG     = ARG_CMDARG_Msk;

  /*******************  Bit definition for CMD register  *******************/
  static constexpr uint32_t CMD_CMDINDEX_Pos = ( 0U );
  static constexpr uint32_t CMD_CMDINDEX_Msk = ( 0x3FUL << CMD_CMDINDEX_Pos );
  static constexpr uint32_t CMD_CMDINDEX     = CMD_CMDINDEX_Msk;

  static constexpr uint32_t CMD_WAITRESP_Pos = ( 6U );
  static constexpr uint32_t CMD_WAITRESP_Msk = ( 0x3UL << CMD_WAITRESP_Pos );
  static constexpr uint32_t CMD_WAITRESP     = CMD_WAITRESP_Msk;
  static constexpr uint32_t CMD_WAITRESP_0   = ( 0x1UL << CMD_WAITRESP_Pos );
  static constexpr uint32_t CMD_WAITRESP_1   = ( 0x2UL << CMD_WAITRESP_Pos );

  static constexpr uint32_t CMD_WAITINT_Pos     = ( 8U );
  static constexpr uint32_t CMD_WAITINT_Msk     = ( 0x1UL << CMD_WAITINT_Pos );
  static constexpr uint32_t CMD_WAITINT         = CMD_WAITINT_Msk;
  static constexpr uint32_t CMD_WAITPEND_Pos    = ( 9U );
  static constexpr uint32_t CMD_WAITPEND_Msk    = ( 0x1UL << CMD_WAITPEND_Pos );
  static constexpr uint32_t CMD_WAITPEND        = CMD_WAITPEND_Msk;
  static constexpr uint32_t CMD_CPSMEN_Pos      = ( 10U );
  static constexpr uint32_t CMD_CPSMEN_Msk      = ( 0x1UL << CMD_CPSMEN_Pos );
  static constexpr uint32_t CMD_CPSMEN          = CMD_CPSMEN_Msk;
  static constexpr uint32_t CMD_SDIOSUSPEND_Pos = ( 11U );
  static constexpr uint32_t CMD_SDIOSUSPEND_Msk = ( 0x1UL << CMD_SDIOSUSPEND_Pos );
  static constexpr uint32_t CMD_SDIOSUSPEND     = CMD_SDIOSUSPEND_Msk;

  /*****************  Bit definition for RESPCMD register  *****************/
  static constexpr uint32_t RESPCMD_RESPCMD_Pos = ( 0U );
  static constexpr uint32_t RESPCMD_RESPCMD_Msk = ( 0x3FUL << RESPCMD_RESPCMD_Pos );
  static constexpr uint32_t RESPCMD_RESPCMD     = RESPCMD_RESPCMD_Msk;

  /******************  Bit definition for RESP0 register  ******************/
  static constexpr uint32_t RESP0_CARDSTATUS0_Pos = ( 0U );
  static constexpr uint32_t RESP0_CARDSTATUS0_Msk = ( 0xFFFFFFFFUL << RESP0_CARDSTATUS0_Pos );
  static constexpr uint32_t RESP0_CARDSTATUS0     = RESP0_CARDSTATUS0_Msk;

  /******************  Bit definition for RESP1 register  ******************/
  static constexpr uint32_t RESP1_CARDSTATUS1_Pos = ( 0U );
  static constexpr uint32_t RESP1_CARDSTATUS1_Msk = ( 0xFFFFFFFFUL << RESP1_CARDSTATUS1_Pos );
  static constexpr uint32_t RESP1_CARDSTATUS1     = RESP1_CARDSTATUS1_Msk;

  /******************  Bit definition for RESP2 register  ******************/
  static constexpr uint32_t RESP2_CARDSTATUS2_Pos = ( 0U );
  static constexpr uint32_t RESP2_CARDSTATUS2_Msk = ( 0xFFFFFFFFUL << RESP2_CARDSTATUS2_Pos );
  static constexpr uint32_t RESP2_CARDSTATUS2     = RESP2_CARDSTATUS2_Msk;

  /******************  Bit definition for RESP3 register  ******************/
  static constexpr uint32_t RESP3_CARDSTATUS3_Pos = ( 0U );
  static constexpr uint32_t RESP3_CARDSTATUS3_Msk = ( 0xFFFFFFFFUL << RESP3_CARDSTATUS3_Pos );
  static constexpr uint32_t RESP3_CARDSTATUS3     = RESP3_CARDSTATUS3_Msk;

  /******************  Bit definition for RESP4 register  ******************/
  static constexpr uint32_t RESP4_CARDSTATUS4_Pos = ( 0U );
  static constexpr uint32_t RESP4_CARDSTATUS4_Msk = ( 0xFFFFFFFFUL << RESP4_CARDSTATUS4_Pos );
  static constexpr uint32_t RESP4_CARDSTATUS4     = RESP4_CARDSTATUS4_Msk;

  /******************  Bit definition for DTIMER register  *****************/
  static constexpr uint32_t DTIMER_DATATIME_Pos = ( 0U );
  static constexpr uint32_t DTIMER_DATATIME_Msk = ( 0xFFFFFFFFUL << DTIMER_DATATIME_Pos );
  static constexpr uint32_t DTIMER_DATATIME     = DTIMER_DATATIME_Msk;

  /******************  Bit definition for DLEN register  *******************/
  static constexpr uint32_t DLEN_DATALENGTH_Pos = ( 0U );
  static constexpr uint32_t DLEN_DATALENGTH_Msk = ( 0x1FFFFFFUL << DLEN_DATALENGTH_Pos );
  static constexpr uint32_t DLEN_DATALENGTH     = DLEN_DATALENGTH_Msk;

  /******************  Bit definition for DCTRL register  ******************/
  static constexpr uint32_t DCTRL_DTEN_Pos   = ( 0U );
  static constexpr uint32_t DCTRL_DTEN_Msk   = ( 0x1UL << DCTRL_DTEN_Pos );
  static constexpr uint32_t DCTRL_DTEN       = DCTRL_DTEN_Msk;
  static constexpr uint32_t DCTRL_DTDIR_Pos  = ( 1U );
  static constexpr uint32_t DCTRL_DTDIR_Msk  = ( 0x1UL << DCTRL_DTDIR_Pos );
  static constexpr uint32_t DCTRL_DTDIR      = DCTRL_DTDIR_Msk;
  static constexpr uint32_t DCTRL_DTMODE_Pos = ( 2U );
  static constexpr uint32_t DCTRL_DTMODE_Msk = ( 0x1UL << DCTRL_DTMODE_Pos );
  static constexpr uint32_t DCTRL_DTMODE     = DCTRL_DTMODE_Msk;
  static constexpr uint32_t DCTRL_DMAEN_Pos  = ( 3U );
  static constexpr uint32_t DCTRL_DMAEN_Msk  = ( 0x1UL << DCTRL_DMAEN_Pos );
  static constexpr uint32_t DCTRL_DMAEN      = DCTRL_DMAEN_Msk;

  static constexpr uint32_t DCTRL_DBLOCKSIZE_Pos = ( 4U );
  static constexpr uint32_t DCTRL_DBLOCKSIZE_Msk = ( 0xFUL << DCTRL_DBLOCKSIZE_Pos );
  static constexpr uint32_t DCTRL_DBLOCKSIZE     = DCTRL_DBLOCKSIZE_Msk;
  static constexpr uint32_t DCTRL_DBLOCKSIZE_0   = ( 0x1UL << DCTRL_DBLOCKSIZE_Pos );
  static constexpr uint32_t DCTRL_DBLOCKSIZE_1   = ( 0x2UL << DCTRL_DBLOCKSIZE_Pos );
  static constexpr uint32_t DCTRL_DBLOCKSIZE_2   = ( 0x4UL << DCTRL_DBLOCKSIZE_Pos );
  static constexpr uint32_t DCTRL_DBLOCKSIZE_3   = ( 0x8UL << DCTRL_DBLOCKSIZE_Pos );

  static constexpr uint32_t DCTRL_RWSTART_Pos = ( 8U );
  static constexpr uint32_t DCTRL_RWSTART_Msk = ( 0x1UL << DCTRL_RWSTART_Pos );
  static constexpr uint32_t DCTRL_RWSTART     = DCTRL_RWSTART_Msk;
  static constexpr uint32_t DCTRL_RWSTOP_Pos  = ( 9U );
  static constexpr uint32_t DCTRL_RWSTOP_Msk  = ( 0x1UL << DCTRL_RWSTOP_Pos );
  static constexpr uint32_t DCTRL_RWSTOP      = DCTRL_RWSTOP_Msk;
  static constexpr uint32_t DCTRL_RWMOD_Pos   = ( 10U );
  static constexpr uint32_t DCTRL_RWMOD_Msk   = ( 0x1UL << DCTRL_RWMOD_Pos );
  static constexpr uint32_t DCTRL_RWMOD       = DCTRL_RWMOD_Msk;
  static constexpr uint32_t DCTRL_SDIOEN_Pos  = ( 11U );
  static constexpr uint32_t DCTRL_SDIOEN_Msk  = ( 0x1UL << DCTRL_SDIOEN_Pos );
  static constexpr uint32_t DCTRL_SDIOEN      = DCTRL_SDIOEN_Msk;

  /******************  Bit definition for DCOUNT register  *****************/
  static constexpr uint32_t DCOUNT_DATACOUNT_Pos = ( 0U );
  static constexpr uint32_t DCOUNT_DATACOUNT_Msk = ( 0x1FFFFFFUL << DCOUNT_DATACOUNT_Pos );
  static constexpr uint32_t DCOUNT_DATACOUNT     = DCOUNT_DATACOUNT_Msk;

  /******************  Bit definition for STA register  ********************/
  static constexpr uint32_t STA_CCRCFAIL_Pos = ( 0U );
  static constexpr uint32_t STA_CCRCFAIL_Msk = ( 0x1UL << STA_CCRCFAIL_Pos );
  static constexpr uint32_t STA_CCRCFAIL     = STA_CCRCFAIL_Msk;
  static constexpr uint32_t STA_DCRCFAIL_Pos = ( 1U );
  static constexpr uint32_t STA_DCRCFAIL_Msk = ( 0x1UL << STA_DCRCFAIL_Pos );
  static constexpr uint32_t STA_DCRCFAIL     = STA_DCRCFAIL_Msk;
  static constexpr uint32_t STA_CTIMEOUT_Pos = ( 2U );
  static constexpr uint32_t STA_CTIMEOUT_Msk = ( 0x1UL << STA_CTIMEOUT_Pos );
  static constexpr uint32_t STA_CTIMEOUT     = STA_CTIMEOUT_Msk;
  static constexpr uint32_t STA_DTIMEOUT_Pos = ( 3U );
  static constexpr uint32_t STA_DTIMEOUT_Msk = ( 0x1UL << STA_DTIMEOUT_Pos );
  static constexpr uint32_t STA_DTIMEOUT     = STA_DTIMEOUT_Msk;
  static constexpr uint32_t STA_TXUNDERR_Pos = ( 4U );
  static constexpr uint32_t STA_TXUNDERR_Msk = ( 0x1UL << STA_TXUNDERR_Pos );
  static constexpr uint32_t STA_TXUNDERR     = STA_TXUNDERR_Msk;
  static constexpr uint32_t STA_RXOVERR_Pos  = ( 5U );
  static constexpr uint32_t STA_RXOVERR_Msk  = ( 0x1UL << STA_RXOVERR_Pos );
  static constexpr uint32_t STA_RXOVERR      = STA_RXOVERR_Msk;
  static constexpr uint32_t STA_CMDREND_Pos  = ( 6U );
  static constexpr uint32_t STA_CMDREND_Msk  = ( 0x1UL << STA_CMDREND_Pos );
  static constexpr uint32_t STA_CMDREND      = STA_CMDREND_Msk;
  static constexpr uint32_t STA_CMDSENT_Pos  = ( 7U );
  static constexpr uint32_t STA_CMDSENT_Msk  = ( 0x1UL << STA_CMDSENT_Pos );
  static constexpr uint32_t STA_CMDSENT      = STA_CMDSENT_Msk;
  static constexpr uint32_t STA_DATAEND_Pos  = ( 8U );
  static constexpr uint32_t STA_DATAEND_Msk  = ( 0x1UL << STA_DATAEND_Pos );
  static constexpr uint32_t STA_DATAEND      = STA_DATAEND_Msk;
  static constexpr uint32_t STA_DBCKEND_Pos  = ( 10U );
  static constexpr uint32_t STA_DBCKEND_Msk  = ( 0x1UL << STA_DBCKEND_Pos );
  static constexpr uint32_t STA_DBCKEND      = STA_DBCKEND_Msk;
  static constexpr uint32_t STA_CMDACT_Pos   = ( 11U );
  static constexpr uint32_t STA_CMDACT_Msk   = ( 0x1UL << STA_CMDACT_Pos );
  static constexpr uint32_t STA_CMDACT       = STA_CMDACT_Msk;
  static constexpr uint32_t STA_TXACT_Pos    = ( 12U );
  static constexpr uint32_t STA_TXACT_Msk    = ( 0x1UL << STA_TXACT_Pos );
  static constexpr uint32_t STA_TXACT        = STA_TXACT_Msk;
  static constexpr uint32_t STA_RXACT_Pos    = ( 13U );
  static constexpr uint32_t STA_RXACT_Msk    = ( 0x1UL << STA_RXACT_Pos );
  static constexpr uint32_t STA_RXACT        = STA_RXACT_Msk;
  static constexpr uint32_t STA_TXFIFOHE_Pos = ( 14U );
  static constexpr uint32_t STA_TXFIFOHE_Msk = ( 0x1UL << STA_TXFIFOHE_Pos );
  static constexpr uint32_t STA_TXFIFOHE     = STA_TXFIFOHE_Msk;
  static constexpr uint32_t STA_RXFIFOHF_Pos = ( 15U );
  static constexpr uint32_t STA_RXFIFOHF_Msk = ( 0x1UL << STA_RXFIFOHF_Pos );
  static constexpr uint32_t STA_RXFIFOHF     = STA_RXFIFOHF_Msk;
  static constexpr uint32_t STA_TXFIFOF_Pos  = ( 16U );
  static constexpr uint32_t STA_TXFIFOF_Msk  = ( 0x1UL << STA_TXFIFOF_Pos );
  static constexpr uint32_t STA_TXFIFOF      = STA_TXFIFOF_Msk;
  static constexpr uint32_t STA_RXFIFOF_Pos  = ( 17U );
  static constexpr uint32_t STA_RXFIFOF_Msk  = ( 0x1UL << STA_RXFIFOF_Pos );
  static constexpr uint32_t STA_RXFIFOF      = STA_RXFIFOF_Msk;
  static constexpr uint32_t STA_TXFIFOE_Pos  = ( 18U );
  static constexpr uint32_t STA_TXFIFOE_Msk  = ( 0x1UL << STA_TXFIFOE_Pos );
  static constexpr uint32_t STA_TXFIFOE      = STA_TXFIFOE_Msk;
  static constexpr uint32_t STA_RXFIFOE_Pos  = ( 19U );
  static constexpr uint32_t STA_RXFIFOE_Msk  = ( 0x1UL << STA_RXFIFOE_Pos );
  static constexpr uint32_t STA_RXFIFOE      = STA_RXFIFOE_Msk;
  static constexpr uint32_t STA_TXDAVL_Pos   = ( 20U );
  static constexpr uint32_t STA_TXDAVL_Msk   = ( 0x1UL << STA_TXDAVL_Pos );
  static constexpr uint32_t STA_TXDAVL       = STA_TXDAVL_Msk;
  static constexpr uint32_t STA_RXDAVL_Pos   = ( 21U );
  static constexpr uint32_t STA_RXDAVL_Msk   = ( 0x1UL << STA_RXDAVL_Pos );
  static constexpr uint32_t STA_RXDAVL       = STA_RXDAVL_Msk;
  static constexpr uint32_t STA_SDIOIT_Pos   = ( 22U );
  static constexpr uint32_t STA_SDIOIT_Msk   = ( 0x1UL << STA_SDIOIT_Pos );
  static constexpr uint32_t STA_SDIOIT       = STA_SDIOIT_Msk;

  /*******************  Bit definition for ICR register  *******************/
  static constexpr uint32_t ICR_CCRCFAILC_Pos = ( 0U );
  static constexpr uint32_t ICR_CCRCFAILC_Msk = ( 0x1UL << ICR_CCRCFAILC_Pos );
  static constexpr uint32_t ICR_CCRCFAILC     = ICR_CCRCFAILC_Msk;
  static constexpr uint32_t ICR_DCRCFAILC_Pos = ( 1U );
  static constexpr uint32_t ICR_DCRCFAILC_Msk = ( 0x1UL << ICR_DCRCFAILC_Pos );
  static constexpr uint32_t ICR_DCRCFAILC     = ICR_DCRCFAILC_Msk;
  static constexpr uint32_t ICR_CTIMEOUTC_Pos = ( 2U );
  static constexpr uint32_t ICR_CTIMEOUTC_Msk = ( 0x1UL << ICR_CTIMEOUTC_Pos );
  static constexpr uint32_t ICR_CTIMEOUTC     = ICR_CTIMEOUTC_Msk;
  static constexpr uint32_t ICR_DTIMEOUTC_Pos = ( 3U );
  static constexpr uint32_t ICR_DTIMEOUTC_Msk = ( 0x1UL << ICR_DTIMEOUTC_Pos );
  static constexpr uint32_t ICR_DTIMEOUTC     = ICR_DTIMEOUTC_Msk;
  static constexpr uint32_t ICR_TXUNDERRC_Pos = ( 4U );
  static constexpr uint32_t ICR_TXUNDERRC_Msk = ( 0x1UL << ICR_TXUNDERRC_Pos );
  static constexpr uint32_t ICR_TXUNDERRC     = ICR_TXUNDERRC_Msk;
  static constexpr uint32_t ICR_RXOVERRC_Pos  = ( 5U );
  static constexpr uint32_t ICR_RXOVERRC_Msk  = ( 0x1UL << ICR_RXOVERRC_Pos );
  static constexpr uint32_t ICR_RXOVERRC      = ICR_RXOVERRC_Msk;
  static constexpr uint32_t ICR_CMDRENDC_Pos  = ( 6U );
  static constexpr uint32_t ICR_CMDRENDC_Msk  = ( 0x1UL << ICR_CMDRENDC_Pos );
  static constexpr uint32_t ICR_CMDRENDC      = ICR_CMDRENDC_Msk;
  static constexpr uint32_t ICR_CMDSENTC_Pos  = ( 7U );
  static constexpr uint32_t ICR_CMDSENTC_Msk  = ( 0x1UL << ICR_CMDSENTC_Pos );
  static constexpr uint32_t ICR_CMDSENTC      = ICR_CMDSENTC_Msk;
  static constexpr uint32_t ICR_DATAENDC_Pos  = ( 8U );
  static constexpr uint32_t ICR_DATAENDC_Msk  = ( 0x1UL << ICR_DATAENDC_Pos );
  static constexpr uint32_t ICR_DATAENDC      = ICR_DATAENDC_Msk;
  static constexpr uint32_t ICR_DBCKENDC_Pos  = ( 10U );
  static constexpr uint32_t ICR_DBCKENDC_Msk  = ( 0x1UL << ICR_DBCKENDC_Pos );
  static constexpr uint32_t ICR_DBCKENDC      = ICR_DBCKENDC_Msk;
  static constexpr uint32_t ICR_SDIOITC_Pos   = ( 22U );
  static constexpr uint32_t ICR_SDIOITC_Msk   = ( 0x1UL << ICR_SDIOITC_Pos );
  static constexpr uint32_t ICR_SDIOITC       = ICR_SDIOITC_Msk;

  /******************  Bit definition for MASK register  *******************/
  static constexpr uint32_t MASK_CCRCFAILIE_Pos = ( 0U );
  static constexpr uint32_t MASK_CCRCFAILIE_Msk = ( 0x1UL << MASK_CCRCFAILIE_Pos );
  static constexpr uint32_t MASK_CCRCFAILIE     = MASK_CCRCFAILIE_Msk;
  static constexpr uint32_t MASK_DCRCFAILIE_Pos = ( 1U );
  static constexpr uint32_t MASK_DCRCFAILIE_Msk = ( 0x1UL << MASK_DCRCFAILIE_Pos );
  static constexpr uint32_t MASK_DCRCFAILIE     = MASK_DCRCFAILIE_Msk;
  static constexpr uint32_t MASK_CTIMEOUTIE_Pos = ( 2U );
  static constexpr uint32_t MASK_CTIMEOUTIE_Msk = ( 0x1UL << MASK_CTIMEOUTIE_Pos );
  static constexpr uint32_t MASK_CTIMEOUTIE     = MASK_CTIMEOUTIE_Msk;
  static constexpr uint32_t MASK_DTIMEOUTIE_Pos = ( 3U );
  static constexpr uint32_t MASK_DTIMEOUTIE_Msk = ( 0x1UL << MASK_DTIMEOUTIE_Pos );
  static constexpr uint32_t MASK_DTIMEOUTIE     = MASK_DTIMEOUTIE_Msk;
  static constexpr uint32_t MASK_TXUNDERRIE_Pos = ( 4U );
  static constexpr uint32_t MASK_TXUNDERRIE_Msk = ( 0x1UL << MASK_TXUNDERRIE_Pos );
  static constexpr uint32_t MASK_TXUNDERRIE     = MASK_TXUNDERRIE_Msk;
  static constexpr uint32_t MASK_RXOVERRIE_Pos  = ( 5U );
  static constexpr uint32_t MASK_RXOVERRIE_Msk  = ( 0x1UL << MASK_RXOVERRIE_Pos );
  static constexpr uint32_t MASK_RXOVERRIE      = MASK_RXOVERRIE_Msk;
  static constexpr uint32_t MASK_CMDRENDIE_Pos  = ( 6U );
  static constexpr uint32_t MASK_CMDRENDIE_Msk  = ( 0x1UL << MASK_CMDRENDIE_Pos );
  static constexpr uint32_t MASK_CMDRENDIE      = MASK_CMDRENDIE_Msk;
  static constexpr uint32_t MASK_CMDSENTIE_Pos  = ( 7U );
  static constexpr uint32_t MASK_CMDSENTIE_Msk  = ( 0x1UL << MASK_CMDSENTIE_Pos );
  static constexpr uint32_t MASK_CMDSENTIE      = MASK_CMDSENTIE_Msk;
  static constexpr uint32_t MASK_DATAENDIE_Pos  = ( 8U );
  static constexpr uint32_t MASK_DATAENDIE_Msk  = ( 0x1UL << MASK_DATAENDIE_Pos );
  static constexpr uint32_t MASK_DATAENDIE      = MASK_DATAENDIE_Msk;
  static constexpr uint32_t MASK_DBCKENDIE_Pos  = ( 10U );
  static constexpr uint32_t MASK_DBCKENDIE_Msk  = ( 0x1UL << MASK_DBCKENDIE_Pos );
  static constexpr uint32_t MASK_DBCKENDIE      = MASK_DBCKENDIE_Msk;
  static constexpr uint32_t MASK_CMDACTIE_Pos   = ( 11U );
  static constexpr uint32_t MASK_CMDACTIE_Msk   = ( 0x1UL << MASK_CMDACTIE_Pos );
  static constexpr uint32_t MASK_CMDACTIE       = MASK_CMDACTIE_Msk;
  static constexpr uint32_t MASK_TXACTIE_Pos    = ( 12U );
  static constexpr uint32_t MASK_TXACTIE_Msk    = ( 0x1UL << MASK_TXACTIE_Pos );
  static constexpr uint32_t MASK_TXACTIE        = MASK_TXACTIE_Msk;
  static constexpr uint32_t MASK_RXACTIE_Pos    = ( 13U );
  static constexpr uint32_t MASK_RXACTIE_Msk    = ( 0x1UL << MASK_RXACTIE_Pos );
  static constexpr uint32_t MASK_RXACTIE        = MASK_RXACTIE_Msk;
  static constexpr uint32_t MASK_TXFIFOHEIE_Pos = ( 14U );
  static constexpr uint32_t MASK_TXFIFOHEIE_Msk = ( 0x1UL << MASK_TXFIFOHEIE_Pos );
  static constexpr uint32_t MASK_TXFIFOHEIE     = MASK_TXFIFOHEIE_Msk;
  static constexpr uint32_t MASK_RXFIFOHFIE_Pos = ( 15U );
  static constexpr uint32_t MASK_RXFIFOHFIE_Msk = ( 0x1UL << MASK_RXFIFOHFIE_Pos );
  static constexpr uint32_t MASK_RXFIFOHFIE     = MASK_RXFIFOHFIE_Msk;
  static constexpr uint32_t MASK_TXFIFOFIE_Pos  = ( 16U );
  static constexpr uint32_t MASK_TXFIFOFIE_Msk  = ( 0x1UL << MASK_TXFIFOFIE_Pos );
  static constexpr uint32_t MASK_TXFIFOFIE      = MASK_TXFIFOFIE_Msk;
  static constexpr uint32_t MASK_RXFIFOFIE_Pos  = ( 17U );
  static constexpr uint32_t MASK_RXFIFOFIE_Msk  = ( 0x1UL << MASK_RXFIFOFIE_Pos );
  static constexpr uint32_t MASK_RXFIFOFIE      = MASK_RXFIFOFIE_Msk;
  static constexpr uint32_t MASK_TXFIFOEIE_Pos  = ( 18U );
  static constexpr uint32_t MASK_TXFIFOEIE_Msk  = ( 0x1UL << MASK_TXFIFOEIE_Pos );
  static constexpr uint32_t MASK_TXFIFOEIE      = MASK_TXFIFOEIE_Msk;
  static constexpr uint32_t MASK_RXFIFOEIE_Pos  = ( 19U );
  static constexpr uint32_t MASK_RXFIFOEIE_Msk  = ( 0x1UL << MASK_RXFIFOEIE_Pos );
  static constexpr uint32_t MASK_RXFIFOEIE      = MASK_RXFIFOEIE_Msk;
  static constexpr uint32_t MASK_TXDAVLIE_Pos   = ( 20U );
  static constexpr uint32_t MASK_TXDAVLIE_Msk   = ( 0x1UL << MASK_TXDAVLIE_Pos );
  static constexpr uint32_t MASK_TXDAVLIE       = MASK_TXDAVLIE_Msk;
  static constexpr uint32_t MASK_RXDAVLIE_Pos   = ( 21U );
  static constexpr uint32_t MASK_RXDAVLIE_Msk   = ( 0x1UL << MASK_RXDAVLIE_Pos );
  static constexpr uint32_t MASK_RXDAVLIE       = MASK_RXDAVLIE_Msk;
  static constexpr uint32_t MASK_SDIOITIE_Pos   = ( 22U );
  static constexpr uint32_t MASK_SDIOITIE_Msk   = ( 0x1UL << MASK_SDIOITIE_Pos );
  static constexpr uint32_t MASK_SDIOITIE       = MASK_SDIOITIE_Msk;

  /*****************  Bit definition for FIFOCNT register  *****************/
  static constexpr uint32_t FIFOCNT_FIFOCOUNT_Pos = ( 0U );
  static constexpr uint32_t FIFOCNT_FIFOCOUNT_Msk = ( 0xFFFFFFUL << FIFOCNT_FIFOCOUNT_Pos );
  static constexpr uint32_t FIFOCNT_FIFOCOUNT     = FIFOCNT_FIFOCOUNT_Msk;

  /******************  Bit definition for FIFO register  *******************/
  static constexpr uint32_t FIFO_FIFODATA_Pos = ( 0U );
  static constexpr uint32_t FIFO_FIFODATA_Msk = ( 0xFFFFFFFFUL << FIFO_FIFODATA_Pos );
  static constexpr uint32_t FIFO_FIFODATA     = FIFO_FIFODATA_Msk;

}    // namespace Thor::LLD::SDIO

#endif /* !THOR_HW_REGISTER_STM32F4XXXX_HPP */
