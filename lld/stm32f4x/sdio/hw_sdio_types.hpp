/******************************************************************************
 *  File Name:
 *    hw_sdio_types.hpp
 *
 *  Description:
 *    Type descriptors for SDIO
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_HW_SDIO_TYPES_HPP
#define THOR_HW_SDIO_TYPES_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Thor/lld/common/registers/field_accessor.hpp>
#include <cstdint>

namespace Thor::LLD::SDIO
{
  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct RegisterMap
  {
    volatile uint32_t POWER;           /**< SDIO power control register,    Address offset: 0x00 */
    volatile uint32_t CLKCR;           /**< SDIO clock control register,    Address offset: 0x04 */
    volatile uint32_t ARG;             /**< SDIO argument register,         Address offset: 0x08 */
    volatile uint32_t CMD;             /**< SDIO command register,          Address offset: 0x0C */
    volatile uint32_t RESPCMD;         /**< SDIO command response register, Address offset: 0x10 */
    volatile uint32_t RESP1;           /**< SDIO response 1 register,       Address offset: 0x14 */
    volatile uint32_t RESP2;           /**< SDIO response 2 register,       Address offset: 0x18 */
    volatile uint32_t RESP3;           /**< SDIO response 3 register,       Address offset: 0x1C */
    volatile uint32_t RESP4;           /**< SDIO response 4 register,       Address offset: 0x20 */
    volatile uint32_t DTIMER;          /**< SDIO data timer register,       Address offset: 0x24 */
    volatile uint32_t DLEN;            /**< SDIO data length register,      Address offset: 0x28 */
    volatile uint32_t DCTRL;           /**< SDIO data control register,     Address offset: 0x2C */
    volatile uint32_t DCOUNT;          /**< SDIO data counter register,     Address offset: 0x30 */
    volatile uint32_t STA;             /**< SDIO status register,           Address offset: 0x34 */
    volatile uint32_t ICR;             /**< SDIO interrupt clear register,  Address offset: 0x38 */
    volatile uint32_t MASK;            /**< SDIO mask register,             Address offset: 0x3C */
    uint32_t          RESERVED0[ 2 ];  /**< Reserved, 0x40-0x44                                  */
    volatile uint32_t FIFOCNT;         /**< SDIO FIFO counter register,     Address offset: 0x48 */
    uint32_t          RESERVED1[ 13 ]; /**< Reserved, 0x4C-0x7C                                  */
    volatile uint32_t FIFO;            /**< SDIO data FIFO register,        Address offset: 0x80 */
  };

  /*-----------------------------------------------------------------------------
  Classes
  -----------------------------------------------------------------------------*/
  /*-------------------------------------------------------------------
  Power Control Register
  -------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, POWER, POWER_PWRCTRL_Msk, PWRCTRL, BIT_ACCESS_RW );
  static constexpr uint32_t POWER_PWRCTRL_OFF = 0u;
  static constexpr uint32_t POWER_PWRCTRL_ON  = POWER_PWRCTRL_1 | POWER_PWRCTRL_0;

  /*-------------------------------------------------------------------
  Clock Control Register
  -------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CLKCR, CLKCR_HWFC_EN_Msk, HWFCEN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CLKCR, CLKCR_NEGEDGE_Msk, NEGEDGE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CLKCR, CLKCR_WIDBUS_Msk, WIDBUS, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CLKCR, CLKCR_BYPASS_Msk, BYPASS, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CLKCR, CLKCR_PWRSAV_Msk, PWRSAV, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CLKCR, CLKCR_CLKDIV_Msk, CLKDIV, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CLKCR, CLKCR_CLKEN_Msk, CLKEN, BIT_ACCESS_RW );

  /*-------------------------------------------------------------------
  Argument Register
  -------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, ARG, ARG_CMDARG_Msk, CMDARG, BIT_ACCESS_RW );

  /*-------------------------------------------------------------------
  Command Register
  -------------------------------------------------------------------*/
  /* WAITRESP */
  REG_ACCESSOR( RegisterMap, CMD, CMD_WAITRESP_Msk, WAITRESP, BIT_ACCESS_RW );
  static constexpr uint32_t CMD_RESPONSE_NO    = 0u;
  static constexpr uint32_t CMD_RESPONSE_SHORT = CMD_WAITRESP_0;
  static constexpr uint32_t CMD_RESPONSE_LONG  = CMD_WAITRESP_1 | CMD_WAITRESP_0;

  REG_ACCESSOR( RegisterMap, CMD, CMD_CMDINDEX_Msk, CMDINDEX, BIT_ACCESS_RW );

  /* WAIT<INT/PEND>*/
  REG_ACCESSOR( RegisterMap, CMD, CMD_WAITINT_Msk, WAITINT, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CMD, CMD_WAITPEND_Msk, WAITPEND, BIT_ACCESS_RW );
  static constexpr uint32_t CMD_WAIT_NO   = 0u;
  static constexpr uint32_t CMD_WAIT_IT   = CMD_WAITINT;
  static constexpr uint32_t CMD_WAIT_PEND = CMD_WAITPEND;

  /* CPSMEN */
  REG_ACCESSOR( RegisterMap, CMD, CMD_CPSMEN_Msk, CPSMEN, BIT_ACCESS_RW );
  static constexpr uint32_t CMD_CPSM_DISABLE = 0u;
  static constexpr uint32_t CMD_CPSM_ENABLE  = CMD_CPSMEN;

  REG_ACCESSOR( RegisterMap, CMD, CMD_SDIOSUSPEND_Msk, SDIOSUSPEND, BIT_ACCESS_RW );

  /*-------------------------------------------------------------------
  Command Response Register
  -------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, RESPCMD, RESPCMD_RESPCMD_Msk, CMDRESP, BIT_ACCESS_RO );

  /*-------------------------------------------------------------------
  Response 1 Register
  -------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, RESP1, RESP1_CARDSTATUS1_Msk, CARDSTATUS1, BIT_ACCESS_RO );

  /*-------------------------------------------------------------------
  Response 2 Register
  -------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, RESP2, RESP2_CARDSTATUS2_Msk, CARDSTATUS2, BIT_ACCESS_RO );

  /*-------------------------------------------------------------------
  Response 3 Register
  -------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, RESP3, RESP3_CARDSTATUS3_Msk, CARDSTATUS3, BIT_ACCESS_RO );

  /*-------------------------------------------------------------------
  Response 4 Register
  -------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, RESP4, RESP4_CARDSTATUS4_Msk, CARDSTATUS4, BIT_ACCESS_RO );

  /*-------------------------------------------------------------------
  Data Timer Register
  -------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, DTIMER, DTIMER_DATATIME_Msk, DATATIME, BIT_ACCESS_RW );

  /*-------------------------------------------------------------------
  Data Length Register
  -------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, DLEN, DLEN_DATALENGTH_Msk, DATALENGTH, BIT_ACCESS_RW );

  /*-------------------------------------------------------------------
  Data Control Register
  -------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, DCTRL, DCTRL_DTEN_Msk, DTEN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, DCTRL, DCTRL_DTDIR_Msk, DTDIR, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, DCTRL, DCTRL_DTMODE_Msk, DTMODE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, DCTRL, DCTRL_DMAEN_Msk, DMAEN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, DCTRL, DCTRL_DBLOCKSIZE_Msk, DBLOCKSIZE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, DCTRL, DCTRL_RWSTART_Msk, RWSTART, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, DCTRL, DCTRL_RWSTOP_Msk, RWSTOP, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, DCTRL, DCTRL_RWMOD_Msk, RWMOD, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, DCTRL, DCTRL_SDIOEN_Msk, SDIOEN, BIT_ACCESS_RW );

  /*-------------------------------------------------------------------
  Data Counter Register
  -------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, DCOUNT, DCOUNT_DATACOUNT_Msk, DATACOUNT, BIT_ACCESS_RO );

  /*-------------------------------------------------------------------
  Status Register
  -------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, STA, STA_ALL_Msk, STA_ALL, BIT_ACCESS_RO );

  REG_ACCESSOR( RegisterMap, STA, STA_CCRCFAIL_Msk, CCRCFAIL, BIT_ACCESS_RO );
  REG_ACCESSOR( RegisterMap, STA, STA_DCRCFAIL_Msk, DCRCFAIL, BIT_ACCESS_RO );
  REG_ACCESSOR( RegisterMap, STA, STA_CTIMEOUT_Msk, CTIMEOUT, BIT_ACCESS_RO );
  REG_ACCESSOR( RegisterMap, STA, STA_DTIMEOUT_Msk, DTIMEOUT, BIT_ACCESS_RO );
  REG_ACCESSOR( RegisterMap, STA, STA_TXUNDERR_Msk, TXUNDERR, BIT_ACCESS_RO );
  REG_ACCESSOR( RegisterMap, STA, STA_RXOVERR_Msk, RXOVERR, BIT_ACCESS_RO );
  REG_ACCESSOR( RegisterMap, STA, STA_CMDREND_Msk, CMDREND, BIT_ACCESS_RO );
  REG_ACCESSOR( RegisterMap, STA, STA_CMDSENT_Msk, CMDSENT, BIT_ACCESS_RO );
  REG_ACCESSOR( RegisterMap, STA, STA_DATAEND_Msk, DATAEND, BIT_ACCESS_RO );
  REG_ACCESSOR( RegisterMap, STA, STA_DBCKEND_Msk, DBCKEND, BIT_ACCESS_RO );
  REG_ACCESSOR( RegisterMap, STA, STA_CMDACT_Msk, CMDACT, BIT_ACCESS_RO );
  REG_ACCESSOR( RegisterMap, STA, STA_TXACT_Msk, TXACT, BIT_ACCESS_RO );
  REG_ACCESSOR( RegisterMap, STA, STA_RXACT_Msk, RXACT, BIT_ACCESS_RO );
  REG_ACCESSOR( RegisterMap, STA, STA_TXFIFOHE_Msk, TXFIFOHE, BIT_ACCESS_RO );
  REG_ACCESSOR( RegisterMap, STA, STA_RXFIFOHF_Msk, RXFIFOHF, BIT_ACCESS_RO );
  REG_ACCESSOR( RegisterMap, STA, STA_TXFIFOF_Msk, TXFIFOF, BIT_ACCESS_RO );
  REG_ACCESSOR( RegisterMap, STA, STA_RXFIFOF_Msk, RXFIFOF, BIT_ACCESS_RO );
  REG_ACCESSOR( RegisterMap, STA, STA_TXFIFOE_Msk, TXFIFOE, BIT_ACCESS_RO );
  REG_ACCESSOR( RegisterMap, STA, STA_RXFIFOE_Msk, RXFIFOE, BIT_ACCESS_RO );
  REG_ACCESSOR( RegisterMap, STA, STA_TXDAVL_Msk, TXDAVL, BIT_ACCESS_RO );
  REG_ACCESSOR( RegisterMap, STA, STA_RXDAVL_Msk, RXDAVL, BIT_ACCESS_RO );

  /*-------------------------------------------------------------------
  Interrupt Clear Register
  -------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, ICR, ICR_ALL_Msk, ICR_ALL, BIT_ACCESS_RW );

  REG_ACCESSOR( RegisterMap, ICR, ICR_CCRCFAILC_Msk, CCRCFAILC, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, ICR, ICR_DCRCFAILC_Msk, DCRCFAILC, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, ICR, ICR_CTIMEOUTC_Msk, CTIMEOUTC, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, ICR, ICR_DTIMEOUTC_Msk, DTIMEOUTC, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, ICR, ICR_TXUNDERRC_Msk, TXUNDERRC, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, ICR, ICR_RXOVERRC_Msk, RXOVERRC, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, ICR, ICR_CMDRENDC_Msk, CMDRENDC, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, ICR, ICR_CMDSENTC_Msk, CMDSENTC, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, ICR, ICR_DATAENDC_Msk, DATAENDC, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, ICR, ICR_DBCKENDC_Msk, DBCKENDC, BIT_ACCESS_RW );

  /*-------------------------------------------------------------------
  Mask Register
  -------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, MASK, MASK_CCRCFAILIE_Msk, CCRCFAILIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, MASK, MASK_DCRCFAILIE_Msk, DCRCFAILIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, MASK, MASK_CTIMEOUTIE_Msk, CTIMEOUTIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, MASK, MASK_DTIMEOUTIE_Msk, DTIMEOUTIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, MASK, MASK_TXUNDERRIE_Msk, TXUNDERRIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, MASK, MASK_RXOVERRIE_Msk, RXOVERRIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, MASK, MASK_CMDRENDIE_Msk, CMDRENDIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, MASK, MASK_CMDSENTIE_Msk, CMDSENTIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, MASK, MASK_DATAENDIE_Msk, DATAENDIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, MASK, MASK_DBCKENDIE_Msk, DBCKENDIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, MASK, MASK_TXFIFOHEIE_Msk, TXFIFOHEIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, MASK, MASK_RXFIFOHFIE_Msk, RXFIFOHFIE, BIT_ACCESS_RW );

  /*-------------------------------------------------------------------
  FIFO Counter Register
  -------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, FIFOCNT, FIFOCNT_FIFOCOUNT_Msk, FIFOCOUNT, BIT_ACCESS_RO );

  /*-------------------------------------------------------------------
  Data FIFO Register
  -------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, FIFO, FIFO_FIFODATA_Msk, FIFODATA, BIT_ACCESS_RW );

}    // namespace Thor::LLD::SDIO

#endif /* !THOR_HW_SDIO_TYPES_HPP */
