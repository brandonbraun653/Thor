/********************************************************************************
 *  File Name:
 *    hw_usb_register_stm32l4xxxx.hpp
 *
 *  Description:
 *    USB register definitions for the STM32L4xxxx series chips.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_HW_USB_REGISTER_STM32L4XXXX_HPP
#define THOR_HW_USB_REGISTER_STM32L4XXXX_HPP

/* C++ Includes */
#include <cstdint>

/* Chimera Includes */
#include <Chimera/common>

/* Driver Includes */
#include <Thor/lld/stm32l4x/system/variant/sys_memory_map_stm32l432kc.hpp>


namespace Thor::LLD::USB
{
  /*-------------------------------------------------
  Peripheral Instance Memory Map Base
  -------------------------------------------------*/
  static constexpr uint32_t USB1_BASE_ADDR = Thor::System::MemoryMap::USB1_FS_PERIPH_START_ADDRESS;
  static constexpr uint32_t USB1_SRAM_ADDR = Thor::System::MemoryMap::USB1_FS_SRAM_START_ADDRESS;


  /*-------------------------------------------------
  Peripheral Register Definitions
  -------------------------------------------------*/
  /*-------------------------------------------------
  Endpoint Registers
  -------------------------------------------------*/
  static constexpr uint32_t EP_CTR_RX    = ( ( uint16_t )0x8000U );
  static constexpr uint32_t EP_DTOG_RX   = ( ( uint16_t )0x4000U );
  static constexpr uint32_t EPRX_STAT    = ( ( uint16_t )0x3000U );
  static constexpr uint32_t EP_SETUP     = ( ( uint16_t )0x0800U );
  static constexpr uint32_t EP_T_FIELD   = ( ( uint16_t )0x0600U );
  static constexpr uint32_t EP_KIND      = ( ( uint16_t )0x0100U );
  static constexpr uint32_t EP_CTR_TX    = ( ( uint16_t )0x0080U );
  static constexpr uint32_t EP_DTOG_TX   = ( ( uint16_t )0x0040U );
  static constexpr uint32_t EPTX_STAT    = ( ( uint16_t )0x0030U );
  static constexpr uint32_t EPADDR_FIELD = ( ( uint16_t )0x000FU );

  /* EndPoint REGister MASK (no toggle fields) */
  static constexpr uint32_t EPREG_MASK     = ( EP_CTR_RX | EP_SETUP | EP_T_FIELD | EP_KIND | EP_CTR_TX | EPADDR_FIELD );
  static constexpr uint32_t EP_TYPE_MASK   = ( ( uint16_t )0x0600U );
  static constexpr uint32_t EP_BULK        = ( ( uint16_t )0x0000U );
  static constexpr uint32_t EP_CONTROL     = ( ( uint16_t )0x0200U );
  static constexpr uint32_t EP_ISOCHRONOUS = ( ( uint16_t )0x0400U );
  static constexpr uint32_t EP_INTERRUPT   = ( ( uint16_t )0x0600U );
  static constexpr uint32_t EP_T_MASK      = ( ( uint16_t )~EP_T_FIELD & EPREG_MASK );
  static constexpr uint32_t EPKIND_MASK    = ( ( uint16_t )~EP_KIND & EPREG_MASK );
  static constexpr uint32_t EP_TX_DIS      = ( ( uint16_t )0x0000U );
  static constexpr uint32_t EP_TX_STALL    = ( ( uint16_t )0x0010U );
  static constexpr uint32_t EP_TX_NAK      = ( ( uint16_t )0x0020U );
  static constexpr uint32_t EP_TX_VALID    = ( ( uint16_t )0x0030U );
  static constexpr uint32_t EPTX_DTOG1     = ( ( uint16_t )0x0010U );
  static constexpr uint32_t EPTX_DTOG2     = ( ( uint16_t )0x0020U );
  static constexpr uint32_t EPTX_DTOGMASK  = ( EPTX_STAT | EPREG_MASK );
  static constexpr uint32_t EP_RX_DIS      = ( ( uint16_t )0x0000U );
  static constexpr uint32_t EP_RX_STALL    = ( ( uint16_t )0x1000U );
  static constexpr uint32_t EP_RX_NAK      = ( ( uint16_t )0x2000U );
  static constexpr uint32_t EP_RX_VALID    = ( ( uint16_t )0x3000U );
  static constexpr uint32_t EPRX_DTOG1     = ( ( uint16_t )0x1000U );
  static constexpr uint32_t EPRX_DTOG2     = ( ( uint16_t )0x2000U );
  static constexpr uint32_t EPRX_DTOGMASK  = ( EPRX_STAT | EPREG_MASK );

  /*-------------------------------------------------
  General Registers
  -------------------------------------------------*/

  /******************  Bits definition for CNTR register  *******************/
  static constexpr uint32_t CNTR_CTRM     = ( ( uint16_t )0x8000U );
  static constexpr uint32_t CNTR_PMAOVRM  = ( ( uint16_t )0x4000U );
  static constexpr uint32_t CNTR_ERRM     = ( ( uint16_t )0x2000U );
  static constexpr uint32_t CNTR_WKUPM    = ( ( uint16_t )0x1000U );
  static constexpr uint32_t CNTR_SUSPM    = ( ( uint16_t )0x0800U );
  static constexpr uint32_t CNTR_RESETM   = ( ( uint16_t )0x0400U );
  static constexpr uint32_t CNTR_SOFM     = ( ( uint16_t )0x0200U );
  static constexpr uint32_t CNTR_ESOFM    = ( ( uint16_t )0x0100U );
  static constexpr uint32_t CNTR_L1REQM   = ( ( uint16_t )0x0080U );
  static constexpr uint32_t CNTR_L1RESUME = ( ( uint16_t )0x0020U );
  static constexpr uint32_t CNTR_RESUME   = ( ( uint16_t )0x0010U );
  static constexpr uint32_t CNTR_FSUSP    = ( ( uint16_t )0x0008U );
  static constexpr uint32_t CNTR_LPMODE   = ( ( uint16_t )0x0004U );
  static constexpr uint32_t CNTR_PDWN     = ( ( uint16_t )0x0002U );
  static constexpr uint32_t CNTR_FRES     = ( ( uint16_t )0x0001U );

  /******************  Bits definition for ISTR register  *******************/
  static constexpr uint32_t ISTR_EP_ID  = ( ( uint16_t )0x000FU );
  static constexpr uint32_t ISTR_DIR    = ( ( uint16_t )0x0010U );
  static constexpr uint32_t ISTR_L1REQ  = ( ( uint16_t )0x0080U );
  static constexpr uint32_t ISTR_ESOF   = ( ( uint16_t )0x0100U );
  static constexpr uint32_t ISTR_SOF    = ( ( uint16_t )0x0200U );
  static constexpr uint32_t ISTR_RESET  = ( ( uint16_t )0x0400U );
  static constexpr uint32_t ISTR_SUSP   = ( ( uint16_t )0x0800U );
  static constexpr uint32_t ISTR_WKUP   = ( ( uint16_t )0x1000U );
  static constexpr uint32_t ISTR_ERR    = ( ( uint16_t )0x2000U );
  static constexpr uint32_t ISTR_PMAOVR = ( ( uint16_t )0x4000U );
  static constexpr uint32_t ISTR_CTR    = ( ( uint16_t )0x8000U );


  /******************  Bits definition for FNR register  ********************/
  static constexpr uint32_t FNR_FN   = ( ( uint16_t )0x07FFU );
  static constexpr uint32_t FNR_LSOF = ( ( uint16_t )0x1800U );
  static constexpr uint32_t FNR_LCK  = ( ( uint16_t )0x2000U );
  static constexpr uint32_t FNR_RXDM = ( ( uint16_t )0x4000U );
  static constexpr uint32_t FNR_RXDP = ( ( uint16_t )0x8000U );

  /******************  Bits definition for DADDR register    ****************/
  static constexpr uint32_t DADDR_ADD  = ( ( uint8_t )0x7FU );
  static constexpr uint32_t DADDR_ADD0 = ( ( uint8_t )0x01U );
  static constexpr uint32_t DADDR_ADD1 = ( ( uint8_t )0x02U );
  static constexpr uint32_t DADDR_ADD2 = ( ( uint8_t )0x04U );
  static constexpr uint32_t DADDR_ADD3 = ( ( uint8_t )0x08U );
  static constexpr uint32_t DADDR_ADD4 = ( ( uint8_t )0x10U );
  static constexpr uint32_t DADDR_ADD5 = ( ( uint8_t )0x20U );
  static constexpr uint32_t DADDR_ADD6 = ( ( uint8_t )0x40U );
  static constexpr uint32_t DADDR_EF   = ( ( uint8_t )0x80U );

  /******************  Bit definition for BTABLE register  ******************/
  static constexpr uint32_t BTABLE_BTABLE = ( ( uint16_t )0xFFF8U );

  /******************  Bits definition for BCDR register  *******************/
  static constexpr uint32_t BCDR_BCDEN  = ( ( uint16_t )0x0001U );
  static constexpr uint32_t BCDR_DCDEN  = ( ( uint16_t )0x0002U );
  static constexpr uint32_t BCDR_PDEN   = ( ( uint16_t )0x0004U );
  static constexpr uint32_t BCDR_SDEN   = ( ( uint16_t )0x0008U );
  static constexpr uint32_t BCDR_DCDET  = ( ( uint16_t )0x0010U );
  static constexpr uint32_t BCDR_PDET   = ( ( uint16_t )0x0020U );
  static constexpr uint32_t BCDR_SDET   = ( ( uint16_t )0x0040U );
  static constexpr uint32_t BCDR_PS2DET = ( ( uint16_t )0x0080U );
  static constexpr uint32_t BCDR_DPPU   = ( ( uint16_t )0x8000U );

  /*******************  Bit definition for LPMCSR register  *********************/
  static constexpr uint32_t LPMCSR_LMPEN   = ( ( uint16_t )0x0001U );
  static constexpr uint32_t LPMCSR_LPMACK  = ( ( uint16_t )0x0002U );
  static constexpr uint32_t LPMCSR_REMWAKE = ( ( uint16_t )0x0008U );
  static constexpr uint32_t LPMCSR_BESL    = ( ( uint16_t )0x00F0U );


  static constexpr uint32_t ADDR0_TX_Pos = ( 1U );
  static constexpr uint32_t ADDR0_TX_Msk = ( 0x7FFFUL << ADDR0_TX_Pos );
  static constexpr uint32_t ADDR0_TX     = ADDR0_TX_Msk;
  static constexpr uint32_t ADDR1_TX_Pos = ( 1U );
  static constexpr uint32_t ADDR1_TX_Msk = ( 0x7FFFUL << ADDR1_TX_Pos );
  static constexpr uint32_t ADDR1_TX     = ADDR1_TX_Msk;
  static constexpr uint32_t ADDR2_TX_Pos = ( 1U );
  static constexpr uint32_t ADDR2_TX_Msk = ( 0x7FFFUL << ADDR2_TX_Pos );
  static constexpr uint32_t ADDR2_TX     = ADDR2_TX_Msk;
  static constexpr uint32_t ADDR3_TX_Pos = ( 1U );
  static constexpr uint32_t ADDR3_TX_Msk = ( 0x7FFFUL << ADDR3_TX_Pos );
  static constexpr uint32_t ADDR3_TX     = ADDR3_TX_Msk;
  static constexpr uint32_t ADDR4_TX_Pos = ( 1U );
  static constexpr uint32_t ADDR4_TX_Msk = ( 0x7FFFUL << ADDR4_TX_Pos );
  static constexpr uint32_t ADDR4_TX     = ADDR4_TX_Msk;
  static constexpr uint32_t ADDR5_TX_Pos = ( 1U );
  static constexpr uint32_t ADDR5_TX_Msk = ( 0x7FFFUL << ADDR5_TX_Pos );
  static constexpr uint32_t ADDR5_TX     = ADDR5_TX_Msk;
  static constexpr uint32_t ADDR6_TX_Pos = ( 1U );
  static constexpr uint32_t ADDR6_TX_Msk = ( 0x7FFFUL << ADDR6_TX_Pos );
  static constexpr uint32_t ADDR6_TX     = ADDR6_TX_Msk;
  static constexpr uint32_t ADDR7_TX_Pos = ( 1U );
  static constexpr uint32_t ADDR7_TX_Msk = ( 0x7FFFUL << ADDR7_TX_Pos );
  static constexpr uint32_t ADDR7_TX     = ADDR7_TX_Msk;

  static constexpr uint32_t COUNT0_TX_Pos = ( 0U );
  static constexpr uint32_t COUNT0_TX_Msk = ( 0x3FFUL << COUNT0_TX_Pos );
  static constexpr uint32_t COUNT0_TX     = COUNT0_TX_Msk;
  static constexpr uint32_t COUNT1_TX_Pos = ( 0U );
  static constexpr uint32_t COUNT1_TX_Msk = ( 0x3FFUL << COUNT1_TX_Pos );
  static constexpr uint32_t COUNT1_TX     = COUNT1_TX_Msk;
  static constexpr uint32_t COUNT2_TX_Pos = ( 0U );
  static constexpr uint32_t COUNT2_TX_Msk = ( 0x3FFUL << COUNT2_TX_Pos );
  static constexpr uint32_t COUNT2_TX     = COUNT2_TX_Msk;
  static constexpr uint32_t COUNT3_TX_Pos = ( 0U );
  static constexpr uint32_t COUNT3_TX_Msk = ( 0x3FFUL << COUNT3_TX_Pos );
  static constexpr uint32_t COUNT3_TX     = COUNT3_TX_Msk;
  static constexpr uint32_t COUNT4_TX_Pos = ( 0U );
  static constexpr uint32_t COUNT4_TX_Msk = ( 0x3FFUL << COUNT4_TX_Pos );
  static constexpr uint32_t COUNT4_TX     = COUNT4_TX_Msk;
  static constexpr uint32_t COUNT5_TX_Pos = ( 0U );
  static constexpr uint32_t COUNT5_TX_Msk = ( 0x3FFUL << COUNT5_TX_Pos );
  static constexpr uint32_t COUNT5_TX     = COUNT5_TX_Msk;
  static constexpr uint32_t COUNT6_TX_Pos = ( 0U );
  static constexpr uint32_t COUNT6_TX_Msk = ( 0x3FFUL << COUNT6_TX_Pos );
  static constexpr uint32_t COUNT6_TX     = COUNT6_TX_Msk;
  static constexpr uint32_t COUNT7_TX_Pos = ( 0U );
  static constexpr uint32_t COUNT7_TX_Msk = ( 0x3FFUL << COUNT7_TX_Pos );
  static constexpr uint32_t COUNT7_TX     = COUNT7_TX_Msk;

  static constexpr uint32_t COUNT0_TX_0 = ( 0x000003FFUL );
  static constexpr uint32_t COUNT0_TX_1 = ( 0x03FF0000UL );
  static constexpr uint32_t COUNT1_TX_0 = ( 0x000003FFUL );
  static constexpr uint32_t COUNT1_TX_1 = ( 0x03FF0000UL );
  static constexpr uint32_t COUNT2_TX_0 = ( 0x000003FFUL );
  static constexpr uint32_t COUNT2_TX_1 = ( 0x03FF0000UL );
  static constexpr uint32_t COUNT3_TX_0 = ( 0x000003FFUL );
  static constexpr uint32_t COUNT3_TX_1 = ( 0x03FF0000UL );
  static constexpr uint32_t COUNT4_TX_0 = ( 0x000003FFUL );
  static constexpr uint32_t COUNT4_TX_1 = ( 0x03FF0000UL );
  static constexpr uint32_t COUNT5_TX_0 = ( 0x000003FFUL );
  static constexpr uint32_t COUNT5_TX_1 = ( 0x03FF0000UL );
  static constexpr uint32_t COUNT6_TX_0 = ( 0x000003FFUL );
  static constexpr uint32_t COUNT6_TX_1 = ( 0x03FF0000UL );
  static constexpr uint32_t COUNT7_TX_0 = ( 0x000003FFUL );
  static constexpr uint32_t COUNT7_TX_1 = ( 0x03FF0000UL );

  static constexpr uint32_t ADDR0_RX_ADDR0_RX_Pos = ( 1U );
  static constexpr uint32_t ADDR0_RX_ADDR0_RX_Msk = ( 0x7FFFUL << ADDR0_RX_ADDR0_RX_Pos );
  static constexpr uint32_t ADDR0_RX_ADDR0_RX     = ADDR0_RX_ADDR0_RX_Msk;
  static constexpr uint32_t ADDR1_RX_ADDR1_RX_Pos = ( 1U );
  static constexpr uint32_t ADDR1_RX_ADDR1_RX_Msk = ( 0x7FFFUL << ADDR1_RX_ADDR1_RX_Pos );
  static constexpr uint32_t ADDR1_RX_ADDR1_RX     = ADDR1_RX_ADDR1_RX_Msk;
  static constexpr uint32_t ADDR2_RX_ADDR2_RX_Pos = ( 1U );
  static constexpr uint32_t ADDR2_RX_ADDR2_RX_Msk = ( 0x7FFFUL << ADDR2_RX_ADDR2_RX_Pos );
  static constexpr uint32_t ADDR2_RX_ADDR2_RX     = ADDR2_RX_ADDR2_RX_Msk;
  static constexpr uint32_t ADDR3_RX_ADDR3_RX_Pos = ( 1U );
  static constexpr uint32_t ADDR3_RX_ADDR3_RX_Msk = ( 0x7FFFUL << ADDR3_RX_ADDR3_RX_Pos );
  static constexpr uint32_t ADDR3_RX_ADDR3_RX     = ADDR3_RX_ADDR3_RX_Msk;
  static constexpr uint32_t ADDR4_RX_ADDR4_RX_Pos = ( 1U );
  static constexpr uint32_t ADDR4_RX_ADDR4_RX_Msk = ( 0x7FFFUL << ADDR4_RX_ADDR4_RX_Pos );
  static constexpr uint32_t ADDR4_RX_ADDR4_RX     = ADDR4_RX_ADDR4_RX_Msk;
  static constexpr uint32_t ADDR5_RX_ADDR5_RX_Pos = ( 1U );
  static constexpr uint32_t ADDR5_RX_ADDR5_RX_Msk = ( 0x7FFFUL << ADDR5_RX_ADDR5_RX_Pos );
  static constexpr uint32_t ADDR5_RX_ADDR5_RX     = ADDR5_RX_ADDR5_RX_Msk;
  static constexpr uint32_t ADDR6_RX_ADDR6_RX_Pos = ( 1U );
  static constexpr uint32_t ADDR6_RX_ADDR6_RX_Msk = ( 0x7FFFUL << ADDR6_RX_ADDR6_RX_Pos );
  static constexpr uint32_t ADDR6_RX_ADDR6_RX     = ADDR6_RX_ADDR6_RX_Msk;
  static constexpr uint32_t ADDR7_RX_ADDR7_RX_Pos = ( 1U );
  static constexpr uint32_t ADDR7_RX_ADDR7_RX_Msk = ( 0x7FFFUL << ADDR7_RX_ADDR7_RX_Pos );
  static constexpr uint32_t ADDR7_RX_ADDR7_RX     = ADDR7_RX_ADDR7_RX_Msk;

  static constexpr uint32_t COUNT0_RX_COUNT0_RX_Pos   = ( 0U );
  static constexpr uint32_t COUNT0_RX_COUNT0_RX_Msk   = ( 0x3FFUL << COUNT0_RX_COUNT0_RX_Pos );
  static constexpr uint32_t COUNT0_RX_COUNT0_RX       = COUNT0_RX_COUNT0_RX_Msk;
  static constexpr uint32_t COUNT0_RX_NUM_BLOCK_Pos   = ( 10U );
  static constexpr uint32_t COUNT0_RX_NUM_BLOCK_Msk   = ( 0x1FUL << COUNT0_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT0_RX_NUM_BLOCK       = COUNT0_RX_NUM_BLOCK_Msk;
  static constexpr uint32_t COUNT0_RX_NUM_BLOCK_0     = ( 0x01UL << COUNT0_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT0_RX_NUM_BLOCK_1     = ( 0x02UL << COUNT0_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT0_RX_NUM_BLOCK_2     = ( 0x04UL << COUNT0_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT0_RX_NUM_BLOCK_3     = ( 0x08UL << COUNT0_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT0_RX_NUM_BLOCK_4     = ( 0x10UL << COUNT0_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT0_RX_BLSIZE_Pos      = ( 15U );
  static constexpr uint32_t COUNT0_RX_BLSIZE_Msk      = ( 0x1UL << COUNT0_RX_BLSIZE_Pos );
  static constexpr uint32_t COUNT0_RX_BLSIZE          = COUNT0_RX_BLSIZE_Msk;
  static constexpr uint32_t COUNT1_RX_COUNT1_RX_Pos   = ( 0U );
  static constexpr uint32_t COUNT1_RX_COUNT1_RX_Msk   = ( 0x3FFUL << COUNT1_RX_COUNT1_RX_Pos );
  static constexpr uint32_t COUNT1_RX_COUNT1_RX       = COUNT1_RX_COUNT1_RX_Msk;
  static constexpr uint32_t COUNT1_RX_NUM_BLOCK_Pos   = ( 10U );
  static constexpr uint32_t COUNT1_RX_NUM_BLOCK_Msk   = ( 0x1FUL << COUNT1_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT1_RX_NUM_BLOCK       = COUNT1_RX_NUM_BLOCK_Msk;
  static constexpr uint32_t COUNT1_RX_NUM_BLOCK_0     = ( 0x01UL << COUNT1_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT1_RX_NUM_BLOCK_1     = ( 0x02UL << COUNT1_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT1_RX_NUM_BLOCK_2     = ( 0x04UL << COUNT1_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT1_RX_NUM_BLOCK_3     = ( 0x08UL << COUNT1_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT1_RX_NUM_BLOCK_4     = ( 0x10UL << COUNT1_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT1_RX_BLSIZE_Pos      = ( 15U );
  static constexpr uint32_t COUNT1_RX_BLSIZE_Msk      = ( 0x1UL << COUNT1_RX_BLSIZE_Pos );
  static constexpr uint32_t COUNT1_RX_BLSIZE          = COUNT1_RX_BLSIZE_Msk;
  static constexpr uint32_t COUNT2_RX_COUNT2_RX_Pos   = ( 0U );
  static constexpr uint32_t COUNT2_RX_COUNT2_RX_Msk   = ( 0x3FFUL << COUNT2_RX_COUNT2_RX_Pos );
  static constexpr uint32_t COUNT2_RX_COUNT2_RX       = COUNT2_RX_COUNT2_RX_Msk;
  static constexpr uint32_t COUNT2_RX_NUM_BLOCK_Pos   = ( 10U );
  static constexpr uint32_t COUNT2_RX_NUM_BLOCK_Msk   = ( 0x1FUL << COUNT2_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT2_RX_NUM_BLOCK       = COUNT2_RX_NUM_BLOCK_Msk;
  static constexpr uint32_t COUNT2_RX_NUM_BLOCK_0     = ( 0x01UL << COUNT2_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT2_RX_NUM_BLOCK_1     = ( 0x02UL << COUNT2_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT2_RX_NUM_BLOCK_2     = ( 0x04UL << COUNT2_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT2_RX_NUM_BLOCK_3     = ( 0x08UL << COUNT2_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT2_RX_NUM_BLOCK_4     = ( 0x10UL << COUNT2_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT2_RX_BLSIZE_Pos      = ( 15U );
  static constexpr uint32_t COUNT2_RX_BLSIZE_Msk      = ( 0x1UL << COUNT2_RX_BLSIZE_Pos );
  static constexpr uint32_t COUNT2_RX_BLSIZE          = COUNT2_RX_BLSIZE_Msk;
  static constexpr uint32_t COUNT3_RX_COUNT3_RX_Pos   = ( 0U );
  static constexpr uint32_t COUNT3_RX_COUNT3_RX_Msk   = ( 0x3FFUL << COUNT3_RX_COUNT3_RX_Pos );
  static constexpr uint32_t COUNT3_RX_COUNT3_RX       = COUNT3_RX_COUNT3_RX_Msk;
  static constexpr uint32_t COUNT3_RX_NUM_BLOCK_Pos   = ( 10U );
  static constexpr uint32_t COUNT3_RX_NUM_BLOCK_Msk   = ( 0x1FUL << COUNT3_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT3_RX_NUM_BLOCK       = COUNT3_RX_NUM_BLOCK_Msk;
  static constexpr uint32_t COUNT3_RX_NUM_BLOCK_0     = ( 0x01UL << COUNT3_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT3_RX_NUM_BLOCK_1     = ( 0x02UL << COUNT3_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT3_RX_NUM_BLOCK_2     = ( 0x04UL << COUNT3_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT3_RX_NUM_BLOCK_3     = ( 0x08UL << COUNT3_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT3_RX_NUM_BLOCK_4     = ( 0x10UL << COUNT3_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT3_RX_BLSIZE_Pos      = ( 15U );
  static constexpr uint32_t COUNT3_RX_BLSIZE_Msk      = ( 0x1UL << COUNT3_RX_BLSIZE_Pos );
  static constexpr uint32_t COUNT3_RX_BLSIZE          = COUNT3_RX_BLSIZE_Msk;
  static constexpr uint32_t COUNT4_RX_COUNT4_RX_Pos   = ( 0U );
  static constexpr uint32_t COUNT4_RX_COUNT4_RX_Msk   = ( 0x3FFUL << COUNT4_RX_COUNT4_RX_Pos );
  static constexpr uint32_t COUNT4_RX_COUNT4_RX       = COUNT4_RX_COUNT4_RX_Msk;
  static constexpr uint32_t COUNT4_RX_NUM_BLOCK_Pos   = ( 10U );
  static constexpr uint32_t COUNT4_RX_NUM_BLOCK_Msk   = ( 0x1FUL << COUNT4_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT4_RX_NUM_BLOCK       = COUNT4_RX_NUM_BLOCK_Msk;
  static constexpr uint32_t COUNT4_RX_NUM_BLOCK_0     = ( 0x01UL << COUNT4_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT4_RX_NUM_BLOCK_1     = ( 0x02UL << COUNT4_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT4_RX_NUM_BLOCK_2     = ( 0x04UL << COUNT4_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT4_RX_NUM_BLOCK_3     = ( 0x08UL << COUNT4_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT4_RX_NUM_BLOCK_4     = ( 0x10UL << COUNT4_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT4_RX_BLSIZE_Pos      = ( 15U );
  static constexpr uint32_t COUNT4_RX_BLSIZE_Msk      = ( 0x1UL << COUNT4_RX_BLSIZE_Pos );
  static constexpr uint32_t COUNT4_RX_BLSIZE          = COUNT4_RX_BLSIZE_Msk;
  static constexpr uint32_t COUNT5_RX_COUNT5_RX_Pos   = ( 0U );
  static constexpr uint32_t COUNT5_RX_COUNT5_RX_Msk   = ( 0x3FFUL << COUNT5_RX_COUNT5_RX_Pos );
  static constexpr uint32_t COUNT5_RX_COUNT5_RX       = COUNT5_RX_COUNT5_RX_Msk;
  static constexpr uint32_t COUNT5_RX_NUM_BLOCK_Pos   = ( 10U );
  static constexpr uint32_t COUNT5_RX_NUM_BLOCK_Msk   = ( 0x1FUL << COUNT5_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT5_RX_NUM_BLOCK       = COUNT5_RX_NUM_BLOCK_Msk;
  static constexpr uint32_t COUNT5_RX_NUM_BLOCK_0     = ( 0x01UL << COUNT5_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT5_RX_NUM_BLOCK_1     = ( 0x02UL << COUNT5_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT5_RX_NUM_BLOCK_2     = ( 0x04UL << COUNT5_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT5_RX_NUM_BLOCK_3     = ( 0x08UL << COUNT5_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT5_RX_NUM_BLOCK_4     = ( 0x10UL << COUNT5_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT5_RX_BLSIZE_Pos      = ( 15U );
  static constexpr uint32_t COUNT5_RX_BLSIZE_Msk      = ( 0x1UL << COUNT5_RX_BLSIZE_Pos );
  static constexpr uint32_t COUNT5_RX_BLSIZE          = COUNT5_RX_BLSIZE_Msk;
  static constexpr uint32_t COUNT6_RX_COUNT6_RX_Pos   = ( 0U );
  static constexpr uint32_t COUNT6_RX_COUNT6_RX_Msk   = ( 0x3FFUL << COUNT6_RX_COUNT6_RX_Pos );
  static constexpr uint32_t COUNT6_RX_COUNT6_RX       = COUNT6_RX_COUNT6_RX_Msk;
  static constexpr uint32_t COUNT6_RX_NUM_BLOCK_Pos   = ( 10U );
  static constexpr uint32_t COUNT6_RX_NUM_BLOCK_Msk   = ( 0x1FUL << COUNT6_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT6_RX_NUM_BLOCK       = COUNT6_RX_NUM_BLOCK_Msk;
  static constexpr uint32_t COUNT6_RX_NUM_BLOCK_0     = ( 0x01UL << COUNT6_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT6_RX_NUM_BLOCK_1     = ( 0x02UL << COUNT6_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT6_RX_NUM_BLOCK_2     = ( 0x04UL << COUNT6_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT6_RX_NUM_BLOCK_3     = ( 0x08UL << COUNT6_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT6_RX_NUM_BLOCK_4     = ( 0x10UL << COUNT6_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT6_RX_BLSIZE_Pos      = ( 15U );
  static constexpr uint32_t COUNT6_RX_BLSIZE_Msk      = ( 0x1UL << COUNT6_RX_BLSIZE_Pos );
  static constexpr uint32_t COUNT6_RX_BLSIZE          = COUNT6_RX_BLSIZE_Msk;
  static constexpr uint32_t COUNT7_RX_COUNT7_RX_Pos   = ( 0U );
  static constexpr uint32_t COUNT7_RX_COUNT7_RX_Msk   = ( 0x3FFUL << COUNT7_RX_COUNT7_RX_Pos );
  static constexpr uint32_t COUNT7_RX_COUNT7_RX       = COUNT7_RX_COUNT7_RX_Msk;
  static constexpr uint32_t COUNT7_RX_NUM_BLOCK_Pos   = ( 10U );
  static constexpr uint32_t COUNT7_RX_NUM_BLOCK_Msk   = ( 0x1FUL << COUNT7_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT7_RX_NUM_BLOCK       = COUNT7_RX_NUM_BLOCK_Msk;
  static constexpr uint32_t COUNT7_RX_NUM_BLOCK_0     = ( 0x01UL << COUNT7_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT7_RX_NUM_BLOCK_1     = ( 0x02UL << COUNT7_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT7_RX_NUM_BLOCK_2     = ( 0x04UL << COUNT7_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT7_RX_NUM_BLOCK_3     = ( 0x08UL << COUNT7_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT7_RX_NUM_BLOCK_4     = ( 0x10UL << COUNT7_RX_NUM_BLOCK_Pos );
  static constexpr uint32_t COUNT7_RX_BLSIZE_Pos      = ( 15U );
  static constexpr uint32_t COUNT7_RX_BLSIZE_Msk      = ( 0x1UL << COUNT7_RX_BLSIZE_Pos );
  static constexpr uint32_t COUNT7_RX_BLSIZE          = COUNT7_RX_BLSIZE_Msk;
  static constexpr uint32_t COUNT0_RX_0_COUNT0_RX_0   = ( 0x000003FFUL );
  static constexpr uint32_t COUNT0_RX_0_NUM_BLOCK_0   = ( 0x00007C00UL );
  static constexpr uint32_t COUNT0_RX_0_NUM_BLOCK_0_0 = ( 0x00000400UL );
  static constexpr uint32_t COUNT0_RX_0_NUM_BLOCK_0_1 = ( 0x00000800UL );
  static constexpr uint32_t COUNT0_RX_0_NUM_BLOCK_0_2 = ( 0x00001000UL );
  static constexpr uint32_t COUNT0_RX_0_NUM_BLOCK_0_3 = ( 0x00002000UL );
  static constexpr uint32_t COUNT0_RX_0_NUM_BLOCK_0_4 = ( 0x00004000UL );
  static constexpr uint32_t COUNT0_RX_0_BLSIZE_0      = ( 0x00008000UL );
  static constexpr uint32_t COUNT0_RX_1_COUNT0_RX_1   = ( 0x03FF0000UL );
  static constexpr uint32_t COUNT0_RX_1_NUM_BLOCK_1   = ( 0x7C000000UL );
  static constexpr uint32_t COUNT0_RX_1_NUM_BLOCK_1_0 = ( 0x04000000UL );
  static constexpr uint32_t COUNT0_RX_1_NUM_BLOCK_1_1 = ( 0x08000000UL );
  static constexpr uint32_t COUNT0_RX_1_NUM_BLOCK_1_2 = ( 0x10000000UL );
  static constexpr uint32_t COUNT0_RX_1_NUM_BLOCK_1_3 = ( 0x20000000UL );
  static constexpr uint32_t COUNT0_RX_1_NUM_BLOCK_1_4 = ( 0x40000000UL );
  static constexpr uint32_t COUNT0_RX_1_BLSIZE_1      = ( 0x80000000UL );
  static constexpr uint32_t COUNT1_RX_0_COUNT1_RX_0   = ( 0x000003FFUL );
  static constexpr uint32_t COUNT1_RX_0_NUM_BLOCK_0   = ( 0x00007C00UL );
  static constexpr uint32_t COUNT1_RX_0_NUM_BLOCK_0_0 = ( 0x00000400UL );
  static constexpr uint32_t COUNT1_RX_0_NUM_BLOCK_0_1 = ( 0x00000800UL );
  static constexpr uint32_t COUNT1_RX_0_NUM_BLOCK_0_2 = ( 0x00001000UL );
  static constexpr uint32_t COUNT1_RX_0_NUM_BLOCK_0_3 = ( 0x00002000UL );
  static constexpr uint32_t COUNT1_RX_0_NUM_BLOCK_0_4 = ( 0x00004000UL );
  static constexpr uint32_t COUNT1_RX_0_BLSIZE_0      = ( 0x00008000UL );
  static constexpr uint32_t COUNT1_RX_1_COUNT1_RX_1   = ( 0x03FF0000UL );
  static constexpr uint32_t COUNT1_RX_1_NUM_BLOCK_1   = ( 0x7C000000UL );
  static constexpr uint32_t COUNT1_RX_1_NUM_BLOCK_1_0 = ( 0x04000000UL );
  static constexpr uint32_t COUNT1_RX_1_NUM_BLOCK_1_1 = ( 0x08000000UL );
  static constexpr uint32_t COUNT1_RX_1_NUM_BLOCK_1_2 = ( 0x10000000UL );
  static constexpr uint32_t COUNT1_RX_1_NUM_BLOCK_1_3 = ( 0x20000000UL );
  static constexpr uint32_t COUNT1_RX_1_NUM_BLOCK_1_4 = ( 0x40000000UL );
  static constexpr uint32_t COUNT1_RX_1_BLSIZE_1      = ( 0x80000000UL );
  static constexpr uint32_t COUNT2_RX_0_COUNT2_RX_0   = ( 0x000003FFUL );
  static constexpr uint32_t COUNT2_RX_0_NUM_BLOCK_0   = ( 0x00007C00UL );
  static constexpr uint32_t COUNT2_RX_0_NUM_BLOCK_0_0 = ( 0x00000400UL );
  static constexpr uint32_t COUNT2_RX_0_NUM_BLOCK_0_1 = ( 0x00000800UL );
  static constexpr uint32_t COUNT2_RX_0_NUM_BLOCK_0_2 = ( 0x00001000UL );
  static constexpr uint32_t COUNT2_RX_0_NUM_BLOCK_0_3 = ( 0x00002000UL );
  static constexpr uint32_t COUNT2_RX_0_NUM_BLOCK_0_4 = ( 0x00004000UL );
  static constexpr uint32_t COUNT2_RX_0_BLSIZE_0      = ( 0x00008000UL );
  static constexpr uint32_t COUNT2_RX_1_COUNT2_RX_1   = ( 0x03FF0000UL );
  static constexpr uint32_t COUNT2_RX_1_NUM_BLOCK_1   = ( 0x7C000000UL );
  static constexpr uint32_t COUNT2_RX_1_NUM_BLOCK_1_0 = ( 0x04000000UL );
  static constexpr uint32_t COUNT2_RX_1_NUM_BLOCK_1_1 = ( 0x08000000UL );
  static constexpr uint32_t COUNT2_RX_1_NUM_BLOCK_1_2 = ( 0x10000000UL );
  static constexpr uint32_t COUNT2_RX_1_NUM_BLOCK_1_3 = ( 0x20000000UL );
  static constexpr uint32_t COUNT2_RX_1_NUM_BLOCK_1_4 = ( 0x40000000UL );
  static constexpr uint32_t COUNT2_RX_1_BLSIZE_1      = ( 0x80000000UL );
  static constexpr uint32_t COUNT3_RX_0_COUNT3_RX_0   = ( 0x000003FFUL );
  static constexpr uint32_t COUNT3_RX_0_NUM_BLOCK_0   = ( 0x00007C00UL );
  static constexpr uint32_t COUNT3_RX_0_NUM_BLOCK_0_0 = ( 0x00000400UL );
  static constexpr uint32_t COUNT3_RX_0_NUM_BLOCK_0_1 = ( 0x00000800UL );
  static constexpr uint32_t COUNT3_RX_0_NUM_BLOCK_0_2 = ( 0x00001000UL );
  static constexpr uint32_t COUNT3_RX_0_NUM_BLOCK_0_3 = ( 0x00002000UL );
  static constexpr uint32_t COUNT3_RX_0_NUM_BLOCK_0_4 = ( 0x00004000UL );
  static constexpr uint32_t COUNT3_RX_0_BLSIZE_0      = ( 0x00008000UL );
  static constexpr uint32_t COUNT3_RX_1_COUNT3_RX_1   = ( 0x03FF0000UL );
  static constexpr uint32_t COUNT3_RX_1_NUM_BLOCK_1   = ( 0x7C000000UL );
  static constexpr uint32_t COUNT3_RX_1_NUM_BLOCK_1_0 = ( 0x04000000UL );
  static constexpr uint32_t COUNT3_RX_1_NUM_BLOCK_1_1 = ( 0x08000000UL );
  static constexpr uint32_t COUNT3_RX_1_NUM_BLOCK_1_2 = ( 0x10000000UL );
  static constexpr uint32_t COUNT3_RX_1_NUM_BLOCK_1_3 = ( 0x20000000UL );
  static constexpr uint32_t COUNT3_RX_1_NUM_BLOCK_1_4 = ( 0x40000000UL );
  static constexpr uint32_t COUNT3_RX_1_BLSIZE_1      = ( 0x80000000UL );
  static constexpr uint32_t COUNT4_RX_0_COUNT4_RX_0   = ( 0x000003FFUL );
  static constexpr uint32_t COUNT4_RX_0_NUM_BLOCK_0   = ( 0x00007C00UL );
  static constexpr uint32_t COUNT4_RX_0_NUM_BLOCK_0_0 = ( 0x00000400UL );
  static constexpr uint32_t COUNT4_RX_0_NUM_BLOCK_0_1 = ( 0x00000800UL );
  static constexpr uint32_t COUNT4_RX_0_NUM_BLOCK_0_2 = ( 0x00001000UL );
  static constexpr uint32_t COUNT4_RX_0_NUM_BLOCK_0_3 = ( 0x00002000UL );
  static constexpr uint32_t COUNT4_RX_0_NUM_BLOCK_0_4 = ( 0x00004000UL );
  static constexpr uint32_t COUNT4_RX_0_BLSIZE_0      = ( 0x00008000UL );
  static constexpr uint32_t COUNT4_RX_1_COUNT4_RX_1   = ( 0x03FF0000UL );
  static constexpr uint32_t COUNT4_RX_1_NUM_BLOCK_1   = ( 0x7C000000UL );
  static constexpr uint32_t COUNT4_RX_1_NUM_BLOCK_1_0 = ( 0x04000000UL );
  static constexpr uint32_t COUNT4_RX_1_NUM_BLOCK_1_1 = ( 0x08000000UL );
  static constexpr uint32_t COUNT4_RX_1_NUM_BLOCK_1_2 = ( 0x10000000UL );
  static constexpr uint32_t COUNT4_RX_1_NUM_BLOCK_1_3 = ( 0x20000000UL );
  static constexpr uint32_t COUNT4_RX_1_NUM_BLOCK_1_4 = ( 0x40000000UL );
  static constexpr uint32_t COUNT4_RX_1_BLSIZE_1      = ( 0x80000000UL );
  static constexpr uint32_t COUNT5_RX_0_COUNT5_RX_0   = ( 0x000003FFUL );
  static constexpr uint32_t COUNT5_RX_0_NUM_BLOCK_0   = ( 0x00007C00UL );
  static constexpr uint32_t COUNT5_RX_0_NUM_BLOCK_0_0 = ( 0x00000400UL );
  static constexpr uint32_t COUNT5_RX_0_NUM_BLOCK_0_1 = ( 0x00000800UL );
  static constexpr uint32_t COUNT5_RX_0_NUM_BLOCK_0_2 = ( 0x00001000UL );
  static constexpr uint32_t COUNT5_RX_0_NUM_BLOCK_0_3 = ( 0x00002000UL );
  static constexpr uint32_t COUNT5_RX_0_NUM_BLOCK_0_4 = ( 0x00004000UL );
  static constexpr uint32_t COUNT5_RX_0_BLSIZE_0      = ( 0x00008000UL );
  static constexpr uint32_t COUNT5_RX_1_COUNT5_RX_1   = ( 0x03FF0000UL );
  static constexpr uint32_t COUNT5_RX_1_NUM_BLOCK_1   = ( 0x7C000000UL );
  static constexpr uint32_t COUNT5_RX_1_NUM_BLOCK_1_0 = ( 0x04000000UL );
  static constexpr uint32_t COUNT5_RX_1_NUM_BLOCK_1_1 = ( 0x08000000UL );
  static constexpr uint32_t COUNT5_RX_1_NUM_BLOCK_1_2 = ( 0x10000000UL );
  static constexpr uint32_t COUNT5_RX_1_NUM_BLOCK_1_3 = ( 0x20000000UL );
  static constexpr uint32_t COUNT5_RX_1_NUM_BLOCK_1_4 = ( 0x40000000UL );
  static constexpr uint32_t COUNT5_RX_1_BLSIZE_1      = ( 0x80000000UL );
  static constexpr uint32_t COUNT6_RX_0_COUNT6_RX_0   = ( 0x000003FFUL );
  static constexpr uint32_t COUNT6_RX_0_NUM_BLOCK_0   = ( 0x00007C00UL );
  static constexpr uint32_t COUNT6_RX_0_NUM_BLOCK_0_0 = ( 0x00000400UL );
  static constexpr uint32_t COUNT6_RX_0_NUM_BLOCK_0_1 = ( 0x00000800UL );
  static constexpr uint32_t COUNT6_RX_0_NUM_BLOCK_0_2 = ( 0x00001000UL );
  static constexpr uint32_t COUNT6_RX_0_NUM_BLOCK_0_3 = ( 0x00002000UL );
  static constexpr uint32_t COUNT6_RX_0_NUM_BLOCK_0_4 = ( 0x00004000UL );
  static constexpr uint32_t COUNT6_RX_0_BLSIZE_0      = ( 0x00008000UL );
  static constexpr uint32_t COUNT6_RX_1_COUNT6_RX_1   = ( 0x03FF0000UL );
  static constexpr uint32_t COUNT6_RX_1_NUM_BLOCK_1   = ( 0x7C000000UL );
  static constexpr uint32_t COUNT6_RX_1_NUM_BLOCK_1_0 = ( 0x04000000UL );
  static constexpr uint32_t COUNT6_RX_1_NUM_BLOCK_1_1 = ( 0x08000000UL );
  static constexpr uint32_t COUNT6_RX_1_NUM_BLOCK_1_2 = ( 0x10000000UL );
  static constexpr uint32_t COUNT6_RX_1_NUM_BLOCK_1_3 = ( 0x20000000UL );
  static constexpr uint32_t COUNT6_RX_1_NUM_BLOCK_1_4 = ( 0x40000000UL );
  static constexpr uint32_t COUNT6_RX_1_BLSIZE_1      = ( 0x80000000UL );
  static constexpr uint32_t COUNT7_RX_0_COUNT7_RX_0   = ( 0x000003FFUL );
  static constexpr uint32_t COUNT7_RX_0_NUM_BLOCK_0   = ( 0x00007C00UL );
  static constexpr uint32_t COUNT7_RX_0_NUM_BLOCK_0_0 = ( 0x00000400UL );
  static constexpr uint32_t COUNT7_RX_0_NUM_BLOCK_0_1 = ( 0x00000800UL );
  static constexpr uint32_t COUNT7_RX_0_NUM_BLOCK_0_2 = ( 0x00001000UL );
  static constexpr uint32_t COUNT7_RX_0_NUM_BLOCK_0_3 = ( 0x00002000UL );
  static constexpr uint32_t COUNT7_RX_0_NUM_BLOCK_0_4 = ( 0x00004000UL );
  static constexpr uint32_t COUNT7_RX_0_BLSIZE_0      = ( 0x00008000UL );
  static constexpr uint32_t COUNT7_RX_1_COUNT7_RX_1   = ( 0x03FF0000UL );
  static constexpr uint32_t COUNT7_RX_1_NUM_BLOCK_1   = ( 0x7C000000UL );
  static constexpr uint32_t COUNT7_RX_1_NUM_BLOCK_1_0 = ( 0x04000000UL );
  static constexpr uint32_t COUNT7_RX_1_NUM_BLOCK_1_1 = ( 0x08000000UL );
  static constexpr uint32_t COUNT7_RX_1_NUM_BLOCK_1_2 = ( 0x10000000UL );
  static constexpr uint32_t COUNT7_RX_1_NUM_BLOCK_1_3 = ( 0x20000000UL );
  static constexpr uint32_t COUNT7_RX_1_NUM_BLOCK_1_4 = ( 0x40000000UL );
  static constexpr uint32_t COUNT7_RX_1_BLSIZE_1      = ( 0x80000000UL );

}    // namespace Thor::LLD::USB

#endif /* !THOR_HW_USB_REGISTER_STM32L4XXXX_HPP */
