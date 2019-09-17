/********************************************************************************
 *   File Name:
 *    hw_usart_register_stm32f446xx.hpp
 *
 *   Description:
 *    Explicit hardware register definitions for the STM32F446xx USART/UART
 *    peripherals. The datasheet for this device makes no distinction between the
 *    UART and USART registers, so the definitions for both are combined here.
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_USART_REGISTER_HPP
#define THOR_HW_USART_REGISTER_HPP

/* C++ Includes */
#include <array>
#include <cstdint>

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/f4/system/sys_memory_map_stm32f446xx.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_USART == 1 )

namespace Thor::Driver::USART
{
  static constexpr uint32_t USART1_BASE_ADDR = Thor::System::MemoryMap::APB2PERIPH_BASE_ADDR + 0x1000U;
  static constexpr uint32_t USART2_BASE_ADDR = Thor::System::MemoryMap::APB1PERIPH_BASE_ADDR + 0x4400U;
  static constexpr uint32_t USART3_BASE_ADDR = Thor::System::MemoryMap::APB1PERIPH_BASE_ADDR + 0x4800U;
  static constexpr uint32_t USART6_BASE_ADDR = Thor::System::MemoryMap::APB2PERIPH_BASE_ADDR + 0x1400U;

  static constexpr uint32_t NUM_USART_PERIPHS = 4u;

  static constexpr std::array<uint32_t, NUM_USART_PERIPHS> periphAddressList = { USART1_BASE_ADDR, USART2_BASE_ADDR,
                                                                                 USART3_BASE_ADDR, USART6_BASE_ADDR };

  /*------------------------------------------------
  USART Status Register
  ------------------------------------------------*/
  static constexpr uint32_t SR_Msk      = ( 0x3FF );
  static constexpr uint32_t SR_Rst      = ( 0x00C00000 );
  static constexpr uint32_t SR_PE_Pos   = ( 0U );
  static constexpr uint32_t SR_PE_Msk   = ( 0x1U << SR_PE_Pos );
  static constexpr uint32_t SR_PE       = SR_PE_Msk;
  static constexpr uint32_t SR_FE_Pos   = ( 1U );
  static constexpr uint32_t SR_FE_Msk   = ( 0x1U << SR_FE_Pos );
  static constexpr uint32_t SR_FE       = SR_FE_Msk;
  static constexpr uint32_t SR_NF_Pos   = ( 2U );
  static constexpr uint32_t SR_NF_Msk   = ( 0x1U << SR_NF_Pos );
  static constexpr uint32_t SR_NF       = SR_NF_Msk;
  static constexpr uint32_t SR_ORE_Pos  = ( 3U );
  static constexpr uint32_t SR_ORE_Msk  = ( 0x1U << SR_ORE_Pos );
  static constexpr uint32_t SR_ORE      = SR_ORE_Msk;
  static constexpr uint32_t SR_IDLE_Pos = ( 4U );
  static constexpr uint32_t SR_IDLE_Msk = ( 0x1U << SR_IDLE_Pos );
  static constexpr uint32_t SR_IDLE     = SR_IDLE_Msk;
  static constexpr uint32_t SR_RXNE_Pos = ( 5U );
  static constexpr uint32_t SR_RXNE_Msk = ( 0x1U << SR_RXNE_Pos );
  static constexpr uint32_t SR_RXNE     = SR_RXNE_Msk;
  static constexpr uint32_t SR_TC_Pos   = ( 6U );
  static constexpr uint32_t SR_TC_Msk   = ( 0x1U << SR_TC_Pos );
  static constexpr uint32_t SR_TC       = SR_TC_Msk;
  static constexpr uint32_t SR_TXE_Pos  = ( 7U );
  static constexpr uint32_t SR_TXE_Msk  = ( 0x1U << SR_TXE_Pos );
  static constexpr uint32_t SR_TXE      = SR_TXE_Msk;
  static constexpr uint32_t SR_LBD_Pos  = ( 8U );
  static constexpr uint32_t SR_LBD_Msk  = ( 0x1U << SR_LBD_Pos );
  static constexpr uint32_t SR_LBD      = SR_LBD_Msk;
  static constexpr uint32_t SR_CTS_Pos  = ( 9U );
  static constexpr uint32_t SR_CTS_Msk  = ( 0x1U << SR_CTS_Pos );
  static constexpr uint32_t SR_CTS      = SR_CTS_Msk;

  /*------------------------------------------------
  USART Data Register
  ------------------------------------------------*/
  static constexpr uint32_t DR_DR_Pos = ( 0U );
  static constexpr uint32_t DR_DR_Msk = ( 0x1FFU << DR_DR_Pos );
  static constexpr uint32_t DR_DR     = DR_DR_Msk;

  /*------------------------------------------------
  USART Baud Rate Register
  ------------------------------------------------*/
  static constexpr uint32_t BRR_Msk              = ( 0xFFFF );
  static constexpr uint32_t BRR_Rst              = ( 0U );
  static constexpr uint32_t BRR_DIV_Fraction_Pos = ( 0U );
  static constexpr uint32_t BRR_DIV_Fraction_Msk = ( 0xFU << BRR_DIV_Fraction_Pos );
  static constexpr uint32_t BRR_DIV_Fraction     = BRR_DIV_Fraction_Msk;
  static constexpr uint32_t BRR_DIV_Mantissa_Pos = ( 4U );
  static constexpr uint32_t BRR_DIV_Mantissa_Msk = ( 0xFFFU << BRR_DIV_Mantissa_Pos );
  static constexpr uint32_t BRR_DIV_Mantissa     = BRR_DIV_Mantissa_Msk;

  /*------------------------------------------------
  USART Control Register 1
  ------------------------------------------------*/
  static constexpr uint32_t CR1_Msk        = ( 0xBFFF );
  static constexpr uint32_t CR1_Rst        = ( 0U );
  static constexpr uint32_t CR1_SBK_Pos    = ( 0U );
  static constexpr uint32_t CR1_SBK_Msk    = ( 0x1U << CR1_SBK_Pos );
  static constexpr uint32_t CR1_SBK        = CR1_SBK_Msk;
  static constexpr uint32_t CR1_RWU_Pos    = ( 1U );
  static constexpr uint32_t CR1_RWU_Msk    = ( 0x1U << CR1_RWU_Pos );
  static constexpr uint32_t CR1_RWU        = CR1_RWU_Msk;
  static constexpr uint32_t CR1_RE_Pos     = ( 2U );
  static constexpr uint32_t CR1_RE_Msk     = ( 0x1U << CR1_RE_Pos );
  static constexpr uint32_t CR1_RE         = CR1_RE_Msk;
  static constexpr uint32_t CR1_TE_Pos     = ( 3U );
  static constexpr uint32_t CR1_TE_Msk     = ( 0x1U << CR1_TE_Pos );
  static constexpr uint32_t CR1_TE         = CR1_TE_Msk;
  static constexpr uint32_t CR1_IDLEIE_Pos = ( 4U );
  static constexpr uint32_t CR1_IDLEIE_Msk = ( 0x1U << CR1_IDLEIE_Pos );
  static constexpr uint32_t CR1_IDLEIE     = CR1_IDLEIE_Msk;
  static constexpr uint32_t CR1_RXNEIE_Pos = ( 5U );
  static constexpr uint32_t CR1_RXNEIE_Msk = ( 0x1U << CR1_RXNEIE_Pos );
  static constexpr uint32_t CR1_RXNEIE     = CR1_RXNEIE_Msk;
  static constexpr uint32_t CR1_TCIE_Pos   = ( 6U );
  static constexpr uint32_t CR1_TCIE_Msk   = ( 0x1U << CR1_TCIE_Pos );
  static constexpr uint32_t CR1_TCIE       = CR1_TCIE_Msk;
  static constexpr uint32_t CR1_TXEIE_Pos  = ( 7U );
  static constexpr uint32_t CR1_TXEIE_Msk  = ( 0x1U << CR1_TXEIE_Pos );
  static constexpr uint32_t CR1_TXEIE      = CR1_TXEIE_Msk;
  static constexpr uint32_t CR1_PEIE_Pos   = ( 8U );
  static constexpr uint32_t CR1_PEIE_Msk   = ( 0x1U << CR1_PEIE_Pos );
  static constexpr uint32_t CR1_PEIE       = CR1_PEIE_Msk;
  static constexpr uint32_t CR1_PS_Pos     = ( 9U );
  static constexpr uint32_t CR1_PS_Msk     = ( 0x1U << CR1_PS_Pos );
  static constexpr uint32_t CR1_PS         = CR1_PS_Msk;
  static constexpr uint32_t CR1_PCE_Pos    = ( 10U );
  static constexpr uint32_t CR1_PCE_Msk    = ( 0x1U << CR1_PCE_Pos );
  static constexpr uint32_t CR1_PCE        = CR1_PCE_Msk;
  static constexpr uint32_t CR1_WAKE_Pos   = ( 11U );
  static constexpr uint32_t CR1_WAKE_Msk   = ( 0x1U << CR1_WAKE_Pos );
  static constexpr uint32_t CR1_WAKE       = CR1_WAKE_Msk;
  static constexpr uint32_t CR1_M_Pos      = ( 12U );
  static constexpr uint32_t CR1_M_Msk      = ( 0x1U << CR1_M_Pos );
  static constexpr uint32_t CR1_M          = CR1_M_Msk;
  static constexpr uint32_t CR1_UE_Pos     = ( 13U );
  static constexpr uint32_t CR1_UE_Msk     = ( 0x1U << CR1_UE_Pos );
  static constexpr uint32_t CR1_UE         = CR1_UE_Msk;
  static constexpr uint32_t CR1_OVER8_Pos  = ( 15U );
  static constexpr uint32_t CR1_OVER8_Msk  = ( 0x1U << CR1_OVER8_Pos );
  static constexpr uint32_t CR1_OVER8      = CR1_OVER8_Msk;

  /*------------------------------------------------
  USART Control Register 2
  ------------------------------------------------*/
  static constexpr uint32_t CR2_Msk       = ( 0x7F6F );
  static constexpr uint32_t CR2_Rst       = ( 0U );
  static constexpr uint32_t CR2_ADD_Pos   = ( 0U );
  static constexpr uint32_t CR2_ADD_Msk   = ( 0xFU << CR2_ADD_Pos );
  static constexpr uint32_t CR2_ADD       = CR2_ADD_Msk;
  static constexpr uint32_t CR2_LBDL_Pos  = ( 5U );
  static constexpr uint32_t CR2_LBDL_Msk  = ( 0x1U << CR2_LBDL_Pos );
  static constexpr uint32_t CR2_LBDL      = CR2_LBDL_Msk;
  static constexpr uint32_t CR2_LBDIE_Pos = ( 6U );
  static constexpr uint32_t CR2_LBDIE_Msk = ( 0x1U << CR2_LBDIE_Pos );
  static constexpr uint32_t CR2_LBDIE     = CR2_LBDIE_Msk;
  static constexpr uint32_t CR2_LBCL_Pos  = ( 8U );
  static constexpr uint32_t CR2_LBCL_Msk  = ( 0x1U << CR2_LBCL_Pos );
  static constexpr uint32_t CR2_LBCL      = CR2_LBCL_Msk;
  static constexpr uint32_t CR2_CPHA_Pos  = ( 9U );
  static constexpr uint32_t CR2_CPHA_Msk  = ( 0x1U << CR2_CPHA_Pos );
  static constexpr uint32_t CR2_CPHA      = CR2_CPHA_Msk;
  static constexpr uint32_t CR2_CPOL_Pos  = ( 10U );
  static constexpr uint32_t CR2_CPOL_Msk  = ( 0x1U << CR2_CPOL_Pos );
  static constexpr uint32_t CR2_CPOL      = CR2_CPOL_Msk;
  static constexpr uint32_t CR2_CLKEN_Pos = ( 11U );
  static constexpr uint32_t CR2_CLKEN_Msk = ( 0x1U << CR2_CLKEN_Pos );
  static constexpr uint32_t CR2_CLKEN     = CR2_CLKEN_Msk;
  static constexpr uint32_t CR2_STOP_Pos  = ( 12U );
  static constexpr uint32_t CR2_STOP_Msk  = ( 0x3U << CR2_STOP_Pos );
  static constexpr uint32_t CR2_STOP      = CR2_STOP_Msk;
  static constexpr uint32_t CR2_STOP_0    = ( 0x1U << CR2_STOP_Pos );
  static constexpr uint32_t CR2_STOP_1    = ( 0x2U << CR2_STOP_Pos );
  static constexpr uint32_t CR2_LINEN_Pos = ( 14U );
  static constexpr uint32_t CR2_LINEN_Msk = ( 0x1U << CR2_LINEN_Pos );
  static constexpr uint32_t CR2_LINEN     = CR2_LINEN_Msk;

  /*------------------------------------------------
  USART Control Register 3
  ------------------------------------------------*/
  static constexpr uint32_t CR3_Msk        = ( 0x0FFF );
  static constexpr uint32_t CR3_Rst        = ( 0U );
  static constexpr uint32_t CR3_EIE_Pos    = ( 0U );
  static constexpr uint32_t CR3_EIE_Msk    = ( 0x1U << CR3_EIE_Pos );
  static constexpr uint32_t CR3_EIE        = CR3_EIE_Msk;
  static constexpr uint32_t CR3_IREN_Pos   = ( 1U );
  static constexpr uint32_t CR3_IREN_Msk   = ( 0x1U << CR3_IREN_Pos );
  static constexpr uint32_t CR3_IREN       = CR3_IREN_Msk;
  static constexpr uint32_t CR3_IRLP_Pos   = ( 2U );
  static constexpr uint32_t CR3_IRLP_Msk   = ( 0x1U << CR3_IRLP_Pos );
  static constexpr uint32_t CR3_IRLP       = CR3_IRLP_Msk;
  static constexpr uint32_t CR3_HDSEL_Pos  = ( 3U );
  static constexpr uint32_t CR3_HDSEL_Msk  = ( 0x1U << CR3_HDSEL_Pos );
  static constexpr uint32_t CR3_HDSEL      = CR3_HDSEL_Msk;
  static constexpr uint32_t CR3_NACK_Pos   = ( 4U );
  static constexpr uint32_t CR3_NACK_Msk   = ( 0x1U << CR3_NACK_Pos );
  static constexpr uint32_t CR3_NACK       = CR3_NACK_Msk;
  static constexpr uint32_t CR3_SCEN_Pos   = ( 5U );
  static constexpr uint32_t CR3_SCEN_Msk   = ( 0x1U << CR3_SCEN_Pos );
  static constexpr uint32_t CR3_SCEN       = CR3_SCEN_Msk;
  static constexpr uint32_t CR3_DMAR_Pos   = ( 6U );
  static constexpr uint32_t CR3_DMAR_Msk   = ( 0x1U << CR3_DMAR_Pos );
  static constexpr uint32_t CR3_DMAR       = CR3_DMAR_Msk;
  static constexpr uint32_t CR3_DMAT_Pos   = ( 7U );
  static constexpr uint32_t CR3_DMAT_Msk   = ( 0x1U << CR3_DMAT_Pos );
  static constexpr uint32_t CR3_DMAT       = CR3_DMAT_Msk;
  static constexpr uint32_t CR3_RTSE_Pos   = ( 8U );
  static constexpr uint32_t CR3_RTSE_Msk   = ( 0x1U << CR3_RTSE_Pos );
  static constexpr uint32_t CR3_RTSE       = CR3_RTSE_Msk;
  static constexpr uint32_t CR3_CTSE_Pos   = ( 9U );
  static constexpr uint32_t CR3_CTSE_Msk   = ( 0x1U << CR3_CTSE_Pos );
  static constexpr uint32_t CR3_CTSE       = CR3_CTSE_Msk;
  static constexpr uint32_t CR3_CTSIE_Pos  = ( 10U );
  static constexpr uint32_t CR3_CTSIE_Msk  = ( 0x1U << CR3_CTSIE_Pos );
  static constexpr uint32_t CR3_CTSIE      = CR3_CTSIE_Msk;
  static constexpr uint32_t CR3_ONEBIT_Pos = ( 11U );
  static constexpr uint32_t CR3_ONEBIT_Msk = ( 0x1U << CR3_ONEBIT_Pos );
  static constexpr uint32_t CR3_ONEBIT     = CR3_ONEBIT_Msk;

  /*------------------------------------------------
  USART Guard Time and Prescaler Register
  ------------------------------------------------*/
  static constexpr uint32_t GTPR_Msk     = ( 0xFFFF );
  static constexpr uint32_t GTPR_Rst     = ( 0U );
  static constexpr uint32_t GTPR_PSC_Pos = ( 0U );
  static constexpr uint32_t GTPR_PSC_Msk = ( 0xFFU << GTPR_PSC_Pos );
  static constexpr uint32_t GTPR_PSC     = GTPR_PSC_Msk;
  static constexpr uint32_t GTPR_PSC_0   = ( 0x01U << GTPR_PSC_Pos );
  static constexpr uint32_t GTPR_PSC_1   = ( 0x02U << GTPR_PSC_Pos );
  static constexpr uint32_t GTPR_PSC_2   = ( 0x04U << GTPR_PSC_Pos );
  static constexpr uint32_t GTPR_PSC_3   = ( 0x08U << GTPR_PSC_Pos );
  static constexpr uint32_t GTPR_PSC_4   = ( 0x10U << GTPR_PSC_Pos );
  static constexpr uint32_t GTPR_PSC_5   = ( 0x20U << GTPR_PSC_Pos );
  static constexpr uint32_t GTPR_PSC_6   = ( 0x40U << GTPR_PSC_Pos );
  static constexpr uint32_t GTPR_PSC_7   = ( 0x80U << GTPR_PSC_Pos );
  static constexpr uint32_t GTPR_GT_Pos  = ( 8U );
  static constexpr uint32_t GTPR_GT_Msk  = ( 0xFFU << GTPR_GT_Pos );
  static constexpr uint32_t GTPR_GT      = GTPR_GT_Msk;
}    // namespace Thor::Driver::USART

#endif /* TARGET_STM32F4 && THOR_DRIVER_USART */
#endif /* !THOR_HW_USART_REGISTER_HPP */