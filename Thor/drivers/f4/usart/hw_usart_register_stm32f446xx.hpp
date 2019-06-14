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
#include <cstdint>

/* Driver Includes */
#include <Thor/drivers/f4/system/sys_memory_map_stm32f446xx.hpp>

namespace Thor::Driver::USART
{
  static constexpr uint32_t USART1_BASE = Thor::System::MemoryMap::APB2PERIPH_BASE + 0x1000U;
  static constexpr uint32_t USART2_BASE = Thor::System::MemoryMap::APB1PERIPH_BASE + 0x4400U;
  static constexpr uint32_t USART3_BASE = Thor::System::MemoryMap::APB1PERIPH_BASE + 0x4800U;
  static constexpr uint32_t UART4_BASE  = Thor::System::MemoryMap::APB1PERIPH_BASE + 0x4C00U;
  static constexpr uint32_t UART5_BASE  = Thor::System::MemoryMap::APB1PERIPH_BASE + 0x5000U;
  static constexpr uint32_t USART6_BASE = Thor::System::MemoryMap::APB2PERIPH_BASE + 0x1400U;

  /*------------------------------------------------
  USART Status Register
  ------------------------------------------------*/
  static constexpr uint32_t USART_SR_PE_Pos   = ( 0U );
  static constexpr uint32_t USART_SR_PE_Msk   = ( 0x1U << USART_SR_PE_Pos ); /**< 0x00000001 */
  static constexpr uint32_t USART_SR_PE       = USART_SR_PE_Msk;             /**<Parity Error                 */
  static constexpr uint32_t USART_SR_FE_Pos   = ( 1U );
  static constexpr uint32_t USART_SR_FE_Msk   = ( 0x1U << USART_SR_FE_Pos ); /**< 0x00000002 */
  static constexpr uint32_t USART_SR_FE       = USART_SR_FE_Msk;             /**<Framing Error                */
  static constexpr uint32_t USART_SR_NE_Pos   = ( 2U );
  static constexpr uint32_t USART_SR_NE_Msk   = ( 0x1U << USART_SR_NE_Pos ); /**< 0x00000004 */
  static constexpr uint32_t USART_SR_NE       = USART_SR_NE_Msk;             /**<Noise Error Flag             */
  static constexpr uint32_t USART_SR_ORE_Pos  = ( 3U );
  static constexpr uint32_t USART_SR_ORE_Msk  = ( 0x1U << USART_SR_ORE_Pos ); /**< 0x00000008 */
  static constexpr uint32_t USART_SR_ORE      = USART_SR_ORE_Msk;             /**<OverRun Error                */
  static constexpr uint32_t USART_SR_IDLE_Pos = ( 4U );
  static constexpr uint32_t USART_SR_IDLE_Msk = ( 0x1U << USART_SR_IDLE_Pos ); /**< 0x00000010 */
  static constexpr uint32_t USART_SR_IDLE     = USART_SR_IDLE_Msk;             /**<IDLE line detected           */
  static constexpr uint32_t USART_SR_RXNE_Pos = ( 5U );
  static constexpr uint32_t USART_SR_RXNE_Msk = ( 0x1U << USART_SR_RXNE_Pos ); /**< 0x00000020 */
  static constexpr uint32_t USART_SR_RXNE     = USART_SR_RXNE_Msk;             /**<Read Data Register Not Empty */
  static constexpr uint32_t USART_SR_TC_Pos   = ( 6U );
  static constexpr uint32_t USART_SR_TC_Msk   = ( 0x1U << USART_SR_TC_Pos ); /**< 0x00000040 */
  static constexpr uint32_t USART_SR_TC       = USART_SR_TC_Msk;             /**<Transmission Complete        */
  static constexpr uint32_t USART_SR_TXE_Pos  = ( 7U );
  static constexpr uint32_t USART_SR_TXE_Msk  = ( 0x1U << USART_SR_TXE_Pos ); /**< 0x00000080 */
  static constexpr uint32_t USART_SR_TXE      = USART_SR_TXE_Msk;             /**<Transmit Data Register Empty */
  static constexpr uint32_t USART_SR_LBD_Pos  = ( 8U );
  static constexpr uint32_t USART_SR_LBD_Msk  = ( 0x1U << USART_SR_LBD_Pos ); /**< 0x00000100 */
  static constexpr uint32_t USART_SR_LBD      = USART_SR_LBD_Msk;             /**<LIN Break Detection Flag     */
  static constexpr uint32_t USART_SR_CTS_Pos  = ( 9U );
  static constexpr uint32_t USART_SR_CTS_Msk  = ( 0x1U << USART_SR_CTS_Pos ); /**< 0x00000200 */
  static constexpr uint32_t USART_SR_CTS      = USART_SR_CTS_Msk;             /**<CTS Flag                     */

  /*------------------------------------------------
  USART Data Register
  ------------------------------------------------*/
  static constexpr uint32_t USART_DR_DR_Pos = ( 0U );
  static constexpr uint32_t USART_DR_DR_Msk = ( 0x1FFU << USART_DR_DR_Pos ); /**< 0x000001FF */
  static constexpr uint32_t USART_DR_DR     = USART_DR_DR_Msk;               /**<Data value */

  /*------------------------------------------------
  USART Baud Rate Register
  ------------------------------------------------*/
  static constexpr uint32_t USART_BRR_DIV_Fraction_Pos = ( 0U );
  static constexpr uint32_t USART_BRR_DIV_Fraction_Msk = ( 0xFU << USART_BRR_DIV_Fraction_Pos ); /**< 0x0000000F */
  static constexpr uint32_t USART_BRR_DIV_Fraction     = USART_BRR_DIV_Fraction_Msk;             /**<Fraction of USARTDIV */
  static constexpr uint32_t USART_BRR_DIV_Mantissa_Pos = ( 4U );
  static constexpr uint32_t USART_BRR_DIV_Mantissa_Msk = ( 0xFFFU << USART_BRR_DIV_Mantissa_Pos ); /**< 0x0000FFF0 */
  static constexpr uint32_t USART_BRR_DIV_Mantissa     = USART_BRR_DIV_Mantissa_Msk;               /**<Mantissa of USARTDIV */

  /*------------------------------------------------
  USART Control Register 1
  ------------------------------------------------*/
  static constexpr uint32_t USART_CR1_SBK_Pos    = ( 0U );
  static constexpr uint32_t USART_CR1_SBK_Msk    = ( 0x1U << USART_CR1_SBK_Pos ); /**< 0x00000001 */
  static constexpr uint32_t USART_CR1_SBK        = USART_CR1_SBK_Msk;             /**<Send Break                             */
  static constexpr uint32_t USART_CR1_RWU_Pos    = ( 1U );
  static constexpr uint32_t USART_CR1_RWU_Msk    = ( 0x1U << USART_CR1_RWU_Pos ); /**< 0x00000002 */
  static constexpr uint32_t USART_CR1_RWU        = USART_CR1_RWU_Msk;             /**<Receiver wakeup                        */
  static constexpr uint32_t USART_CR1_RE_Pos     = ( 2U );
  static constexpr uint32_t USART_CR1_RE_Msk     = ( 0x1U << USART_CR1_RE_Pos ); /**< 0x00000004 */
  static constexpr uint32_t USART_CR1_RE         = USART_CR1_RE_Msk;             /**<Receiver Enable                        */
  static constexpr uint32_t USART_CR1_TE_Pos     = ( 3U );
  static constexpr uint32_t USART_CR1_TE_Msk     = ( 0x1U << USART_CR1_TE_Pos ); /**< 0x00000008 */
  static constexpr uint32_t USART_CR1_TE         = USART_CR1_TE_Msk;             /**<Transmitter Enable                     */
  static constexpr uint32_t USART_CR1_IDLEIE_Pos = ( 4U );
  static constexpr uint32_t USART_CR1_IDLEIE_Msk = ( 0x1U << USART_CR1_IDLEIE_Pos ); /**< 0x00000010 */
  static constexpr uint32_t USART_CR1_IDLEIE     = USART_CR1_IDLEIE_Msk; /**<IDLE Interrupt Enable                  */
  static constexpr uint32_t USART_CR1_RXNEIE_Pos = ( 5U );
  static constexpr uint32_t USART_CR1_RXNEIE_Msk = ( 0x1U << USART_CR1_RXNEIE_Pos ); /**< 0x00000020 */
  static constexpr uint32_t USART_CR1_RXNEIE     = USART_CR1_RXNEIE_Msk; /**<RXNE Interrupt Enable                  */
  static constexpr uint32_t USART_CR1_TCIE_Pos   = ( 6U );
  static constexpr uint32_t USART_CR1_TCIE_Msk   = ( 0x1U << USART_CR1_TCIE_Pos ); /**< 0x00000040 */
  static constexpr uint32_t USART_CR1_TCIE       = USART_CR1_TCIE_Msk;             /**<Transmission Complete Interrupt Enable */
  static constexpr uint32_t USART_CR1_TXEIE_Pos  = ( 7U );
  static constexpr uint32_t USART_CR1_TXEIE_Msk  = ( 0x1U << USART_CR1_TXEIE_Pos ); /**< 0x00000080 */
  static constexpr uint32_t USART_CR1_TXEIE      = USART_CR1_TXEIE_Msk; /**<TXE Interrupt Enable                   */
  static constexpr uint32_t USART_CR1_PEIE_Pos   = ( 8U );
  static constexpr uint32_t USART_CR1_PEIE_Msk   = ( 0x1U << USART_CR1_PEIE_Pos ); /**< 0x00000100 */
  static constexpr uint32_t USART_CR1_PEIE       = USART_CR1_PEIE_Msk;             /**<PE Interrupt Enable                    */
  static constexpr uint32_t USART_CR1_PS_Pos     = ( 9U );
  static constexpr uint32_t USART_CR1_PS_Msk     = ( 0x1U << USART_CR1_PS_Pos ); /**< 0x00000200 */
  static constexpr uint32_t USART_CR1_PS         = USART_CR1_PS_Msk;             /**<Parity Selection                       */
  static constexpr uint32_t USART_CR1_PCE_Pos    = ( 10U );
  static constexpr uint32_t USART_CR1_PCE_Msk    = ( 0x1U << USART_CR1_PCE_Pos ); /**< 0x00000400 */
  static constexpr uint32_t USART_CR1_PCE        = USART_CR1_PCE_Msk;             /**<Parity Control Enable                  */
  static constexpr uint32_t USART_CR1_WAKE_Pos   = ( 11U );
  static constexpr uint32_t USART_CR1_WAKE_Msk   = ( 0x1U << USART_CR1_WAKE_Pos ); /**< 0x00000800 */
  static constexpr uint32_t USART_CR1_WAKE       = USART_CR1_WAKE_Msk;             /**<Wakeup method                          */
  static constexpr uint32_t USART_CR1_M_Pos      = ( 12U );
  static constexpr uint32_t USART_CR1_M_Msk      = ( 0x1U << USART_CR1_M_Pos ); /**< 0x00001000 */
  static constexpr uint32_t USART_CR1_M          = USART_CR1_M_Msk;             /**<Word length                            */
  static constexpr uint32_t USART_CR1_UE_Pos     = ( 13U );
  static constexpr uint32_t USART_CR1_UE_Msk     = ( 0x1U << USART_CR1_UE_Pos ); /**< 0x00002000 */
  static constexpr uint32_t USART_CR1_UE         = USART_CR1_UE_Msk;             /**<USART Enable                           */
  static constexpr uint32_t USART_CR1_OVER8_Pos  = ( 15U );
  static constexpr uint32_t USART_CR1_OVER8_Msk  = ( 0x1U << USART_CR1_OVER8_Pos ); /**< 0x00008000 */
  static constexpr uint32_t USART_CR1_OVER8      = USART_CR1_OVER8_Msk; /**<USART Oversampling by 8 enable         */

  /*------------------------------------------------
  USART Control Register 2
  ------------------------------------------------*/
  static constexpr uint32_t USART_CR2_ADD_Pos   = ( 0U );
  static constexpr uint32_t USART_CR2_ADD_Msk   = ( 0xFU << USART_CR2_ADD_Pos ); /**< 0x0000000F */
  static constexpr uint32_t USART_CR2_ADD       = USART_CR2_ADD_Msk;             /**<Address of the USART node            */
  static constexpr uint32_t USART_CR2_LBDL_Pos  = ( 5U );
  static constexpr uint32_t USART_CR2_LBDL_Msk  = ( 0x1U << USART_CR2_LBDL_Pos ); /**< 0x00000020 */
  static constexpr uint32_t USART_CR2_LBDL      = USART_CR2_LBDL_Msk;             /**<LIN Break Detection Length           */
  static constexpr uint32_t USART_CR2_LBDIE_Pos = ( 6U );
  static constexpr uint32_t USART_CR2_LBDIE_Msk = ( 0x1U << USART_CR2_LBDIE_Pos ); /**< 0x00000040 */
  static constexpr uint32_t USART_CR2_LBDIE     = USART_CR2_LBDIE_Msk;             /**<LIN Break Detection Interrupt Enable */
  static constexpr uint32_t USART_CR2_LBCL_Pos  = ( 8U );
  static constexpr uint32_t USART_CR2_LBCL_Msk  = ( 0x1U << USART_CR2_LBCL_Pos ); /**< 0x00000100 */
  static constexpr uint32_t USART_CR2_LBCL      = USART_CR2_LBCL_Msk;             /**<Last Bit Clock pulse                 */
  static constexpr uint32_t USART_CR2_CPHA_Pos  = ( 9U );
  static constexpr uint32_t USART_CR2_CPHA_Msk  = ( 0x1U << USART_CR2_CPHA_Pos ); /**< 0x00000200 */
  static constexpr uint32_t USART_CR2_CPHA      = USART_CR2_CPHA_Msk;             /**<Clock Phase                          */
  static constexpr uint32_t USART_CR2_CPOL_Pos  = ( 10U );
  static constexpr uint32_t USART_CR2_CPOL_Msk  = ( 0x1U << USART_CR2_CPOL_Pos ); /**< 0x00000400 */
  static constexpr uint32_t USART_CR2_CPOL      = USART_CR2_CPOL_Msk;             /**<Clock Polarity                       */
  static constexpr uint32_t USART_CR2_CLKEN_Pos = ( 11U );
  static constexpr uint32_t USART_CR2_CLKEN_Msk = ( 0x1U << USART_CR2_CLKEN_Pos ); /**< 0x00000800 */
  static constexpr uint32_t USART_CR2_CLKEN     = USART_CR2_CLKEN_Msk;             /**<Clock Enable                         */
  static constexpr uint32_t USART_CR2_STOP_Pos  = ( 12U );
  static constexpr uint32_t USART_CR2_STOP_Msk  = ( 0x3U << USART_CR2_STOP_Pos ); /**< 0x00003000 */
  static constexpr uint32_t USART_CR2_STOP      = USART_CR2_STOP_Msk;             /**<STOP[1:0] bits (STOP bits) */
  static constexpr uint32_t USART_CR2_STOP_0    = ( 0x1U << USART_CR2_STOP_Pos ); /**< 0x1000 */
  static constexpr uint32_t USART_CR2_STOP_1    = ( 0x2U << USART_CR2_STOP_Pos ); /**< 0x2000 */
  static constexpr uint32_t USART_CR2_LINEN_Pos = ( 14U );
  static constexpr uint32_t USART_CR2_LINEN_Msk = ( 0x1U << USART_CR2_LINEN_Pos ); /**< 0x00004000 */
  static constexpr uint32_t USART_CR2_LINEN     = USART_CR2_LINEN_Msk;             /**<LIN mode enable */

  /*------------------------------------------------
  USART Control Register 3
  ------------------------------------------------*/
  static constexpr uint32_t USART_CR3_EIE_Pos    = ( 0U );
  static constexpr uint32_t USART_CR3_EIE_Msk    = ( 0x1U << USART_CR3_EIE_Pos ); /**< 0x00000001 */
  static constexpr uint32_t USART_CR3_EIE        = USART_CR3_EIE_Msk;             /**<Error Interrupt Enable      */
  static constexpr uint32_t USART_CR3_IREN_Pos   = ( 1U );
  static constexpr uint32_t USART_CR3_IREN_Msk   = ( 0x1U << USART_CR3_IREN_Pos ); /**< 0x00000002 */
  static constexpr uint32_t USART_CR3_IREN       = USART_CR3_IREN_Msk;             /**<IrDA mode Enable            */
  static constexpr uint32_t USART_CR3_IRLP_Pos   = ( 2U );
  static constexpr uint32_t USART_CR3_IRLP_Msk   = ( 0x1U << USART_CR3_IRLP_Pos ); /**< 0x00000004 */
  static constexpr uint32_t USART_CR3_IRLP       = USART_CR3_IRLP_Msk;             /**<IrDA Low-Power              */
  static constexpr uint32_t USART_CR3_HDSEL_Pos  = ( 3U );
  static constexpr uint32_t USART_CR3_HDSEL_Msk  = ( 0x1U << USART_CR3_HDSEL_Pos ); /**< 0x00000008 */
  static constexpr uint32_t USART_CR3_HDSEL      = USART_CR3_HDSEL_Msk;             /**<Half-Duplex Selection       */
  static constexpr uint32_t USART_CR3_NACK_Pos   = ( 4U );
  static constexpr uint32_t USART_CR3_NACK_Msk   = ( 0x1U << USART_CR3_NACK_Pos ); /**< 0x00000010 */
  static constexpr uint32_t USART_CR3_NACK       = USART_CR3_NACK_Msk;             /**<Smartcard NACK enable       */
  static constexpr uint32_t USART_CR3_SCEN_Pos   = ( 5U );
  static constexpr uint32_t USART_CR3_SCEN_Msk   = ( 0x1U << USART_CR3_SCEN_Pos ); /**< 0x00000020 */
  static constexpr uint32_t USART_CR3_SCEN       = USART_CR3_SCEN_Msk;             /**<Smartcard mode enable       */
  static constexpr uint32_t USART_CR3_DMAR_Pos   = ( 6U );
  static constexpr uint32_t USART_CR3_DMAR_Msk   = ( 0x1U << USART_CR3_DMAR_Pos ); /**< 0x00000040 */
  static constexpr uint32_t USART_CR3_DMAR       = USART_CR3_DMAR_Msk;             /**<DMA Enable Receiver         */
  static constexpr uint32_t USART_CR3_DMAT_Pos   = ( 7U );
  static constexpr uint32_t USART_CR3_DMAT_Msk   = ( 0x1U << USART_CR3_DMAT_Pos ); /**< 0x00000080 */
  static constexpr uint32_t USART_CR3_DMAT       = USART_CR3_DMAT_Msk;             /**<DMA Enable Transmitter      */
  static constexpr uint32_t USART_CR3_RTSE_Pos   = ( 8U );
  static constexpr uint32_t USART_CR3_RTSE_Msk   = ( 0x1U << USART_CR3_RTSE_Pos ); /**< 0x00000100 */
  static constexpr uint32_t USART_CR3_RTSE       = USART_CR3_RTSE_Msk;             /**<RTS Enable                  */
  static constexpr uint32_t USART_CR3_CTSE_Pos   = ( 9U );
  static constexpr uint32_t USART_CR3_CTSE_Msk   = ( 0x1U << USART_CR3_CTSE_Pos ); /**< 0x00000200 */
  static constexpr uint32_t USART_CR3_CTSE       = USART_CR3_CTSE_Msk;             /**<CTS Enable                  */
  static constexpr uint32_t USART_CR3_CTSIE_Pos  = ( 10U );
  static constexpr uint32_t USART_CR3_CTSIE_Msk  = ( 0x1U << USART_CR3_CTSIE_Pos ); /**< 0x00000400 */
  static constexpr uint32_t USART_CR3_CTSIE      = USART_CR3_CTSIE_Msk;             /**<CTS Interrupt Enable        */
  static constexpr uint32_t USART_CR3_ONEBIT_Pos = ( 11U );
  static constexpr uint32_t USART_CR3_ONEBIT_Msk = ( 0x1U << USART_CR3_ONEBIT_Pos ); /**< 0x00000800 */
  static constexpr uint32_t USART_CR3_ONEBIT     = USART_CR3_ONEBIT_Msk;             /**<USART One bit method enable */

  /*------------------------------------------------
  USART Guard Time and Prescaler Register
  ------------------------------------------------*/
  static constexpr uint32_t USART_GTPR_PSC_Pos = ( 0U );
  static constexpr uint32_t USART_GTPR_PSC_Msk = ( 0xFFU << USART_GTPR_PSC_Pos ); /**< 0x000000FF */
  static constexpr uint32_t USART_GTPR_PSC     = USART_GTPR_PSC_Msk;              /**<PSC[7:0] bits (Prescaler value) */
  static constexpr uint32_t USART_GTPR_PSC_0   = ( 0x01U << USART_GTPR_PSC_Pos ); /**< 0x0001 */
  static constexpr uint32_t USART_GTPR_PSC_1   = ( 0x02U << USART_GTPR_PSC_Pos ); /**< 0x0002 */
  static constexpr uint32_t USART_GTPR_PSC_2   = ( 0x04U << USART_GTPR_PSC_Pos ); /**< 0x0004 */
  static constexpr uint32_t USART_GTPR_PSC_3   = ( 0x08U << USART_GTPR_PSC_Pos ); /**< 0x0008 */
  static constexpr uint32_t USART_GTPR_PSC_4   = ( 0x10U << USART_GTPR_PSC_Pos ); /**< 0x0010 */
  static constexpr uint32_t USART_GTPR_PSC_5   = ( 0x20U << USART_GTPR_PSC_Pos ); /**< 0x0020 */
  static constexpr uint32_t USART_GTPR_PSC_6   = ( 0x40U << USART_GTPR_PSC_Pos ); /**< 0x0040 */
  static constexpr uint32_t USART_GTPR_PSC_7   = ( 0x80U << USART_GTPR_PSC_Pos ); /**< 0x0080 */

  static constexpr uint32_t USART_GTPR_GT_Pos = ( 8U );
  static constexpr uint32_t USART_GTPR_GT_Msk = ( 0xFFU << USART_GTPR_GT_Pos ); /**< 0x0000FF00 */
  static constexpr uint32_t USART_GTPR_GT     = USART_GTPR_GT_Msk;              /**<Guard time value */
}    // namespace Thor::Driver::USART

#endif /* !THOR_HW_USART_REGISTER_HPP */