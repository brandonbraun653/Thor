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
  static constexpr uint32_t USART1_BASE_ADDR = Thor::System::MemoryMap::APB2PERIPH_BASE_ADDR + 0x1000U;
  static constexpr uint32_t USART2_BASE_ADDR = Thor::System::MemoryMap::APB1PERIPH_BASE_ADDR + 0x4400U;
  static constexpr uint32_t USART3_BASE_ADDR = Thor::System::MemoryMap::APB1PERIPH_BASE_ADDR + 0x4800U;
  static constexpr uint32_t USART6_BASE_ADDR = Thor::System::MemoryMap::APB2PERIPH_BASE_ADDR + 0x1400U;

  /*------------------------------------------------
  USART Status Register
  ------------------------------------------------*/
  static constexpr uint32_t BASE_ADDRSR_PE_Pos   = ( 0U );
  static constexpr uint32_t BASE_ADDRSR_PE_Msk   = ( 0x1U << BASE_ADDRSR_PE_Pos ); /**< 0x00000001 */
  static constexpr uint32_t BASE_ADDRSR_PE       = BASE_ADDRSR_PE_Msk;             /**<Parity Error                 */
  static constexpr uint32_t BASE_ADDRSR_FE_Pos   = ( 1U );
  static constexpr uint32_t BASE_ADDRSR_FE_Msk   = ( 0x1U << BASE_ADDRSR_FE_Pos ); /**< 0x00000002 */
  static constexpr uint32_t BASE_ADDRSR_FE       = BASE_ADDRSR_FE_Msk;             /**<Framing Error                */
  static constexpr uint32_t BASE_ADDRSR_NE_Pos   = ( 2U );
  static constexpr uint32_t BASE_ADDRSR_NE_Msk   = ( 0x1U << BASE_ADDRSR_NE_Pos ); /**< 0x00000004 */
  static constexpr uint32_t BASE_ADDRSR_NE       = BASE_ADDRSR_NE_Msk;             /**<Noise Error Flag             */
  static constexpr uint32_t BASE_ADDRSR_ORE_Pos  = ( 3U );
  static constexpr uint32_t BASE_ADDRSR_ORE_Msk  = ( 0x1U << BASE_ADDRSR_ORE_Pos ); /**< 0x00000008 */
  static constexpr uint32_t BASE_ADDRSR_ORE      = BASE_ADDRSR_ORE_Msk;             /**<OverRun Error                */
  static constexpr uint32_t BASE_ADDRSR_IDLE_Pos = ( 4U );
  static constexpr uint32_t BASE_ADDRSR_IDLE_Msk = ( 0x1U << BASE_ADDRSR_IDLE_Pos ); /**< 0x00000010 */
  static constexpr uint32_t BASE_ADDRSR_IDLE     = BASE_ADDRSR_IDLE_Msk;             /**<IDLE line detected           */
  static constexpr uint32_t BASE_ADDRSR_RXNE_Pos = ( 5U );
  static constexpr uint32_t BASE_ADDRSR_RXNE_Msk = ( 0x1U << BASE_ADDRSR_RXNE_Pos ); /**< 0x00000020 */
  static constexpr uint32_t BASE_ADDRSR_RXNE     = BASE_ADDRSR_RXNE_Msk;             /**<Read Data Register Not Empty */
  static constexpr uint32_t BASE_ADDRSR_TC_Pos   = ( 6U );
  static constexpr uint32_t BASE_ADDRSR_TC_Msk   = ( 0x1U << BASE_ADDRSR_TC_Pos ); /**< 0x00000040 */
  static constexpr uint32_t BASE_ADDRSR_TC       = BASE_ADDRSR_TC_Msk;             /**<Transmission Complete        */
  static constexpr uint32_t BASE_ADDRSR_TXE_Pos  = ( 7U );
  static constexpr uint32_t BASE_ADDRSR_TXE_Msk  = ( 0x1U << BASE_ADDRSR_TXE_Pos ); /**< 0x00000080 */
  static constexpr uint32_t BASE_ADDRSR_TXE      = BASE_ADDRSR_TXE_Msk;             /**<Transmit Data Register Empty */
  static constexpr uint32_t BASE_ADDRSR_LBD_Pos  = ( 8U );
  static constexpr uint32_t BASE_ADDRSR_LBD_Msk  = ( 0x1U << BASE_ADDRSR_LBD_Pos ); /**< 0x00000100 */
  static constexpr uint32_t BASE_ADDRSR_LBD      = BASE_ADDRSR_LBD_Msk;             /**<LIN Break Detection Flag     */
  static constexpr uint32_t BASE_ADDRSR_CTS_Pos  = ( 9U );
  static constexpr uint32_t BASE_ADDRSR_CTS_Msk  = ( 0x1U << BASE_ADDRSR_CTS_Pos ); /**< 0x00000200 */
  static constexpr uint32_t BASE_ADDRSR_CTS      = BASE_ADDRSR_CTS_Msk;             /**<CTS Flag                     */

  /*------------------------------------------------
  USART Data Register
  ------------------------------------------------*/
  static constexpr uint32_t BASE_ADDRDR_DR_Pos = ( 0U );
  static constexpr uint32_t BASE_ADDRDR_DR_Msk = ( 0x1FFU << BASE_ADDRDR_DR_Pos ); /**< 0x000001FF */
  static constexpr uint32_t BASE_ADDRDR_DR     = BASE_ADDRDR_DR_Msk;               /**<Data value */

  /*------------------------------------------------
  USART Baud Rate Register
  ------------------------------------------------*/
  static constexpr uint32_t BASE_ADDRBRR_DIV_Fraction_Pos = ( 0U );
  static constexpr uint32_t BASE_ADDRBRR_DIV_Fraction_Msk = ( 0xFU << BASE_ADDRBRR_DIV_Fraction_Pos ); /**< 0x0000000F */
  static constexpr uint32_t BASE_ADDRBRR_DIV_Fraction     = BASE_ADDRBRR_DIV_Fraction_Msk;             /**<Fraction of USARTDIV */
  static constexpr uint32_t BASE_ADDRBRR_DIV_Mantissa_Pos = ( 4U );
  static constexpr uint32_t BASE_ADDRBRR_DIV_Mantissa_Msk = ( 0xFFFU << BASE_ADDRBRR_DIV_Mantissa_Pos ); /**< 0x0000FFF0 */
  static constexpr uint32_t BASE_ADDRBRR_DIV_Mantissa     = BASE_ADDRBRR_DIV_Mantissa_Msk;               /**<Mantissa of USARTDIV */

  /*------------------------------------------------
  USART Control Register 1
  ------------------------------------------------*/
  static constexpr uint32_t BASE_ADDRCR1_SBK_Pos    = ( 0U );
  static constexpr uint32_t BASE_ADDRCR1_SBK_Msk    = ( 0x1U << BASE_ADDRCR1_SBK_Pos ); /**< 0x00000001 */
  static constexpr uint32_t BASE_ADDRCR1_SBK        = BASE_ADDRCR1_SBK_Msk;             /**<Send Break                             */
  static constexpr uint32_t BASE_ADDRCR1_RWU_Pos    = ( 1U );
  static constexpr uint32_t BASE_ADDRCR1_RWU_Msk    = ( 0x1U << BASE_ADDRCR1_RWU_Pos ); /**< 0x00000002 */
  static constexpr uint32_t BASE_ADDRCR1_RWU        = BASE_ADDRCR1_RWU_Msk;             /**<Receiver wakeup                        */
  static constexpr uint32_t BASE_ADDRCR1_RE_Pos     = ( 2U );
  static constexpr uint32_t BASE_ADDRCR1_RE_Msk     = ( 0x1U << BASE_ADDRCR1_RE_Pos ); /**< 0x00000004 */
  static constexpr uint32_t BASE_ADDRCR1_RE         = BASE_ADDRCR1_RE_Msk;             /**<Receiver Enable                        */
  static constexpr uint32_t BASE_ADDRCR1_TE_Pos     = ( 3U );
  static constexpr uint32_t BASE_ADDRCR1_TE_Msk     = ( 0x1U << BASE_ADDRCR1_TE_Pos ); /**< 0x00000008 */
  static constexpr uint32_t BASE_ADDRCR1_TE         = BASE_ADDRCR1_TE_Msk;             /**<Transmitter Enable                     */
  static constexpr uint32_t BASE_ADDRCR1_IDLEIE_Pos = ( 4U );
  static constexpr uint32_t BASE_ADDRCR1_IDLEIE_Msk = ( 0x1U << BASE_ADDRCR1_IDLEIE_Pos ); /**< 0x00000010 */
  static constexpr uint32_t BASE_ADDRCR1_IDLEIE     = BASE_ADDRCR1_IDLEIE_Msk; /**<IDLE Interrupt Enable                  */
  static constexpr uint32_t BASE_ADDRCR1_RXNEIE_Pos = ( 5U );
  static constexpr uint32_t BASE_ADDRCR1_RXNEIE_Msk = ( 0x1U << BASE_ADDRCR1_RXNEIE_Pos ); /**< 0x00000020 */
  static constexpr uint32_t BASE_ADDRCR1_RXNEIE     = BASE_ADDRCR1_RXNEIE_Msk; /**<RXNE Interrupt Enable                  */
  static constexpr uint32_t BASE_ADDRCR1_TCIE_Pos   = ( 6U );
  static constexpr uint32_t BASE_ADDRCR1_TCIE_Msk   = ( 0x1U << BASE_ADDRCR1_TCIE_Pos ); /**< 0x00000040 */
  static constexpr uint32_t BASE_ADDRCR1_TCIE       = BASE_ADDRCR1_TCIE_Msk;             /**<Transmission Complete Interrupt Enable */
  static constexpr uint32_t BASE_ADDRCR1_TXEIE_Pos  = ( 7U );
  static constexpr uint32_t BASE_ADDRCR1_TXEIE_Msk  = ( 0x1U << BASE_ADDRCR1_TXEIE_Pos ); /**< 0x00000080 */
  static constexpr uint32_t BASE_ADDRCR1_TXEIE      = BASE_ADDRCR1_TXEIE_Msk; /**<TXE Interrupt Enable                   */
  static constexpr uint32_t BASE_ADDRCR1_PEIE_Pos   = ( 8U );
  static constexpr uint32_t BASE_ADDRCR1_PEIE_Msk   = ( 0x1U << BASE_ADDRCR1_PEIE_Pos ); /**< 0x00000100 */
  static constexpr uint32_t BASE_ADDRCR1_PEIE       = BASE_ADDRCR1_PEIE_Msk;             /**<PE Interrupt Enable                    */
  static constexpr uint32_t BASE_ADDRCR1_PS_Pos     = ( 9U );
  static constexpr uint32_t BASE_ADDRCR1_PS_Msk     = ( 0x1U << BASE_ADDRCR1_PS_Pos ); /**< 0x00000200 */
  static constexpr uint32_t BASE_ADDRCR1_PS         = BASE_ADDRCR1_PS_Msk;             /**<Parity Selection                       */
  static constexpr uint32_t BASE_ADDRCR1_PCE_Pos    = ( 10U );
  static constexpr uint32_t BASE_ADDRCR1_PCE_Msk    = ( 0x1U << BASE_ADDRCR1_PCE_Pos ); /**< 0x00000400 */
  static constexpr uint32_t BASE_ADDRCR1_PCE        = BASE_ADDRCR1_PCE_Msk;             /**<Parity Control Enable                  */
  static constexpr uint32_t BASE_ADDRCR1_WAKE_Pos   = ( 11U );
  static constexpr uint32_t BASE_ADDRCR1_WAKE_Msk   = ( 0x1U << BASE_ADDRCR1_WAKE_Pos ); /**< 0x00000800 */
  static constexpr uint32_t BASE_ADDRCR1_WAKE       = BASE_ADDRCR1_WAKE_Msk;             /**<Wakeup method                          */
  static constexpr uint32_t BASE_ADDRCR1_M_Pos      = ( 12U );
  static constexpr uint32_t BASE_ADDRCR1_M_Msk      = ( 0x1U << BASE_ADDRCR1_M_Pos ); /**< 0x00001000 */
  static constexpr uint32_t BASE_ADDRCR1_M          = BASE_ADDRCR1_M_Msk;             /**<Word length                            */
  static constexpr uint32_t BASE_ADDRCR1_UE_Pos     = ( 13U );
  static constexpr uint32_t BASE_ADDRCR1_UE_Msk     = ( 0x1U << BASE_ADDRCR1_UE_Pos ); /**< 0x00002000 */
  static constexpr uint32_t BASE_ADDRCR1_UE         = BASE_ADDRCR1_UE_Msk;             /**<USART Enable                           */
  static constexpr uint32_t BASE_ADDRCR1_OVER8_Pos  = ( 15U );
  static constexpr uint32_t BASE_ADDRCR1_OVER8_Msk  = ( 0x1U << BASE_ADDRCR1_OVER8_Pos ); /**< 0x00008000 */
  static constexpr uint32_t BASE_ADDRCR1_OVER8      = BASE_ADDRCR1_OVER8_Msk; /**<USART Oversampling by 8 enable         */

  /*------------------------------------------------
  USART Control Register 2
  ------------------------------------------------*/
  static constexpr uint32_t BASE_ADDRCR2_ADD_Pos   = ( 0U );
  static constexpr uint32_t BASE_ADDRCR2_ADD_Msk   = ( 0xFU << BASE_ADDRCR2_ADD_Pos ); /**< 0x0000000F */
  static constexpr uint32_t BASE_ADDRCR2_ADD       = BASE_ADDRCR2_ADD_Msk;             /**<Address of the USART node            */
  static constexpr uint32_t BASE_ADDRCR2_LBDL_Pos  = ( 5U );
  static constexpr uint32_t BASE_ADDRCR2_LBDL_Msk  = ( 0x1U << BASE_ADDRCR2_LBDL_Pos ); /**< 0x00000020 */
  static constexpr uint32_t BASE_ADDRCR2_LBDL      = BASE_ADDRCR2_LBDL_Msk;             /**<LIN Break Detection Length           */
  static constexpr uint32_t BASE_ADDRCR2_LBDIE_Pos = ( 6U );
  static constexpr uint32_t BASE_ADDRCR2_LBDIE_Msk = ( 0x1U << BASE_ADDRCR2_LBDIE_Pos ); /**< 0x00000040 */
  static constexpr uint32_t BASE_ADDRCR2_LBDIE     = BASE_ADDRCR2_LBDIE_Msk;             /**<LIN Break Detection Interrupt Enable */
  static constexpr uint32_t BASE_ADDRCR2_LBCL_Pos  = ( 8U );
  static constexpr uint32_t BASE_ADDRCR2_LBCL_Msk  = ( 0x1U << BASE_ADDRCR2_LBCL_Pos ); /**< 0x00000100 */
  static constexpr uint32_t BASE_ADDRCR2_LBCL      = BASE_ADDRCR2_LBCL_Msk;             /**<Last Bit Clock pulse                 */
  static constexpr uint32_t BASE_ADDRCR2_CPHA_Pos  = ( 9U );
  static constexpr uint32_t BASE_ADDRCR2_CPHA_Msk  = ( 0x1U << BASE_ADDRCR2_CPHA_Pos ); /**< 0x00000200 */
  static constexpr uint32_t BASE_ADDRCR2_CPHA      = BASE_ADDRCR2_CPHA_Msk;             /**<Clock Phase                          */
  static constexpr uint32_t BASE_ADDRCR2_CPOL_Pos  = ( 10U );
  static constexpr uint32_t BASE_ADDRCR2_CPOL_Msk  = ( 0x1U << BASE_ADDRCR2_CPOL_Pos ); /**< 0x00000400 */
  static constexpr uint32_t BASE_ADDRCR2_CPOL      = BASE_ADDRCR2_CPOL_Msk;             /**<Clock Polarity                       */
  static constexpr uint32_t BASE_ADDRCR2_CLKEN_Pos = ( 11U );
  static constexpr uint32_t BASE_ADDRCR2_CLKEN_Msk = ( 0x1U << BASE_ADDRCR2_CLKEN_Pos ); /**< 0x00000800 */
  static constexpr uint32_t BASE_ADDRCR2_CLKEN     = BASE_ADDRCR2_CLKEN_Msk;             /**<Clock Enable                         */
  static constexpr uint32_t BASE_ADDRCR2_STOP_Pos  = ( 12U );
  static constexpr uint32_t BASE_ADDRCR2_STOP_Msk  = ( 0x3U << BASE_ADDRCR2_STOP_Pos ); /**< 0x00003000 */
  static constexpr uint32_t BASE_ADDRCR2_STOP      = BASE_ADDRCR2_STOP_Msk;             /**<STOP[1:0] bits (STOP bits) */
  static constexpr uint32_t BASE_ADDRCR2_STOP_0    = ( 0x1U << BASE_ADDRCR2_STOP_Pos ); /**< 0x1000 */
  static constexpr uint32_t BASE_ADDRCR2_STOP_1    = ( 0x2U << BASE_ADDRCR2_STOP_Pos ); /**< 0x2000 */
  static constexpr uint32_t BASE_ADDRCR2_LINEN_Pos = ( 14U );
  static constexpr uint32_t BASE_ADDRCR2_LINEN_Msk = ( 0x1U << BASE_ADDRCR2_LINEN_Pos ); /**< 0x00004000 */
  static constexpr uint32_t BASE_ADDRCR2_LINEN     = BASE_ADDRCR2_LINEN_Msk;             /**<LIN mode enable */

  /*------------------------------------------------
  USART Control Register 3
  ------------------------------------------------*/
  static constexpr uint32_t BASE_ADDRCR3_EIE_Pos    = ( 0U );
  static constexpr uint32_t BASE_ADDRCR3_EIE_Msk    = ( 0x1U << BASE_ADDRCR3_EIE_Pos ); /**< 0x00000001 */
  static constexpr uint32_t BASE_ADDRCR3_EIE        = BASE_ADDRCR3_EIE_Msk;             /**<Error Interrupt Enable      */
  static constexpr uint32_t BASE_ADDRCR3_IREN_Pos   = ( 1U );
  static constexpr uint32_t BASE_ADDRCR3_IREN_Msk   = ( 0x1U << BASE_ADDRCR3_IREN_Pos ); /**< 0x00000002 */
  static constexpr uint32_t BASE_ADDRCR3_IREN       = BASE_ADDRCR3_IREN_Msk;             /**<IrDA mode Enable            */
  static constexpr uint32_t BASE_ADDRCR3_IRLP_Pos   = ( 2U );
  static constexpr uint32_t BASE_ADDRCR3_IRLP_Msk   = ( 0x1U << BASE_ADDRCR3_IRLP_Pos ); /**< 0x00000004 */
  static constexpr uint32_t BASE_ADDRCR3_IRLP       = BASE_ADDRCR3_IRLP_Msk;             /**<IrDA Low-Power              */
  static constexpr uint32_t BASE_ADDRCR3_HDSEL_Pos  = ( 3U );
  static constexpr uint32_t BASE_ADDRCR3_HDSEL_Msk  = ( 0x1U << BASE_ADDRCR3_HDSEL_Pos ); /**< 0x00000008 */
  static constexpr uint32_t BASE_ADDRCR3_HDSEL      = BASE_ADDRCR3_HDSEL_Msk;             /**<Half-Duplex Selection       */
  static constexpr uint32_t BASE_ADDRCR3_NACK_Pos   = ( 4U );
  static constexpr uint32_t BASE_ADDRCR3_NACK_Msk   = ( 0x1U << BASE_ADDRCR3_NACK_Pos ); /**< 0x00000010 */
  static constexpr uint32_t BASE_ADDRCR3_NACK       = BASE_ADDRCR3_NACK_Msk;             /**<Smartcard NACK enable       */
  static constexpr uint32_t BASE_ADDRCR3_SCEN_Pos   = ( 5U );
  static constexpr uint32_t BASE_ADDRCR3_SCEN_Msk   = ( 0x1U << BASE_ADDRCR3_SCEN_Pos ); /**< 0x00000020 */
  static constexpr uint32_t BASE_ADDRCR3_SCEN       = BASE_ADDRCR3_SCEN_Msk;             /**<Smartcard mode enable       */
  static constexpr uint32_t BASE_ADDRCR3_DMAR_Pos   = ( 6U );
  static constexpr uint32_t BASE_ADDRCR3_DMAR_Msk   = ( 0x1U << BASE_ADDRCR3_DMAR_Pos ); /**< 0x00000040 */
  static constexpr uint32_t BASE_ADDRCR3_DMAR       = BASE_ADDRCR3_DMAR_Msk;             /**<DMA Enable Receiver         */
  static constexpr uint32_t BASE_ADDRCR3_DMAT_Pos   = ( 7U );
  static constexpr uint32_t BASE_ADDRCR3_DMAT_Msk   = ( 0x1U << BASE_ADDRCR3_DMAT_Pos ); /**< 0x00000080 */
  static constexpr uint32_t BASE_ADDRCR3_DMAT       = BASE_ADDRCR3_DMAT_Msk;             /**<DMA Enable Transmitter      */
  static constexpr uint32_t BASE_ADDRCR3_RTSE_Pos   = ( 8U );
  static constexpr uint32_t BASE_ADDRCR3_RTSE_Msk   = ( 0x1U << BASE_ADDRCR3_RTSE_Pos ); /**< 0x00000100 */
  static constexpr uint32_t BASE_ADDRCR3_RTSE       = BASE_ADDRCR3_RTSE_Msk;             /**<RTS Enable                  */
  static constexpr uint32_t BASE_ADDRCR3_CTSE_Pos   = ( 9U );
  static constexpr uint32_t BASE_ADDRCR3_CTSE_Msk   = ( 0x1U << BASE_ADDRCR3_CTSE_Pos ); /**< 0x00000200 */
  static constexpr uint32_t BASE_ADDRCR3_CTSE       = BASE_ADDRCR3_CTSE_Msk;             /**<CTS Enable                  */
  static constexpr uint32_t BASE_ADDRCR3_CTSIE_Pos  = ( 10U );
  static constexpr uint32_t BASE_ADDRCR3_CTSIE_Msk  = ( 0x1U << BASE_ADDRCR3_CTSIE_Pos ); /**< 0x00000400 */
  static constexpr uint32_t BASE_ADDRCR3_CTSIE      = BASE_ADDRCR3_CTSIE_Msk;             /**<CTS Interrupt Enable        */
  static constexpr uint32_t BASE_ADDRCR3_ONEBIT_Pos = ( 11U );
  static constexpr uint32_t BASE_ADDRCR3_ONEBIT_Msk = ( 0x1U << BASE_ADDRCR3_ONEBIT_Pos ); /**< 0x00000800 */
  static constexpr uint32_t BASE_ADDRCR3_ONEBIT     = BASE_ADDRCR3_ONEBIT_Msk;             /**<USART One bit method enable */

  /*------------------------------------------------
  USART Guard Time and Prescaler Register
  ------------------------------------------------*/
  static constexpr uint32_t BASE_ADDRGTPR_PSC_Pos = ( 0U );
  static constexpr uint32_t BASE_ADDRGTPR_PSC_Msk = ( 0xFFU << BASE_ADDRGTPR_PSC_Pos ); /**< 0x000000FF */
  static constexpr uint32_t BASE_ADDRGTPR_PSC     = BASE_ADDRGTPR_PSC_Msk;              /**<PSC[7:0] bits (Prescaler value) */
  static constexpr uint32_t BASE_ADDRGTPR_PSC_0   = ( 0x01U << BASE_ADDRGTPR_PSC_Pos ); /**< 0x0001 */
  static constexpr uint32_t BASE_ADDRGTPR_PSC_1   = ( 0x02U << BASE_ADDRGTPR_PSC_Pos ); /**< 0x0002 */
  static constexpr uint32_t BASE_ADDRGTPR_PSC_2   = ( 0x04U << BASE_ADDRGTPR_PSC_Pos ); /**< 0x0004 */
  static constexpr uint32_t BASE_ADDRGTPR_PSC_3   = ( 0x08U << BASE_ADDRGTPR_PSC_Pos ); /**< 0x0008 */
  static constexpr uint32_t BASE_ADDRGTPR_PSC_4   = ( 0x10U << BASE_ADDRGTPR_PSC_Pos ); /**< 0x0010 */
  static constexpr uint32_t BASE_ADDRGTPR_PSC_5   = ( 0x20U << BASE_ADDRGTPR_PSC_Pos ); /**< 0x0020 */
  static constexpr uint32_t BASE_ADDRGTPR_PSC_6   = ( 0x40U << BASE_ADDRGTPR_PSC_Pos ); /**< 0x0040 */
  static constexpr uint32_t BASE_ADDRGTPR_PSC_7   = ( 0x80U << BASE_ADDRGTPR_PSC_Pos ); /**< 0x0080 */

  static constexpr uint32_t BASE_ADDRGTPR_GT_Pos = ( 8U );
  static constexpr uint32_t BASE_ADDRGTPR_GT_Msk = ( 0xFFU << BASE_ADDRGTPR_GT_Pos ); /**< 0x0000FF00 */
  static constexpr uint32_t BASE_ADDRGTPR_GT     = BASE_ADDRGTPR_GT_Msk;              /**<Guard time value */
}    // namespace Thor::Driver::USART

#endif /* !THOR_HW_USART_REGISTER_HPP */