/********************************************************************************
 *  File Name:
 *    hw_usart_register_stm32l432kc.hpp
 *
 *  Description:
 *    USART register definitions for the STM32L432KC series chips.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_USART_REGISTER_STM32L432KC_HPP
#define THOR_HW_USART_REGISTER_STM32L432KC_HPP

/* C++ Includes */
#include <array>
#include <cstdint>
#include <cstddef>

/* Chimera Includes */
#include <Chimera/serial>

/* Driver Includes */
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/stm32l4x/system/variant/sys_memory_map_stm32l432kc.hpp>

/*-------------------------------------------------
Peripheral Availability
-------------------------------------------------*/
// Available on every STM32L4 device
#define STM32_USART1_PERIPH_AVAILABLE
#define STM32_USART2_PERIPH_AVAILABLE

#if defined( STM32L432xx )
// Device doesn't have USART3
#else
#define STM32_USART3_PERIPH_AVAILABLE
#endif

namespace Thor::LLD::USART
{
  /**
   *  Initializes the LLD register resources and memory
   *
   *  @return void
   */
  void initializeRegisters();

  /*-------------------------------------------------
  Peripheral Instance Memory Map Base
  -------------------------------------------------*/
  static constexpr RIndexType NUM_USART_PERIPHS = 3;

#if defined( STM32_USART1_PERIPH_AVAILABLE )
  static constexpr uint32_t USART1_BASE_ADDR = Thor::System::MemoryMap::USART1_PERIPH_START_ADDRESS;
#else
  static constexpr uint32_t USART1_BASE_ADDR = std::numeric_limits<uint32_t>::max();
#endif

#if defined( STM32_USART2_PERIPH_AVAILABLE )
  static constexpr uint32_t USART2_BASE_ADDR = Thor::System::MemoryMap::USART2_PERIPH_START_ADDRESS;
#else
  static constexpr uint32_t USART2_BASE_ADDR = std::numeric_limits<uint32_t>::max();
#endif

#if defined( STM32_USART3_PERIPH_AVAILABLE )
  static constexpr uint32_t USART3_BASE_ADDR = Thor::System::MemoryMap::USART3_PERIPH_START_ADDRESS;
#else
  static constexpr uint32_t USART3_BASE_ADDR = std::numeric_limits<uint32_t>::max();
#endif

  /*-------------------------------------------------
  Lookup addresses
  -------------------------------------------------*/
  static constexpr std::array<uint32_t, NUM_USART_PERIPHS> periphAddressList = { USART1_BASE_ADDR, USART2_BASE_ADDR,
                                                                                 USART3_BASE_ADDR };

  /*-------------------------------------------------
  Peripheral Register Definitions
  -------------------------------------------------*/
  /******************  Bit definition for CR1 register  *******************/
  static constexpr Reg32_t CR1_Msk = 0x1FFFFFFF;
  static constexpr Reg32_t CR1_Rst = 0x00000000;

  static constexpr uint32_t CR1_UE_Pos     = ( 0U );
  static constexpr uint32_t CR1_UE_Msk     = ( 0x1UL << CR1_UE_Pos );
  static constexpr uint32_t CR1_UE         = CR1_UE_Msk;
  static constexpr uint32_t CR1_UESM_Pos   = ( 1U );
  static constexpr uint32_t CR1_UESM_Msk   = ( 0x1UL << CR1_UESM_Pos );
  static constexpr uint32_t CR1_UESM       = CR1_UESM_Msk;
  static constexpr uint32_t CR1_RE_Pos     = ( 2U );
  static constexpr uint32_t CR1_RE_Msk     = ( 0x1UL << CR1_RE_Pos );
  static constexpr uint32_t CR1_RE         = CR1_RE_Msk;
  static constexpr uint32_t CR1_TE_Pos     = ( 3U );
  static constexpr uint32_t CR1_TE_Msk     = ( 0x1UL << CR1_TE_Pos );
  static constexpr uint32_t CR1_TE         = CR1_TE_Msk;
  static constexpr uint32_t CR1_IDLEIE_Pos = ( 4U );
  static constexpr uint32_t CR1_IDLEIE_Msk = ( 0x1UL << CR1_IDLEIE_Pos );
  static constexpr uint32_t CR1_IDLEIE     = CR1_IDLEIE_Msk;
  static constexpr uint32_t CR1_RXNEIE_Pos = ( 5U );
  static constexpr uint32_t CR1_RXNEIE_Msk = ( 0x1UL << CR1_RXNEIE_Pos );
  static constexpr uint32_t CR1_RXNEIE     = CR1_RXNEIE_Msk;
  static constexpr uint32_t CR1_TCIE_Pos   = ( 6U );
  static constexpr uint32_t CR1_TCIE_Msk   = ( 0x1UL << CR1_TCIE_Pos );
  static constexpr uint32_t CR1_TCIE       = CR1_TCIE_Msk;
  static constexpr uint32_t CR1_TXEIE_Pos  = ( 7U );
  static constexpr uint32_t CR1_TXEIE_Msk  = ( 0x1UL << CR1_TXEIE_Pos );
  static constexpr uint32_t CR1_TXEIE      = CR1_TXEIE_Msk;
  static constexpr uint32_t CR1_PEIE_Pos   = ( 8U );
  static constexpr uint32_t CR1_PEIE_Msk   = ( 0x1UL << CR1_PEIE_Pos );
  static constexpr uint32_t CR1_PEIE       = CR1_PEIE_Msk;
  static constexpr uint32_t CR1_PS_Pos     = ( 9U );
  static constexpr uint32_t CR1_PS_Msk     = ( 0x1UL << CR1_PS_Pos );
  static constexpr uint32_t CR1_PS         = CR1_PS_Msk;
  static constexpr uint32_t CR1_PCE_Pos    = ( 10U );
  static constexpr uint32_t CR1_PCE_Msk    = ( 0x1UL << CR1_PCE_Pos );
  static constexpr uint32_t CR1_PCE        = CR1_PCE_Msk;
  static constexpr uint32_t CR1_WAKE_Pos   = ( 11U );
  static constexpr uint32_t CR1_WAKE_Msk   = ( 0x1UL << CR1_WAKE_Pos );
  static constexpr uint32_t CR1_WAKE       = CR1_WAKE_Msk;
  static constexpr uint32_t CR1_M_Pos      = ( 12U );
  static constexpr uint32_t CR1_M_Msk      = ( 0x10001UL << CR1_M_Pos );
  static constexpr uint32_t CR1_M          = CR1_M_Msk;
  static constexpr uint32_t CR1_M0_Pos     = ( 12U );
  static constexpr uint32_t CR1_M0_Msk     = ( 0x1UL << CR1_M0_Pos );
  static constexpr uint32_t CR1_M0         = CR1_M0_Msk;
  static constexpr uint32_t CR1_MME_Pos    = ( 13U );
  static constexpr uint32_t CR1_MME_Msk    = ( 0x1UL << CR1_MME_Pos );
  static constexpr uint32_t CR1_MME        = CR1_MME_Msk;
  static constexpr uint32_t CR1_CMIE_Pos   = ( 14U );
  static constexpr uint32_t CR1_CMIE_Msk   = ( 0x1UL << CR1_CMIE_Pos );
  static constexpr uint32_t CR1_CMIE       = CR1_CMIE_Msk;
  static constexpr uint32_t CR1_OVER8_Pos  = ( 15U );
  static constexpr uint32_t CR1_OVER8_Msk  = ( 0x1UL << CR1_OVER8_Pos );
  static constexpr uint32_t CR1_OVER8      = CR1_OVER8_Msk;
  static constexpr uint32_t CR1_DEDT_Pos   = ( 16U );
  static constexpr uint32_t CR1_DEDT_Msk   = ( 0x1FUL << CR1_DEDT_Pos );
  static constexpr uint32_t CR1_DEDT       = CR1_DEDT_Msk;
  static constexpr uint32_t CR1_DEDT_0     = ( 0x01UL << CR1_DEDT_Pos );
  static constexpr uint32_t CR1_DEDT_1     = ( 0x02UL << CR1_DEDT_Pos );
  static constexpr uint32_t CR1_DEDT_2     = ( 0x04UL << CR1_DEDT_Pos );
  static constexpr uint32_t CR1_DEDT_3     = ( 0x08UL << CR1_DEDT_Pos );
  static constexpr uint32_t CR1_DEDT_4     = ( 0x10UL << CR1_DEDT_Pos );
  static constexpr uint32_t CR1_DEAT_Pos   = ( 21U );
  static constexpr uint32_t CR1_DEAT_Msk   = ( 0x1FUL << CR1_DEAT_Pos );
  static constexpr uint32_t CR1_DEAT       = CR1_DEAT_Msk;
  static constexpr uint32_t CR1_DEAT_0     = ( 0x01UL << CR1_DEAT_Pos );
  static constexpr uint32_t CR1_DEAT_1     = ( 0x02UL << CR1_DEAT_Pos );
  static constexpr uint32_t CR1_DEAT_2     = ( 0x04UL << CR1_DEAT_Pos );
  static constexpr uint32_t CR1_DEAT_3     = ( 0x08UL << CR1_DEAT_Pos );
  static constexpr uint32_t CR1_DEAT_4     = ( 0x10UL << CR1_DEAT_Pos );
  static constexpr uint32_t CR1_RTOIE_Pos  = ( 26U );
  static constexpr uint32_t CR1_RTOIE_Msk  = ( 0x1UL << CR1_RTOIE_Pos );
  static constexpr uint32_t CR1_RTOIE      = CR1_RTOIE_Msk;
  static constexpr uint32_t CR1_EOBIE_Pos  = ( 27U );
  static constexpr uint32_t CR1_EOBIE_Msk  = ( 0x1UL << CR1_EOBIE_Pos );
  static constexpr uint32_t CR1_EOBIE      = CR1_EOBIE_Msk;
  static constexpr uint32_t CR1_M1_Pos     = ( 28U );
  static constexpr uint32_t CR1_M1_Msk     = ( 0x1UL << CR1_M1_Pos );
  static constexpr uint32_t CR1_M1         = CR1_M1_Msk;

  /******************  Bit definition for CR2 register  *******************/
  static constexpr Reg32_t CR2_Msk = 0xFFFFFF70;
  static constexpr Reg32_t CR2_Rst = 0x00000000;

  static constexpr uint32_t CR2_ADDM7_Pos    = ( 4U );
  static constexpr uint32_t CR2_ADDM7_Msk    = ( 0x1UL << CR2_ADDM7_Pos );
  static constexpr uint32_t CR2_ADDM7        = CR2_ADDM7_Msk;
  static constexpr uint32_t CR2_LBDL_Pos     = ( 5U );
  static constexpr uint32_t CR2_LBDL_Msk     = ( 0x1UL << CR2_LBDL_Pos );
  static constexpr uint32_t CR2_LBDL         = CR2_LBDL_Msk;
  static constexpr uint32_t CR2_LBDIE_Pos    = ( 6U );
  static constexpr uint32_t CR2_LBDIE_Msk    = ( 0x1UL << CR2_LBDIE_Pos );
  static constexpr uint32_t CR2_LBDIE        = CR2_LBDIE_Msk;
  static constexpr uint32_t CR2_LBCL_Pos     = ( 8U );
  static constexpr uint32_t CR2_LBCL_Msk     = ( 0x1UL << CR2_LBCL_Pos );
  static constexpr uint32_t CR2_LBCL         = CR2_LBCL_Msk;
  static constexpr uint32_t CR2_CPHA_Pos     = ( 9U );
  static constexpr uint32_t CR2_CPHA_Msk     = ( 0x1UL << CR2_CPHA_Pos );
  static constexpr uint32_t CR2_CPHA         = CR2_CPHA_Msk;
  static constexpr uint32_t CR2_CPOL_Pos     = ( 10U );
  static constexpr uint32_t CR2_CPOL_Msk     = ( 0x1UL << CR2_CPOL_Pos );
  static constexpr uint32_t CR2_CPOL         = CR2_CPOL_Msk;
  static constexpr uint32_t CR2_CLKEN_Pos    = ( 11U );
  static constexpr uint32_t CR2_CLKEN_Msk    = ( 0x1UL << CR2_CLKEN_Pos );
  static constexpr uint32_t CR2_CLKEN        = CR2_CLKEN_Msk;
  static constexpr uint32_t CR2_STOP_Pos     = ( 12U );
  static constexpr uint32_t CR2_STOP_Msk     = ( 0x3UL << CR2_STOP_Pos );
  static constexpr uint32_t CR2_STOP         = CR2_STOP_Msk;
  static constexpr uint32_t CR2_STOP_0       = ( 0x1UL << CR2_STOP_Pos );
  static constexpr uint32_t CR2_STOP_1       = ( 0x2UL << CR2_STOP_Pos );
  static constexpr uint32_t CR2_LINEN_Pos    = ( 14U );
  static constexpr uint32_t CR2_LINEN_Msk    = ( 0x1UL << CR2_LINEN_Pos );
  static constexpr uint32_t CR2_LINEN        = CR2_LINEN_Msk;
  static constexpr uint32_t CR2_SWAP_Pos     = ( 15U );
  static constexpr uint32_t CR2_SWAP_Msk     = ( 0x1UL << CR2_SWAP_Pos );
  static constexpr uint32_t CR2_SWAP         = CR2_SWAP_Msk;
  static constexpr uint32_t CR2_RXINV_Pos    = ( 16U );
  static constexpr uint32_t CR2_RXINV_Msk    = ( 0x1UL << CR2_RXINV_Pos );
  static constexpr uint32_t CR2_RXINV        = CR2_RXINV_Msk;
  static constexpr uint32_t CR2_TXINV_Pos    = ( 17U );
  static constexpr uint32_t CR2_TXINV_Msk    = ( 0x1UL << CR2_TXINV_Pos );
  static constexpr uint32_t CR2_TXINV        = CR2_TXINV_Msk;
  static constexpr uint32_t CR2_DATAINV_Pos  = ( 18U );
  static constexpr uint32_t CR2_DATAINV_Msk  = ( 0x1UL << CR2_DATAINV_Pos );
  static constexpr uint32_t CR2_DATAINV      = CR2_DATAINV_Msk;
  static constexpr uint32_t CR2_MSBFIRST_Pos = ( 19U );
  static constexpr uint32_t CR2_MSBFIRST_Msk = ( 0x1UL << CR2_MSBFIRST_Pos );
  static constexpr uint32_t CR2_MSBFIRST     = CR2_MSBFIRST_Msk;
  static constexpr uint32_t CR2_ABREN_Pos    = ( 20U );
  static constexpr uint32_t CR2_ABREN_Msk    = ( 0x1UL << CR2_ABREN_Pos );
  static constexpr uint32_t CR2_ABREN        = CR2_ABREN_Msk;
  static constexpr uint32_t CR2_ABRMODE_Pos  = ( 21U );
  static constexpr uint32_t CR2_ABRMODE_Msk  = ( 0x3UL << CR2_ABRMODE_Pos );
  static constexpr uint32_t CR2_ABRMODE      = CR2_ABRMODE_Msk;
  static constexpr uint32_t CR2_ABRMODE_0    = ( 0x1UL << CR2_ABRMODE_Pos );
  static constexpr uint32_t CR2_ABRMODE_1    = ( 0x2UL << CR2_ABRMODE_Pos );
  static constexpr uint32_t CR2_RTOEN_Pos    = ( 23U );
  static constexpr uint32_t CR2_RTOEN_Msk    = ( 0x1UL << CR2_RTOEN_Pos );
  static constexpr uint32_t CR2_RTOEN        = CR2_RTOEN_Msk;
  static constexpr uint32_t CR2_ADD_Pos      = ( 24U );
  static constexpr uint32_t CR2_ADD_Msk      = ( 0xFFUL << CR2_ADD_Pos );
  static constexpr uint32_t CR2_ADD          = CR2_ADD_Msk;

  /******************  Bit definition for CR3 register  *******************/
  static constexpr Reg32_t CR3_Msk = 0x01FEFFFF;
  static constexpr Reg32_t CR3_Rst = 0x00000000;

  static constexpr uint32_t CR3_EIE_Pos     = ( 0U );
  static constexpr uint32_t CR3_EIE_Msk     = ( 0x1UL << CR3_EIE_Pos );
  static constexpr uint32_t CR3_EIE         = CR3_EIE_Msk;
  static constexpr uint32_t CR3_IREN_Pos    = ( 1U );
  static constexpr uint32_t CR3_IREN_Msk    = ( 0x1UL << CR3_IREN_Pos );
  static constexpr uint32_t CR3_IREN        = CR3_IREN_Msk;
  static constexpr uint32_t CR3_IRLP_Pos    = ( 2U );
  static constexpr uint32_t CR3_IRLP_Msk    = ( 0x1UL << CR3_IRLP_Pos );
  static constexpr uint32_t CR3_IRLP        = CR3_IRLP_Msk;
  static constexpr uint32_t CR3_HDSEL_Pos   = ( 3U );
  static constexpr uint32_t CR3_HDSEL_Msk   = ( 0x1UL << CR3_HDSEL_Pos );
  static constexpr uint32_t CR3_HDSEL       = CR3_HDSEL_Msk;
  static constexpr uint32_t CR3_NACK_Pos    = ( 4U );
  static constexpr uint32_t CR3_NACK_Msk    = ( 0x1UL << CR3_NACK_Pos );
  static constexpr uint32_t CR3_NACK        = CR3_NACK_Msk;
  static constexpr uint32_t CR3_SCEN_Pos    = ( 5U );
  static constexpr uint32_t CR3_SCEN_Msk    = ( 0x1UL << CR3_SCEN_Pos );
  static constexpr uint32_t CR3_SCEN        = CR3_SCEN_Msk;
  static constexpr uint32_t CR3_DMAR_Pos    = ( 6U );
  static constexpr uint32_t CR3_DMAR_Msk    = ( 0x1UL << CR3_DMAR_Pos );
  static constexpr uint32_t CR3_DMAR        = CR3_DMAR_Msk;
  static constexpr uint32_t CR3_DMAT_Pos    = ( 7U );
  static constexpr uint32_t CR3_DMAT_Msk    = ( 0x1UL << CR3_DMAT_Pos );
  static constexpr uint32_t CR3_DMAT        = CR3_DMAT_Msk;
  static constexpr uint32_t CR3_RTSE_Pos    = ( 8U );
  static constexpr uint32_t CR3_RTSE_Msk    = ( 0x1UL << CR3_RTSE_Pos );
  static constexpr uint32_t CR3_RTSE        = CR3_RTSE_Msk;
  static constexpr uint32_t CR3_CTSE_Pos    = ( 9U );
  static constexpr uint32_t CR3_CTSE_Msk    = ( 0x1UL << CR3_CTSE_Pos );
  static constexpr uint32_t CR3_CTSE        = CR3_CTSE_Msk;
  static constexpr uint32_t CR3_CTSIE_Pos   = ( 10U );
  static constexpr uint32_t CR3_CTSIE_Msk   = ( 0x1UL << CR3_CTSIE_Pos );
  static constexpr uint32_t CR3_CTSIE       = CR3_CTSIE_Msk;
  static constexpr uint32_t CR3_ONEBIT_Pos  = ( 11U );
  static constexpr uint32_t CR3_ONEBIT_Msk  = ( 0x1UL << CR3_ONEBIT_Pos );
  static constexpr uint32_t CR3_ONEBIT      = CR3_ONEBIT_Msk;
  static constexpr uint32_t CR3_OVRDIS_Pos  = ( 12U );
  static constexpr uint32_t CR3_OVRDIS_Msk  = ( 0x1UL << CR3_OVRDIS_Pos );
  static constexpr uint32_t CR3_OVRDIS      = CR3_OVRDIS_Msk;
  static constexpr uint32_t CR3_DDRE_Pos    = ( 13U );
  static constexpr uint32_t CR3_DDRE_Msk    = ( 0x1UL << CR3_DDRE_Pos );
  static constexpr uint32_t CR3_DDRE        = CR3_DDRE_Msk;
  static constexpr uint32_t CR3_DEM_Pos     = ( 14U );
  static constexpr uint32_t CR3_DEM_Msk     = ( 0x1UL << CR3_DEM_Pos );
  static constexpr uint32_t CR3_DEM         = CR3_DEM_Msk;
  static constexpr uint32_t CR3_DEP_Pos     = ( 15U );
  static constexpr uint32_t CR3_DEP_Msk     = ( 0x1UL << CR3_DEP_Pos );
  static constexpr uint32_t CR3_DEP         = CR3_DEP_Msk;
  static constexpr uint32_t CR3_SCARCNT_Pos = ( 17U );
  static constexpr uint32_t CR3_SCARCNT_Msk = ( 0x7UL << CR3_SCARCNT_Pos );
  static constexpr uint32_t CR3_SCARCNT     = CR3_SCARCNT_Msk;
  static constexpr uint32_t CR3_SCARCNT_0   = ( 0x1UL << CR3_SCARCNT_Pos );
  static constexpr uint32_t CR3_SCARCNT_1   = ( 0x2UL << CR3_SCARCNT_Pos );
  static constexpr uint32_t CR3_SCARCNT_2   = ( 0x4UL << CR3_SCARCNT_Pos );
  static constexpr uint32_t CR3_WUS_Pos     = ( 20U );
  static constexpr uint32_t CR3_WUS_Msk     = ( 0x3UL << CR3_WUS_Pos );
  static constexpr uint32_t CR3_WUS         = CR3_WUS_Msk;
  static constexpr uint32_t CR3_WUS_0       = ( 0x1UL << CR3_WUS_Pos );
  static constexpr uint32_t CR3_WUS_1       = ( 0x2UL << CR3_WUS_Pos );
  static constexpr uint32_t CR3_WUFIE_Pos   = ( 22U );
  static constexpr uint32_t CR3_WUFIE_Msk   = ( 0x1UL << CR3_WUFIE_Pos );
  static constexpr uint32_t CR3_WUFIE       = CR3_WUFIE_Msk;
  static constexpr uint32_t CR3_UCESM_Pos   = ( 23U );
  static constexpr uint32_t CR3_UCESM_Msk   = ( 0x1UL << CR3_UCESM_Pos );
  static constexpr uint32_t CR3_UCESM       = CR3_UCESM_Msk;
  static constexpr uint32_t CR3_TCBGTIE_Pos = ( 24U );
  static constexpr uint32_t CR3_TCBGTIE_Msk = ( 0x1UL << CR3_TCBGTIE_Pos );
  static constexpr uint32_t CR3_TCBGTIE     = CR3_TCBGTIE_Msk;

  /******************  Bit definition for BRR register  *******************/
  static constexpr Reg32_t BRR_Msk = 0x0000FFFF;
  static constexpr Reg32_t BRR_Rst = 0x00000000;

  static constexpr uint32_t BRR_DIV_FRACTION_Pos = ( 0U );
  static constexpr uint32_t BRR_DIV_FRACTION_Msk = ( 0xFUL << BRR_DIV_FRACTION_Pos );
  static constexpr uint32_t BRR_DIV_FRACTION     = BRR_DIV_FRACTION_Msk;
  static constexpr uint32_t BRR_DIV_MANTISSA_Pos = ( 4U );
  static constexpr uint32_t BRR_DIV_MANTISSA_Msk = ( 0xFFFUL << BRR_DIV_MANTISSA_Pos );
  static constexpr uint32_t BRR_DIV_MANTISSA     = BRR_DIV_MANTISSA_Msk;

  /******************  Bit definition for GTPR register  ******************/
  static constexpr Reg32_t GTPR_Msk = 0x0000FFFF;
  static constexpr Reg32_t GTPR_Rst = 0x00000000;

  static constexpr uint32_t GTPR_PSC_Pos = ( 0U );
  static constexpr uint32_t GTPR_PSC_Msk = ( 0xFFUL << GTPR_PSC_Pos );
  static constexpr uint32_t GTPR_PSC     = GTPR_PSC_Msk;
  static constexpr uint32_t GTPR_GT_Pos  = ( 8U );
  static constexpr uint32_t GTPR_GT_Msk  = ( 0xFFUL << GTPR_GT_Pos );
  static constexpr uint32_t GTPR_GT      = GTPR_GT_Msk;

  /*******************  Bit definition for RTOR register  *****************/
  static constexpr Reg32_t RTOR_Msk = 0xFFFFFFFF;
  static constexpr Reg32_t RTOR_Rst = 0x00000000;

  static constexpr uint32_t RTOR_RTO_Pos  = ( 0U );
  static constexpr uint32_t RTOR_RTO_Msk  = ( 0xFFFFFFUL << RTOR_RTO_Pos );
  static constexpr uint32_t RTOR_RTO      = RTOR_RTO_Msk;
  static constexpr uint32_t RTOR_BLEN_Pos = ( 24U );
  static constexpr uint32_t RTOR_BLEN_Msk = ( 0xFFUL << RTOR_BLEN_Pos );
  static constexpr uint32_t RTOR_BLEN     = RTOR_BLEN_Msk;

  /*******************  Bit definition for RQR register  ******************/
  static constexpr Reg32_t RQR_Msk = 0x0000001F;
  static constexpr Reg32_t RQR_Rst = 0x00000000;

  static constexpr uint32_t RQR_ABRRQ_Pos = ( 0U );
  static constexpr uint32_t RQR_ABRRQ_Msk = ( 0x1UL << RQR_ABRRQ_Pos );
  static constexpr uint32_t RQR_ABRRQ     = RQR_ABRRQ_Msk;
  static constexpr uint32_t RQR_SBKRQ_Pos = ( 1U );
  static constexpr uint32_t RQR_SBKRQ_Msk = ( 0x1UL << RQR_SBKRQ_Pos );
  static constexpr uint32_t RQR_SBKRQ     = RQR_SBKRQ_Msk;
  static constexpr uint32_t RQR_MMRQ_Pos  = ( 2U );
  static constexpr uint32_t RQR_MMRQ_Msk  = ( 0x1UL << RQR_MMRQ_Pos );
  static constexpr uint32_t RQR_MMRQ      = RQR_MMRQ_Msk;
  static constexpr uint32_t RQR_RXFRQ_Pos = ( 3U );
  static constexpr uint32_t RQR_RXFRQ_Msk = ( 0x1UL << RQR_RXFRQ_Pos );
  static constexpr uint32_t RQR_RXFRQ     = RQR_RXFRQ_Msk;
  static constexpr uint32_t RQR_TXFRQ_Pos = ( 4U );
  static constexpr uint32_t RQR_TXFRQ_Msk = ( 0x1UL << RQR_TXFRQ_Pos );
  static constexpr uint32_t RQR_TXFRQ     = RQR_TXFRQ_Msk;

  /*******************  Bit definition for ISR register  ******************/
  static constexpr Reg32_t ISR_Msk = 0x027EDFFF;
  static constexpr Reg32_t ISR_Rst = 0x020000C0;

  static constexpr uint32_t ISR_PE_Pos    = ( 0U );
  static constexpr uint32_t ISR_PE_Msk    = ( 0x1UL << ISR_PE_Pos );
  static constexpr uint32_t ISR_PE        = ISR_PE_Msk;
  static constexpr uint32_t ISR_FE_Pos    = ( 1U );
  static constexpr uint32_t ISR_FE_Msk    = ( 0x1UL << ISR_FE_Pos );
  static constexpr uint32_t ISR_FE        = ISR_FE_Msk;
  static constexpr uint32_t ISR_NE_Pos    = ( 2U );
  static constexpr uint32_t ISR_NE_Msk    = ( 0x1UL << ISR_NE_Pos );
  static constexpr uint32_t ISR_NE        = ISR_NE_Msk;
  static constexpr uint32_t ISR_ORE_Pos   = ( 3U );
  static constexpr uint32_t ISR_ORE_Msk   = ( 0x1UL << ISR_ORE_Pos );
  static constexpr uint32_t ISR_ORE       = ISR_ORE_Msk;
  static constexpr uint32_t ISR_IDLE_Pos  = ( 4U );
  static constexpr uint32_t ISR_IDLE_Msk  = ( 0x1UL << ISR_IDLE_Pos );
  static constexpr uint32_t ISR_IDLE      = ISR_IDLE_Msk;
  static constexpr uint32_t ISR_RXNE_Pos  = ( 5U );
  static constexpr uint32_t ISR_RXNE_Msk  = ( 0x1UL << ISR_RXNE_Pos );
  static constexpr uint32_t ISR_RXNE      = ISR_RXNE_Msk;
  static constexpr uint32_t ISR_TC_Pos    = ( 6U );
  static constexpr uint32_t ISR_TC_Msk    = ( 0x1UL << ISR_TC_Pos );
  static constexpr uint32_t ISR_TC        = ISR_TC_Msk;
  static constexpr uint32_t ISR_TXE_Pos   = ( 7U );
  static constexpr uint32_t ISR_TXE_Msk   = ( 0x1UL << ISR_TXE_Pos );
  static constexpr uint32_t ISR_TXE       = ISR_TXE_Msk;
  static constexpr uint32_t ISR_LBDF_Pos  = ( 8U );
  static constexpr uint32_t ISR_LBDF_Msk  = ( 0x1UL << ISR_LBDF_Pos );
  static constexpr uint32_t ISR_LBDF      = ISR_LBDF_Msk;
  static constexpr uint32_t ISR_CTSIF_Pos = ( 9U );
  static constexpr uint32_t ISR_CTSIF_Msk = ( 0x1UL << ISR_CTSIF_Pos );
  static constexpr uint32_t ISR_CTSIF     = ISR_CTSIF_Msk;
  static constexpr uint32_t ISR_CTS_Pos   = ( 10U );
  static constexpr uint32_t ISR_CTS_Msk   = ( 0x1UL << ISR_CTS_Pos );
  static constexpr uint32_t ISR_CTS       = ISR_CTS_Msk;
  static constexpr uint32_t ISR_RTOF_Pos  = ( 11U );
  static constexpr uint32_t ISR_RTOF_Msk  = ( 0x1UL << ISR_RTOF_Pos );
  static constexpr uint32_t ISR_RTOF      = ISR_RTOF_Msk;
  static constexpr uint32_t ISR_EOBF_Pos  = ( 12U );
  static constexpr uint32_t ISR_EOBF_Msk  = ( 0x1UL << ISR_EOBF_Pos );
  static constexpr uint32_t ISR_EOBF      = ISR_EOBF_Msk;
  static constexpr uint32_t ISR_ABRE_Pos  = ( 14U );
  static constexpr uint32_t ISR_ABRE_Msk  = ( 0x1UL << ISR_ABRE_Pos );
  static constexpr uint32_t ISR_ABRE      = ISR_ABRE_Msk;
  static constexpr uint32_t ISR_ABRF_Pos  = ( 15U );
  static constexpr uint32_t ISR_ABRF_Msk  = ( 0x1UL << ISR_ABRF_Pos );
  static constexpr uint32_t ISR_ABRF      = ISR_ABRF_Msk;
  static constexpr uint32_t ISR_BUSY_Pos  = ( 16U );
  static constexpr uint32_t ISR_BUSY_Msk  = ( 0x1UL << ISR_BUSY_Pos );
  static constexpr uint32_t ISR_BUSY      = ISR_BUSY_Msk;
  static constexpr uint32_t ISR_CMF_Pos   = ( 17U );
  static constexpr uint32_t ISR_CMF_Msk   = ( 0x1UL << ISR_CMF_Pos );
  static constexpr uint32_t ISR_CMF       = ISR_CMF_Msk;
  static constexpr uint32_t ISR_SBKF_Pos  = ( 18U );
  static constexpr uint32_t ISR_SBKF_Msk  = ( 0x1UL << ISR_SBKF_Pos );
  static constexpr uint32_t ISR_SBKF      = ISR_SBKF_Msk;
  static constexpr uint32_t ISR_RWU_Pos   = ( 19U );
  static constexpr uint32_t ISR_RWU_Msk   = ( 0x1UL << ISR_RWU_Pos );
  static constexpr uint32_t ISR_RWU       = ISR_RWU_Msk;
  static constexpr uint32_t ISR_WUF_Pos   = ( 20U );
  static constexpr uint32_t ISR_WUF_Msk   = ( 0x1UL << ISR_WUF_Pos );
  static constexpr uint32_t ISR_WUF       = ISR_WUF_Msk;
  static constexpr uint32_t ISR_TEACK_Pos = ( 21U );
  static constexpr uint32_t ISR_TEACK_Msk = ( 0x1UL << ISR_TEACK_Pos );
  static constexpr uint32_t ISR_TEACK     = ISR_TEACK_Msk;
  static constexpr uint32_t ISR_REACK_Pos = ( 22U );
  static constexpr uint32_t ISR_REACK_Msk = ( 0x1UL << ISR_REACK_Pos );
  static constexpr uint32_t ISR_REACK     = ISR_REACK_Msk;
  static constexpr uint32_t ISR_TCBGT_Pos = ( 25U );
  static constexpr uint32_t ISR_TCBGT_Msk = ( 0x1UL << ISR_TCBGT_Pos );
  static constexpr uint32_t ISR_TCBGT     = ISR_TCBGT_Msk;

  /*******************  Bit definition for ICR register  ******************/
  static constexpr Reg32_t ICR_Msk = 0x00121BDF;
  static constexpr Reg32_t ICR_Rst = 0x00000000;

  static constexpr uint32_t ICR_PECF_Pos    = ( 0U );
  static constexpr uint32_t ICR_PECF_Msk    = ( 0x1UL << ICR_PECF_Pos );
  static constexpr uint32_t ICR_PECF        = ICR_PECF_Msk;
  static constexpr uint32_t ICR_FECF_Pos    = ( 1U );
  static constexpr uint32_t ICR_FECF_Msk    = ( 0x1UL << ICR_FECF_Pos );
  static constexpr uint32_t ICR_FECF        = ICR_FECF_Msk;
  static constexpr uint32_t ICR_NECF_Pos    = ( 2U );
  static constexpr uint32_t ICR_NECF_Msk    = ( 0x1UL << ICR_NECF_Pos );
  static constexpr uint32_t ICR_NECF        = ICR_NECF_Msk;
  static constexpr uint32_t ICR_ORECF_Pos   = ( 3U );
  static constexpr uint32_t ICR_ORECF_Msk   = ( 0x1UL << ICR_ORECF_Pos );
  static constexpr uint32_t ICR_ORECF       = ICR_ORECF_Msk;
  static constexpr uint32_t ICR_IDLECF_Pos  = ( 4U );
  static constexpr uint32_t ICR_IDLECF_Msk  = ( 0x1UL << ICR_IDLECF_Pos );
  static constexpr uint32_t ICR_IDLECF      = ICR_IDLECF_Msk;
  static constexpr uint32_t ICR_TCCF_Pos    = ( 6U );
  static constexpr uint32_t ICR_TCCF_Msk    = ( 0x1UL << ICR_TCCF_Pos );
  static constexpr uint32_t ICR_TCCF        = ICR_TCCF_Msk;
  static constexpr uint32_t ICR_TCBGTCF_Pos = ( 7U );
  static constexpr uint32_t ICR_TCBGTCF_Msk = ( 0x1UL << ICR_TCBGTCF_Pos );
  static constexpr uint32_t ICR_TCBGTCF     = ICR_TCBGTCF_Msk;
  static constexpr uint32_t ICR_LBDCF_Pos   = ( 8U );
  static constexpr uint32_t ICR_LBDCF_Msk   = ( 0x1UL << ICR_LBDCF_Pos );
  static constexpr uint32_t ICR_LBDCF       = ICR_LBDCF_Msk;
  static constexpr uint32_t ICR_CTSCF_Pos   = ( 9U );
  static constexpr uint32_t ICR_CTSCF_Msk   = ( 0x1UL << ICR_CTSCF_Pos );
  static constexpr uint32_t ICR_CTSCF       = ICR_CTSCF_Msk;
  static constexpr uint32_t ICR_RTOCF_Pos   = ( 11U );
  static constexpr uint32_t ICR_RTOCF_Msk   = ( 0x1UL << ICR_RTOCF_Pos );
  static constexpr uint32_t ICR_RTOCF       = ICR_RTOCF_Msk;
  static constexpr uint32_t ICR_EOBCF_Pos   = ( 12U );
  static constexpr uint32_t ICR_EOBCF_Msk   = ( 0x1UL << ICR_EOBCF_Pos );
  static constexpr uint32_t ICR_EOBCF       = ICR_EOBCF_Msk;
  static constexpr uint32_t ICR_CMCF_Pos    = ( 17U );
  static constexpr uint32_t ICR_CMCF_Msk    = ( 0x1UL << ICR_CMCF_Pos );
  static constexpr uint32_t ICR_CMCF        = ICR_CMCF_Msk;
  static constexpr uint32_t ICR_WUCF_Pos    = ( 20U );
  static constexpr uint32_t ICR_WUCF_Msk    = ( 0x1UL << ICR_WUCF_Pos );
  static constexpr uint32_t ICR_WUCF        = ICR_WUCF_Msk;

  /*******************  Bit definition for RDR register  ******************/
  static constexpr uint32_t RDR_RDR_Pos = ( 0U );
  static constexpr uint32_t RDR_RDR_Msk = ( 0x1FFUL << RDR_RDR_Pos );
  static constexpr uint32_t RDR_RDR     = RDR_RDR_Msk;

  /*******************  Bit definition for TDR register  ******************/
  static constexpr uint32_t TDR_TDR_Pos = ( 0U );
  static constexpr uint32_t TDR_TDR_Msk = ( 0x1FFUL << TDR_TDR_Pos );
  static constexpr uint32_t TDR_TDR     = TDR_TDR_Msk;

}    // namespace Thor::LLD::USART

#endif /* !THOR_HW_USART_REGISTER_STM32L432KC_HPP */