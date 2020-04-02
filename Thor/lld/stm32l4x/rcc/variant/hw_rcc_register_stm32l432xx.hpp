/********************************************************************************
 *  File Name:
 *    hw_rcc_register_stm32l432xx.hpp
 *
 *  Description:
 *    RCC register definitions for the STM32L432xx series chips
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_RCC_REGISTER_HPP
#define THOR_LLD_RCC_REGISTER_HPP

/* C++ Includes */
#include <cstdint>

/* Chimera Includes */
#include <Chimera/common>

/* Driver Includes */
#include <Thor/lld/stm32l4x/system/sys_memory_map_prj.hpp>

#define STM32_RCC1_PERIPH_AVAILABLE

namespace Thor::LLD::RCC
{
  void initializeRegisters();

  static constexpr Reg32_t NUM_RCC_PERIPHS     = 1;
  static constexpr uint8_t RCC1_RESOURCE_INDEX = 0u;
  static constexpr Reg32_t RCC1_BASE_ADDR      = Thor::System::MemoryMap::RCC_PERIPH_START_ADDRESS;

  static constexpr std::array<Reg32_t, NUM_RCC_PERIPHS> periphAddressList = { RCC1_BASE_ADDR };

  /******************************************************************************/
  /*                                                                            */
  /*                         Reset and Clock Control                            */
  /*                                                                            */
  /******************************************************************************/
  /*
   * @brief Specific device feature definitions  = (not present on all devices in the STM32L4 serie);
   */
  static constexpr bool AHB1ENRPLLSAI1_SUPPORT = true;
  static constexpr bool PLLP_SUPPORT           = true;
  static constexpr bool HSI48_SUPPORT          = true;
  static constexpr bool PLLP_DIV_2_31_SUPPORT  = true;
  static constexpr bool PLLSAI1P_DIV_2_31_SUPPORT = true;

  /********************  Bit definition for CR register  ********************/
  static constexpr Reg32_t CR_MSION_Pos     = ( 0U );
  static constexpr Reg32_t CR_MSION_Msk     = ( 0x1UL << CR_MSION_Pos );
  static constexpr Reg32_t CR_MSION         = CR_MSION_Msk;
  static constexpr Reg32_t CR_MSIRDY_Pos    = ( 1U );
  static constexpr Reg32_t CR_MSIRDY_Msk    = ( 0x1UL << CR_MSIRDY_Pos );
  static constexpr Reg32_t CR_MSIRDY        = CR_MSIRDY_Msk;
  static constexpr Reg32_t CR_MSIPLLEN_Pos  = ( 2U );
  static constexpr Reg32_t CR_MSIPLLEN_Msk  = ( 0x1UL << CR_MSIPLLEN_Pos );
  static constexpr Reg32_t CR_MSIPLLEN      = CR_MSIPLLEN_Msk;
  static constexpr Reg32_t CR_MSIRGSEL_Pos  = ( 3U );
  static constexpr Reg32_t CR_MSIRGSEL_Msk  = ( 0x1UL << CR_MSIRGSEL_Pos );
  static constexpr Reg32_t CR_MSIRGSEL      = CR_MSIRGSEL_Msk;


  static constexpr Reg32_t CR_MSIRANGE_Pos = ( 4U );
  static constexpr Reg32_t CR_MSIRANGE_Msk = ( 0xFUL << CR_MSIRANGE_Pos );
  static constexpr Reg32_t CR_MSIRANGE     = CR_MSIRANGE_Msk;
  static constexpr Reg32_t CR_MSIRANGE_0   = ( 0x0UL << CR_MSIRANGE_Pos );
  static constexpr Reg32_t CR_MSIRANGE_1   = ( 0x1UL << CR_MSIRANGE_Pos );
  static constexpr Reg32_t CR_MSIRANGE_2   = ( 0x2UL << CR_MSIRANGE_Pos );
  static constexpr Reg32_t CR_MSIRANGE_3   = ( 0x3UL << CR_MSIRANGE_Pos );
  static constexpr Reg32_t CR_MSIRANGE_4   = ( 0x4UL << CR_MSIRANGE_Pos );
  static constexpr Reg32_t CR_MSIRANGE_5   = ( 0x5UL << CR_MSIRANGE_Pos );
  static constexpr Reg32_t CR_MSIRANGE_6   = ( 0x6UL << CR_MSIRANGE_Pos );
  static constexpr Reg32_t CR_MSIRANGE_7   = ( 0x7UL << CR_MSIRANGE_Pos );
  static constexpr Reg32_t CR_MSIRANGE_8   = ( 0x8UL << CR_MSIRANGE_Pos );
  static constexpr Reg32_t CR_MSIRANGE_9   = ( 0x9UL << CR_MSIRANGE_Pos );
  static constexpr Reg32_t CR_MSIRANGE_10  = ( 0xAUL << CR_MSIRANGE_Pos );
  static constexpr Reg32_t CR_MSIRANGE_11  = ( 0xBUL << CR_MSIRANGE_Pos );

  static constexpr Reg32_t CR_HSION_Pos    = ( 8U );
  static constexpr Reg32_t CR_HSION_Msk    = ( 0x1UL << CR_HSION_Pos );
  static constexpr Reg32_t CR_HSION        = CR_HSION_Msk;
  static constexpr Reg32_t CR_HSIKERON_Pos = ( 9U );
  static constexpr Reg32_t CR_HSIKERON_Msk = ( 0x1UL << CR_HSIKERON_Pos );
  static constexpr Reg32_t CR_HSIKERON     = CR_HSIKERON_Msk;
  static constexpr Reg32_t CR_HSIRDY_Pos   = ( 10U );
  static constexpr Reg32_t CR_HSIRDY_Msk   = ( 0x1UL << CR_HSIRDY_Pos );
  static constexpr Reg32_t CR_HSIRDY       = CR_HSIRDY_Msk;
  static constexpr Reg32_t CR_HSIASFS_Pos  = ( 11U );
  static constexpr Reg32_t CR_HSIASFS_Msk  = ( 0x1UL << CR_HSIASFS_Pos );
  static constexpr Reg32_t CR_HSIASFS      = CR_HSIASFS_Msk;

  static constexpr Reg32_t CR_HSEON_Pos  = ( 16U );
  static constexpr Reg32_t CR_HSEON_Msk  = ( 0x1UL << CR_HSEON_Pos );
  static constexpr Reg32_t CR_HSEON      = CR_HSEON_Msk;
  static constexpr Reg32_t CR_HSERDY_Pos = ( 17U );
  static constexpr Reg32_t CR_HSERDY_Msk = ( 0x1UL << CR_HSERDY_Pos );
  static constexpr Reg32_t CR_HSERDY     = CR_HSERDY_Msk;
  static constexpr Reg32_t CR_HSEBYP_Pos = ( 18U );
  static constexpr Reg32_t CR_HSEBYP_Msk = ( 0x1UL << CR_HSEBYP_Pos );
  static constexpr Reg32_t CR_HSEBYP     = CR_HSEBYP_Msk;
  static constexpr Reg32_t CR_CSSON_Pos  = ( 19U );
  static constexpr Reg32_t CR_CSSON_Msk  = ( 0x1UL << CR_CSSON_Pos );
  static constexpr Reg32_t CR_CSSON      = CR_CSSON_Msk;

  static constexpr Reg32_t CR_PLLON_Pos      = ( 24U );
  static constexpr Reg32_t CR_PLLON_Msk      = ( 0x1UL << CR_PLLON_Pos );
  static constexpr Reg32_t CR_PLLON          = CR_PLLON_Msk;
  static constexpr Reg32_t CR_PLLRDY_Pos     = ( 25U );
  static constexpr Reg32_t CR_PLLRDY_Msk     = ( 0x1UL << CR_PLLRDY_Pos );
  static constexpr Reg32_t CR_PLLRDY         = CR_PLLRDY_Msk;
  static constexpr Reg32_t CR_PLLSAI1ON_Pos  = ( 26U );
  static constexpr Reg32_t CR_PLLSAI1ON_Msk  = ( 0x1UL << CR_PLLSAI1ON_Pos );
  static constexpr Reg32_t CR_PLLSAI1ON      = CR_PLLSAI1ON_Msk;
  static constexpr Reg32_t CR_PLLSAI1RDY_Pos = ( 27U );
  static constexpr Reg32_t CR_PLLSAI1RDY_Msk = ( 0x1UL << CR_PLLSAI1RDY_Pos );
  static constexpr Reg32_t CR_PLLSAI1RDY     = CR_PLLSAI1RDY_Msk;

  /********************  Bit definition for ICSCR register  ***************/

  static constexpr Reg32_t ICSCR_MSICAL_Pos = ( 0U );
  static constexpr Reg32_t ICSCR_MSICAL_Msk = ( 0xFFUL << ICSCR_MSICAL_Pos );
  static constexpr Reg32_t ICSCR_MSICAL     = ICSCR_MSICAL_Msk;
  static constexpr Reg32_t ICSCR_MSICAL_0   = ( 0x01UL << ICSCR_MSICAL_Pos );
  static constexpr Reg32_t ICSCR_MSICAL_1   = ( 0x02UL << ICSCR_MSICAL_Pos );
  static constexpr Reg32_t ICSCR_MSICAL_2   = ( 0x04UL << ICSCR_MSICAL_Pos );
  static constexpr Reg32_t ICSCR_MSICAL_3   = ( 0x08UL << ICSCR_MSICAL_Pos );
  static constexpr Reg32_t ICSCR_MSICAL_4   = ( 0x10UL << ICSCR_MSICAL_Pos );
  static constexpr Reg32_t ICSCR_MSICAL_5   = ( 0x20UL << ICSCR_MSICAL_Pos );
  static constexpr Reg32_t ICSCR_MSICAL_6   = ( 0x40UL << ICSCR_MSICAL_Pos );
  static constexpr Reg32_t ICSCR_MSICAL_7   = ( 0x80UL << ICSCR_MSICAL_Pos );


  static constexpr Reg32_t ICSCR_MSITRIM_Pos = ( 8U );
  static constexpr Reg32_t ICSCR_MSITRIM_Msk = ( 0xFFUL << ICSCR_MSITRIM_Pos );
  static constexpr Reg32_t ICSCR_MSITRIM     = ICSCR_MSITRIM_Msk;
  static constexpr Reg32_t ICSCR_MSITRIM_0   = ( 0x01UL << ICSCR_MSITRIM_Pos );
  static constexpr Reg32_t ICSCR_MSITRIM_1   = ( 0x02UL << ICSCR_MSITRIM_Pos );
  static constexpr Reg32_t ICSCR_MSITRIM_2   = ( 0x04UL << ICSCR_MSITRIM_Pos );
  static constexpr Reg32_t ICSCR_MSITRIM_3   = ( 0x08UL << ICSCR_MSITRIM_Pos );
  static constexpr Reg32_t ICSCR_MSITRIM_4   = ( 0x10UL << ICSCR_MSITRIM_Pos );
  static constexpr Reg32_t ICSCR_MSITRIM_5   = ( 0x20UL << ICSCR_MSITRIM_Pos );
  static constexpr Reg32_t ICSCR_MSITRIM_6   = ( 0x40UL << ICSCR_MSITRIM_Pos );
  static constexpr Reg32_t ICSCR_MSITRIM_7   = ( 0x80UL << ICSCR_MSITRIM_Pos );


  static constexpr Reg32_t ICSCR_HSICAL_Pos = ( 16U );
  static constexpr Reg32_t ICSCR_HSICAL_Msk = ( 0xFFUL << ICSCR_HSICAL_Pos );
  static constexpr Reg32_t ICSCR_HSICAL     = ICSCR_HSICAL_Msk;
  static constexpr Reg32_t ICSCR_HSICAL_0   = ( 0x01UL << ICSCR_HSICAL_Pos );
  static constexpr Reg32_t ICSCR_HSICAL_1   = ( 0x02UL << ICSCR_HSICAL_Pos );
  static constexpr Reg32_t ICSCR_HSICAL_2   = ( 0x04UL << ICSCR_HSICAL_Pos );
  static constexpr Reg32_t ICSCR_HSICAL_3   = ( 0x08UL << ICSCR_HSICAL_Pos );
  static constexpr Reg32_t ICSCR_HSICAL_4   = ( 0x10UL << ICSCR_HSICAL_Pos );
  static constexpr Reg32_t ICSCR_HSICAL_5   = ( 0x20UL << ICSCR_HSICAL_Pos );
  static constexpr Reg32_t ICSCR_HSICAL_6   = ( 0x40UL << ICSCR_HSICAL_Pos );
  static constexpr Reg32_t ICSCR_HSICAL_7   = ( 0x80UL << ICSCR_HSICAL_Pos );


  static constexpr Reg32_t ICSCR_HSITRIM_Pos = ( 24U );
  static constexpr Reg32_t ICSCR_HSITRIM_Msk = ( 0x1FUL << ICSCR_HSITRIM_Pos );
  static constexpr Reg32_t ICSCR_HSITRIM     = ICSCR_HSITRIM_Msk;
  static constexpr Reg32_t ICSCR_HSITRIM_0   = ( 0x01UL << ICSCR_HSITRIM_Pos );
  static constexpr Reg32_t ICSCR_HSITRIM_1   = ( 0x02UL << ICSCR_HSITRIM_Pos );
  static constexpr Reg32_t ICSCR_HSITRIM_2   = ( 0x04UL << ICSCR_HSITRIM_Pos );
  static constexpr Reg32_t ICSCR_HSITRIM_3   = ( 0x08UL << ICSCR_HSITRIM_Pos );
  static constexpr Reg32_t ICSCR_HSITRIM_4   = ( 0x10UL << ICSCR_HSITRIM_Pos );

  /********************  Bit definition for CFGR register  ******************/

  static constexpr Reg32_t CFGR_SW_Pos = ( 0U );
  static constexpr Reg32_t CFGR_SW_Msk = ( 0x3UL << CFGR_SW_Pos );
  static constexpr Reg32_t CFGR_SW     = CFGR_SW_Msk;
  static constexpr Reg32_t CFGR_SW_0   = ( 0x1UL << CFGR_SW_Pos );
  static constexpr Reg32_t CFGR_SW_1   = ( 0x2UL << CFGR_SW_Pos );

  static constexpr Reg32_t CFGR_SW_MSI = ( 0x00000000UL );
  static constexpr Reg32_t CFGR_SW_HSI = ( 0x00000001UL );
  static constexpr Reg32_t CFGR_SW_HSE = ( 0x00000002UL );
  static constexpr Reg32_t CFGR_SW_PLL = ( 0x00000003UL );


  static constexpr Reg32_t CFGR_SWS_Pos = ( 2U );
  static constexpr Reg32_t CFGR_SWS_Msk = ( 0x3UL << CFGR_SWS_Pos );
  static constexpr Reg32_t CFGR_SWS     = CFGR_SWS_Msk;
  static constexpr Reg32_t CFGR_SWS_0   = ( 0x1UL << CFGR_SWS_Pos );
  static constexpr Reg32_t CFGR_SWS_1   = ( 0x2UL << CFGR_SWS_Pos );

  static constexpr Reg32_t CFGR_SWS_MSI = ( 0x00000000UL );
  static constexpr Reg32_t CFGR_SWS_HSI = ( 0x00000004UL );
  static constexpr Reg32_t CFGR_SWS_HSE = ( 0x00000008UL );
  static constexpr Reg32_t CFGR_SWS_PLL = ( 0x0000000CUL );


  static constexpr Reg32_t CFGR_HPRE_Pos = ( 4U );
  static constexpr Reg32_t CFGR_HPRE_Msk = ( 0xFUL << CFGR_HPRE_Pos );
  static constexpr Reg32_t CFGR_HPRE     = CFGR_HPRE_Msk;
  static constexpr Reg32_t CFGR_HPRE_0   = ( 0x1UL << CFGR_HPRE_Pos );
  static constexpr Reg32_t CFGR_HPRE_1   = ( 0x2UL << CFGR_HPRE_Pos );
  static constexpr Reg32_t CFGR_HPRE_2   = ( 0x4UL << CFGR_HPRE_Pos );
  static constexpr Reg32_t CFGR_HPRE_3   = ( 0x8UL << CFGR_HPRE_Pos );

  static constexpr Reg32_t CFGR_HPRE_DIV1   = ( 0x00000000UL );
  static constexpr Reg32_t CFGR_HPRE_DIV2   = ( 0x00000080UL );
  static constexpr Reg32_t CFGR_HPRE_DIV4   = ( 0x00000090UL );
  static constexpr Reg32_t CFGR_HPRE_DIV8   = ( 0x000000A0UL );
  static constexpr Reg32_t CFGR_HPRE_DIV16  = ( 0x000000B0UL );
  static constexpr Reg32_t CFGR_HPRE_DIV64  = ( 0x000000C0UL );
  static constexpr Reg32_t CFGR_HPRE_DIV128 = ( 0x000000D0UL );
  static constexpr Reg32_t CFGR_HPRE_DIV256 = ( 0x000000E0UL );
  static constexpr Reg32_t CFGR_HPRE_DIV512 = ( 0x000000F0UL );


  static constexpr Reg32_t CFGR_PPRE1_Pos = ( 8U );
  static constexpr Reg32_t CFGR_PPRE1_Msk = ( 0x7UL << CFGR_PPRE1_Pos );
  static constexpr Reg32_t CFGR_PPRE1     = CFGR_PPRE1_Msk;
  static constexpr Reg32_t CFGR_PPRE1_0   = ( 0x1UL << CFGR_PPRE1_Pos );
  static constexpr Reg32_t CFGR_PPRE1_1   = ( 0x2UL << CFGR_PPRE1_Pos );
  static constexpr Reg32_t CFGR_PPRE1_2   = ( 0x4UL << CFGR_PPRE1_Pos );

  static constexpr Reg32_t CFGR_PPRE1_DIV1  = ( 0x00000000UL );
  static constexpr Reg32_t CFGR_PPRE1_DIV2  = ( 0x00000400UL );
  static constexpr Reg32_t CFGR_PPRE1_DIV4  = ( 0x00000500UL );
  static constexpr Reg32_t CFGR_PPRE1_DIV8  = ( 0x00000600UL );
  static constexpr Reg32_t CFGR_PPRE1_DIV16 = ( 0x00000700UL );


  static constexpr Reg32_t CFGR_PPRE2_Pos = ( 11U );
  static constexpr Reg32_t CFGR_PPRE2_Msk = ( 0x7UL << CFGR_PPRE2_Pos );
  static constexpr Reg32_t CFGR_PPRE2     = CFGR_PPRE2_Msk;
  static constexpr Reg32_t CFGR_PPRE2_0   = ( 0x1UL << CFGR_PPRE2_Pos );
  static constexpr Reg32_t CFGR_PPRE2_1   = ( 0x2UL << CFGR_PPRE2_Pos );
  static constexpr Reg32_t CFGR_PPRE2_2   = ( 0x4UL << CFGR_PPRE2_Pos );

  static constexpr Reg32_t CFGR_PPRE2_DIV1  = ( 0x00000000UL );
  static constexpr Reg32_t CFGR_PPRE2_DIV2  = ( 0x00002000UL );
  static constexpr Reg32_t CFGR_PPRE2_DIV4  = ( 0x00002800UL );
  static constexpr Reg32_t CFGR_PPRE2_DIV8  = ( 0x00003000UL );
  static constexpr Reg32_t CFGR_PPRE2_DIV16 = ( 0x00003800UL );

  static constexpr Reg32_t CFGR_STOPWUCK_Pos = ( 15U );
  static constexpr Reg32_t CFGR_STOPWUCK_Msk = ( 0x1UL << CFGR_STOPWUCK_Pos );
  static constexpr Reg32_t CFGR_STOPWUCK     = CFGR_STOPWUCK_Msk;


  static constexpr Reg32_t CFGR_MCOSEL_Pos = ( 24U );
  static constexpr Reg32_t CFGR_MCOSEL_Msk = ( 0xFUL << CFGR_MCOSEL_Pos );
  static constexpr Reg32_t CFGR_MCOSEL     = CFGR_MCOSEL_Msk;
  static constexpr Reg32_t CFGR_MCOSEL_0   = ( 0x1UL << CFGR_MCOSEL_Pos );
  static constexpr Reg32_t CFGR_MCOSEL_1   = ( 0x2UL << CFGR_MCOSEL_Pos );
  static constexpr Reg32_t CFGR_MCOSEL_2   = ( 0x4UL << CFGR_MCOSEL_Pos );
  static constexpr Reg32_t CFGR_MCOSEL_3   = ( 0x8UL << CFGR_MCOSEL_Pos );

  static constexpr Reg32_t CFGR_MCOPRE_Pos = ( 28U );
  static constexpr Reg32_t CFGR_MCOPRE_Msk = ( 0x7UL << CFGR_MCOPRE_Pos );
  static constexpr Reg32_t CFGR_MCOPRE     = CFGR_MCOPRE_Msk;
  static constexpr Reg32_t CFGR_MCOPRE_0   = ( 0x1UL << CFGR_MCOPRE_Pos );
  static constexpr Reg32_t CFGR_MCOPRE_1   = ( 0x2UL << CFGR_MCOPRE_Pos );
  static constexpr Reg32_t CFGR_MCOPRE_2   = ( 0x4UL << CFGR_MCOPRE_Pos );

  static constexpr Reg32_t CFGR_MCOPRE_DIV1  = ( 0x00000000UL );
  static constexpr Reg32_t CFGR_MCOPRE_DIV2  = ( 0x10000000UL );
  static constexpr Reg32_t CFGR_MCOPRE_DIV4  = ( 0x20000000UL );
  static constexpr Reg32_t CFGR_MCOPRE_DIV8  = ( 0x30000000UL );
  static constexpr Reg32_t CFGR_MCOPRE_DIV16 = ( 0x40000000UL );

  /********************  Bit definition for PLLCFGR register  ***************/
  static constexpr Reg32_t PLLCFGR_PLLSRC_Pos = ( 0U );
  static constexpr Reg32_t PLLCFGR_PLLSRC_Msk = ( 0x3UL << PLLCFGR_PLLSRC_Pos );
  static constexpr Reg32_t PLLCFGR_PLLSRC     = PLLCFGR_PLLSRC_Msk;

  static constexpr Reg32_t PLLCFGR_PLLSRC_MSI_Pos = ( 0U );
  static constexpr Reg32_t PLLCFGR_PLLSRC_MSI_Msk = ( 0x1UL << PLLCFGR_PLLSRC_MSI_Pos );
  static constexpr Reg32_t PLLCFGR_PLLSRC_MSI     = PLLCFGR_PLLSRC_MSI_Msk;
  static constexpr Reg32_t PLLCFGR_PLLSRC_HSI_Pos = ( 1U );
  static constexpr Reg32_t PLLCFGR_PLLSRC_HSI_Msk = ( 0x1UL << PLLCFGR_PLLSRC_HSI_Pos );
  static constexpr Reg32_t PLLCFGR_PLLSRC_HSI     = PLLCFGR_PLLSRC_HSI_Msk;
  static constexpr Reg32_t PLLCFGR_PLLSRC_HSE_Pos = ( 0U );
  static constexpr Reg32_t PLLCFGR_PLLSRC_HSE_Msk = ( 0x3UL << PLLCFGR_PLLSRC_HSE_Pos );
  static constexpr Reg32_t PLLCFGR_PLLSRC_HSE     = PLLCFGR_PLLSRC_HSE_Msk;

  static constexpr Reg32_t PLLCFGR_PLLM_Pos = ( 4U );
  static constexpr Reg32_t PLLCFGR_PLLM_Msk = ( 0x7UL << PLLCFGR_PLLM_Pos );
  static constexpr Reg32_t PLLCFGR_PLLM     = PLLCFGR_PLLM_Msk;
  static constexpr Reg32_t PLLCFGR_PLLM_0   = ( 0x1UL << PLLCFGR_PLLM_Pos );
  static constexpr Reg32_t PLLCFGR_PLLM_1   = ( 0x2UL << PLLCFGR_PLLM_Pos );
  static constexpr Reg32_t PLLCFGR_PLLM_2   = ( 0x4UL << PLLCFGR_PLLM_Pos );

  static constexpr Reg32_t PLLCFGR_PLLN_Pos = ( 8U );
  static constexpr Reg32_t PLLCFGR_PLLN_Msk = ( 0x7FUL << PLLCFGR_PLLN_Pos );
  static constexpr Reg32_t PLLCFGR_PLLN     = PLLCFGR_PLLN_Msk;
  static constexpr Reg32_t PLLCFGR_PLLN_0   = ( 0x01UL << PLLCFGR_PLLN_Pos );
  static constexpr Reg32_t PLLCFGR_PLLN_1   = ( 0x02UL << PLLCFGR_PLLN_Pos );
  static constexpr Reg32_t PLLCFGR_PLLN_2   = ( 0x04UL << PLLCFGR_PLLN_Pos );
  static constexpr Reg32_t PLLCFGR_PLLN_3   = ( 0x08UL << PLLCFGR_PLLN_Pos );
  static constexpr Reg32_t PLLCFGR_PLLN_4   = ( 0x10UL << PLLCFGR_PLLN_Pos );
  static constexpr Reg32_t PLLCFGR_PLLN_5   = ( 0x20UL << PLLCFGR_PLLN_Pos );
  static constexpr Reg32_t PLLCFGR_PLLN_6   = ( 0x40UL << PLLCFGR_PLLN_Pos );

  static constexpr Reg32_t PLLCFGR_PLLPEN_Pos = ( 16U );
  static constexpr Reg32_t PLLCFGR_PLLPEN_Msk = ( 0x1UL << PLLCFGR_PLLPEN_Pos );
  static constexpr Reg32_t PLLCFGR_PLLPEN     = PLLCFGR_PLLPEN_Msk;
  static constexpr Reg32_t PLLCFGR_PLLP_Pos   = ( 17U );
  static constexpr Reg32_t PLLCFGR_PLLP_Msk   = ( 0x1UL << PLLCFGR_PLLP_Pos );
  static constexpr Reg32_t PLLCFGR_PLLP       = PLLCFGR_PLLP_Msk;
  static constexpr Reg32_t PLLCFGR_PLLQEN_Pos = ( 20U );
  static constexpr Reg32_t PLLCFGR_PLLQEN_Msk = ( 0x1UL << PLLCFGR_PLLQEN_Pos );
  static constexpr Reg32_t PLLCFGR_PLLQEN     = PLLCFGR_PLLQEN_Msk;

  static constexpr Reg32_t PLLCFGR_PLLQ_Pos = ( 21U );
  static constexpr Reg32_t PLLCFGR_PLLQ_Msk = ( 0x3UL << PLLCFGR_PLLQ_Pos );
  static constexpr Reg32_t PLLCFGR_PLLQ     = PLLCFGR_PLLQ_Msk;
  static constexpr Reg32_t PLLCFGR_PLLQ_0   = ( 0x1UL << PLLCFGR_PLLQ_Pos );
  static constexpr Reg32_t PLLCFGR_PLLQ_1   = ( 0x2UL << PLLCFGR_PLLQ_Pos );

  static constexpr Reg32_t PLLCFGR_PLLREN_Pos = ( 24U );
  static constexpr Reg32_t PLLCFGR_PLLREN_Msk = ( 0x1UL << PLLCFGR_PLLREN_Pos );
  static constexpr Reg32_t PLLCFGR_PLLREN     = PLLCFGR_PLLREN_Msk;
  static constexpr Reg32_t PLLCFGR_PLLR_Pos   = ( 25U );
  static constexpr Reg32_t PLLCFGR_PLLR_Msk   = ( 0x3UL << PLLCFGR_PLLR_Pos );
  static constexpr Reg32_t PLLCFGR_PLLR       = PLLCFGR_PLLR_Msk;
  static constexpr Reg32_t PLLCFGR_PLLR_0     = ( 0x1UL << PLLCFGR_PLLR_Pos );
  static constexpr Reg32_t PLLCFGR_PLLR_1     = ( 0x2UL << PLLCFGR_PLLR_Pos );

  static constexpr Reg32_t PLLCFGR_PLLPDIV_Pos = ( 27U );
  static constexpr Reg32_t PLLCFGR_PLLPDIV_Msk = ( 0x1FUL << PLLCFGR_PLLPDIV_Pos );
  static constexpr Reg32_t PLLCFGR_PLLPDIV     = PLLCFGR_PLLPDIV_Msk;
  static constexpr Reg32_t PLLCFGR_PLLPDIV_0   = ( 0x01UL << PLLCFGR_PLLPDIV_Pos );
  static constexpr Reg32_t PLLCFGR_PLLPDIV_1   = ( 0x02UL << PLLCFGR_PLLPDIV_Pos );
  static constexpr Reg32_t PLLCFGR_PLLPDIV_2   = ( 0x04UL << PLLCFGR_PLLPDIV_Pos );
  static constexpr Reg32_t PLLCFGR_PLLPDIV_3   = ( 0x08UL << PLLCFGR_PLLPDIV_Pos );
  static constexpr Reg32_t PLLCFGR_PLLPDIV_4   = ( 0x10UL << PLLCFGR_PLLPDIV_Pos );

  /********************  Bit definition for PLLSAI1CFGR register  ************/
  static constexpr Reg32_t PLLSAI1CFGR_PLLSAI1N_Pos = ( 8U );
  static constexpr Reg32_t PLLSAI1CFGR_PLLSAI1N_Msk = ( 0x7FUL << PLLSAI1CFGR_PLLSAI1N_Pos );
  static constexpr Reg32_t PLLSAI1CFGR_PLLSAI1N     = PLLSAI1CFGR_PLLSAI1N_Msk;
  static constexpr Reg32_t PLLSAI1CFGR_PLLSAI1N_0   = ( 0x01UL << PLLSAI1CFGR_PLLSAI1N_Pos );
  static constexpr Reg32_t PLLSAI1CFGR_PLLSAI1N_1   = ( 0x02UL << PLLSAI1CFGR_PLLSAI1N_Pos );
  static constexpr Reg32_t PLLSAI1CFGR_PLLSAI1N_2   = ( 0x04UL << PLLSAI1CFGR_PLLSAI1N_Pos );
  static constexpr Reg32_t PLLSAI1CFGR_PLLSAI1N_3   = ( 0x08UL << PLLSAI1CFGR_PLLSAI1N_Pos );
  static constexpr Reg32_t PLLSAI1CFGR_PLLSAI1N_4   = ( 0x10UL << PLLSAI1CFGR_PLLSAI1N_Pos );
  static constexpr Reg32_t PLLSAI1CFGR_PLLSAI1N_5   = ( 0x20UL << PLLSAI1CFGR_PLLSAI1N_Pos );
  static constexpr Reg32_t PLLSAI1CFGR_PLLSAI1N_6   = ( 0x40UL << PLLSAI1CFGR_PLLSAI1N_Pos );

  static constexpr Reg32_t PLLSAI1CFGR_PLLSAI1PEN_Pos = ( 16U );
  static constexpr Reg32_t PLLSAI1CFGR_PLLSAI1PEN_Msk = ( 0x1UL << PLLSAI1CFGR_PLLSAI1PEN_Pos );
  static constexpr Reg32_t PLLSAI1CFGR_PLLSAI1PEN     = PLLSAI1CFGR_PLLSAI1PEN_Msk;
  static constexpr Reg32_t PLLSAI1CFGR_PLLSAI1P_Pos   = ( 17U );
  static constexpr Reg32_t PLLSAI1CFGR_PLLSAI1P_Msk   = ( 0x1UL << PLLSAI1CFGR_PLLSAI1P_Pos );
  static constexpr Reg32_t PLLSAI1CFGR_PLLSAI1P       = PLLSAI1CFGR_PLLSAI1P_Msk;

  static constexpr Reg32_t PLLSAI1CFGR_PLLSAI1QEN_Pos = ( 20U );
  static constexpr Reg32_t PLLSAI1CFGR_PLLSAI1QEN_Msk = ( 0x1UL << PLLSAI1CFGR_PLLSAI1QEN_Pos );
  static constexpr Reg32_t PLLSAI1CFGR_PLLSAI1QEN     = PLLSAI1CFGR_PLLSAI1QEN_Msk;
  static constexpr Reg32_t PLLSAI1CFGR_PLLSAI1Q_Pos   = ( 21U );
  static constexpr Reg32_t PLLSAI1CFGR_PLLSAI1Q_Msk   = ( 0x3UL << PLLSAI1CFGR_PLLSAI1Q_Pos );
  static constexpr Reg32_t PLLSAI1CFGR_PLLSAI1Q       = PLLSAI1CFGR_PLLSAI1Q_Msk;
  static constexpr Reg32_t PLLSAI1CFGR_PLLSAI1Q_0     = ( 0x1UL << PLLSAI1CFGR_PLLSAI1Q_Pos );
  static constexpr Reg32_t PLLSAI1CFGR_PLLSAI1Q_1     = ( 0x2UL << PLLSAI1CFGR_PLLSAI1Q_Pos );

  static constexpr Reg32_t PLLSAI1CFGR_PLLSAI1REN_Pos = ( 24U );
  static constexpr Reg32_t PLLSAI1CFGR_PLLSAI1REN_Msk = ( 0x1UL << PLLSAI1CFGR_PLLSAI1REN_Pos );
  static constexpr Reg32_t PLLSAI1CFGR_PLLSAI1REN     = PLLSAI1CFGR_PLLSAI1REN_Msk;
  static constexpr Reg32_t PLLSAI1CFGR_PLLSAI1R_Pos   = ( 25U );
  static constexpr Reg32_t PLLSAI1CFGR_PLLSAI1R_Msk   = ( 0x3UL << PLLSAI1CFGR_PLLSAI1R_Pos );
  static constexpr Reg32_t PLLSAI1CFGR_PLLSAI1R       = PLLSAI1CFGR_PLLSAI1R_Msk;
  static constexpr Reg32_t PLLSAI1CFGR_PLLSAI1R_0     = ( 0x1UL << PLLSAI1CFGR_PLLSAI1R_Pos );
  static constexpr Reg32_t PLLSAI1CFGR_PLLSAI1R_1     = ( 0x2UL << PLLSAI1CFGR_PLLSAI1R_Pos );

  static constexpr Reg32_t PLLSAI1CFGR_PLLSAI1PDIV_Pos = ( 27U );
  static constexpr Reg32_t PLLSAI1CFGR_PLLSAI1PDIV_Msk = ( 0x1FUL << PLLSAI1CFGR_PLLSAI1PDIV_Pos );
  static constexpr Reg32_t PLLSAI1CFGR_PLLSAI1PDIV     = PLLSAI1CFGR_PLLSAI1PDIV_Msk;
  static constexpr Reg32_t PLLSAI1CFGR_PLLSAI1PDIV_0   = ( 0x01UL << PLLSAI1CFGR_PLLSAI1PDIV_Pos );
  static constexpr Reg32_t PLLSAI1CFGR_PLLSAI1PDIV_1   = ( 0x02UL << PLLSAI1CFGR_PLLSAI1PDIV_Pos );
  static constexpr Reg32_t PLLSAI1CFGR_PLLSAI1PDIV_2   = ( 0x04UL << PLLSAI1CFGR_PLLSAI1PDIV_Pos );
  static constexpr Reg32_t PLLSAI1CFGR_PLLSAI1PDIV_3   = ( 0x08UL << PLLSAI1CFGR_PLLSAI1PDIV_Pos );
  static constexpr Reg32_t PLLSAI1CFGR_PLLSAI1PDIV_4   = ( 0x10UL << PLLSAI1CFGR_PLLSAI1PDIV_Pos );

  /********************  Bit definition for CIER register  ******************/
  static constexpr Reg32_t CIER_LSIRDYIE_Pos     = ( 0U );
  static constexpr Reg32_t CIER_LSIRDYIE_Msk     = ( 0x1UL << CIER_LSIRDYIE_Pos );
  static constexpr Reg32_t CIER_LSIRDYIE         = CIER_LSIRDYIE_Msk;
  static constexpr Reg32_t CIER_LSERDYIE_Pos     = ( 1U );
  static constexpr Reg32_t CIER_LSERDYIE_Msk     = ( 0x1UL << CIER_LSERDYIE_Pos );
  static constexpr Reg32_t CIER_LSERDYIE         = CIER_LSERDYIE_Msk;
  static constexpr Reg32_t CIER_MSIRDYIE_Pos     = ( 2U );
  static constexpr Reg32_t CIER_MSIRDYIE_Msk     = ( 0x1UL << CIER_MSIRDYIE_Pos );
  static constexpr Reg32_t CIER_MSIRDYIE         = CIER_MSIRDYIE_Msk;
  static constexpr Reg32_t CIER_HSIRDYIE_Pos     = ( 3U );
  static constexpr Reg32_t CIER_HSIRDYIE_Msk     = ( 0x1UL << CIER_HSIRDYIE_Pos );
  static constexpr Reg32_t CIER_HSIRDYIE         = CIER_HSIRDYIE_Msk;
  static constexpr Reg32_t CIER_HSERDYIE_Pos     = ( 4U );
  static constexpr Reg32_t CIER_HSERDYIE_Msk     = ( 0x1UL << CIER_HSERDYIE_Pos );
  static constexpr Reg32_t CIER_HSERDYIE         = CIER_HSERDYIE_Msk;
  static constexpr Reg32_t CIER_PLLRDYIE_Pos     = ( 5U );
  static constexpr Reg32_t CIER_PLLRDYIE_Msk     = ( 0x1UL << CIER_PLLRDYIE_Pos );
  static constexpr Reg32_t CIER_PLLRDYIE         = CIER_PLLRDYIE_Msk;
  static constexpr Reg32_t CIER_PLLSAI1RDYIE_Pos = ( 6U );
  static constexpr Reg32_t CIER_PLLSAI1RDYIE_Msk = ( 0x1UL << CIER_PLLSAI1RDYIE_Pos );
  static constexpr Reg32_t CIER_PLLSAI1RDYIE     = CIER_PLLSAI1RDYIE_Msk;
  static constexpr Reg32_t CIER_LSECSSIE_Pos     = ( 9U );
  static constexpr Reg32_t CIER_LSECSSIE_Msk     = ( 0x1UL << CIER_LSECSSIE_Pos );
  static constexpr Reg32_t CIER_LSECSSIE         = CIER_LSECSSIE_Msk;
  static constexpr Reg32_t CIER_HSI48RDYIE_Pos   = ( 10U );
  static constexpr Reg32_t CIER_HSI48RDYIE_Msk   = ( 0x1UL << CIER_HSI48RDYIE_Pos );
  static constexpr Reg32_t CIER_HSI48RDYIE       = CIER_HSI48RDYIE_Msk;

  /********************  Bit definition for CIFR register  ******************/
  static constexpr Reg32_t CIFR_LSIRDYF_Pos     = ( 0U );
  static constexpr Reg32_t CIFR_LSIRDYF_Msk     = ( 0x1UL << CIFR_LSIRDYF_Pos );
  static constexpr Reg32_t CIFR_LSIRDYF         = CIFR_LSIRDYF_Msk;
  static constexpr Reg32_t CIFR_LSERDYF_Pos     = ( 1U );
  static constexpr Reg32_t CIFR_LSERDYF_Msk     = ( 0x1UL << CIFR_LSERDYF_Pos );
  static constexpr Reg32_t CIFR_LSERDYF         = CIFR_LSERDYF_Msk;
  static constexpr Reg32_t CIFR_MSIRDYF_Pos     = ( 2U );
  static constexpr Reg32_t CIFR_MSIRDYF_Msk     = ( 0x1UL << CIFR_MSIRDYF_Pos );
  static constexpr Reg32_t CIFR_MSIRDYF         = CIFR_MSIRDYF_Msk;
  static constexpr Reg32_t CIFR_HSIRDYF_Pos     = ( 3U );
  static constexpr Reg32_t CIFR_HSIRDYF_Msk     = ( 0x1UL << CIFR_HSIRDYF_Pos );
  static constexpr Reg32_t CIFR_HSIRDYF         = CIFR_HSIRDYF_Msk;
  static constexpr Reg32_t CIFR_HSERDYF_Pos     = ( 4U );
  static constexpr Reg32_t CIFR_HSERDYF_Msk     = ( 0x1UL << CIFR_HSERDYF_Pos );
  static constexpr Reg32_t CIFR_HSERDYF         = CIFR_HSERDYF_Msk;
  static constexpr Reg32_t CIFR_PLLRDYF_Pos     = ( 5U );
  static constexpr Reg32_t CIFR_PLLRDYF_Msk     = ( 0x1UL << CIFR_PLLRDYF_Pos );
  static constexpr Reg32_t CIFR_PLLRDYF         = CIFR_PLLRDYF_Msk;
  static constexpr Reg32_t CIFR_PLLSAI1RDYF_Pos = ( 6U );
  static constexpr Reg32_t CIFR_PLLSAI1RDYF_Msk = ( 0x1UL << CIFR_PLLSAI1RDYF_Pos );
  static constexpr Reg32_t CIFR_PLLSAI1RDYF     = CIFR_PLLSAI1RDYF_Msk;
  static constexpr Reg32_t CIFR_CSSF_Pos        = ( 8U );
  static constexpr Reg32_t CIFR_CSSF_Msk        = ( 0x1UL << CIFR_CSSF_Pos );
  static constexpr Reg32_t CIFR_CSSF            = CIFR_CSSF_Msk;
  static constexpr Reg32_t CIFR_LSECSSF_Pos     = ( 9U );
  static constexpr Reg32_t CIFR_LSECSSF_Msk     = ( 0x1UL << CIFR_LSECSSF_Pos );
  static constexpr Reg32_t CIFR_LSECSSF         = CIFR_LSECSSF_Msk;
  static constexpr Reg32_t CIFR_HSI48RDYF_Pos   = ( 10U );
  static constexpr Reg32_t CIFR_HSI48RDYF_Msk   = ( 0x1UL << CIFR_HSI48RDYF_Pos );
  static constexpr Reg32_t CIFR_HSI48RDYF       = CIFR_HSI48RDYF_Msk;

  /********************  Bit definition for CICR register  ******************/
  static constexpr Reg32_t CICR_LSIRDYC_Pos     = ( 0U );
  static constexpr Reg32_t CICR_LSIRDYC_Msk     = ( 0x1UL << CICR_LSIRDYC_Pos );
  static constexpr Reg32_t CICR_LSIRDYC         = CICR_LSIRDYC_Msk;
  static constexpr Reg32_t CICR_LSERDYC_Pos     = ( 1U );
  static constexpr Reg32_t CICR_LSERDYC_Msk     = ( 0x1UL << CICR_LSERDYC_Pos );
  static constexpr Reg32_t CICR_LSERDYC         = CICR_LSERDYC_Msk;
  static constexpr Reg32_t CICR_MSIRDYC_Pos     = ( 2U );
  static constexpr Reg32_t CICR_MSIRDYC_Msk     = ( 0x1UL << CICR_MSIRDYC_Pos );
  static constexpr Reg32_t CICR_MSIRDYC         = CICR_MSIRDYC_Msk;
  static constexpr Reg32_t CICR_HSIRDYC_Pos     = ( 3U );
  static constexpr Reg32_t CICR_HSIRDYC_Msk     = ( 0x1UL << CICR_HSIRDYC_Pos );
  static constexpr Reg32_t CICR_HSIRDYC         = CICR_HSIRDYC_Msk;
  static constexpr Reg32_t CICR_HSERDYC_Pos     = ( 4U );
  static constexpr Reg32_t CICR_HSERDYC_Msk     = ( 0x1UL << CICR_HSERDYC_Pos );
  static constexpr Reg32_t CICR_HSERDYC         = CICR_HSERDYC_Msk;
  static constexpr Reg32_t CICR_PLLRDYC_Pos     = ( 5U );
  static constexpr Reg32_t CICR_PLLRDYC_Msk     = ( 0x1UL << CICR_PLLRDYC_Pos );
  static constexpr Reg32_t CICR_PLLRDYC         = CICR_PLLRDYC_Msk;
  static constexpr Reg32_t CICR_PLLSAI1RDYC_Pos = ( 6U );
  static constexpr Reg32_t CICR_PLLSAI1RDYC_Msk = ( 0x1UL << CICR_PLLSAI1RDYC_Pos );
  static constexpr Reg32_t CICR_PLLSAI1RDYC     = CICR_PLLSAI1RDYC_Msk;
  static constexpr Reg32_t CICR_CSSC_Pos        = ( 8U );
  static constexpr Reg32_t CICR_CSSC_Msk        = ( 0x1UL << CICR_CSSC_Pos );
  static constexpr Reg32_t CICR_CSSC            = CICR_CSSC_Msk;
  static constexpr Reg32_t CICR_LSECSSC_Pos     = ( 9U );
  static constexpr Reg32_t CICR_LSECSSC_Msk     = ( 0x1UL << CICR_LSECSSC_Pos );
  static constexpr Reg32_t CICR_LSECSSC         = CICR_LSECSSC_Msk;
  static constexpr Reg32_t CICR_HSI48RDYC_Pos   = ( 10U );
  static constexpr Reg32_t CICR_HSI48RDYC_Msk   = ( 0x1UL << CICR_HSI48RDYC_Pos );
  static constexpr Reg32_t CICR_HSI48RDYC       = CICR_HSI48RDYC_Msk;

  /********************  Bit definition for AHB1RSTR register  **************/
  static constexpr Reg32_t AHB1RSTR_DMA1RST_Pos  = ( 0U );
  static constexpr Reg32_t AHB1RSTR_DMA1RST_Msk  = ( 0x1UL << AHB1RSTR_DMA1RST_Pos );
  static constexpr Reg32_t AHB1RSTR_DMA1RST      = AHB1RSTR_DMA1RST_Msk;
  static constexpr Reg32_t AHB1RSTR_DMA2RST_Pos  = ( 1U );
  static constexpr Reg32_t AHB1RSTR_DMA2RST_Msk  = ( 0x1UL << AHB1RSTR_DMA2RST_Pos );
  static constexpr Reg32_t AHB1RSTR_DMA2RST      = AHB1RSTR_DMA2RST_Msk;
  static constexpr Reg32_t AHB1RSTR_FLASHRST_Pos = ( 8U );
  static constexpr Reg32_t AHB1RSTR_FLASHRST_Msk = ( 0x1UL << AHB1RSTR_FLASHRST_Pos );
  static constexpr Reg32_t AHB1RSTR_FLASHRST     = AHB1RSTR_FLASHRST_Msk;
  static constexpr Reg32_t AHB1RSTR_CRCRST_Pos   = ( 12U );
  static constexpr Reg32_t AHB1RSTR_CRCRST_Msk   = ( 0x1UL << AHB1RSTR_CRCRST_Pos );
  static constexpr Reg32_t AHB1RSTR_CRCRST       = AHB1RSTR_CRCRST_Msk;
  static constexpr Reg32_t AHB1RSTR_TSCRST_Pos   = ( 16U );
  static constexpr Reg32_t AHB1RSTR_TSCRST_Msk   = ( 0x1UL << AHB1RSTR_TSCRST_Pos );
  static constexpr Reg32_t AHB1RSTR_TSCRST       = AHB1RSTR_TSCRST_Msk;

  /********************  Bit definition for AHB2RSTR register  **************/
  static constexpr Reg32_t AHB2RSTR_GPIOARST_Pos = ( 0U );
  static constexpr Reg32_t AHB2RSTR_GPIOARST_Msk = ( 0x1UL << AHB2RSTR_GPIOARST_Pos );
  static constexpr Reg32_t AHB2RSTR_GPIOARST     = AHB2RSTR_GPIOARST_Msk;
  static constexpr Reg32_t AHB2RSTR_GPIOBRST_Pos = ( 1U );
  static constexpr Reg32_t AHB2RSTR_GPIOBRST_Msk = ( 0x1UL << AHB2RSTR_GPIOBRST_Pos );
  static constexpr Reg32_t AHB2RSTR_GPIOBRST     = AHB2RSTR_GPIOBRST_Msk;
  static constexpr Reg32_t AHB2RSTR_GPIOCRST_Pos = ( 2U );
  static constexpr Reg32_t AHB2RSTR_GPIOCRST_Msk = ( 0x1UL << AHB2RSTR_GPIOCRST_Pos );
  static constexpr Reg32_t AHB2RSTR_GPIOCRST     = AHB2RSTR_GPIOCRST_Msk;
  static constexpr Reg32_t AHB2RSTR_GPIOHRST_Pos = ( 7U );
  static constexpr Reg32_t AHB2RSTR_GPIOHRST_Msk = ( 0x1UL << AHB2RSTR_GPIOHRST_Pos );
  static constexpr Reg32_t AHB2RSTR_GPIOHRST     = AHB2RSTR_GPIOHRST_Msk;
  static constexpr Reg32_t AHB2RSTR_ADCRST_Pos   = ( 13U );
  static constexpr Reg32_t AHB2RSTR_ADCRST_Msk   = ( 0x1UL << AHB2RSTR_ADCRST_Pos );
  static constexpr Reg32_t AHB2RSTR_ADCRST       = AHB2RSTR_ADCRST_Msk;
  static constexpr Reg32_t AHB2RSTR_RNGRST_Pos   = ( 18U );
  static constexpr Reg32_t AHB2RSTR_RNGRST_Msk   = ( 0x1UL << AHB2RSTR_RNGRST_Pos );
  static constexpr Reg32_t AHB2RSTR_RNGRST       = AHB2RSTR_RNGRST_Msk;

  /********************  Bit definition for AHB3RSTR register  **************/
  static constexpr Reg32_t AHB3RSTR_QSPIRST_Pos = ( 8U );
  static constexpr Reg32_t AHB3RSTR_QSPIRST_Msk = ( 0x1UL << AHB3RSTR_QSPIRST_Pos );
  static constexpr Reg32_t AHB3RSTR_QSPIRST     = AHB3RSTR_QSPIRST_Msk;

  /********************  Bit definition for APB1RSTR1 register  **************/
  static constexpr Reg32_t APB1RSTR1_TIM2RST_Pos   = ( 0U );
  static constexpr Reg32_t APB1RSTR1_TIM2RST_Msk   = ( 0x1UL << APB1RSTR1_TIM2RST_Pos );
  static constexpr Reg32_t APB1RSTR1_TIM2RST       = APB1RSTR1_TIM2RST_Msk;
  static constexpr Reg32_t APB1RSTR1_TIM6RST_Pos   = ( 4U );
  static constexpr Reg32_t APB1RSTR1_TIM6RST_Msk   = ( 0x1UL << APB1RSTR1_TIM6RST_Pos );
  static constexpr Reg32_t APB1RSTR1_TIM6RST       = APB1RSTR1_TIM6RST_Msk;
  static constexpr Reg32_t APB1RSTR1_TIM7RST_Pos   = ( 5U );
  static constexpr Reg32_t APB1RSTR1_TIM7RST_Msk   = ( 0x1UL << APB1RSTR1_TIM7RST_Pos );
  static constexpr Reg32_t APB1RSTR1_TIM7RST       = APB1RSTR1_TIM7RST_Msk;
  static constexpr Reg32_t APB1RSTR1_SPI3RST_Pos   = ( 15U );
  static constexpr Reg32_t APB1RSTR1_SPI3RST_Msk   = ( 0x1UL << APB1RSTR1_SPI3RST_Pos );
  static constexpr Reg32_t APB1RSTR1_SPI3RST       = APB1RSTR1_SPI3RST_Msk;
  static constexpr Reg32_t APB1RSTR1_USART2RST_Pos = ( 17U );
  static constexpr Reg32_t APB1RSTR1_USART2RST_Msk = ( 0x1UL << APB1RSTR1_USART2RST_Pos );
  static constexpr Reg32_t APB1RSTR1_USART2RST     = APB1RSTR1_USART2RST_Msk;
  static constexpr Reg32_t APB1RSTR1_I2C1RST_Pos   = ( 21U );
  static constexpr Reg32_t APB1RSTR1_I2C1RST_Msk   = ( 0x1UL << APB1RSTR1_I2C1RST_Pos );
  static constexpr Reg32_t APB1RSTR1_I2C1RST       = APB1RSTR1_I2C1RST_Msk;
  static constexpr Reg32_t APB1RSTR1_I2C3RST_Pos   = ( 23U );
  static constexpr Reg32_t APB1RSTR1_I2C3RST_Msk   = ( 0x1UL << APB1RSTR1_I2C3RST_Pos );
  static constexpr Reg32_t APB1RSTR1_I2C3RST       = APB1RSTR1_I2C3RST_Msk;
  static constexpr Reg32_t APB1RSTR1_CRSRST_Pos    = ( 24U );
  static constexpr Reg32_t APB1RSTR1_CRSRST_Msk    = ( 0x1UL << APB1RSTR1_CRSRST_Pos );
  static constexpr Reg32_t APB1RSTR1_CRSRST        = APB1RSTR1_CRSRST_Msk;
  static constexpr Reg32_t APB1RSTR1_CAN1RST_Pos   = ( 25U );
  static constexpr Reg32_t APB1RSTR1_CAN1RST_Msk   = ( 0x1UL << APB1RSTR1_CAN1RST_Pos );
  static constexpr Reg32_t APB1RSTR1_CAN1RST       = APB1RSTR1_CAN1RST_Msk;
  static constexpr Reg32_t APB1RSTR1_USBFSRST_Pos  = ( 26U );
  static constexpr Reg32_t APB1RSTR1_USBFSRST_Msk  = ( 0x1UL << APB1RSTR1_USBFSRST_Pos );
  static constexpr Reg32_t APB1RSTR1_USBFSRST      = APB1RSTR1_USBFSRST_Msk;
  static constexpr Reg32_t APB1RSTR1_PWRRST_Pos    = ( 28U );
  static constexpr Reg32_t APB1RSTR1_PWRRST_Msk    = ( 0x1UL << APB1RSTR1_PWRRST_Pos );
  static constexpr Reg32_t APB1RSTR1_PWRRST        = APB1RSTR1_PWRRST_Msk;
  static constexpr Reg32_t APB1RSTR1_DAC1RST_Pos   = ( 29U );
  static constexpr Reg32_t APB1RSTR1_DAC1RST_Msk   = ( 0x1UL << APB1RSTR1_DAC1RST_Pos );
  static constexpr Reg32_t APB1RSTR1_DAC1RST       = APB1RSTR1_DAC1RST_Msk;
  static constexpr Reg32_t APB1RSTR1_OPAMPRST_Pos  = ( 30U );
  static constexpr Reg32_t APB1RSTR1_OPAMPRST_Msk  = ( 0x1UL << APB1RSTR1_OPAMPRST_Pos );
  static constexpr Reg32_t APB1RSTR1_OPAMPRST      = APB1RSTR1_OPAMPRST_Msk;
  static constexpr Reg32_t APB1RSTR1_LPTIM1RST_Pos = ( 31U );
  static constexpr Reg32_t APB1RSTR1_LPTIM1RST_Msk = ( 0x1UL << APB1RSTR1_LPTIM1RST_Pos );
  static constexpr Reg32_t APB1RSTR1_LPTIM1RST     = APB1RSTR1_LPTIM1RST_Msk;

  /********************  Bit definition for APB1RSTR2 register  **************/
  static constexpr Reg32_t APB1RSTR2_LPUART1RST_Pos = ( 0U );
  static constexpr Reg32_t APB1RSTR2_LPUART1RST_Msk = ( 0x1UL << APB1RSTR2_LPUART1RST_Pos );
  static constexpr Reg32_t APB1RSTR2_LPUART1RST     = APB1RSTR2_LPUART1RST_Msk;
  static constexpr Reg32_t APB1RSTR2_SWPMI1RST_Pos  = ( 2U );
  static constexpr Reg32_t APB1RSTR2_SWPMI1RST_Msk  = ( 0x1UL << APB1RSTR2_SWPMI1RST_Pos );
  static constexpr Reg32_t APB1RSTR2_SWPMI1RST      = APB1RSTR2_SWPMI1RST_Msk;
  static constexpr Reg32_t APB1RSTR2_LPTIM2RST_Pos  = ( 5U );
  static constexpr Reg32_t APB1RSTR2_LPTIM2RST_Msk  = ( 0x1UL << APB1RSTR2_LPTIM2RST_Pos );
  static constexpr Reg32_t APB1RSTR2_LPTIM2RST      = APB1RSTR2_LPTIM2RST_Msk;

  /********************  Bit definition for APB2RSTR register  **************/
  static constexpr Reg32_t APB2RSTR_SYSCFGRST_Pos = ( 0U );
  static constexpr Reg32_t APB2RSTR_SYSCFGRST_Msk = ( 0x1UL << APB2RSTR_SYSCFGRST_Pos );
  static constexpr Reg32_t APB2RSTR_SYSCFGRST     = APB2RSTR_SYSCFGRST_Msk;
  static constexpr Reg32_t APB2RSTR_TIM1RST_Pos   = ( 11U );
  static constexpr Reg32_t APB2RSTR_TIM1RST_Msk   = ( 0x1UL << APB2RSTR_TIM1RST_Pos );
  static constexpr Reg32_t APB2RSTR_TIM1RST       = APB2RSTR_TIM1RST_Msk;
  static constexpr Reg32_t APB2RSTR_SPI1RST_Pos   = ( 12U );
  static constexpr Reg32_t APB2RSTR_SPI1RST_Msk   = ( 0x1UL << APB2RSTR_SPI1RST_Pos );
  static constexpr Reg32_t APB2RSTR_SPI1RST       = APB2RSTR_SPI1RST_Msk;
  static constexpr Reg32_t APB2RSTR_USART1RST_Pos = ( 14U );
  static constexpr Reg32_t APB2RSTR_USART1RST_Msk = ( 0x1UL << APB2RSTR_USART1RST_Pos );
  static constexpr Reg32_t APB2RSTR_USART1RST     = APB2RSTR_USART1RST_Msk;
  static constexpr Reg32_t APB2RSTR_TIM15RST_Pos  = ( 16U );
  static constexpr Reg32_t APB2RSTR_TIM15RST_Msk  = ( 0x1UL << APB2RSTR_TIM15RST_Pos );
  static constexpr Reg32_t APB2RSTR_TIM15RST      = APB2RSTR_TIM15RST_Msk;
  static constexpr Reg32_t APB2RSTR_TIM16RST_Pos  = ( 17U );
  static constexpr Reg32_t APB2RSTR_TIM16RST_Msk  = ( 0x1UL << APB2RSTR_TIM16RST_Pos );
  static constexpr Reg32_t APB2RSTR_TIM16RST      = APB2RSTR_TIM16RST_Msk;
  static constexpr Reg32_t APB2RSTR_SAI1RST_Pos   = ( 21U );
  static constexpr Reg32_t APB2RSTR_SAI1RST_Msk   = ( 0x1UL << APB2RSTR_SAI1RST_Pos );
  static constexpr Reg32_t APB2RSTR_SAI1RST       = APB2RSTR_SAI1RST_Msk;

  /********************  Bit definition for AHB1ENR register  ***************/
  static constexpr Reg32_t AHB1ENR_DMA1EN_Pos  = ( 0U );
  static constexpr Reg32_t AHB1ENR_DMA1EN_Msk  = ( 0x1UL << AHB1ENR_DMA1EN_Pos );
  static constexpr Reg32_t AHB1ENR_DMA1EN      = AHB1ENR_DMA1EN_Msk;
  static constexpr Reg32_t AHB1ENR_DMA2EN_Pos  = ( 1U );
  static constexpr Reg32_t AHB1ENR_DMA2EN_Msk  = ( 0x1UL << AHB1ENR_DMA2EN_Pos );
  static constexpr Reg32_t AHB1ENR_DMA2EN      = AHB1ENR_DMA2EN_Msk;
  static constexpr Reg32_t AHB1ENR_FLASHEN_Pos = ( 8U );
  static constexpr Reg32_t AHB1ENR_FLASHEN_Msk = ( 0x1UL << AHB1ENR_FLASHEN_Pos );
  static constexpr Reg32_t AHB1ENR_FLASHEN     = AHB1ENR_FLASHEN_Msk;
  static constexpr Reg32_t AHB1ENR_CRCEN_Pos   = ( 12U );
  static constexpr Reg32_t AHB1ENR_CRCEN_Msk   = ( 0x1UL << AHB1ENR_CRCEN_Pos );
  static constexpr Reg32_t AHB1ENR_CRCEN       = AHB1ENR_CRCEN_Msk;
  static constexpr Reg32_t AHB1ENR_TSCEN_Pos   = ( 16U );
  static constexpr Reg32_t AHB1ENR_TSCEN_Msk   = ( 0x1UL << AHB1ENR_TSCEN_Pos );
  static constexpr Reg32_t AHB1ENR_TSCEN       = AHB1ENR_TSCEN_Msk;

  /********************  Bit definition for AHB2ENR register  ***************/
  static constexpr Reg32_t AHB2ENR_GPIOAEN_Pos = ( 0U );
  static constexpr Reg32_t AHB2ENR_GPIOAEN_Msk = ( 0x1UL << AHB2ENR_GPIOAEN_Pos );
  static constexpr Reg32_t AHB2ENR_GPIOAEN     = AHB2ENR_GPIOAEN_Msk;
  static constexpr Reg32_t AHB2ENR_GPIOBEN_Pos = ( 1U );
  static constexpr Reg32_t AHB2ENR_GPIOBEN_Msk = ( 0x1UL << AHB2ENR_GPIOBEN_Pos );
  static constexpr Reg32_t AHB2ENR_GPIOBEN     = AHB2ENR_GPIOBEN_Msk;
  static constexpr Reg32_t AHB2ENR_GPIOCEN_Pos = ( 2U );
  static constexpr Reg32_t AHB2ENR_GPIOCEN_Msk = ( 0x1UL << AHB2ENR_GPIOCEN_Pos );
  static constexpr Reg32_t AHB2ENR_GPIOCEN     = AHB2ENR_GPIOCEN_Msk;
  static constexpr Reg32_t AHB2ENR_GPIOHEN_Pos = ( 7U );
  static constexpr Reg32_t AHB2ENR_GPIOHEN_Msk = ( 0x1UL << AHB2ENR_GPIOHEN_Pos );
  static constexpr Reg32_t AHB2ENR_GPIOHEN     = AHB2ENR_GPIOHEN_Msk;
  static constexpr Reg32_t AHB2ENR_ADCEN_Pos   = ( 13U );
  static constexpr Reg32_t AHB2ENR_ADCEN_Msk   = ( 0x1UL << AHB2ENR_ADCEN_Pos );
  static constexpr Reg32_t AHB2ENR_ADCEN       = AHB2ENR_ADCEN_Msk;
  static constexpr Reg32_t AHB2ENR_RNGEN_Pos   = ( 18U );
  static constexpr Reg32_t AHB2ENR_RNGEN_Msk   = ( 0x1UL << AHB2ENR_RNGEN_Pos );
  static constexpr Reg32_t AHB2ENR_RNGEN       = AHB2ENR_RNGEN_Msk;

  /********************  Bit definition for AHB3ENR register  ***************/
  static constexpr Reg32_t AHB3ENR_QSPIEN_Pos = ( 8U );
  static constexpr Reg32_t AHB3ENR_QSPIEN_Msk = ( 0x1UL << AHB3ENR_QSPIEN_Pos );
  static constexpr Reg32_t AHB3ENR_QSPIEN     = AHB3ENR_QSPIEN_Msk;

  /********************  Bit definition for APB1ENR1 register  ***************/
  static constexpr Reg32_t APB1ENR1_TIM2EN_Pos   = ( 0U );
  static constexpr Reg32_t APB1ENR1_TIM2EN_Msk   = ( 0x1UL << APB1ENR1_TIM2EN_Pos );
  static constexpr Reg32_t APB1ENR1_TIM2EN       = APB1ENR1_TIM2EN_Msk;
  static constexpr Reg32_t APB1ENR1_TIM6EN_Pos   = ( 4U );
  static constexpr Reg32_t APB1ENR1_TIM6EN_Msk   = ( 0x1UL << APB1ENR1_TIM6EN_Pos );
  static constexpr Reg32_t APB1ENR1_TIM6EN       = APB1ENR1_TIM6EN_Msk;
  static constexpr Reg32_t APB1ENR1_TIM7EN_Pos   = ( 5U );
  static constexpr Reg32_t APB1ENR1_TIM7EN_Msk   = ( 0x1UL << APB1ENR1_TIM7EN_Pos );
  static constexpr Reg32_t APB1ENR1_TIM7EN       = APB1ENR1_TIM7EN_Msk;
  static constexpr Reg32_t APB1ENR1_RTCAPBEN_Pos = ( 10U );
  static constexpr Reg32_t APB1ENR1_RTCAPBEN_Msk = ( 0x1UL << APB1ENR1_RTCAPBEN_Pos );
  static constexpr Reg32_t APB1ENR1_RTCAPBEN     = APB1ENR1_RTCAPBEN_Msk;
  static constexpr Reg32_t APB1ENR1_WWDGEN_Pos   = ( 11U );
  static constexpr Reg32_t APB1ENR1_WWDGEN_Msk   = ( 0x1UL << APB1ENR1_WWDGEN_Pos );
  static constexpr Reg32_t APB1ENR1_WWDGEN       = APB1ENR1_WWDGEN_Msk;
  static constexpr Reg32_t APB1ENR1_SPI3EN_Pos   = ( 15U );
  static constexpr Reg32_t APB1ENR1_SPI3EN_Msk   = ( 0x1UL << APB1ENR1_SPI3EN_Pos );
  static constexpr Reg32_t APB1ENR1_SPI3EN       = APB1ENR1_SPI3EN_Msk;
  static constexpr Reg32_t APB1ENR1_USART2EN_Pos = ( 17U );
  static constexpr Reg32_t APB1ENR1_USART2EN_Msk = ( 0x1UL << APB1ENR1_USART2EN_Pos );
  static constexpr Reg32_t APB1ENR1_USART2EN     = APB1ENR1_USART2EN_Msk;
  static constexpr Reg32_t APB1ENR1_I2C1EN_Pos   = ( 21U );
  static constexpr Reg32_t APB1ENR1_I2C1EN_Msk   = ( 0x1UL << APB1ENR1_I2C1EN_Pos );
  static constexpr Reg32_t APB1ENR1_I2C1EN       = APB1ENR1_I2C1EN_Msk;
  static constexpr Reg32_t APB1ENR1_I2C3EN_Pos   = ( 23U );
  static constexpr Reg32_t APB1ENR1_I2C3EN_Msk   = ( 0x1UL << APB1ENR1_I2C3EN_Pos );
  static constexpr Reg32_t APB1ENR1_I2C3EN       = APB1ENR1_I2C3EN_Msk;
  static constexpr Reg32_t APB1ENR1_CRSEN_Pos    = ( 24U );
  static constexpr Reg32_t APB1ENR1_CRSEN_Msk    = ( 0x1UL << APB1ENR1_CRSEN_Pos );
  static constexpr Reg32_t APB1ENR1_CRSEN        = APB1ENR1_CRSEN_Msk;
  static constexpr Reg32_t APB1ENR1_CAN1EN_Pos   = ( 25U );
  static constexpr Reg32_t APB1ENR1_CAN1EN_Msk   = ( 0x1UL << APB1ENR1_CAN1EN_Pos );
  static constexpr Reg32_t APB1ENR1_CAN1EN       = APB1ENR1_CAN1EN_Msk;
  static constexpr Reg32_t APB1ENR1_USBFSEN_Pos  = ( 26U );
  static constexpr Reg32_t APB1ENR1_USBFSEN_Msk  = ( 0x1UL << APB1ENR1_USBFSEN_Pos );
  static constexpr Reg32_t APB1ENR1_USBFSEN      = APB1ENR1_USBFSEN_Msk;
  static constexpr Reg32_t APB1ENR1_PWREN_Pos    = ( 28U );
  static constexpr Reg32_t APB1ENR1_PWREN_Msk    = ( 0x1UL << APB1ENR1_PWREN_Pos );
  static constexpr Reg32_t APB1ENR1_PWREN        = APB1ENR1_PWREN_Msk;
  static constexpr Reg32_t APB1ENR1_DAC1EN_Pos   = ( 29U );
  static constexpr Reg32_t APB1ENR1_DAC1EN_Msk   = ( 0x1UL << APB1ENR1_DAC1EN_Pos );
  static constexpr Reg32_t APB1ENR1_DAC1EN       = APB1ENR1_DAC1EN_Msk;
  static constexpr Reg32_t APB1ENR1_OPAMPEN_Pos  = ( 30U );
  static constexpr Reg32_t APB1ENR1_OPAMPEN_Msk  = ( 0x1UL << APB1ENR1_OPAMPEN_Pos );
  static constexpr Reg32_t APB1ENR1_OPAMPEN      = APB1ENR1_OPAMPEN_Msk;
  static constexpr Reg32_t APB1ENR1_LPTIM1EN_Pos = ( 31U );
  static constexpr Reg32_t APB1ENR1_LPTIM1EN_Msk = ( 0x1UL << APB1ENR1_LPTIM1EN_Pos );
  static constexpr Reg32_t APB1ENR1_LPTIM1EN     = APB1ENR1_LPTIM1EN_Msk;

  /********************  Bit definition for APB1RSTR2 register  **************/
  static constexpr Reg32_t APB1ENR2_LPUART1EN_Pos = ( 0U );
  static constexpr Reg32_t APB1ENR2_LPUART1EN_Msk = ( 0x1UL << APB1ENR2_LPUART1EN_Pos );
  static constexpr Reg32_t APB1ENR2_LPUART1EN     = APB1ENR2_LPUART1EN_Msk;
  static constexpr Reg32_t APB1ENR2_SWPMI1EN_Pos  = ( 2U );
  static constexpr Reg32_t APB1ENR2_SWPMI1EN_Msk  = ( 0x1UL << APB1ENR2_SWPMI1EN_Pos );
  static constexpr Reg32_t APB1ENR2_SWPMI1EN      = APB1ENR2_SWPMI1EN_Msk;
  static constexpr Reg32_t APB1ENR2_LPTIM2EN_Pos  = ( 5U );
  static constexpr Reg32_t APB1ENR2_LPTIM2EN_Msk  = ( 0x1UL << APB1ENR2_LPTIM2EN_Pos );
  static constexpr Reg32_t APB1ENR2_LPTIM2EN      = APB1ENR2_LPTIM2EN_Msk;

  /********************  Bit definition for APB2ENR register  ***************/
  static constexpr Reg32_t APB2ENR_SYSCFGEN_Pos = ( 0U );
  static constexpr Reg32_t APB2ENR_SYSCFGEN_Msk = ( 0x1UL << APB2ENR_SYSCFGEN_Pos );
  static constexpr Reg32_t APB2ENR_SYSCFGEN     = APB2ENR_SYSCFGEN_Msk;
  static constexpr Reg32_t APB2ENR_FWEN_Pos     = ( 7U );
  static constexpr Reg32_t APB2ENR_FWEN_Msk     = ( 0x1UL << APB2ENR_FWEN_Pos );
  static constexpr Reg32_t APB2ENR_FWEN         = APB2ENR_FWEN_Msk;
  static constexpr Reg32_t APB2ENR_TIM1EN_Pos   = ( 11U );
  static constexpr Reg32_t APB2ENR_TIM1EN_Msk   = ( 0x1UL << APB2ENR_TIM1EN_Pos );
  static constexpr Reg32_t APB2ENR_TIM1EN       = APB2ENR_TIM1EN_Msk;
  static constexpr Reg32_t APB2ENR_SPI1EN_Pos   = ( 12U );
  static constexpr Reg32_t APB2ENR_SPI1EN_Msk   = ( 0x1UL << APB2ENR_SPI1EN_Pos );
  static constexpr Reg32_t APB2ENR_SPI1EN       = APB2ENR_SPI1EN_Msk;
  static constexpr Reg32_t APB2ENR_USART1EN_Pos = ( 14U );
  static constexpr Reg32_t APB2ENR_USART1EN_Msk = ( 0x1UL << APB2ENR_USART1EN_Pos );
  static constexpr Reg32_t APB2ENR_USART1EN     = APB2ENR_USART1EN_Msk;
  static constexpr Reg32_t APB2ENR_TIM15EN_Pos  = ( 16U );
  static constexpr Reg32_t APB2ENR_TIM15EN_Msk  = ( 0x1UL << APB2ENR_TIM15EN_Pos );
  static constexpr Reg32_t APB2ENR_TIM15EN      = APB2ENR_TIM15EN_Msk;
  static constexpr Reg32_t APB2ENR_TIM16EN_Pos  = ( 17U );
  static constexpr Reg32_t APB2ENR_TIM16EN_Msk  = ( 0x1UL << APB2ENR_TIM16EN_Pos );
  static constexpr Reg32_t APB2ENR_TIM16EN      = APB2ENR_TIM16EN_Msk;
  static constexpr Reg32_t APB2ENR_SAI1EN_Pos   = ( 21U );
  static constexpr Reg32_t APB2ENR_SAI1EN_Msk   = ( 0x1UL << APB2ENR_SAI1EN_Pos );
  static constexpr Reg32_t APB2ENR_SAI1EN       = APB2ENR_SAI1EN_Msk;

  /********************  Bit definition for AHB1SMENR register  ***************/
  static constexpr Reg32_t AHB1SMENR_DMA1SMEN_Pos  = ( 0U );
  static constexpr Reg32_t AHB1SMENR_DMA1SMEN_Msk  = ( 0x1UL << AHB1SMENR_DMA1SMEN_Pos );
  static constexpr Reg32_t AHB1SMENR_DMA1SMEN      = AHB1SMENR_DMA1SMEN_Msk;
  static constexpr Reg32_t AHB1SMENR_DMA2SMEN_Pos  = ( 1U );
  static constexpr Reg32_t AHB1SMENR_DMA2SMEN_Msk  = ( 0x1UL << AHB1SMENR_DMA2SMEN_Pos );
  static constexpr Reg32_t AHB1SMENR_DMA2SMEN      = AHB1SMENR_DMA2SMEN_Msk;
  static constexpr Reg32_t AHB1SMENR_FLASHSMEN_Pos = ( 8U );
  static constexpr Reg32_t AHB1SMENR_FLASHSMEN_Msk = ( 0x1UL << AHB1SMENR_FLASHSMEN_Pos );
  static constexpr Reg32_t AHB1SMENR_FLASHSMEN     = AHB1SMENR_FLASHSMEN_Msk;
  static constexpr Reg32_t AHB1SMENR_SRAM1SMEN_Pos = ( 9U );
  static constexpr Reg32_t AHB1SMENR_SRAM1SMEN_Msk = ( 0x1UL << AHB1SMENR_SRAM1SMEN_Pos );
  static constexpr Reg32_t AHB1SMENR_SRAM1SMEN     = AHB1SMENR_SRAM1SMEN_Msk;
  static constexpr Reg32_t AHB1SMENR_CRCSMEN_Pos   = ( 12U );
  static constexpr Reg32_t AHB1SMENR_CRCSMEN_Msk   = ( 0x1UL << AHB1SMENR_CRCSMEN_Pos );
  static constexpr Reg32_t AHB1SMENR_CRCSMEN       = AHB1SMENR_CRCSMEN_Msk;
  static constexpr Reg32_t AHB1SMENR_TSCSMEN_Pos   = ( 16U );
  static constexpr Reg32_t AHB1SMENR_TSCSMEN_Msk   = ( 0x1UL << AHB1SMENR_TSCSMEN_Pos );
  static constexpr Reg32_t AHB1SMENR_TSCSMEN       = AHB1SMENR_TSCSMEN_Msk;

  /********************  Bit definition for AHB2SMENR register  *************/
  static constexpr Reg32_t AHB2SMENR_GPIOASMEN_Pos = ( 0U );
  static constexpr Reg32_t AHB2SMENR_GPIOASMEN_Msk = ( 0x1UL << AHB2SMENR_GPIOASMEN_Pos );
  static constexpr Reg32_t AHB2SMENR_GPIOASMEN     = AHB2SMENR_GPIOASMEN_Msk;
  static constexpr Reg32_t AHB2SMENR_GPIOBSMEN_Pos = ( 1U );
  static constexpr Reg32_t AHB2SMENR_GPIOBSMEN_Msk = ( 0x1UL << AHB2SMENR_GPIOBSMEN_Pos );
  static constexpr Reg32_t AHB2SMENR_GPIOBSMEN     = AHB2SMENR_GPIOBSMEN_Msk;
  static constexpr Reg32_t AHB2SMENR_GPIOCSMEN_Pos = ( 2U );
  static constexpr Reg32_t AHB2SMENR_GPIOCSMEN_Msk = ( 0x1UL << AHB2SMENR_GPIOCSMEN_Pos );
  static constexpr Reg32_t AHB2SMENR_GPIOCSMEN     = AHB2SMENR_GPIOCSMEN_Msk;
  static constexpr Reg32_t AHB2SMENR_GPIOHSMEN_Pos = ( 7U );
  static constexpr Reg32_t AHB2SMENR_GPIOHSMEN_Msk = ( 0x1UL << AHB2SMENR_GPIOHSMEN_Pos );
  static constexpr Reg32_t AHB2SMENR_GPIOHSMEN     = AHB2SMENR_GPIOHSMEN_Msk;
  static constexpr Reg32_t AHB2SMENR_SRAM2SMEN_Pos = ( 9U );
  static constexpr Reg32_t AHB2SMENR_SRAM2SMEN_Msk = ( 0x1UL << AHB2SMENR_SRAM2SMEN_Pos );
  static constexpr Reg32_t AHB2SMENR_SRAM2SMEN     = AHB2SMENR_SRAM2SMEN_Msk;
  static constexpr Reg32_t AHB2SMENR_ADCSMEN_Pos   = ( 13U );
  static constexpr Reg32_t AHB2SMENR_ADCSMEN_Msk   = ( 0x1UL << AHB2SMENR_ADCSMEN_Pos );
  static constexpr Reg32_t AHB2SMENR_ADCSMEN       = AHB2SMENR_ADCSMEN_Msk;
  static constexpr Reg32_t AHB2SMENR_RNGSMEN_Pos   = ( 18U );
  static constexpr Reg32_t AHB2SMENR_RNGSMEN_Msk   = ( 0x1UL << AHB2SMENR_RNGSMEN_Pos );
  static constexpr Reg32_t AHB2SMENR_RNGSMEN       = AHB2SMENR_RNGSMEN_Msk;

  /********************  Bit definition for AHB3SMENR register  *************/
  static constexpr Reg32_t AHB3SMENR_QSPISMEN_Pos = ( 8U );
  static constexpr Reg32_t AHB3SMENR_QSPISMEN_Msk = ( 0x1UL << AHB3SMENR_QSPISMEN_Pos );
  static constexpr Reg32_t AHB3SMENR_QSPISMEN     = AHB3SMENR_QSPISMEN_Msk;

  /********************  Bit definition for APB1SMENR1 register  *************/
  static constexpr Reg32_t APB1SMENR1_TIM2SMEN_Pos   = ( 0U );
  static constexpr Reg32_t APB1SMENR1_TIM2SMEN_Msk   = ( 0x1UL << APB1SMENR1_TIM2SMEN_Pos );
  static constexpr Reg32_t APB1SMENR1_TIM2SMEN       = APB1SMENR1_TIM2SMEN_Msk;
  static constexpr Reg32_t APB1SMENR1_TIM6SMEN_Pos   = ( 4U );
  static constexpr Reg32_t APB1SMENR1_TIM6SMEN_Msk   = ( 0x1UL << APB1SMENR1_TIM6SMEN_Pos );
  static constexpr Reg32_t APB1SMENR1_TIM6SMEN       = APB1SMENR1_TIM6SMEN_Msk;
  static constexpr Reg32_t APB1SMENR1_TIM7SMEN_Pos   = ( 5U );
  static constexpr Reg32_t APB1SMENR1_TIM7SMEN_Msk   = ( 0x1UL << APB1SMENR1_TIM7SMEN_Pos );
  static constexpr Reg32_t APB1SMENR1_TIM7SMEN       = APB1SMENR1_TIM7SMEN_Msk;
  static constexpr Reg32_t APB1SMENR1_RTCAPBSMEN_Pos = ( 10U );
  static constexpr Reg32_t APB1SMENR1_RTCAPBSMEN_Msk = ( 0x1UL << APB1SMENR1_RTCAPBSMEN_Pos );
  static constexpr Reg32_t APB1SMENR1_RTCAPBSMEN     = APB1SMENR1_RTCAPBSMEN_Msk;
  static constexpr Reg32_t APB1SMENR1_WWDGSMEN_Pos   = ( 11U );
  static constexpr Reg32_t APB1SMENR1_WWDGSMEN_Msk   = ( 0x1UL << APB1SMENR1_WWDGSMEN_Pos );
  static constexpr Reg32_t APB1SMENR1_WWDGSMEN       = APB1SMENR1_WWDGSMEN_Msk;
  static constexpr Reg32_t APB1SMENR1_SPI3SMEN_Pos   = ( 15U );
  static constexpr Reg32_t APB1SMENR1_SPI3SMEN_Msk   = ( 0x1UL << APB1SMENR1_SPI3SMEN_Pos );
  static constexpr Reg32_t APB1SMENR1_SPI3SMEN       = APB1SMENR1_SPI3SMEN_Msk;
  static constexpr Reg32_t APB1SMENR1_USART2SMEN_Pos = ( 17U );
  static constexpr Reg32_t APB1SMENR1_USART2SMEN_Msk = ( 0x1UL << APB1SMENR1_USART2SMEN_Pos );
  static constexpr Reg32_t APB1SMENR1_USART2SMEN     = APB1SMENR1_USART2SMEN_Msk;
  static constexpr Reg32_t APB1SMENR1_I2C1SMEN_Pos   = ( 21U );
  static constexpr Reg32_t APB1SMENR1_I2C1SMEN_Msk   = ( 0x1UL << APB1SMENR1_I2C1SMEN_Pos );
  static constexpr Reg32_t APB1SMENR1_I2C1SMEN       = APB1SMENR1_I2C1SMEN_Msk;
  static constexpr Reg32_t APB1SMENR1_I2C3SMEN_Pos   = ( 23U );
  static constexpr Reg32_t APB1SMENR1_I2C3SMEN_Msk   = ( 0x1UL << APB1SMENR1_I2C3SMEN_Pos );
  static constexpr Reg32_t APB1SMENR1_I2C3SMEN       = APB1SMENR1_I2C3SMEN_Msk;
  static constexpr Reg32_t APB1SMENR1_CRSSMEN_Pos    = ( 24U );
  static constexpr Reg32_t APB1SMENR1_CRSSMEN_Msk    = ( 0x1UL << APB1SMENR1_CRSSMEN_Pos );
  static constexpr Reg32_t APB1SMENR1_CRSSMEN        = APB1SMENR1_CRSSMEN_Msk;
  static constexpr Reg32_t APB1SMENR1_CAN1SMEN_Pos   = ( 25U );
  static constexpr Reg32_t APB1SMENR1_CAN1SMEN_Msk   = ( 0x1UL << APB1SMENR1_CAN1SMEN_Pos );
  static constexpr Reg32_t APB1SMENR1_CAN1SMEN       = APB1SMENR1_CAN1SMEN_Msk;
  static constexpr Reg32_t APB1SMENR1_USBFSSMEN_Pos  = ( 26U );
  static constexpr Reg32_t APB1SMENR1_USBFSSMEN_Msk  = ( 0x1UL << APB1SMENR1_USBFSSMEN_Pos );
  static constexpr Reg32_t APB1SMENR1_USBFSSMEN      = APB1SMENR1_USBFSSMEN_Msk;
  static constexpr Reg32_t APB1SMENR1_PWRSMEN_Pos    = ( 28U );
  static constexpr Reg32_t APB1SMENR1_PWRSMEN_Msk    = ( 0x1UL << APB1SMENR1_PWRSMEN_Pos );
  static constexpr Reg32_t APB1SMENR1_PWRSMEN        = APB1SMENR1_PWRSMEN_Msk;
  static constexpr Reg32_t APB1SMENR1_DAC1SMEN_Pos   = ( 29U );
  static constexpr Reg32_t APB1SMENR1_DAC1SMEN_Msk   = ( 0x1UL << APB1SMENR1_DAC1SMEN_Pos );
  static constexpr Reg32_t APB1SMENR1_DAC1SMEN       = APB1SMENR1_DAC1SMEN_Msk;
  static constexpr Reg32_t APB1SMENR1_OPAMPSMEN_Pos  = ( 30U );
  static constexpr Reg32_t APB1SMENR1_OPAMPSMEN_Msk  = ( 0x1UL << APB1SMENR1_OPAMPSMEN_Pos );
  static constexpr Reg32_t APB1SMENR1_OPAMPSMEN      = APB1SMENR1_OPAMPSMEN_Msk;
  static constexpr Reg32_t APB1SMENR1_LPTIM1SMEN_Pos = ( 31U );
  static constexpr Reg32_t APB1SMENR1_LPTIM1SMEN_Msk = ( 0x1UL << APB1SMENR1_LPTIM1SMEN_Pos );
  static constexpr Reg32_t APB1SMENR1_LPTIM1SMEN     = APB1SMENR1_LPTIM1SMEN_Msk;

  /********************  Bit definition for APB1SMENR2 register  *************/
  static constexpr Reg32_t APB1SMENR2_LPUART1SMEN_Pos = ( 0U );
  static constexpr Reg32_t APB1SMENR2_LPUART1SMEN_Msk = ( 0x1UL << APB1SMENR2_LPUART1SMEN_Pos );
  static constexpr Reg32_t APB1SMENR2_LPUART1SMEN     = APB1SMENR2_LPUART1SMEN_Msk;
  static constexpr Reg32_t APB1SMENR2_SWPMI1SMEN_Pos  = ( 2U );
  static constexpr Reg32_t APB1SMENR2_SWPMI1SMEN_Msk  = ( 0x1UL << APB1SMENR2_SWPMI1SMEN_Pos );
  static constexpr Reg32_t APB1SMENR2_SWPMI1SMEN      = APB1SMENR2_SWPMI1SMEN_Msk;
  static constexpr Reg32_t APB1SMENR2_LPTIM2SMEN_Pos  = ( 5U );
  static constexpr Reg32_t APB1SMENR2_LPTIM2SMEN_Msk  = ( 0x1UL << APB1SMENR2_LPTIM2SMEN_Pos );
  static constexpr Reg32_t APB1SMENR2_LPTIM2SMEN      = APB1SMENR2_LPTIM2SMEN_Msk;

  /********************  Bit definition for APB2SMENR register  *************/
  static constexpr Reg32_t APB2SMENR_SYSCFGSMEN_Pos = ( 0U );
  static constexpr Reg32_t APB2SMENR_SYSCFGSMEN_Msk = ( 0x1UL << APB2SMENR_SYSCFGSMEN_Pos );
  static constexpr Reg32_t APB2SMENR_SYSCFGSMEN     = APB2SMENR_SYSCFGSMEN_Msk;
  static constexpr Reg32_t APB2SMENR_TIM1SMEN_Pos   = ( 11U );
  static constexpr Reg32_t APB2SMENR_TIM1SMEN_Msk   = ( 0x1UL << APB2SMENR_TIM1SMEN_Pos );
  static constexpr Reg32_t APB2SMENR_TIM1SMEN       = APB2SMENR_TIM1SMEN_Msk;
  static constexpr Reg32_t APB2SMENR_SPI1SMEN_Pos   = ( 12U );
  static constexpr Reg32_t APB2SMENR_SPI1SMEN_Msk   = ( 0x1UL << APB2SMENR_SPI1SMEN_Pos );
  static constexpr Reg32_t APB2SMENR_SPI1SMEN       = APB2SMENR_SPI1SMEN_Msk;
  static constexpr Reg32_t APB2SMENR_USART1SMEN_Pos = ( 14U );
  static constexpr Reg32_t APB2SMENR_USART1SMEN_Msk = ( 0x1UL << APB2SMENR_USART1SMEN_Pos );
  static constexpr Reg32_t APB2SMENR_USART1SMEN     = APB2SMENR_USART1SMEN_Msk;
  static constexpr Reg32_t APB2SMENR_TIM15SMEN_Pos  = ( 16U );
  static constexpr Reg32_t APB2SMENR_TIM15SMEN_Msk  = ( 0x1UL << APB2SMENR_TIM15SMEN_Pos );
  static constexpr Reg32_t APB2SMENR_TIM15SMEN      = APB2SMENR_TIM15SMEN_Msk;
  static constexpr Reg32_t APB2SMENR_TIM16SMEN_Pos  = ( 17U );
  static constexpr Reg32_t APB2SMENR_TIM16SMEN_Msk  = ( 0x1UL << APB2SMENR_TIM16SMEN_Pos );
  static constexpr Reg32_t APB2SMENR_TIM16SMEN      = APB2SMENR_TIM16SMEN_Msk;
  static constexpr Reg32_t APB2SMENR_SAI1SMEN_Pos   = ( 21U );
  static constexpr Reg32_t APB2SMENR_SAI1SMEN_Msk   = ( 0x1UL << APB2SMENR_SAI1SMEN_Pos );
  static constexpr Reg32_t APB2SMENR_SAI1SMEN       = APB2SMENR_SAI1SMEN_Msk;

  /********************  Bit definition for CCIPR register  ******************/
  static constexpr Reg32_t CCIPR_USART1SEL_Pos = ( 0U );
  static constexpr Reg32_t CCIPR_USART1SEL_Msk = ( 0x3UL << CCIPR_USART1SEL_Pos );
  static constexpr Reg32_t CCIPR_USART1SEL     = CCIPR_USART1SEL_Msk;
  static constexpr Reg32_t CCIPR_USART1SEL_0   = ( 0x1UL << CCIPR_USART1SEL_Pos );
  static constexpr Reg32_t CCIPR_USART1SEL_1   = ( 0x2UL << CCIPR_USART1SEL_Pos );

  static constexpr Reg32_t CCIPR_USART2SEL_Pos = ( 2U );
  static constexpr Reg32_t CCIPR_USART2SEL_Msk = ( 0x3UL << CCIPR_USART2SEL_Pos );
  static constexpr Reg32_t CCIPR_USART2SEL     = CCIPR_USART2SEL_Msk;
  static constexpr Reg32_t CCIPR_USART2SEL_0   = ( 0x1UL << CCIPR_USART2SEL_Pos );
  static constexpr Reg32_t CCIPR_USART2SEL_1   = ( 0x2UL << CCIPR_USART2SEL_Pos );

  static constexpr Reg32_t CCIPR_LPUART1SEL_Pos = ( 10U );
  static constexpr Reg32_t CCIPR_LPUART1SEL_Msk = ( 0x3UL << CCIPR_LPUART1SEL_Pos );
  static constexpr Reg32_t CCIPR_LPUART1SEL     = CCIPR_LPUART1SEL_Msk;
  static constexpr Reg32_t CCIPR_LPUART1SEL_0   = ( 0x1UL << CCIPR_LPUART1SEL_Pos );
  static constexpr Reg32_t CCIPR_LPUART1SEL_1   = ( 0x2UL << CCIPR_LPUART1SEL_Pos );

  static constexpr Reg32_t CCIPR_I2C1SEL_Pos = ( 12U );
  static constexpr Reg32_t CCIPR_I2C1SEL_Msk = ( 0x3UL << CCIPR_I2C1SEL_Pos );
  static constexpr Reg32_t CCIPR_I2C1SEL     = CCIPR_I2C1SEL_Msk;
  static constexpr Reg32_t CCIPR_I2C1SEL_0   = ( 0x1UL << CCIPR_I2C1SEL_Pos );
  static constexpr Reg32_t CCIPR_I2C1SEL_1   = ( 0x2UL << CCIPR_I2C1SEL_Pos );

  static constexpr Reg32_t CCIPR_I2C3SEL_Pos = ( 16U );
  static constexpr Reg32_t CCIPR_I2C3SEL_Msk = ( 0x3UL << CCIPR_I2C3SEL_Pos );
  static constexpr Reg32_t CCIPR_I2C3SEL     = CCIPR_I2C3SEL_Msk;
  static constexpr Reg32_t CCIPR_I2C3SEL_0   = ( 0x1UL << CCIPR_I2C3SEL_Pos );
  static constexpr Reg32_t CCIPR_I2C3SEL_1   = ( 0x2UL << CCIPR_I2C3SEL_Pos );

  static constexpr Reg32_t CCIPR_LPTIM1SEL_Pos = ( 18U );
  static constexpr Reg32_t CCIPR_LPTIM1SEL_Msk = ( 0x3UL << CCIPR_LPTIM1SEL_Pos );
  static constexpr Reg32_t CCIPR_LPTIM1SEL     = CCIPR_LPTIM1SEL_Msk;
  static constexpr Reg32_t CCIPR_LPTIM1SEL_0   = ( 0x1UL << CCIPR_LPTIM1SEL_Pos );
  static constexpr Reg32_t CCIPR_LPTIM1SEL_1   = ( 0x2UL << CCIPR_LPTIM1SEL_Pos );

  static constexpr Reg32_t CCIPR_LPTIM2SEL_Pos = ( 20U );
  static constexpr Reg32_t CCIPR_LPTIM2SEL_Msk = ( 0x3UL << CCIPR_LPTIM2SEL_Pos );
  static constexpr Reg32_t CCIPR_LPTIM2SEL     = CCIPR_LPTIM2SEL_Msk;
  static constexpr Reg32_t CCIPR_LPTIM2SEL_0   = ( 0x1UL << CCIPR_LPTIM2SEL_Pos );
  static constexpr Reg32_t CCIPR_LPTIM2SEL_1   = ( 0x2UL << CCIPR_LPTIM2SEL_Pos );

  static constexpr Reg32_t CCIPR_SAI1SEL_Pos = ( 22U );
  static constexpr Reg32_t CCIPR_SAI1SEL_Msk = ( 0x3UL << CCIPR_SAI1SEL_Pos );
  static constexpr Reg32_t CCIPR_SAI1SEL     = CCIPR_SAI1SEL_Msk;
  static constexpr Reg32_t CCIPR_SAI1SEL_0   = ( 0x1UL << CCIPR_SAI1SEL_Pos );
  static constexpr Reg32_t CCIPR_SAI1SEL_1   = ( 0x2UL << CCIPR_SAI1SEL_Pos );

  static constexpr Reg32_t CCIPR_CLK48SEL_Pos = ( 26U );
  static constexpr Reg32_t CCIPR_CLK48SEL_Msk = ( 0x3UL << CCIPR_CLK48SEL_Pos );
  static constexpr Reg32_t CCIPR_CLK48SEL     = CCIPR_CLK48SEL_Msk;
  static constexpr Reg32_t CCIPR_CLK48SEL_0   = ( 0x1UL << CCIPR_CLK48SEL_Pos );
  static constexpr Reg32_t CCIPR_CLK48SEL_1   = ( 0x2UL << CCIPR_CLK48SEL_Pos );

  static constexpr Reg32_t CCIPR_ADCSEL_Pos = ( 28U );
  static constexpr Reg32_t CCIPR_ADCSEL_Msk = ( 0x3UL << CCIPR_ADCSEL_Pos );
  static constexpr Reg32_t CCIPR_ADCSEL     = CCIPR_ADCSEL_Msk;
  static constexpr Reg32_t CCIPR_ADCSEL_0   = ( 0x1UL << CCIPR_ADCSEL_Pos );
  static constexpr Reg32_t CCIPR_ADCSEL_1   = ( 0x2UL << CCIPR_ADCSEL_Pos );

  static constexpr Reg32_t CCIPR_SWPMI1SEL_Pos = ( 30U );
  static constexpr Reg32_t CCIPR_SWPMI1SEL_Msk = ( 0x1UL << CCIPR_SWPMI1SEL_Pos );
  static constexpr Reg32_t CCIPR_SWPMI1SEL     = CCIPR_SWPMI1SEL_Msk;

  /********************  Bit definition for BDCR register  ******************/
  static constexpr Reg32_t BDCR_LSEON_Pos  = ( 0U );
  static constexpr Reg32_t BDCR_LSEON_Msk  = ( 0x1UL << BDCR_LSEON_Pos );
  static constexpr Reg32_t BDCR_LSEON      = BDCR_LSEON_Msk;
  static constexpr Reg32_t BDCR_LSERDY_Pos = ( 1U );
  static constexpr Reg32_t BDCR_LSERDY_Msk = ( 0x1UL << BDCR_LSERDY_Pos );
  static constexpr Reg32_t BDCR_LSERDY     = BDCR_LSERDY_Msk;
  static constexpr Reg32_t BDCR_LSEBYP_Pos = ( 2U );
  static constexpr Reg32_t BDCR_LSEBYP_Msk = ( 0x1UL << BDCR_LSEBYP_Pos );
  static constexpr Reg32_t BDCR_LSEBYP     = BDCR_LSEBYP_Msk;

  static constexpr Reg32_t BDCR_LSEDRV_Pos = ( 3U );
  static constexpr Reg32_t BDCR_LSEDRV_Msk = ( 0x3UL << BDCR_LSEDRV_Pos );
  static constexpr Reg32_t BDCR_LSEDRV     = BDCR_LSEDRV_Msk;
  static constexpr Reg32_t BDCR_LSEDRV_0   = ( 0x1UL << BDCR_LSEDRV_Pos );
  static constexpr Reg32_t BDCR_LSEDRV_1   = ( 0x2UL << BDCR_LSEDRV_Pos );

  static constexpr Reg32_t BDCR_LSECSSON_Pos = ( 5U );
  static constexpr Reg32_t BDCR_LSECSSON_Msk = ( 0x1UL << BDCR_LSECSSON_Pos );
  static constexpr Reg32_t BDCR_LSECSSON     = BDCR_LSECSSON_Msk;
  static constexpr Reg32_t BDCR_LSECSSD_Pos  = ( 6U );
  static constexpr Reg32_t BDCR_LSECSSD_Msk  = ( 0x1UL << BDCR_LSECSSD_Pos );
  static constexpr Reg32_t BDCR_LSECSSD      = BDCR_LSECSSD_Msk;

  static constexpr Reg32_t BDCR_RTCSEL_Pos = ( 8U );
  static constexpr Reg32_t BDCR_RTCSEL_Msk = ( 0x3UL << BDCR_RTCSEL_Pos );
  static constexpr Reg32_t BDCR_RTCSEL     = BDCR_RTCSEL_Msk;
  static constexpr Reg32_t BDCR_RTCSEL_0   = ( 0x1UL << BDCR_RTCSEL_Pos );
  static constexpr Reg32_t BDCR_RTCSEL_1   = ( 0x2UL << BDCR_RTCSEL_Pos );

  static constexpr Reg32_t BDCR_RTCEN_Pos   = ( 15U );
  static constexpr Reg32_t BDCR_RTCEN_Msk   = ( 0x1UL << BDCR_RTCEN_Pos );
  static constexpr Reg32_t BDCR_RTCEN       = BDCR_RTCEN_Msk;
  static constexpr Reg32_t BDCR_BDRST_Pos   = ( 16U );
  static constexpr Reg32_t BDCR_BDRST_Msk   = ( 0x1UL << BDCR_BDRST_Pos );
  static constexpr Reg32_t BDCR_BDRST       = BDCR_BDRST_Msk;
  static constexpr Reg32_t BDCR_LSCOEN_Pos  = ( 24U );
  static constexpr Reg32_t BDCR_LSCOEN_Msk  = ( 0x1UL << BDCR_LSCOEN_Pos );
  static constexpr Reg32_t BDCR_LSCOEN      = BDCR_LSCOEN_Msk;
  static constexpr Reg32_t BDCR_LSCOSEL_Pos = ( 25U );
  static constexpr Reg32_t BDCR_LSCOSEL_Msk = ( 0x1UL << BDCR_LSCOSEL_Pos );
  static constexpr Reg32_t BDCR_LSCOSEL     = BDCR_LSCOSEL_Msk;

  /********************  Bit definition for CSR register  *******************/
  static constexpr Reg32_t CSR_Msk = 0xFF800F13;
  static constexpr Reg32_t CSR_Rst = 0x0C000600;

  static constexpr Reg32_t CSR_ResetFlags_Msk = 0xFF000000;

  static constexpr Reg32_t CSR_LSION_Pos  = ( 0U );
  static constexpr Reg32_t CSR_LSION_Msk  = ( 0x1UL << CSR_LSION_Pos );
  static constexpr Reg32_t CSR_LSION      = CSR_LSION_Msk;
  static constexpr Reg32_t CSR_LSIRDY_Pos = ( 1U );
  static constexpr Reg32_t CSR_LSIRDY_Msk = ( 0x1UL << CSR_LSIRDY_Pos );
  static constexpr Reg32_t CSR_LSIRDY     = CSR_LSIRDY_Msk;

  static constexpr Reg32_t CSR_MSISRANGE_Pos = ( 8U );
  static constexpr Reg32_t CSR_MSISRANGE_Msk = ( 0xFUL << CSR_MSISRANGE_Pos );
  static constexpr Reg32_t CSR_MSISRANGE     = CSR_MSISRANGE_Msk;
  static constexpr Reg32_t CSR_MSISRANGE_1   = ( 0x4UL << CSR_MSISRANGE_Pos );
  static constexpr Reg32_t CSR_MSISRANGE_2   = ( 0x5UL << CSR_MSISRANGE_Pos );
  static constexpr Reg32_t CSR_MSISRANGE_4   = ( 0x6UL << CSR_MSISRANGE_Pos );
  static constexpr Reg32_t CSR_MSISRANGE_8   = ( 0x7UL << CSR_MSISRANGE_Pos );

  static constexpr Reg32_t CSR_RMVF_Pos     = ( 23U );
  static constexpr Reg32_t CSR_RMVF_Msk     = ( 0x1UL << CSR_RMVF_Pos );
  static constexpr Reg32_t CSR_RMVF         = CSR_RMVF_Msk;
  static constexpr Reg32_t CSR_FWRSTF_Pos   = ( 24U );
  static constexpr Reg32_t CSR_FWRSTF_Msk   = ( 0x1UL << CSR_FWRSTF_Pos );
  static constexpr Reg32_t CSR_FWRSTF       = CSR_FWRSTF_Msk;
  static constexpr Reg32_t CSR_OBLRSTF_Pos  = ( 25U );
  static constexpr Reg32_t CSR_OBLRSTF_Msk  = ( 0x1UL << CSR_OBLRSTF_Pos );
  static constexpr Reg32_t CSR_OBLRSTF      = CSR_OBLRSTF_Msk;
  static constexpr Reg32_t CSR_PINRSTF_Pos  = ( 26U );
  static constexpr Reg32_t CSR_PINRSTF_Msk  = ( 0x1UL << CSR_PINRSTF_Pos );
  static constexpr Reg32_t CSR_PINRSTF      = CSR_PINRSTF_Msk;
  static constexpr Reg32_t CSR_BORRSTF_Pos  = ( 27U );
  static constexpr Reg32_t CSR_BORRSTF_Msk  = ( 0x1UL << CSR_BORRSTF_Pos );
  static constexpr Reg32_t CSR_BORRSTF      = CSR_BORRSTF_Msk;
  static constexpr Reg32_t CSR_SFTRSTF_Pos  = ( 28U );
  static constexpr Reg32_t CSR_SFTRSTF_Msk  = ( 0x1UL << CSR_SFTRSTF_Pos );
  static constexpr Reg32_t CSR_SFTRSTF      = CSR_SFTRSTF_Msk;
  static constexpr Reg32_t CSR_IWDGRSTF_Pos = ( 29U );
  static constexpr Reg32_t CSR_IWDGRSTF_Msk = ( 0x1UL << CSR_IWDGRSTF_Pos );
  static constexpr Reg32_t CSR_IWDGRSTF     = CSR_IWDGRSTF_Msk;
  static constexpr Reg32_t CSR_WWDGRSTF_Pos = ( 30U );
  static constexpr Reg32_t CSR_WWDGRSTF_Msk = ( 0x1UL << CSR_WWDGRSTF_Pos );
  static constexpr Reg32_t CSR_WWDGRSTF     = CSR_WWDGRSTF_Msk;
  static constexpr Reg32_t CSR_LPWRRSTF_Pos = ( 31U );
  static constexpr Reg32_t CSR_LPWRRSTF_Msk = ( 0x1UL << CSR_LPWRRSTF_Pos );
  static constexpr Reg32_t CSR_LPWRRSTF     = CSR_LPWRRSTF_Msk;

  /********************  Bit definition for CRRCR register  *****************/
  static constexpr Reg32_t CRRCR_HSI48ON_Pos  = ( 0U );
  static constexpr Reg32_t CRRCR_HSI48ON_Msk  = ( 0x1UL << CRRCR_HSI48ON_Pos );
  static constexpr Reg32_t CRRCR_HSI48ON      = CRRCR_HSI48ON_Msk;
  static constexpr Reg32_t CRRCR_HSI48RDY_Pos = ( 1U );
  static constexpr Reg32_t CRRCR_HSI48RDY_Msk = ( 0x1UL << CRRCR_HSI48RDY_Pos );
  static constexpr Reg32_t CRRCR_HSI48RDY     = CRRCR_HSI48RDY_Msk;


  static constexpr Reg32_t CRRCR_HSI48CAL_Pos = ( 7U );
  static constexpr Reg32_t CRRCR_HSI48CAL_Msk = ( 0x1FFUL << CRRCR_HSI48CAL_Pos );
  static constexpr Reg32_t CRRCR_HSI48CAL     = CRRCR_HSI48CAL_Msk;
  static constexpr Reg32_t CRRCR_HSI48CAL_0   = ( 0x001UL << CRRCR_HSI48CAL_Pos );
  static constexpr Reg32_t CRRCR_HSI48CAL_1   = ( 0x002UL << CRRCR_HSI48CAL_Pos );
  static constexpr Reg32_t CRRCR_HSI48CAL_2   = ( 0x004UL << CRRCR_HSI48CAL_Pos );
  static constexpr Reg32_t CRRCR_HSI48CAL_3   = ( 0x008UL << CRRCR_HSI48CAL_Pos );
  static constexpr Reg32_t CRRCR_HSI48CAL_4   = ( 0x010UL << CRRCR_HSI48CAL_Pos );
  static constexpr Reg32_t CRRCR_HSI48CAL_5   = ( 0x020UL << CRRCR_HSI48CAL_Pos );
  static constexpr Reg32_t CRRCR_HSI48CAL_6   = ( 0x040UL << CRRCR_HSI48CAL_Pos );
  static constexpr Reg32_t CRRCR_HSI48CAL_7   = ( 0x080UL << CRRCR_HSI48CAL_Pos );
  static constexpr Reg32_t CRRCR_HSI48CAL_8   = ( 0x100UL << CRRCR_HSI48CAL_Pos );

}    // namespace Thor::LLD::RCC

#endif /* !THOR_LLD_RCC_REGISTER_HPP */
