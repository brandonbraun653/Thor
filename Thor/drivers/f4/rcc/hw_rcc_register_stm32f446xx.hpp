/********************************************************************************
 *   File Name:
 *    hw_rcc_register_stm32f446xx.hpp
 *
 *   Description:
 *    RCC register definitions for the STM32F446xx series chips.
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_RCC_REGISTER_HPP
#define THOR_HW_RCC_REGISTER_HPP

/* C++ Includes */
#include <cstdint>

/* Driver Includes */
#include <Thor/drivers/f4/system/sys_memory_map_stm32f446xx.hpp>

/* ARM Includes */
#include <Thor/drivers/common/cmsis/core/include/core_cm4.h>

namespace Thor::Driver::RCC
{
  static constexpr uint32_t RCC_BASE_ADDR = Thor::System::MemoryMap::AHB1PERIPH_BASE_ADDR + 0x3800U;

  /*------------------------------------------------
  CR Register
  ------------------------------------------------*/
  static constexpr uint32_t CR_HSION_Pos     = ( 0U );
  static constexpr uint32_t CR_HSION_Msk     = ( 0x1U << CR_HSION_Pos );
  static constexpr uint32_t CR_HSION         = CR_HSION_Msk;
  static constexpr uint32_t CR_HSIRDY_Pos    = ( 1U );
  static constexpr uint32_t CR_HSIRDY_Msk    = ( 0x1U << CR_HSIRDY_Pos );
  static constexpr uint32_t CR_HSIRDY        = CR_HSIRDY_Msk;
  static constexpr uint32_t CR_HSITRIM_Pos   = ( 3U );
  static constexpr uint32_t CR_HSITRIM_Msk   = ( 0x1FU << CR_HSITRIM_Pos );
  static constexpr uint32_t CR_HSITRIM       = CR_HSITRIM_Msk;
  static constexpr uint32_t CR_HSITRIM_0     = ( 0x01U << CR_HSITRIM_Pos );
  static constexpr uint32_t CR_HSITRIM_1     = ( 0x02U << CR_HSITRIM_Pos );
  static constexpr uint32_t CR_HSITRIM_2     = ( 0x04U << CR_HSITRIM_Pos );
  static constexpr uint32_t CR_HSITRIM_3     = ( 0x08U << CR_HSITRIM_Pos );
  static constexpr uint32_t CR_HSITRIM_4     = ( 0x10U << CR_HSITRIM_Pos );
  static constexpr uint32_t CR_HSICAL_Pos    = ( 8U );
  static constexpr uint32_t CR_HSICAL_Msk    = ( 0xFFU << CR_HSICAL_Pos );
  static constexpr uint32_t CR_HSICAL        = CR_HSICAL_Msk;
  static constexpr uint32_t CR_HSICAL_0      = ( 0x01U << CR_HSICAL_Pos );
  static constexpr uint32_t CR_HSICAL_1      = ( 0x02U << CR_HSICAL_Pos );
  static constexpr uint32_t CR_HSICAL_2      = ( 0x04U << CR_HSICAL_Pos );
  static constexpr uint32_t CR_HSICAL_3      = ( 0x08U << CR_HSICAL_Pos );
  static constexpr uint32_t CR_HSICAL_4      = ( 0x10U << CR_HSICAL_Pos );
  static constexpr uint32_t CR_HSICAL_5      = ( 0x20U << CR_HSICAL_Pos );
  static constexpr uint32_t CR_HSICAL_6      = ( 0x40U << CR_HSICAL_Pos );
  static constexpr uint32_t CR_HSICAL_7      = ( 0x80U << CR_HSICAL_Pos );
  static constexpr uint32_t CR_HSEON_Pos     = ( 16U );
  static constexpr uint32_t CR_HSEON_Msk     = ( 0x1U << CR_HSEON_Pos );
  static constexpr uint32_t CR_HSEON         = CR_HSEON_Msk;
  static constexpr uint32_t CR_HSERDY_Pos    = ( 17U );
  static constexpr uint32_t CR_HSERDY_Msk    = ( 0x1U << CR_HSERDY_Pos );
  static constexpr uint32_t CR_HSERDY        = CR_HSERDY_Msk;
  static constexpr uint32_t CR_HSEBYP_Pos    = ( 18U );
  static constexpr uint32_t CR_HSEBYP_Msk    = ( 0x1U << CR_HSEBYP_Pos );
  static constexpr uint32_t CR_HSEBYP        = CR_HSEBYP_Msk;
  static constexpr uint32_t CR_CSSON_Pos     = ( 19U );
  static constexpr uint32_t CR_CSSON_Msk     = ( 0x1U << CR_CSSON_Pos );
  static constexpr uint32_t CR_CSSON         = CR_CSSON_Msk;
  static constexpr uint32_t CR_PLLON_Pos     = ( 24U );
  static constexpr uint32_t CR_PLLON_Msk     = ( 0x1U << CR_PLLON_Pos );
  static constexpr uint32_t CR_PLLON         = CR_PLLON_Msk;
  static constexpr uint32_t CR_PLLRDY_Pos    = ( 25U );
  static constexpr uint32_t CR_PLLRDY_Msk    = ( 0x1U << CR_PLLRDY_Pos );
  static constexpr uint32_t CR_PLLRDY        = CR_PLLRDY_Msk;
  static constexpr uint32_t CR_PLLI2SON_Pos  = ( 26U );
  static constexpr uint32_t CR_PLLI2SON_Msk  = ( 0x1U << CR_PLLI2SON_Pos );
  static constexpr uint32_t CR_PLLI2SON      = CR_PLLI2SON_Msk;
  static constexpr uint32_t CR_PLLI2SRDY_Pos = ( 27U );
  static constexpr uint32_t CR_PLLI2SRDY_Msk = ( 0x1U << CR_PLLI2SRDY_Pos );
  static constexpr uint32_t CR_PLLI2SRDY     = CR_PLLI2SRDY_Msk;
  static constexpr uint32_t CR_PLLSAION_Pos  = ( 28U );
  static constexpr uint32_t CR_PLLSAION_Msk  = ( 0x1U << CR_PLLSAION_Pos );
  static constexpr uint32_t CR_PLLSAION      = CR_PLLSAION_Msk;
  static constexpr uint32_t CR_PLLSAIRDY_Pos = ( 29U );
  static constexpr uint32_t CR_PLLSAIRDY_Msk = ( 0x1U << CR_PLLSAIRDY_Pos );
  static constexpr uint32_t CR_PLLSAIRDY     = CR_PLLSAIRDY_Msk;

  /*------------------------------------------------
  PLLCFGR Register
  ------------------------------------------------*/
  static constexpr uint32_t PLLCFGR_PLLM_Pos       = ( 0U );
  static constexpr uint32_t PLLCFGR_PLLM_Msk       = ( 0x3FU << PLLCFGR_PLLM_Pos );
  static constexpr uint32_t PLLCFGR_PLLM           = PLLCFGR_PLLM_Msk;
  static constexpr uint32_t PLLCFGR_PLLM_0         = ( 0x01U << PLLCFGR_PLLM_Pos );
  static constexpr uint32_t PLLCFGR_PLLM_1         = ( 0x02U << PLLCFGR_PLLM_Pos );
  static constexpr uint32_t PLLCFGR_PLLM_2         = ( 0x04U << PLLCFGR_PLLM_Pos );
  static constexpr uint32_t PLLCFGR_PLLM_3         = ( 0x08U << PLLCFGR_PLLM_Pos );
  static constexpr uint32_t PLLCFGR_PLLM_4         = ( 0x10U << PLLCFGR_PLLM_Pos );
  static constexpr uint32_t PLLCFGR_PLLM_5         = ( 0x20U << PLLCFGR_PLLM_Pos );
  static constexpr uint32_t PLLCFGR_PLLN_Pos       = ( 6U );
  static constexpr uint32_t PLLCFGR_PLLN_Msk       = ( 0x1FFU << PLLCFGR_PLLN_Pos );
  static constexpr uint32_t PLLCFGR_PLLN           = PLLCFGR_PLLN_Msk;
  static constexpr uint32_t PLLCFGR_PLLN_0         = ( 0x001U << PLLCFGR_PLLN_Pos );
  static constexpr uint32_t PLLCFGR_PLLN_1         = ( 0x002U << PLLCFGR_PLLN_Pos );
  static constexpr uint32_t PLLCFGR_PLLN_2         = ( 0x004U << PLLCFGR_PLLN_Pos );
  static constexpr uint32_t PLLCFGR_PLLN_3         = ( 0x008U << PLLCFGR_PLLN_Pos );
  static constexpr uint32_t PLLCFGR_PLLN_4         = ( 0x010U << PLLCFGR_PLLN_Pos );
  static constexpr uint32_t PLLCFGR_PLLN_5         = ( 0x020U << PLLCFGR_PLLN_Pos );
  static constexpr uint32_t PLLCFGR_PLLN_6         = ( 0x040U << PLLCFGR_PLLN_Pos );
  static constexpr uint32_t PLLCFGR_PLLN_7         = ( 0x080U << PLLCFGR_PLLN_Pos );
  static constexpr uint32_t PLLCFGR_PLLN_8         = ( 0x100U << PLLCFGR_PLLN_Pos );
  static constexpr uint32_t PLLCFGR_PLLP_Pos       = ( 16U );
  static constexpr uint32_t PLLCFGR_PLLP_Msk       = ( 0x3U << PLLCFGR_PLLP_Pos );
  static constexpr uint32_t PLLCFGR_PLLP           = PLLCFGR_PLLP_Msk;
  static constexpr uint32_t PLLCFGR_PLLP_0         = ( 0x1U << PLLCFGR_PLLP_Pos );
  static constexpr uint32_t PLLCFGR_PLLP_1         = ( 0x2U << PLLCFGR_PLLP_Pos );
  static constexpr uint32_t PLLCFGR_PLLSRC_Pos     = ( 22U );
  static constexpr uint32_t PLLCFGR_PLLSRC_Msk     = ( 0x1U << PLLCFGR_PLLSRC_Pos );
  static constexpr uint32_t PLLCFGR_PLLSRC         = PLLCFGR_PLLSRC_Msk;
  static constexpr uint32_t PLLCFGR_PLLSRC_HSE_Pos = ( 22U );
  static constexpr uint32_t PLLCFGR_PLLSRC_HSE_Msk = ( 0x1U << PLLCFGR_PLLSRC_HSE_Pos );
  static constexpr uint32_t PLLCFGR_PLLSRC_HSE     = PLLCFGR_PLLSRC_HSE_Msk;
  static constexpr uint32_t PLLCFGR_PLLSRC_HSI     = 0x00000000U;
  static constexpr uint32_t PLLCFGR_PLLQ_Pos       = ( 24U );
  static constexpr uint32_t PLLCFGR_PLLQ_Msk       = ( 0xFU << PLLCFGR_PLLQ_Pos );
  static constexpr uint32_t PLLCFGR_PLLQ           = PLLCFGR_PLLQ_Msk;
  static constexpr uint32_t PLLCFGR_PLLQ_0         = ( 0x1U << PLLCFGR_PLLQ_Pos );
  static constexpr uint32_t PLLCFGR_PLLQ_1         = ( 0x2U << PLLCFGR_PLLQ_Pos );
  static constexpr uint32_t PLLCFGR_PLLQ_2         = ( 0x4U << PLLCFGR_PLLQ_Pos );
  static constexpr uint32_t PLLCFGR_PLLQ_3         = ( 0x8U << PLLCFGR_PLLQ_Pos );
  static constexpr uint32_t PLLCFGR_PLLR_Pos       = ( 28U );
  static constexpr uint32_t PLLCFGR_PLLR_Msk       = ( 0x7U << PLLCFGR_PLLR_Pos );
  static constexpr uint32_t PLLCFGR_PLLR           = PLLCFGR_PLLR_Msk;
  static constexpr uint32_t PLLCFGR_PLLR_0         = ( 0x1U << PLLCFGR_PLLR_Pos );
  static constexpr uint32_t PLLCFGR_PLLR_1         = ( 0x2U << PLLCFGR_PLLR_Pos );
  static constexpr uint32_t PLLCFGR_PLLR_2         = ( 0x4U << PLLCFGR_PLLR_Pos );

  /*------------------------------------------------
  CFGR Register
  ------------------------------------------------*/
  static constexpr uint32_t CFGR_SW_Pos      = ( 0U );
  static constexpr uint32_t CFGR_SW_Msk      = ( 0x3U << CFGR_SW_Pos );
  static constexpr uint32_t CFGR_SW          = CFGR_SW_Msk;
  static constexpr uint32_t CFGR_SW_0        = ( 0x1U << CFGR_SW_Pos );
  static constexpr uint32_t CFGR_SW_1        = ( 0x2U << CFGR_SW_Pos );
  static constexpr uint32_t CFGR_SW_HSI      = 0x00000000U;
  static constexpr uint32_t CFGR_SW_HSE      = 0x00000001U;
  static constexpr uint32_t CFGR_SW_PLL      = 0x00000002U;
  static constexpr uint32_t CFGR_SW_PLLR     = 0x00000003U;
  static constexpr uint32_t CFGR_SWS_Pos     = ( 2U );
  static constexpr uint32_t CFGR_SWS_Msk     = ( 0x3U << CFGR_SWS_Pos );
  static constexpr uint32_t CFGR_SWS         = CFGR_SWS_Msk;
  static constexpr uint32_t CFGR_SWS_0       = ( 0x1U << CFGR_SWS_Pos );
  static constexpr uint32_t CFGR_SWS_1       = ( 0x2U << CFGR_SWS_Pos );
  static constexpr uint32_t CFGR_SWS_HSI     = 0x00000000U;
  static constexpr uint32_t CFGR_SWS_HSE     = 0x00000004U;
  static constexpr uint32_t CFGR_SWS_PLL     = 0x00000008U;
  static constexpr uint32_t CFGR_SWS_PLLR    = 0x0000000CU;
  static constexpr uint32_t CFGR_HPRE_Pos    = ( 4U );
  static constexpr uint32_t CFGR_HPRE_Msk    = ( 0xFU << CFGR_HPRE_Pos );
  static constexpr uint32_t CFGR_HPRE        = CFGR_HPRE_Msk;
  static constexpr uint32_t CFGR_HPRE_0      = ( 0x1U << CFGR_HPRE_Pos );
  static constexpr uint32_t CFGR_HPRE_1      = ( 0x2U << CFGR_HPRE_Pos );
  static constexpr uint32_t CFGR_HPRE_2      = ( 0x4U << CFGR_HPRE_Pos );
  static constexpr uint32_t CFGR_HPRE_3      = ( 0x8U << CFGR_HPRE_Pos );
  static constexpr uint32_t CFGR_HPRE_DIV1   = 0x00000000U;
  static constexpr uint32_t CFGR_HPRE_DIV2   = 0x00000080U;
  static constexpr uint32_t CFGR_HPRE_DIV4   = 0x00000090U;
  static constexpr uint32_t CFGR_HPRE_DIV8   = 0x000000A0U;
  static constexpr uint32_t CFGR_HPRE_DIV16  = 0x000000B0U;
  static constexpr uint32_t CFGR_HPRE_DIV64  = 0x000000C0U;
  static constexpr uint32_t CFGR_HPRE_DIV128 = 0x000000D0U;
  static constexpr uint32_t CFGR_HPRE_DIV256 = 0x000000E0U;
  static constexpr uint32_t CFGR_HPRE_DIV512 = 0x000000F0U;
  static constexpr uint32_t CFGR_PPRE1_Pos   = ( 10U );
  static constexpr uint32_t CFGR_PPRE1_Msk   = ( 0x7U << CFGR_PPRE1_Pos );
  static constexpr uint32_t CFGR_PPRE1       = CFGR_PPRE1_Msk;
  static constexpr uint32_t CFGR_PPRE1_0     = ( 0x1U << CFGR_PPRE1_Pos );
  static constexpr uint32_t CFGR_PPRE1_1     = ( 0x2U << CFGR_PPRE1_Pos );
  static constexpr uint32_t CFGR_PPRE1_2     = ( 0x4U << CFGR_PPRE1_Pos );
  static constexpr uint32_t CFGR_PPRE1_DIV1  = 0x00000000U;
  static constexpr uint32_t CFGR_PPRE1_DIV2  = 0x00001000U;
  static constexpr uint32_t CFGR_PPRE1_DIV4  = 0x00001400U;
  static constexpr uint32_t CFGR_PPRE1_DIV8  = 0x00001800U;
  static constexpr uint32_t CFGR_PPRE1_DIV16 = 0x00001C00U;
  static constexpr uint32_t CFGR_PPRE2_Pos   = ( 13U );
  static constexpr uint32_t CFGR_PPRE2_Msk   = ( 0x7U << CFGR_PPRE2_Pos );
  static constexpr uint32_t CFGR_PPRE2       = CFGR_PPRE2_Msk;
  static constexpr uint32_t CFGR_PPRE2_0     = ( 0x1U << CFGR_PPRE2_Pos );
  static constexpr uint32_t CFGR_PPRE2_1     = ( 0x2U << CFGR_PPRE2_Pos );
  static constexpr uint32_t CFGR_PPRE2_2     = ( 0x4U << CFGR_PPRE2_Pos );
  static constexpr uint32_t CFGR_PPRE2_DIV1  = 0x00000000U;
  static constexpr uint32_t CFGR_PPRE2_DIV2  = 0x00008000U;
  static constexpr uint32_t CFGR_PPRE2_DIV4  = 0x0000A000U;
  static constexpr uint32_t CFGR_PPRE2_DIV8  = 0x0000C000U;
  static constexpr uint32_t CFGR_PPRE2_DIV16 = 0x0000E000U;
  static constexpr uint32_t CFGR_RTCPRE_Pos  = ( 16U );
  static constexpr uint32_t CFGR_RTCPRE_Msk  = ( 0x1FU << CFGR_RTCPRE_Pos );
  static constexpr uint32_t CFGR_RTCPRE      = CFGR_RTCPRE_Msk;
  static constexpr uint32_t CFGR_RTCPRE_0    = ( 0x01U << CFGR_RTCPRE_Pos );
  static constexpr uint32_t CFGR_RTCPRE_1    = ( 0x02U << CFGR_RTCPRE_Pos );
  static constexpr uint32_t CFGR_RTCPRE_2    = ( 0x04U << CFGR_RTCPRE_Pos );
  static constexpr uint32_t CFGR_RTCPRE_3    = ( 0x08U << CFGR_RTCPRE_Pos );
  static constexpr uint32_t CFGR_RTCPRE_4    = ( 0x10U << CFGR_RTCPRE_Pos );
  static constexpr uint32_t CFGR_MCO1_Pos    = ( 21U );
  static constexpr uint32_t CFGR_MCO1_Msk    = ( 0x3U << CFGR_MCO1_Pos );
  static constexpr uint32_t CFGR_MCO1        = CFGR_MCO1_Msk;
  static constexpr uint32_t CFGR_MCO1_0      = ( 0x1U << CFGR_MCO1_Pos );
  static constexpr uint32_t CFGR_MCO1_1      = ( 0x2U << CFGR_MCO1_Pos );
  static constexpr uint32_t CFGR_MCO1PRE_Pos = ( 24U );
  static constexpr uint32_t CFGR_MCO1PRE_Msk = ( 0x7U << CFGR_MCO1PRE_Pos );
  static constexpr uint32_t CFGR_MCO1PRE     = CFGR_MCO1PRE_Msk;
  static constexpr uint32_t CFGR_MCO1PRE_0   = ( 0x1U << CFGR_MCO1PRE_Pos );
  static constexpr uint32_t CFGR_MCO1PRE_1   = ( 0x2U << CFGR_MCO1PRE_Pos );
  static constexpr uint32_t CFGR_MCO1PRE_2   = ( 0x4U << CFGR_MCO1PRE_Pos );
  static constexpr uint32_t CFGR_MCO2PRE_Pos = ( 27U );
  static constexpr uint32_t CFGR_MCO2PRE_Msk = ( 0x7U << CFGR_MCO2PRE_Pos );
  static constexpr uint32_t CFGR_MCO2PRE     = CFGR_MCO2PRE_Msk;
  static constexpr uint32_t CFGR_MCO2PRE_0   = ( 0x1U << CFGR_MCO2PRE_Pos );
  static constexpr uint32_t CFGR_MCO2PRE_1   = ( 0x2U << CFGR_MCO2PRE_Pos );
  static constexpr uint32_t CFGR_MCO2PRE_2   = ( 0x4U << CFGR_MCO2PRE_Pos );
  static constexpr uint32_t CFGR_MCO2_Pos    = ( 30U );
  static constexpr uint32_t CFGR_MCO2_Msk    = ( 0x3U << CFGR_MCO2_Pos );
  static constexpr uint32_t CFGR_MCO2        = CFGR_MCO2_Msk;
  static constexpr uint32_t CFGR_MCO2_0      = ( 0x1U << CFGR_MCO2_Pos );
  static constexpr uint32_t CFGR_MCO2_1      = ( 0x2U << CFGR_MCO2_Pos );




  /*------------------------------------------------
  CIR Register
  ------------------------------------------------*/
  static constexpr uint32_t CIR_LSIRDYF_Pos     = ( 0U );
  static constexpr uint32_t CIR_LSIRDYF_Msk     = ( 0x1U << CIR_LSIRDYF_Pos );
  static constexpr uint32_t CIR_LSIRDYF         = CIR_LSIRDYF_Msk;
  static constexpr uint32_t CIR_LSERDYF_Pos     = ( 1U );
  static constexpr uint32_t CIR_LSERDYF_Msk     = ( 0x1U << CIR_LSERDYF_Pos );
  static constexpr uint32_t CIR_LSERDYF         = CIR_LSERDYF_Msk;
  static constexpr uint32_t CIR_HSIRDYF_Pos     = ( 2U );
  static constexpr uint32_t CIR_HSIRDYF_Msk     = ( 0x1U << CIR_HSIRDYF_Pos );
  static constexpr uint32_t CIR_HSIRDYF         = CIR_HSIRDYF_Msk;
  static constexpr uint32_t CIR_HSERDYF_Pos     = ( 3U );
  static constexpr uint32_t CIR_HSERDYF_Msk     = ( 0x1U << CIR_HSERDYF_Pos );
  static constexpr uint32_t CIR_HSERDYF         = CIR_HSERDYF_Msk;
  static constexpr uint32_t CIR_PLLRDYF_Pos     = ( 4U );
  static constexpr uint32_t CIR_PLLRDYF_Msk     = ( 0x1U << CIR_PLLRDYF_Pos );
  static constexpr uint32_t CIR_PLLRDYF         = CIR_PLLRDYF_Msk;
  static constexpr uint32_t CIR_PLLI2SRDYF_Pos  = ( 5U );
  static constexpr uint32_t CIR_PLLI2SRDYF_Msk  = ( 0x1U << CIR_PLLI2SRDYF_Pos );
  static constexpr uint32_t CIR_PLLI2SRDYF      = CIR_PLLI2SRDYF_Msk;
  static constexpr uint32_t CIR_PLLSAIRDYF_Pos  = ( 6U );
  static constexpr uint32_t CIR_PLLSAIRDYF_Msk  = ( 0x1U << CIR_PLLSAIRDYF_Pos );
  static constexpr uint32_t CIR_PLLSAIRDYF      = CIR_PLLSAIRDYF_Msk;
  static constexpr uint32_t CIR_CSSF_Pos        = ( 7U );
  static constexpr uint32_t CIR_CSSF_Msk        = ( 0x1U << CIR_CSSF_Pos );
  static constexpr uint32_t CIR_CSSF            = CIR_CSSF_Msk;
  static constexpr uint32_t CIR_LSIRDYIE_Pos    = ( 8U );
  static constexpr uint32_t CIR_LSIRDYIE_Msk    = ( 0x1U << CIR_LSIRDYIE_Pos );
  static constexpr uint32_t CIR_LSIRDYIE        = CIR_LSIRDYIE_Msk;
  static constexpr uint32_t CIR_LSERDYIE_Pos    = ( 9U );
  static constexpr uint32_t CIR_LSERDYIE_Msk    = ( 0x1U << CIR_LSERDYIE_Pos );
  static constexpr uint32_t CIR_LSERDYIE        = CIR_LSERDYIE_Msk;
  static constexpr uint32_t CIR_HSIRDYIE_Pos    = ( 10U );
  static constexpr uint32_t CIR_HSIRDYIE_Msk    = ( 0x1U << CIR_HSIRDYIE_Pos );
  static constexpr uint32_t CIR_HSIRDYIE        = CIR_HSIRDYIE_Msk;
  static constexpr uint32_t CIR_HSERDYIE_Pos    = ( 11U );
  static constexpr uint32_t CIR_HSERDYIE_Msk    = ( 0x1U << CIR_HSERDYIE_Pos );
  static constexpr uint32_t CIR_HSERDYIE        = CIR_HSERDYIE_Msk;
  static constexpr uint32_t CIR_PLLRDYIE_Pos    = ( 12U );
  static constexpr uint32_t CIR_PLLRDYIE_Msk    = ( 0x1U << CIR_PLLRDYIE_Pos );
  static constexpr uint32_t CIR_PLLRDYIE        = CIR_PLLRDYIE_Msk;
  static constexpr uint32_t CIR_PLLI2SRDYIE_Pos = ( 13U );
  static constexpr uint32_t CIR_PLLI2SRDYIE_Msk = ( 0x1U << CIR_PLLI2SRDYIE_Pos );
  static constexpr uint32_t CIR_PLLI2SRDYIE     = CIR_PLLI2SRDYIE_Msk;
  static constexpr uint32_t CIR_PLLSAIRDYIE_Pos = ( 14U );
  static constexpr uint32_t CIR_PLLSAIRDYIE_Msk = ( 0x1U << CIR_PLLSAIRDYIE_Pos );
  static constexpr uint32_t CIR_PLLSAIRDYIE     = CIR_PLLSAIRDYIE_Msk;
  static constexpr uint32_t CIR_LSIRDYC_Pos     = ( 16U );
  static constexpr uint32_t CIR_LSIRDYC_Msk     = ( 0x1U << CIR_LSIRDYC_Pos );
  static constexpr uint32_t CIR_LSIRDYC         = CIR_LSIRDYC_Msk;
  static constexpr uint32_t CIR_LSERDYC_Pos     = ( 17U );
  static constexpr uint32_t CIR_LSERDYC_Msk     = ( 0x1U << CIR_LSERDYC_Pos );
  static constexpr uint32_t CIR_LSERDYC         = CIR_LSERDYC_Msk;
  static constexpr uint32_t CIR_HSIRDYC_Pos     = ( 18U );
  static constexpr uint32_t CIR_HSIRDYC_Msk     = ( 0x1U << CIR_HSIRDYC_Pos );
  static constexpr uint32_t CIR_HSIRDYC         = CIR_HSIRDYC_Msk;
  static constexpr uint32_t CIR_HSERDYC_Pos     = ( 19U );
  static constexpr uint32_t CIR_HSERDYC_Msk     = ( 0x1U << CIR_HSERDYC_Pos );
  static constexpr uint32_t CIR_HSERDYC         = CIR_HSERDYC_Msk;
  static constexpr uint32_t CIR_PLLRDYC_Pos     = ( 20U );
  static constexpr uint32_t CIR_PLLRDYC_Msk     = ( 0x1U << CIR_PLLRDYC_Pos );
  static constexpr uint32_t CIR_PLLRDYC         = CIR_PLLRDYC_Msk;
  static constexpr uint32_t CIR_PLLI2SRDYC_Pos  = ( 21U );
  static constexpr uint32_t CIR_PLLI2SRDYC_Msk  = ( 0x1U << CIR_PLLI2SRDYC_Pos );
  static constexpr uint32_t CIR_PLLI2SRDYC      = CIR_PLLI2SRDYC_Msk;
  static constexpr uint32_t CIR_PLLSAIRDYC_Pos  = ( 22U );
  static constexpr uint32_t CIR_PLLSAIRDYC_Msk  = ( 0x1U << CIR_PLLSAIRDYC_Pos );
  static constexpr uint32_t CIR_PLLSAIRDYC      = CIR_PLLSAIRDYC_Msk;
  static constexpr uint32_t CIR_CSSC_Pos        = ( 23U );
  static constexpr uint32_t CIR_CSSC_Msk        = ( 0x1U << CIR_CSSC_Pos );
  static constexpr uint32_t CIR_CSSC            = CIR_CSSC_Msk;

  /*------------------------------------------------
  AHB1RSTR Register
  ------------------------------------------------*/
  static constexpr uint32_t AHB1RSTR_GPIOARST_Pos = ( 0U );
  static constexpr uint32_t AHB1RSTR_GPIOARST_Msk = ( 0x1U << AHB1RSTR_GPIOARST_Pos );
  static constexpr uint32_t AHB1RSTR_GPIOARST     = AHB1RSTR_GPIOARST_Msk;
  static constexpr uint32_t AHB1RSTR_GPIOBRST_Pos = ( 1U );
  static constexpr uint32_t AHB1RSTR_GPIOBRST_Msk = ( 0x1U << AHB1RSTR_GPIOBRST_Pos );
  static constexpr uint32_t AHB1RSTR_GPIOBRST     = AHB1RSTR_GPIOBRST_Msk;
  static constexpr uint32_t AHB1RSTR_GPIOCRST_Pos = ( 2U );
  static constexpr uint32_t AHB1RSTR_GPIOCRST_Msk = ( 0x1U << AHB1RSTR_GPIOCRST_Pos );
  static constexpr uint32_t AHB1RSTR_GPIOCRST     = AHB1RSTR_GPIOCRST_Msk;
  static constexpr uint32_t AHB1RSTR_GPIODRST_Pos = ( 3U );
  static constexpr uint32_t AHB1RSTR_GPIODRST_Msk = ( 0x1U << AHB1RSTR_GPIODRST_Pos );
  static constexpr uint32_t AHB1RSTR_GPIODRST     = AHB1RSTR_GPIODRST_Msk;
  static constexpr uint32_t AHB1RSTR_GPIOERST_Pos = ( 4U );
  static constexpr uint32_t AHB1RSTR_GPIOERST_Msk = ( 0x1U << AHB1RSTR_GPIOERST_Pos );
  static constexpr uint32_t AHB1RSTR_GPIOERST     = AHB1RSTR_GPIOERST_Msk;
  static constexpr uint32_t AHB1RSTR_GPIOFRST_Pos = ( 5U );
  static constexpr uint32_t AHB1RSTR_GPIOFRST_Msk = ( 0x1U << AHB1RSTR_GPIOFRST_Pos );
  static constexpr uint32_t AHB1RSTR_GPIOFRST     = AHB1RSTR_GPIOFRST_Msk;
  static constexpr uint32_t AHB1RSTR_GPIOGRST_Pos = ( 6U );
  static constexpr uint32_t AHB1RSTR_GPIOGRST_Msk = ( 0x1U << AHB1RSTR_GPIOGRST_Pos );
  static constexpr uint32_t AHB1RSTR_GPIOGRST     = AHB1RSTR_GPIOGRST_Msk;
  static constexpr uint32_t AHB1RSTR_GPIOHRST_Pos = ( 7U );
  static constexpr uint32_t AHB1RSTR_GPIOHRST_Msk = ( 0x1U << AHB1RSTR_GPIOHRST_Pos );
  static constexpr uint32_t AHB1RSTR_GPIOHRST     = AHB1RSTR_GPIOHRST_Msk;
  static constexpr uint32_t AHB1RSTR_CRCRST_Pos   = ( 12U );
  static constexpr uint32_t AHB1RSTR_CRCRST_Msk   = ( 0x1U << AHB1RSTR_CRCRST_Pos );
  static constexpr uint32_t AHB1RSTR_CRCRST       = AHB1RSTR_CRCRST_Msk;
  static constexpr uint32_t AHB1RSTR_DMA1RST_Pos  = ( 21U );
  static constexpr uint32_t AHB1RSTR_DMA1RST_Msk  = ( 0x1U << AHB1RSTR_DMA1RST_Pos );
  static constexpr uint32_t AHB1RSTR_DMA1RST      = AHB1RSTR_DMA1RST_Msk;
  static constexpr uint32_t AHB1RSTR_DMA2RST_Pos  = ( 22U );
  static constexpr uint32_t AHB1RSTR_DMA2RST_Msk  = ( 0x1U << AHB1RSTR_DMA2RST_Pos );
  static constexpr uint32_t AHB1RSTR_DMA2RST      = AHB1RSTR_DMA2RST_Msk;
  static constexpr uint32_t AHB1RSTR_OTGHRST_Pos  = ( 29U );
  static constexpr uint32_t AHB1RSTR_OTGHRST_Msk  = ( 0x1U << AHB1RSTR_OTGHRST_Pos );
  static constexpr uint32_t AHB1RSTR_OTGHRST      = AHB1RSTR_OTGHRST_Msk;

  /*------------------------------------------------
  AHB2RSTR Register
  ------------------------------------------------*/
  static constexpr uint32_t AHB2RSTR_DCMIRST_Pos  = ( 0U );
  static constexpr uint32_t AHB2RSTR_DCMIRST_Msk  = ( 0x1U << AHB2RSTR_DCMIRST_Pos );
  static constexpr uint32_t AHB2RSTR_DCMIRST      = AHB2RSTR_DCMIRST_Msk;
  static constexpr uint32_t AHB2RSTR_OTGFSRST_Pos = ( 7U );
  static constexpr uint32_t AHB2RSTR_OTGFSRST_Msk = ( 0x1U << AHB2RSTR_OTGFSRST_Pos );
  static constexpr uint32_t AHB2RSTR_OTGFSRST     = AHB2RSTR_OTGFSRST_Msk;

  /*------------------------------------------------
  AHB3RSTR Register
  ------------------------------------------------*/
  static constexpr uint32_t AHB3RSTR_FMCRST_Pos  = ( 0U );
  static constexpr uint32_t AHB3RSTR_FMCRST_Msk  = ( 0x1U << AHB3RSTR_FMCRST_Pos );
  static constexpr uint32_t AHB3RSTR_FMCRST      = AHB3RSTR_FMCRST_Msk;
  static constexpr uint32_t AHB3RSTR_QSPIRST_Pos = ( 1U );
  static constexpr uint32_t AHB3RSTR_QSPIRST_Msk = ( 0x1U << AHB3RSTR_QSPIRST_Pos );
  static constexpr uint32_t AHB3RSTR_QSPIRST     = AHB3RSTR_QSPIRST_Msk;

  /*------------------------------------------------
  APB1RSTR Register
  ------------------------------------------------*/
  static constexpr uint32_t APB1RSTR_TIM2RST_Pos    = ( 0U );
  static constexpr uint32_t APB1RSTR_TIM2RST_Msk    = ( 0x1U << APB1RSTR_TIM2RST_Pos );
  static constexpr uint32_t APB1RSTR_TIM2RST        = APB1RSTR_TIM2RST_Msk;
  static constexpr uint32_t APB1RSTR_TIM3RST_Pos    = ( 1U );
  static constexpr uint32_t APB1RSTR_TIM3RST_Msk    = ( 0x1U << APB1RSTR_TIM3RST_Pos );
  static constexpr uint32_t APB1RSTR_TIM3RST        = APB1RSTR_TIM3RST_Msk;
  static constexpr uint32_t APB1RSTR_TIM4RST_Pos    = ( 2U );
  static constexpr uint32_t APB1RSTR_TIM4RST_Msk    = ( 0x1U << APB1RSTR_TIM4RST_Pos );
  static constexpr uint32_t APB1RSTR_TIM4RST        = APB1RSTR_TIM4RST_Msk;
  static constexpr uint32_t APB1RSTR_TIM5RST_Pos    = ( 3U );
  static constexpr uint32_t APB1RSTR_TIM5RST_Msk    = ( 0x1U << APB1RSTR_TIM5RST_Pos );
  static constexpr uint32_t APB1RSTR_TIM5RST        = APB1RSTR_TIM5RST_Msk;
  static constexpr uint32_t APB1RSTR_TIM6RST_Pos    = ( 4U );
  static constexpr uint32_t APB1RSTR_TIM6RST_Msk    = ( 0x1U << APB1RSTR_TIM6RST_Pos );
  static constexpr uint32_t APB1RSTR_TIM6RST        = APB1RSTR_TIM6RST_Msk;
  static constexpr uint32_t APB1RSTR_TIM7RST_Pos    = ( 5U );
  static constexpr uint32_t APB1RSTR_TIM7RST_Msk    = ( 0x1U << APB1RSTR_TIM7RST_Pos );
  static constexpr uint32_t APB1RSTR_TIM7RST        = APB1RSTR_TIM7RST_Msk;
  static constexpr uint32_t APB1RSTR_TIM12RST_Pos   = ( 6U );
  static constexpr uint32_t APB1RSTR_TIM12RST_Msk   = ( 0x1U << APB1RSTR_TIM12RST_Pos );
  static constexpr uint32_t APB1RSTR_TIM12RST       = APB1RSTR_TIM12RST_Msk;
  static constexpr uint32_t APB1RSTR_TIM13RST_Pos   = ( 7U );
  static constexpr uint32_t APB1RSTR_TIM13RST_Msk   = ( 0x1U << APB1RSTR_TIM13RST_Pos );
  static constexpr uint32_t APB1RSTR_TIM13RST       = APB1RSTR_TIM13RST_Msk;
  static constexpr uint32_t APB1RSTR_TIM14RST_Pos   = ( 8U );
  static constexpr uint32_t APB1RSTR_TIM14RST_Msk   = ( 0x1U << APB1RSTR_TIM14RST_Pos );
  static constexpr uint32_t APB1RSTR_TIM14RST       = APB1RSTR_TIM14RST_Msk;
  static constexpr uint32_t APB1RSTR_WWDGRST_Pos    = ( 11U );
  static constexpr uint32_t APB1RSTR_WWDGRST_Msk    = ( 0x1U << APB1RSTR_WWDGRST_Pos );
  static constexpr uint32_t APB1RSTR_WWDGRST        = APB1RSTR_WWDGRST_Msk;
  static constexpr uint32_t APB1RSTR_SPI2RST_Pos    = ( 14U );
  static constexpr uint32_t APB1RSTR_SPI2RST_Msk    = ( 0x1U << APB1RSTR_SPI2RST_Pos );
  static constexpr uint32_t APB1RSTR_SPI2RST        = APB1RSTR_SPI2RST_Msk;
  static constexpr uint32_t APB1RSTR_SPI3RST_Pos    = ( 15U );
  static constexpr uint32_t APB1RSTR_SPI3RST_Msk    = ( 0x1U << APB1RSTR_SPI3RST_Pos );
  static constexpr uint32_t APB1RSTR_SPI3RST        = APB1RSTR_SPI3RST_Msk;
  static constexpr uint32_t APB1RSTR_SPDIFRXRST_Pos = ( 16U );
  static constexpr uint32_t APB1RSTR_SPDIFRXRST_Msk = ( 0x1U << APB1RSTR_SPDIFRXRST_Pos );
  static constexpr uint32_t APB1RSTR_SPDIFRXRST     = APB1RSTR_SPDIFRXRST_Msk;
  static constexpr uint32_t APB1RSTR_USART2RST_Pos  = ( 17U );
  static constexpr uint32_t APB1RSTR_USART2RST_Msk  = ( 0x1U << APB1RSTR_USART2RST_Pos );
  static constexpr uint32_t APB1RSTR_USART2RST      = APB1RSTR_USART2RST_Msk;
  static constexpr uint32_t APB1RSTR_USART3RST_Pos  = ( 18U );
  static constexpr uint32_t APB1RSTR_USART3RST_Msk  = ( 0x1U << APB1RSTR_USART3RST_Pos );
  static constexpr uint32_t APB1RSTR_USART3RST      = APB1RSTR_USART3RST_Msk;
  static constexpr uint32_t APB1RSTR_UART4RST_Pos   = ( 19U );
  static constexpr uint32_t APB1RSTR_UART4RST_Msk   = ( 0x1U << APB1RSTR_UART4RST_Pos );
  static constexpr uint32_t APB1RSTR_UART4RST       = APB1RSTR_UART4RST_Msk;
  static constexpr uint32_t APB1RSTR_UART5RST_Pos   = ( 20U );
  static constexpr uint32_t APB1RSTR_UART5RST_Msk   = ( 0x1U << APB1RSTR_UART5RST_Pos );
  static constexpr uint32_t APB1RSTR_UART5RST       = APB1RSTR_UART5RST_Msk;
  static constexpr uint32_t APB1RSTR_I2C1RST_Pos    = ( 21U );
  static constexpr uint32_t APB1RSTR_I2C1RST_Msk    = ( 0x1U << APB1RSTR_I2C1RST_Pos );
  static constexpr uint32_t APB1RSTR_I2C1RST        = APB1RSTR_I2C1RST_Msk;
  static constexpr uint32_t APB1RSTR_I2C2RST_Pos    = ( 22U );
  static constexpr uint32_t APB1RSTR_I2C2RST_Msk    = ( 0x1U << APB1RSTR_I2C2RST_Pos );
  static constexpr uint32_t APB1RSTR_I2C2RST        = APB1RSTR_I2C2RST_Msk;
  static constexpr uint32_t APB1RSTR_I2C3RST_Pos    = ( 23U );
  static constexpr uint32_t APB1RSTR_I2C3RST_Msk    = ( 0x1U << APB1RSTR_I2C3RST_Pos );
  static constexpr uint32_t APB1RSTR_I2C3RST        = APB1RSTR_I2C3RST_Msk;
  static constexpr uint32_t APB1RSTR_FMPI2C1RST_Pos = ( 24U );
  static constexpr uint32_t APB1RSTR_FMPI2C1RST_Msk = ( 0x1U << APB1RSTR_FMPI2C1RST_Pos );
  static constexpr uint32_t APB1RSTR_FMPI2C1RST     = APB1RSTR_FMPI2C1RST_Msk;
  static constexpr uint32_t APB1RSTR_CAN1RST_Pos    = ( 25U );
  static constexpr uint32_t APB1RSTR_CAN1RST_Msk    = ( 0x1U << APB1RSTR_CAN1RST_Pos );
  static constexpr uint32_t APB1RSTR_CAN1RST        = APB1RSTR_CAN1RST_Msk;
  static constexpr uint32_t APB1RSTR_CAN2RST_Pos    = ( 26U );
  static constexpr uint32_t APB1RSTR_CAN2RST_Msk    = ( 0x1U << APB1RSTR_CAN2RST_Pos );
  static constexpr uint32_t APB1RSTR_CAN2RST        = APB1RSTR_CAN2RST_Msk;
  static constexpr uint32_t APB1RSTR_CECRST_Pos     = ( 27U );
  static constexpr uint32_t APB1RSTR_CECRST_Msk     = ( 0x1U << APB1RSTR_CECRST_Pos );
  static constexpr uint32_t APB1RSTR_CECRST         = APB1RSTR_CECRST_Msk;
  static constexpr uint32_t APB1RSTR_PWRRST_Pos     = ( 28U );
  static constexpr uint32_t APB1RSTR_PWRRST_Msk     = ( 0x1U << APB1RSTR_PWRRST_Pos );
  static constexpr uint32_t APB1RSTR_PWRRST         = APB1RSTR_PWRRST_Msk;
  static constexpr uint32_t APB1RSTR_DACRST_Pos     = ( 29U );
  static constexpr uint32_t APB1RSTR_DACRST_Msk     = ( 0x1U << APB1RSTR_DACRST_Pos );
  static constexpr uint32_t APB1RSTR_DACRST         = APB1RSTR_DACRST_Msk;

  /*------------------------------------------------
  APB2RSTR Register
  ------------------------------------------------*/
  static constexpr uint32_t APB2RSTR_TIM1RST_Pos   = ( 0U );
  static constexpr uint32_t APB2RSTR_TIM1RST_Msk   = ( 0x1U << APB2RSTR_TIM1RST_Pos );
  static constexpr uint32_t APB2RSTR_TIM1RST       = APB2RSTR_TIM1RST_Msk;
  static constexpr uint32_t APB2RSTR_TIM8RST_Pos   = ( 1U );
  static constexpr uint32_t APB2RSTR_TIM8RST_Msk   = ( 0x1U << APB2RSTR_TIM8RST_Pos );
  static constexpr uint32_t APB2RSTR_TIM8RST       = APB2RSTR_TIM8RST_Msk;
  static constexpr uint32_t APB2RSTR_USART1RST_Pos = ( 4U );
  static constexpr uint32_t APB2RSTR_USART1RST_Msk = ( 0x1U << APB2RSTR_USART1RST_Pos );
  static constexpr uint32_t APB2RSTR_USART1RST     = APB2RSTR_USART1RST_Msk;
  static constexpr uint32_t APB2RSTR_USART6RST_Pos = ( 5U );
  static constexpr uint32_t APB2RSTR_USART6RST_Msk = ( 0x1U << APB2RSTR_USART6RST_Pos );
  static constexpr uint32_t APB2RSTR_USART6RST     = APB2RSTR_USART6RST_Msk;
  static constexpr uint32_t APB2RSTR_ADCRST_Pos    = ( 8U );
  static constexpr uint32_t APB2RSTR_ADCRST_Msk    = ( 0x1U << APB2RSTR_ADCRST_Pos );
  static constexpr uint32_t APB2RSTR_ADCRST        = APB2RSTR_ADCRST_Msk;
  static constexpr uint32_t APB2RSTR_SDIORST_Pos   = ( 11U );
  static constexpr uint32_t APB2RSTR_SDIORST_Msk   = ( 0x1U << APB2RSTR_SDIORST_Pos );
  static constexpr uint32_t APB2RSTR_SDIORST       = APB2RSTR_SDIORST_Msk;
  static constexpr uint32_t APB2RSTR_SPI1RST_Pos   = ( 12U );
  static constexpr uint32_t APB2RSTR_SPI1RST_Msk   = ( 0x1U << APB2RSTR_SPI1RST_Pos );
  static constexpr uint32_t APB2RSTR_SPI1RST       = APB2RSTR_SPI1RST_Msk;
  static constexpr uint32_t APB2RSTR_SPI4RST_Pos   = ( 13U );
  static constexpr uint32_t APB2RSTR_SPI4RST_Msk   = ( 0x1U << APB2RSTR_SPI4RST_Pos );
  static constexpr uint32_t APB2RSTR_SPI4RST       = APB2RSTR_SPI4RST_Msk;
  static constexpr uint32_t APB2RSTR_SYSCFGRST_Pos = ( 14U );
  static constexpr uint32_t APB2RSTR_SYSCFGRST_Msk = ( 0x1U << APB2RSTR_SYSCFGRST_Pos );
  static constexpr uint32_t APB2RSTR_SYSCFGRST     = APB2RSTR_SYSCFGRST_Msk;
  static constexpr uint32_t APB2RSTR_TIM9RST_Pos   = ( 16U );
  static constexpr uint32_t APB2RSTR_TIM9RST_Msk   = ( 0x1U << APB2RSTR_TIM9RST_Pos );
  static constexpr uint32_t APB2RSTR_TIM9RST       = APB2RSTR_TIM9RST_Msk;
  static constexpr uint32_t APB2RSTR_TIM10RST_Pos  = ( 17U );
  static constexpr uint32_t APB2RSTR_TIM10RST_Msk  = ( 0x1U << APB2RSTR_TIM10RST_Pos );
  static constexpr uint32_t APB2RSTR_TIM10RST      = APB2RSTR_TIM10RST_Msk;
  static constexpr uint32_t APB2RSTR_TIM11RST_Pos  = ( 18U );
  static constexpr uint32_t APB2RSTR_TIM11RST_Msk  = ( 0x1U << APB2RSTR_TIM11RST_Pos );
  static constexpr uint32_t APB2RSTR_TIM11RST      = APB2RSTR_TIM11RST_Msk;
  static constexpr uint32_t APB2RSTR_SAI1RST_Pos   = ( 22U );
  static constexpr uint32_t APB2RSTR_SAI1RST_Msk   = ( 0x1U << APB2RSTR_SAI1RST_Pos );
  static constexpr uint32_t APB2RSTR_SAI1RST       = APB2RSTR_SAI1RST_Msk;
  static constexpr uint32_t APB2RSTR_SAI2RST_Pos   = ( 23U );
  static constexpr uint32_t APB2RSTR_SAI2RST_Msk   = ( 0x1U << APB2RSTR_SAI2RST_Pos );
  static constexpr uint32_t APB2RSTR_SAI2RST       = APB2RSTR_SAI2RST_Msk;

  /*------------------------------------------------
  AHB1ENR Register
  ------------------------------------------------*/
  static constexpr uint32_t AHB1ENR_GPIOAEN_Pos     = ( 0U );
  static constexpr uint32_t AHB1ENR_GPIOAEN_Msk     = ( 0x1U << AHB1ENR_GPIOAEN_Pos );
  static constexpr uint32_t AHB1ENR_GPIOAEN         = AHB1ENR_GPIOAEN_Msk;
  static constexpr uint32_t AHB1ENR_GPIOBEN_Pos     = ( 1U );
  static constexpr uint32_t AHB1ENR_GPIOBEN_Msk     = ( 0x1U << AHB1ENR_GPIOBEN_Pos );
  static constexpr uint32_t AHB1ENR_GPIOBEN         = AHB1ENR_GPIOBEN_Msk;
  static constexpr uint32_t AHB1ENR_GPIOCEN_Pos     = ( 2U );
  static constexpr uint32_t AHB1ENR_GPIOCEN_Msk     = ( 0x1U << AHB1ENR_GPIOCEN_Pos );
  static constexpr uint32_t AHB1ENR_GPIOCEN         = AHB1ENR_GPIOCEN_Msk;
  static constexpr uint32_t AHB1ENR_GPIODEN_Pos     = ( 3U );
  static constexpr uint32_t AHB1ENR_GPIODEN_Msk     = ( 0x1U << AHB1ENR_GPIODEN_Pos );
  static constexpr uint32_t AHB1ENR_GPIODEN         = AHB1ENR_GPIODEN_Msk;
  static constexpr uint32_t AHB1ENR_GPIOEEN_Pos     = ( 4U );
  static constexpr uint32_t AHB1ENR_GPIOEEN_Msk     = ( 0x1U << AHB1ENR_GPIOEEN_Pos );
  static constexpr uint32_t AHB1ENR_GPIOEEN         = AHB1ENR_GPIOEEN_Msk;
  static constexpr uint32_t AHB1ENR_GPIOFEN_Pos     = ( 5U );
  static constexpr uint32_t AHB1ENR_GPIOFEN_Msk     = ( 0x1U << AHB1ENR_GPIOFEN_Pos );
  static constexpr uint32_t AHB1ENR_GPIOFEN         = AHB1ENR_GPIOFEN_Msk;
  static constexpr uint32_t AHB1ENR_GPIOGEN_Pos     = ( 6U );
  static constexpr uint32_t AHB1ENR_GPIOGEN_Msk     = ( 0x1U << AHB1ENR_GPIOGEN_Pos );
  static constexpr uint32_t AHB1ENR_GPIOGEN         = AHB1ENR_GPIOGEN_Msk;
  static constexpr uint32_t AHB1ENR_GPIOHEN_Pos     = ( 7U );
  static constexpr uint32_t AHB1ENR_GPIOHEN_Msk     = ( 0x1U << AHB1ENR_GPIOHEN_Pos );
  static constexpr uint32_t AHB1ENR_GPIOHEN         = AHB1ENR_GPIOHEN_Msk;
  static constexpr uint32_t AHB1ENR_CRCEN_Pos       = ( 12U );
  static constexpr uint32_t AHB1ENR_CRCEN_Msk       = ( 0x1U << AHB1ENR_CRCEN_Pos );
  static constexpr uint32_t AHB1ENR_CRCEN           = AHB1ENR_CRCEN_Msk;
  static constexpr uint32_t AHB1ENR_BKPSRAMEN_Pos   = ( 18U );
  static constexpr uint32_t AHB1ENR_BKPSRAMEN_Msk   = ( 0x1U << AHB1ENR_BKPSRAMEN_Pos );
  static constexpr uint32_t AHB1ENR_BKPSRAMEN       = AHB1ENR_BKPSRAMEN_Msk;
  static constexpr uint32_t AHB1ENR_DMA1EN_Pos      = ( 21U );
  static constexpr uint32_t AHB1ENR_DMA1EN_Msk      = ( 0x1U << AHB1ENR_DMA1EN_Pos );
  static constexpr uint32_t AHB1ENR_DMA1EN          = AHB1ENR_DMA1EN_Msk;
  static constexpr uint32_t AHB1ENR_DMA2EN_Pos      = ( 22U );
  static constexpr uint32_t AHB1ENR_DMA2EN_Msk      = ( 0x1U << AHB1ENR_DMA2EN_Pos );
  static constexpr uint32_t AHB1ENR_DMA2EN          = AHB1ENR_DMA2EN_Msk;
  static constexpr uint32_t AHB1ENR_OTGHSEN_Pos     = ( 29U );
  static constexpr uint32_t AHB1ENR_OTGHSEN_Msk     = ( 0x1U << AHB1ENR_OTGHSEN_Pos );
  static constexpr uint32_t AHB1ENR_OTGHSEN         = AHB1ENR_OTGHSEN_Msk;
  static constexpr uint32_t AHB1ENR_OTGHSULPIEN_Pos = ( 30U );
  static constexpr uint32_t AHB1ENR_OTGHSULPIEN_Msk = ( 0x1U << AHB1ENR_OTGHSULPIEN_Pos );
  static constexpr uint32_t AHB1ENR_OTGHSULPIEN     = AHB1ENR_OTGHSULPIEN_Msk;

  /*------------------------------------------------
  AHB2ENR Register
  ------------------------------------------------*/
  static constexpr uint32_t AHB2ENR_DCMIEN_Pos  = ( 0U );
  static constexpr uint32_t AHB2ENR_DCMIEN_Msk  = ( 0x1U << AHB2ENR_DCMIEN_Pos );
  static constexpr uint32_t AHB2ENR_DCMIEN      = AHB2ENR_DCMIEN_Msk;
  static constexpr uint32_t AHB2ENR_OTGFSEN_Pos = ( 7U );
  static constexpr uint32_t AHB2ENR_OTGFSEN_Msk = ( 0x1U << AHB2ENR_OTGFSEN_Pos );
  static constexpr uint32_t AHB2ENR_OTGFSEN     = AHB2ENR_OTGFSEN_Msk;

  /*------------------------------------------------
  AHB3ENR Register
  ------------------------------------------------*/
  static constexpr uint32_t AHB3ENR_FMCEN_Pos  = ( 0U );
  static constexpr uint32_t AHB3ENR_FMCEN_Msk  = ( 0x1U << AHB3ENR_FMCEN_Pos );
  static constexpr uint32_t AHB3ENR_FMCEN      = AHB3ENR_FMCEN_Msk;
  static constexpr uint32_t AHB3ENR_QSPIEN_Pos = ( 1U );
  static constexpr uint32_t AHB3ENR_QSPIEN_Msk = ( 0x1U << AHB3ENR_QSPIEN_Pos );
  static constexpr uint32_t AHB3ENR_QSPIEN     = AHB3ENR_QSPIEN_Msk;

  /*------------------------------------------------
  APB1ENR Register
  ------------------------------------------------*/
  static constexpr uint32_t APB1ENR_TIM2EN_Pos    = ( 0U );
  static constexpr uint32_t APB1ENR_TIM2EN_Msk    = ( 0x1U << APB1ENR_TIM2EN_Pos );
  static constexpr uint32_t APB1ENR_TIM2EN        = APB1ENR_TIM2EN_Msk;
  static constexpr uint32_t APB1ENR_TIM3EN_Pos    = ( 1U );
  static constexpr uint32_t APB1ENR_TIM3EN_Msk    = ( 0x1U << APB1ENR_TIM3EN_Pos );
  static constexpr uint32_t APB1ENR_TIM3EN        = APB1ENR_TIM3EN_Msk;
  static constexpr uint32_t APB1ENR_TIM4EN_Pos    = ( 2U );
  static constexpr uint32_t APB1ENR_TIM4EN_Msk    = ( 0x1U << APB1ENR_TIM4EN_Pos );
  static constexpr uint32_t APB1ENR_TIM4EN        = APB1ENR_TIM4EN_Msk;
  static constexpr uint32_t APB1ENR_TIM5EN_Pos    = ( 3U );
  static constexpr uint32_t APB1ENR_TIM5EN_Msk    = ( 0x1U << APB1ENR_TIM5EN_Pos );
  static constexpr uint32_t APB1ENR_TIM5EN        = APB1ENR_TIM5EN_Msk;
  static constexpr uint32_t APB1ENR_TIM6EN_Pos    = ( 4U );
  static constexpr uint32_t APB1ENR_TIM6EN_Msk    = ( 0x1U << APB1ENR_TIM6EN_Pos );
  static constexpr uint32_t APB1ENR_TIM6EN        = APB1ENR_TIM6EN_Msk;
  static constexpr uint32_t APB1ENR_TIM7EN_Pos    = ( 5U );
  static constexpr uint32_t APB1ENR_TIM7EN_Msk    = ( 0x1U << APB1ENR_TIM7EN_Pos );
  static constexpr uint32_t APB1ENR_TIM7EN        = APB1ENR_TIM7EN_Msk;
  static constexpr uint32_t APB1ENR_TIM12EN_Pos   = ( 6U );
  static constexpr uint32_t APB1ENR_TIM12EN_Msk   = ( 0x1U << APB1ENR_TIM12EN_Pos );
  static constexpr uint32_t APB1ENR_TIM12EN       = APB1ENR_TIM12EN_Msk;
  static constexpr uint32_t APB1ENR_TIM13EN_Pos   = ( 7U );
  static constexpr uint32_t APB1ENR_TIM13EN_Msk   = ( 0x1U << APB1ENR_TIM13EN_Pos );
  static constexpr uint32_t APB1ENR_TIM13EN       = APB1ENR_TIM13EN_Msk;
  static constexpr uint32_t APB1ENR_TIM14EN_Pos   = ( 8U );
  static constexpr uint32_t APB1ENR_TIM14EN_Msk   = ( 0x1U << APB1ENR_TIM14EN_Pos );
  static constexpr uint32_t APB1ENR_TIM14EN       = APB1ENR_TIM14EN_Msk;
  static constexpr uint32_t APB1ENR_WWDGEN_Pos    = ( 11U );
  static constexpr uint32_t APB1ENR_WWDGEN_Msk    = ( 0x1U << APB1ENR_WWDGEN_Pos );
  static constexpr uint32_t APB1ENR_WWDGEN        = APB1ENR_WWDGEN_Msk;
  static constexpr uint32_t APB1ENR_SPI2EN_Pos    = ( 14U );
  static constexpr uint32_t APB1ENR_SPI2EN_Msk    = ( 0x1U << APB1ENR_SPI2EN_Pos );
  static constexpr uint32_t APB1ENR_SPI2EN        = APB1ENR_SPI2EN_Msk;
  static constexpr uint32_t APB1ENR_SPI3EN_Pos    = ( 15U );
  static constexpr uint32_t APB1ENR_SPI3EN_Msk    = ( 0x1U << APB1ENR_SPI3EN_Pos );
  static constexpr uint32_t APB1ENR_SPI3EN        = APB1ENR_SPI3EN_Msk;
  static constexpr uint32_t APB1ENR_SPDIFRXEN_Pos = ( 16U );
  static constexpr uint32_t APB1ENR_SPDIFRXEN_Msk = ( 0x1U << APB1ENR_SPDIFRXEN_Pos );
  static constexpr uint32_t APB1ENR_SPDIFRXEN     = APB1ENR_SPDIFRXEN_Msk;
  static constexpr uint32_t APB1ENR_USART2EN_Pos  = ( 17U );
  static constexpr uint32_t APB1ENR_USART2EN_Msk  = ( 0x1U << APB1ENR_USART2EN_Pos );
  static constexpr uint32_t APB1ENR_USART2EN      = APB1ENR_USART2EN_Msk;
  static constexpr uint32_t APB1ENR_USART3EN_Pos  = ( 18U );
  static constexpr uint32_t APB1ENR_USART3EN_Msk  = ( 0x1U << APB1ENR_USART3EN_Pos );
  static constexpr uint32_t APB1ENR_USART3EN      = APB1ENR_USART3EN_Msk;
  static constexpr uint32_t APB1ENR_UART4EN_Pos   = ( 19U );
  static constexpr uint32_t APB1ENR_UART4EN_Msk   = ( 0x1U << APB1ENR_UART4EN_Pos );
  static constexpr uint32_t APB1ENR_UART4EN       = APB1ENR_UART4EN_Msk;
  static constexpr uint32_t APB1ENR_UART5EN_Pos   = ( 20U );
  static constexpr uint32_t APB1ENR_UART5EN_Msk   = ( 0x1U << APB1ENR_UART5EN_Pos );
  static constexpr uint32_t APB1ENR_UART5EN       = APB1ENR_UART5EN_Msk;
  static constexpr uint32_t APB1ENR_I2C1EN_Pos    = ( 21U );
  static constexpr uint32_t APB1ENR_I2C1EN_Msk    = ( 0x1U << APB1ENR_I2C1EN_Pos );
  static constexpr uint32_t APB1ENR_I2C1EN        = APB1ENR_I2C1EN_Msk;
  static constexpr uint32_t APB1ENR_I2C2EN_Pos    = ( 22U );
  static constexpr uint32_t APB1ENR_I2C2EN_Msk    = ( 0x1U << APB1ENR_I2C2EN_Pos );
  static constexpr uint32_t APB1ENR_I2C2EN        = APB1ENR_I2C2EN_Msk;
  static constexpr uint32_t APB1ENR_I2C3EN_Pos    = ( 23U );
  static constexpr uint32_t APB1ENR_I2C3EN_Msk    = ( 0x1U << APB1ENR_I2C3EN_Pos );
  static constexpr uint32_t APB1ENR_I2C3EN        = APB1ENR_I2C3EN_Msk;
  static constexpr uint32_t APB1ENR_FMPI2C1EN_Pos = ( 24U );
  static constexpr uint32_t APB1ENR_FMPI2C1EN_Msk = ( 0x1U << APB1ENR_FMPI2C1EN_Pos );
  static constexpr uint32_t APB1ENR_FMPI2C1EN     = APB1ENR_FMPI2C1EN_Msk;
  static constexpr uint32_t APB1ENR_CAN1EN_Pos    = ( 25U );
  static constexpr uint32_t APB1ENR_CAN1EN_Msk    = ( 0x1U << APB1ENR_CAN1EN_Pos );
  static constexpr uint32_t APB1ENR_CAN1EN        = APB1ENR_CAN1EN_Msk;
  static constexpr uint32_t APB1ENR_CAN2EN_Pos    = ( 26U );
  static constexpr uint32_t APB1ENR_CAN2EN_Msk    = ( 0x1U << APB1ENR_CAN2EN_Pos );
  static constexpr uint32_t APB1ENR_CAN2EN        = APB1ENR_CAN2EN_Msk;
  static constexpr uint32_t APB1ENR_CECEN_Pos     = ( 27U );
  static constexpr uint32_t APB1ENR_CECEN_Msk     = ( 0x1U << APB1ENR_CECEN_Pos );
  static constexpr uint32_t APB1ENR_CECEN         = APB1ENR_CECEN_Msk;
  static constexpr uint32_t APB1ENR_PWREN_Pos     = ( 28U );
  static constexpr uint32_t APB1ENR_PWREN_Msk     = ( 0x1U << APB1ENR_PWREN_Pos );
  static constexpr uint32_t APB1ENR_PWREN         = APB1ENR_PWREN_Msk;
  static constexpr uint32_t APB1ENR_DACEN_Pos     = ( 29U );
  static constexpr uint32_t APB1ENR_DACEN_Msk     = ( 0x1U << APB1ENR_DACEN_Pos );
  static constexpr uint32_t APB1ENR_DACEN         = APB1ENR_DACEN_Msk;

  /*------------------------------------------------
  APB2ENR Register
  ------------------------------------------------*/
  static constexpr uint32_t APB2ENR_TIM1EN_Pos   = ( 0U );
  static constexpr uint32_t APB2ENR_TIM1EN_Msk   = ( 0x1U << APB2ENR_TIM1EN_Pos );
  static constexpr uint32_t APB2ENR_TIM1EN       = APB2ENR_TIM1EN_Msk;
  static constexpr uint32_t APB2ENR_TIM8EN_Pos   = ( 1U );
  static constexpr uint32_t APB2ENR_TIM8EN_Msk   = ( 0x1U << APB2ENR_TIM8EN_Pos );
  static constexpr uint32_t APB2ENR_TIM8EN       = APB2ENR_TIM8EN_Msk;
  static constexpr uint32_t APB2ENR_USART1EN_Pos = ( 4U );
  static constexpr uint32_t APB2ENR_USART1EN_Msk = ( 0x1U << APB2ENR_USART1EN_Pos );
  static constexpr uint32_t APB2ENR_USART1EN     = APB2ENR_USART1EN_Msk;
  static constexpr uint32_t APB2ENR_USART6EN_Pos = ( 5U );
  static constexpr uint32_t APB2ENR_USART6EN_Msk = ( 0x1U << APB2ENR_USART6EN_Pos );
  static constexpr uint32_t APB2ENR_USART6EN     = APB2ENR_USART6EN_Msk;
  static constexpr uint32_t APB2ENR_ADC1EN_Pos   = ( 8U );
  static constexpr uint32_t APB2ENR_ADC1EN_Msk   = ( 0x1U << APB2ENR_ADC1EN_Pos );
  static constexpr uint32_t APB2ENR_ADC1EN       = APB2ENR_ADC1EN_Msk;
  static constexpr uint32_t APB2ENR_ADC2EN_Pos   = ( 9U );
  static constexpr uint32_t APB2ENR_ADC2EN_Msk   = ( 0x1U << APB2ENR_ADC2EN_Pos );
  static constexpr uint32_t APB2ENR_ADC2EN       = APB2ENR_ADC2EN_Msk;
  static constexpr uint32_t APB2ENR_ADC3EN_Pos   = ( 10U );
  static constexpr uint32_t APB2ENR_ADC3EN_Msk   = ( 0x1U << APB2ENR_ADC3EN_Pos );
  static constexpr uint32_t APB2ENR_ADC3EN       = APB2ENR_ADC3EN_Msk;
  static constexpr uint32_t APB2ENR_SDIOEN_Pos   = ( 11U );
  static constexpr uint32_t APB2ENR_SDIOEN_Msk   = ( 0x1U << APB2ENR_SDIOEN_Pos );
  static constexpr uint32_t APB2ENR_SDIOEN       = APB2ENR_SDIOEN_Msk;
  static constexpr uint32_t APB2ENR_SPI1EN_Pos   = ( 12U );
  static constexpr uint32_t APB2ENR_SPI1EN_Msk   = ( 0x1U << APB2ENR_SPI1EN_Pos );
  static constexpr uint32_t APB2ENR_SPI1EN       = APB2ENR_SPI1EN_Msk;
  static constexpr uint32_t APB2ENR_SPI4EN_Pos   = ( 13U );
  static constexpr uint32_t APB2ENR_SPI4EN_Msk   = ( 0x1U << APB2ENR_SPI4EN_Pos );
  static constexpr uint32_t APB2ENR_SPI4EN       = APB2ENR_SPI4EN_Msk;
  static constexpr uint32_t APB2ENR_SYSCFGEN_Pos = ( 14U );
  static constexpr uint32_t APB2ENR_SYSCFGEN_Msk = ( 0x1U << APB2ENR_SYSCFGEN_Pos );
  static constexpr uint32_t APB2ENR_SYSCFGEN     = APB2ENR_SYSCFGEN_Msk;
  static constexpr uint32_t APB2ENR_TIM9EN_Pos   = ( 16U );
  static constexpr uint32_t APB2ENR_TIM9EN_Msk   = ( 0x1U << APB2ENR_TIM9EN_Pos );
  static constexpr uint32_t APB2ENR_TIM9EN       = APB2ENR_TIM9EN_Msk;
  static constexpr uint32_t APB2ENR_TIM10EN_Pos  = ( 17U );
  static constexpr uint32_t APB2ENR_TIM10EN_Msk  = ( 0x1U << APB2ENR_TIM10EN_Pos );
  static constexpr uint32_t APB2ENR_TIM10EN      = APB2ENR_TIM10EN_Msk;
  static constexpr uint32_t APB2ENR_TIM11EN_Pos  = ( 18U );
  static constexpr uint32_t APB2ENR_TIM11EN_Msk  = ( 0x1U << APB2ENR_TIM11EN_Pos );
  static constexpr uint32_t APB2ENR_TIM11EN      = APB2ENR_TIM11EN_Msk;
  static constexpr uint32_t APB2ENR_SAI1EN_Pos   = ( 22U );
  static constexpr uint32_t APB2ENR_SAI1EN_Msk   = ( 0x1U << APB2ENR_SAI1EN_Pos );
  static constexpr uint32_t APB2ENR_SAI1EN       = APB2ENR_SAI1EN_Msk;
  static constexpr uint32_t APB2ENR_SAI2EN_Pos   = ( 23U );
  static constexpr uint32_t APB2ENR_SAI2EN_Msk   = ( 0x1U << APB2ENR_SAI2EN_Pos );
  static constexpr uint32_t APB2ENR_SAI2EN       = APB2ENR_SAI2EN_Msk;

  /*------------------------------------------------
  AHB1LPENR Register
  ------------------------------------------------*/
  static constexpr uint32_t AHB1LPENR_GPIOALPEN_Pos     = ( 0U );
  static constexpr uint32_t AHB1LPENR_GPIOALPEN_Msk     = ( 0x1U << AHB1LPENR_GPIOALPEN_Pos );
  static constexpr uint32_t AHB1LPENR_GPIOALPEN         = AHB1LPENR_GPIOALPEN_Msk;
  static constexpr uint32_t AHB1LPENR_GPIOBLPEN_Pos     = ( 1U );
  static constexpr uint32_t AHB1LPENR_GPIOBLPEN_Msk     = ( 0x1U << AHB1LPENR_GPIOBLPEN_Pos );
  static constexpr uint32_t AHB1LPENR_GPIOBLPEN         = AHB1LPENR_GPIOBLPEN_Msk;
  static constexpr uint32_t AHB1LPENR_GPIOCLPEN_Pos     = ( 2U );
  static constexpr uint32_t AHB1LPENR_GPIOCLPEN_Msk     = ( 0x1U << AHB1LPENR_GPIOCLPEN_Pos );
  static constexpr uint32_t AHB1LPENR_GPIOCLPEN         = AHB1LPENR_GPIOCLPEN_Msk;
  static constexpr uint32_t AHB1LPENR_GPIODLPEN_Pos     = ( 3U );
  static constexpr uint32_t AHB1LPENR_GPIODLPEN_Msk     = ( 0x1U << AHB1LPENR_GPIODLPEN_Pos );
  static constexpr uint32_t AHB1LPENR_GPIODLPEN         = AHB1LPENR_GPIODLPEN_Msk;
  static constexpr uint32_t AHB1LPENR_GPIOELPEN_Pos     = ( 4U );
  static constexpr uint32_t AHB1LPENR_GPIOELPEN_Msk     = ( 0x1U << AHB1LPENR_GPIOELPEN_Pos );
  static constexpr uint32_t AHB1LPENR_GPIOELPEN         = AHB1LPENR_GPIOELPEN_Msk;
  static constexpr uint32_t AHB1LPENR_GPIOFLPEN_Pos     = ( 5U );
  static constexpr uint32_t AHB1LPENR_GPIOFLPEN_Msk     = ( 0x1U << AHB1LPENR_GPIOFLPEN_Pos );
  static constexpr uint32_t AHB1LPENR_GPIOFLPEN         = AHB1LPENR_GPIOFLPEN_Msk;
  static constexpr uint32_t AHB1LPENR_GPIOGLPEN_Pos     = ( 6U );
  static constexpr uint32_t AHB1LPENR_GPIOGLPEN_Msk     = ( 0x1U << AHB1LPENR_GPIOGLPEN_Pos );
  static constexpr uint32_t AHB1LPENR_GPIOGLPEN         = AHB1LPENR_GPIOGLPEN_Msk;
  static constexpr uint32_t AHB1LPENR_GPIOHLPEN_Pos     = ( 7U );
  static constexpr uint32_t AHB1LPENR_GPIOHLPEN_Msk     = ( 0x1U << AHB1LPENR_GPIOHLPEN_Pos );
  static constexpr uint32_t AHB1LPENR_GPIOHLPEN         = AHB1LPENR_GPIOHLPEN_Msk;
  static constexpr uint32_t AHB1LPENR_CRCLPEN_Pos       = ( 12U );
  static constexpr uint32_t AHB1LPENR_CRCLPEN_Msk       = ( 0x1U << AHB1LPENR_CRCLPEN_Pos );
  static constexpr uint32_t AHB1LPENR_CRCLPEN           = AHB1LPENR_CRCLPEN_Msk;
  static constexpr uint32_t AHB1LPENR_FLITFLPEN_Pos     = ( 15U );
  static constexpr uint32_t AHB1LPENR_FLITFLPEN_Msk     = ( 0x1U << AHB1LPENR_FLITFLPEN_Pos );
  static constexpr uint32_t AHB1LPENR_FLITFLPEN         = AHB1LPENR_FLITFLPEN_Msk;
  static constexpr uint32_t AHB1LPENR_SRAM1LPEN_Pos     = ( 16U );
  static constexpr uint32_t AHB1LPENR_SRAM1LPEN_Msk     = ( 0x1U << AHB1LPENR_SRAM1LPEN_Pos );
  static constexpr uint32_t AHB1LPENR_SRAM1LPEN         = AHB1LPENR_SRAM1LPEN_Msk;
  static constexpr uint32_t AHB1LPENR_SRAM2LPEN_Pos     = ( 17U );
  static constexpr uint32_t AHB1LPENR_SRAM2LPEN_Msk     = ( 0x1U << AHB1LPENR_SRAM2LPEN_Pos );
  static constexpr uint32_t AHB1LPENR_SRAM2LPEN         = AHB1LPENR_SRAM2LPEN_Msk;
  static constexpr uint32_t AHB1LPENR_BKPSRAMLPEN_Pos   = ( 18U );
  static constexpr uint32_t AHB1LPENR_BKPSRAMLPEN_Msk   = ( 0x1U << AHB1LPENR_BKPSRAMLPEN_Pos );
  static constexpr uint32_t AHB1LPENR_BKPSRAMLPEN       = AHB1LPENR_BKPSRAMLPEN_Msk;
  static constexpr uint32_t AHB1LPENR_DMA1LPEN_Pos      = ( 21U );
  static constexpr uint32_t AHB1LPENR_DMA1LPEN_Msk      = ( 0x1U << AHB1LPENR_DMA1LPEN_Pos );
  static constexpr uint32_t AHB1LPENR_DMA1LPEN          = AHB1LPENR_DMA1LPEN_Msk;
  static constexpr uint32_t AHB1LPENR_DMA2LPEN_Pos      = ( 22U );
  static constexpr uint32_t AHB1LPENR_DMA2LPEN_Msk      = ( 0x1U << AHB1LPENR_DMA2LPEN_Pos );
  static constexpr uint32_t AHB1LPENR_DMA2LPEN          = AHB1LPENR_DMA2LPEN_Msk;
  static constexpr uint32_t AHB1LPENR_OTGHSLPEN_Pos     = ( 29U );
  static constexpr uint32_t AHB1LPENR_OTGHSLPEN_Msk     = ( 0x1U << AHB1LPENR_OTGHSLPEN_Pos );
  static constexpr uint32_t AHB1LPENR_OTGHSLPEN         = AHB1LPENR_OTGHSLPEN_Msk;
  static constexpr uint32_t AHB1LPENR_OTGHSULPILPEN_Pos = ( 30U );
  static constexpr uint32_t AHB1LPENR_OTGHSULPILPEN_Msk = ( 0x1U << AHB1LPENR_OTGHSULPILPEN_Pos );
  static constexpr uint32_t AHB1LPENR_OTGHSULPILPEN     = AHB1LPENR_OTGHSULPILPEN_Msk;

  /*------------------------------------------------
  AHB2LPENR Register
  ------------------------------------------------*/
  static constexpr uint32_t AHB2LPENR_DCMILPEN_Pos  = ( 0U );
  static constexpr uint32_t AHB2LPENR_DCMILPEN_Msk  = ( 0x1U << AHB2LPENR_DCMILPEN_Pos );
  static constexpr uint32_t AHB2LPENR_DCMILPEN      = AHB2LPENR_DCMILPEN_Msk;
  static constexpr uint32_t AHB2LPENR_OTGFSLPEN_Pos = ( 7U );
  static constexpr uint32_t AHB2LPENR_OTGFSLPEN_Msk = ( 0x1U << AHB2LPENR_OTGFSLPEN_Pos );
  static constexpr uint32_t AHB2LPENR_OTGFSLPEN     = AHB2LPENR_OTGFSLPEN_Msk;

  /*------------------------------------------------
  AHB3LPENR Register
  ------------------------------------------------*/
  static constexpr uint32_t AHB3LPENR_FMCLPEN_Pos  = ( 0U );
  static constexpr uint32_t AHB3LPENR_FMCLPEN_Msk  = ( 0x1U << AHB3LPENR_FMCLPEN_Pos );
  static constexpr uint32_t AHB3LPENR_FMCLPEN      = AHB3LPENR_FMCLPEN_Msk;
  static constexpr uint32_t AHB3LPENR_QSPILPEN_Pos = ( 1U );
  static constexpr uint32_t AHB3LPENR_QSPILPEN_Msk = ( 0x1U << AHB3LPENR_QSPILPEN_Pos );
  static constexpr uint32_t AHB3LPENR_QSPILPEN     = AHB3LPENR_QSPILPEN_Msk;

  /*------------------------------------------------
  APB1LPENR Register
  ------------------------------------------------*/
  static constexpr uint32_t APB1LPENR_TIM2LPEN_Pos    = ( 0U );
  static constexpr uint32_t APB1LPENR_TIM2LPEN_Msk    = ( 0x1U << APB1LPENR_TIM2LPEN_Pos );
  static constexpr uint32_t APB1LPENR_TIM2LPEN        = APB1LPENR_TIM2LPEN_Msk;
  static constexpr uint32_t APB1LPENR_TIM3LPEN_Pos    = ( 1U );
  static constexpr uint32_t APB1LPENR_TIM3LPEN_Msk    = ( 0x1U << APB1LPENR_TIM3LPEN_Pos );
  static constexpr uint32_t APB1LPENR_TIM3LPEN        = APB1LPENR_TIM3LPEN_Msk;
  static constexpr uint32_t APB1LPENR_TIM4LPEN_Pos    = ( 2U );
  static constexpr uint32_t APB1LPENR_TIM4LPEN_Msk    = ( 0x1U << APB1LPENR_TIM4LPEN_Pos );
  static constexpr uint32_t APB1LPENR_TIM4LPEN        = APB1LPENR_TIM4LPEN_Msk;
  static constexpr uint32_t APB1LPENR_TIM5LPEN_Pos    = ( 3U );
  static constexpr uint32_t APB1LPENR_TIM5LPEN_Msk    = ( 0x1U << APB1LPENR_TIM5LPEN_Pos );
  static constexpr uint32_t APB1LPENR_TIM5LPEN        = APB1LPENR_TIM5LPEN_Msk;
  static constexpr uint32_t APB1LPENR_TIM6LPEN_Pos    = ( 4U );
  static constexpr uint32_t APB1LPENR_TIM6LPEN_Msk    = ( 0x1U << APB1LPENR_TIM6LPEN_Pos );
  static constexpr uint32_t APB1LPENR_TIM6LPEN        = APB1LPENR_TIM6LPEN_Msk;
  static constexpr uint32_t APB1LPENR_TIM7LPEN_Pos    = ( 5U );
  static constexpr uint32_t APB1LPENR_TIM7LPEN_Msk    = ( 0x1U << APB1LPENR_TIM7LPEN_Pos );
  static constexpr uint32_t APB1LPENR_TIM7LPEN        = APB1LPENR_TIM7LPEN_Msk;
  static constexpr uint32_t APB1LPENR_TIM12LPEN_Pos   = ( 6U );
  static constexpr uint32_t APB1LPENR_TIM12LPEN_Msk   = ( 0x1U << APB1LPENR_TIM12LPEN_Pos );
  static constexpr uint32_t APB1LPENR_TIM12LPEN       = APB1LPENR_TIM12LPEN_Msk;
  static constexpr uint32_t APB1LPENR_TIM13LPEN_Pos   = ( 7U );
  static constexpr uint32_t APB1LPENR_TIM13LPEN_Msk   = ( 0x1U << APB1LPENR_TIM13LPEN_Pos );
  static constexpr uint32_t APB1LPENR_TIM13LPEN       = APB1LPENR_TIM13LPEN_Msk;
  static constexpr uint32_t APB1LPENR_TIM14LPEN_Pos   = ( 8U );
  static constexpr uint32_t APB1LPENR_TIM14LPEN_Msk   = ( 0x1U << APB1LPENR_TIM14LPEN_Pos );
  static constexpr uint32_t APB1LPENR_TIM14LPEN       = APB1LPENR_TIM14LPEN_Msk;
  static constexpr uint32_t APB1LPENR_WWDGLPEN_Pos    = ( 11U );
  static constexpr uint32_t APB1LPENR_WWDGLPEN_Msk    = ( 0x1U << APB1LPENR_WWDGLPEN_Pos );
  static constexpr uint32_t APB1LPENR_WWDGLPEN        = APB1LPENR_WWDGLPEN_Msk;
  static constexpr uint32_t APB1LPENR_SPI2LPEN_Pos    = ( 14U );
  static constexpr uint32_t APB1LPENR_SPI2LPEN_Msk    = ( 0x1U << APB1LPENR_SPI2LPEN_Pos );
  static constexpr uint32_t APB1LPENR_SPI2LPEN        = APB1LPENR_SPI2LPEN_Msk;
  static constexpr uint32_t APB1LPENR_SPI3LPEN_Pos    = ( 15U );
  static constexpr uint32_t APB1LPENR_SPI3LPEN_Msk    = ( 0x1U << APB1LPENR_SPI3LPEN_Pos );
  static constexpr uint32_t APB1LPENR_SPI3LPEN        = APB1LPENR_SPI3LPEN_Msk;
  static constexpr uint32_t APB1LPENR_SPDIFRXLPEN_Pos = ( 16U );
  static constexpr uint32_t APB1LPENR_SPDIFRXLPEN_Msk = ( 0x1U << APB1LPENR_SPDIFRXLPEN_Pos );
  static constexpr uint32_t APB1LPENR_SPDIFRXLPEN     = APB1LPENR_SPDIFRXLPEN_Msk;
  static constexpr uint32_t APB1LPENR_USART2LPEN_Pos  = ( 17U );
  static constexpr uint32_t APB1LPENR_USART2LPEN_Msk  = ( 0x1U << APB1LPENR_USART2LPEN_Pos );
  static constexpr uint32_t APB1LPENR_USART2LPEN      = APB1LPENR_USART2LPEN_Msk;
  static constexpr uint32_t APB1LPENR_USART3LPEN_Pos  = ( 18U );
  static constexpr uint32_t APB1LPENR_USART3LPEN_Msk  = ( 0x1U << APB1LPENR_USART3LPEN_Pos );
  static constexpr uint32_t APB1LPENR_USART3LPEN      = APB1LPENR_USART3LPEN_Msk;
  static constexpr uint32_t APB1LPENR_UART4LPEN_Pos   = ( 19U );
  static constexpr uint32_t APB1LPENR_UART4LPEN_Msk   = ( 0x1U << APB1LPENR_UART4LPEN_Pos );
  static constexpr uint32_t APB1LPENR_UART4LPEN       = APB1LPENR_UART4LPEN_Msk;
  static constexpr uint32_t APB1LPENR_UART5LPEN_Pos   = ( 20U );
  static constexpr uint32_t APB1LPENR_UART5LPEN_Msk   = ( 0x1U << APB1LPENR_UART5LPEN_Pos );
  static constexpr uint32_t APB1LPENR_UART5LPEN       = APB1LPENR_UART5LPEN_Msk;
  static constexpr uint32_t APB1LPENR_I2C1LPEN_Pos    = ( 21U );
  static constexpr uint32_t APB1LPENR_I2C1LPEN_Msk    = ( 0x1U << APB1LPENR_I2C1LPEN_Pos );
  static constexpr uint32_t APB1LPENR_I2C1LPEN        = APB1LPENR_I2C1LPEN_Msk;
  static constexpr uint32_t APB1LPENR_I2C2LPEN_Pos    = ( 22U );
  static constexpr uint32_t APB1LPENR_I2C2LPEN_Msk    = ( 0x1U << APB1LPENR_I2C2LPEN_Pos );
  static constexpr uint32_t APB1LPENR_I2C2LPEN        = APB1LPENR_I2C2LPEN_Msk;
  static constexpr uint32_t APB1LPENR_I2C3LPEN_Pos    = ( 23U );
  static constexpr uint32_t APB1LPENR_I2C3LPEN_Msk    = ( 0x1U << APB1LPENR_I2C3LPEN_Pos );
  static constexpr uint32_t APB1LPENR_I2C3LPEN        = APB1LPENR_I2C3LPEN_Msk;
  static constexpr uint32_t APB1LPENR_FMPI2C1LPEN_Pos = ( 24U );
  static constexpr uint32_t APB1LPENR_FMPI2C1LPEN_Msk = ( 0x1U << APB1LPENR_FMPI2C1LPEN_Pos );
  static constexpr uint32_t APB1LPENR_FMPI2C1LPEN     = APB1LPENR_FMPI2C1LPEN_Msk;
  static constexpr uint32_t APB1LPENR_CAN1LPEN_Pos    = ( 25U );
  static constexpr uint32_t APB1LPENR_CAN1LPEN_Msk    = ( 0x1U << APB1LPENR_CAN1LPEN_Pos );
  static constexpr uint32_t APB1LPENR_CAN1LPEN        = APB1LPENR_CAN1LPEN_Msk;
  static constexpr uint32_t APB1LPENR_CAN2LPEN_Pos    = ( 26U );
  static constexpr uint32_t APB1LPENR_CAN2LPEN_Msk    = ( 0x1U << APB1LPENR_CAN2LPEN_Pos );
  static constexpr uint32_t APB1LPENR_CAN2LPEN        = APB1LPENR_CAN2LPEN_Msk;
  static constexpr uint32_t APB1LPENR_CECLPEN_Pos     = ( 27U );
  static constexpr uint32_t APB1LPENR_CECLPEN_Msk     = ( 0x1U << APB1LPENR_CECLPEN_Pos );
  static constexpr uint32_t APB1LPENR_CECLPEN         = APB1LPENR_CECLPEN_Msk;
  static constexpr uint32_t APB1LPENR_PWRLPEN_Pos     = ( 28U );
  static constexpr uint32_t APB1LPENR_PWRLPEN_Msk     = ( 0x1U << APB1LPENR_PWRLPEN_Pos );
  static constexpr uint32_t APB1LPENR_PWRLPEN         = APB1LPENR_PWRLPEN_Msk;
  static constexpr uint32_t APB1LPENR_DACLPEN_Pos     = ( 29U );
  static constexpr uint32_t APB1LPENR_DACLPEN_Msk     = ( 0x1U << APB1LPENR_DACLPEN_Pos );
  static constexpr uint32_t APB1LPENR_DACLPEN         = APB1LPENR_DACLPEN_Msk;

  /*------------------------------------------------
  APB2LPENR Register
  ------------------------------------------------*/
  static constexpr uint32_t APB2LPENR_TIM1LPEN_Pos   = ( 0U );
  static constexpr uint32_t APB2LPENR_TIM1LPEN_Msk   = ( 0x1U << APB2LPENR_TIM1LPEN_Pos );
  static constexpr uint32_t APB2LPENR_TIM1LPEN       = APB2LPENR_TIM1LPEN_Msk;
  static constexpr uint32_t APB2LPENR_TIM8LPEN_Pos   = ( 1U );
  static constexpr uint32_t APB2LPENR_TIM8LPEN_Msk   = ( 0x1U << APB2LPENR_TIM8LPEN_Pos );
  static constexpr uint32_t APB2LPENR_TIM8LPEN       = APB2LPENR_TIM8LPEN_Msk;
  static constexpr uint32_t APB2LPENR_USART1LPEN_Pos = ( 4U );
  static constexpr uint32_t APB2LPENR_USART1LPEN_Msk = ( 0x1U << APB2LPENR_USART1LPEN_Pos );
  static constexpr uint32_t APB2LPENR_USART1LPEN     = APB2LPENR_USART1LPEN_Msk;
  static constexpr uint32_t APB2LPENR_USART6LPEN_Pos = ( 5U );
  static constexpr uint32_t APB2LPENR_USART6LPEN_Msk = ( 0x1U << APB2LPENR_USART6LPEN_Pos );
  static constexpr uint32_t APB2LPENR_USART6LPEN     = APB2LPENR_USART6LPEN_Msk;
  static constexpr uint32_t APB2LPENR_ADC1LPEN_Pos   = ( 8U );
  static constexpr uint32_t APB2LPENR_ADC1LPEN_Msk   = ( 0x1U << APB2LPENR_ADC1LPEN_Pos );
  static constexpr uint32_t APB2LPENR_ADC1LPEN       = APB2LPENR_ADC1LPEN_Msk;
  static constexpr uint32_t APB2LPENR_ADC2LPEN_Pos   = ( 9U );
  static constexpr uint32_t APB2LPENR_ADC2LPEN_Msk   = ( 0x1U << APB2LPENR_ADC2LPEN_Pos );
  static constexpr uint32_t APB2LPENR_ADC2LPEN       = APB2LPENR_ADC2LPEN_Msk;
  static constexpr uint32_t APB2LPENR_ADC3LPEN_Pos   = ( 10U );
  static constexpr uint32_t APB2LPENR_ADC3LPEN_Msk   = ( 0x1U << APB2LPENR_ADC3LPEN_Pos );
  static constexpr uint32_t APB2LPENR_ADC3LPEN       = APB2LPENR_ADC3LPEN_Msk;
  static constexpr uint32_t APB2LPENR_SDIOLPEN_Pos   = ( 11U );
  static constexpr uint32_t APB2LPENR_SDIOLPEN_Msk   = ( 0x1U << APB2LPENR_SDIOLPEN_Pos );
  static constexpr uint32_t APB2LPENR_SDIOLPEN       = APB2LPENR_SDIOLPEN_Msk;
  static constexpr uint32_t APB2LPENR_SPI1LPEN_Pos   = ( 12U );
  static constexpr uint32_t APB2LPENR_SPI1LPEN_Msk   = ( 0x1U << APB2LPENR_SPI1LPEN_Pos );
  static constexpr uint32_t APB2LPENR_SPI1LPEN       = APB2LPENR_SPI1LPEN_Msk;
  static constexpr uint32_t APB2LPENR_SPI4LPEN_Pos   = ( 13U );
  static constexpr uint32_t APB2LPENR_SPI4LPEN_Msk   = ( 0x1U << APB2LPENR_SPI4LPEN_Pos );
  static constexpr uint32_t APB2LPENR_SPI4LPEN       = APB2LPENR_SPI4LPEN_Msk;
  static constexpr uint32_t APB2LPENR_SYSCFGLPEN_Pos = ( 14U );
  static constexpr uint32_t APB2LPENR_SYSCFGLPEN_Msk = ( 0x1U << APB2LPENR_SYSCFGLPEN_Pos );
  static constexpr uint32_t APB2LPENR_SYSCFGLPEN     = APB2LPENR_SYSCFGLPEN_Msk;
  static constexpr uint32_t APB2LPENR_TIM9LPEN_Pos   = ( 16U );
  static constexpr uint32_t APB2LPENR_TIM9LPEN_Msk   = ( 0x1U << APB2LPENR_TIM9LPEN_Pos );
  static constexpr uint32_t APB2LPENR_TIM9LPEN       = APB2LPENR_TIM9LPEN_Msk;
  static constexpr uint32_t APB2LPENR_TIM10LPEN_Pos  = ( 17U );
  static constexpr uint32_t APB2LPENR_TIM10LPEN_Msk  = ( 0x1U << APB2LPENR_TIM10LPEN_Pos );
  static constexpr uint32_t APB2LPENR_TIM10LPEN      = APB2LPENR_TIM10LPEN_Msk;
  static constexpr uint32_t APB2LPENR_TIM11LPEN_Pos  = ( 18U );
  static constexpr uint32_t APB2LPENR_TIM11LPEN_Msk  = ( 0x1U << APB2LPENR_TIM11LPEN_Pos );
  static constexpr uint32_t APB2LPENR_TIM11LPEN      = APB2LPENR_TIM11LPEN_Msk;
  static constexpr uint32_t APB2LPENR_SAI1LPEN_Pos   = ( 22U );
  static constexpr uint32_t APB2LPENR_SAI1LPEN_Msk   = ( 0x1U << APB2LPENR_SAI1LPEN_Pos );
  static constexpr uint32_t APB2LPENR_SAI1LPEN       = APB2LPENR_SAI1LPEN_Msk;
  static constexpr uint32_t APB2LPENR_SAI2LPEN_Pos   = ( 23U );
  static constexpr uint32_t APB2LPENR_SAI2LPEN_Msk   = ( 0x1U << APB2LPENR_SAI2LPEN_Pos );
  static constexpr uint32_t APB2LPENR_SAI2LPEN       = APB2LPENR_SAI2LPEN_Msk;

  /*------------------------------------------------
  BDCR Register
  ------------------------------------------------*/
  static constexpr uint32_t BDCR_LSEON_Pos  = ( 0U );
  static constexpr uint32_t BDCR_LSEON_Msk  = ( 0x1U << BDCR_LSEON_Pos );
  static constexpr uint32_t BDCR_LSEON      = BDCR_LSEON_Msk;
  static constexpr uint32_t BDCR_LSERDY_Pos = ( 1U );
  static constexpr uint32_t BDCR_LSERDY_Msk = ( 0x1U << BDCR_LSERDY_Pos );
  static constexpr uint32_t BDCR_LSERDY     = BDCR_LSERDY_Msk;
  static constexpr uint32_t BDCR_LSEBYP_Pos = ( 2U );
  static constexpr uint32_t BDCR_LSEBYP_Msk = ( 0x1U << BDCR_LSEBYP_Pos );
  static constexpr uint32_t BDCR_LSEBYP     = BDCR_LSEBYP_Msk;
  static constexpr uint32_t BDCR_LSEMOD_Pos = ( 3U );
  static constexpr uint32_t BDCR_LSEMOD_Msk = ( 0x1U << BDCR_LSEMOD_Pos );
  static constexpr uint32_t BDCR_LSEMOD     = BDCR_LSEMOD_Msk;
  static constexpr uint32_t BDCR_RTCSEL_Pos = ( 8U );
  static constexpr uint32_t BDCR_RTCSEL_Msk = ( 0x3U << BDCR_RTCSEL_Pos );
  static constexpr uint32_t BDCR_RTCSEL     = BDCR_RTCSEL_Msk;
  static constexpr uint32_t BDCR_RTCSEL_0   = ( 0x1U << BDCR_RTCSEL_Pos );
  static constexpr uint32_t BDCR_RTCSEL_1   = ( 0x2U << BDCR_RTCSEL_Pos );
  static constexpr uint32_t BDCR_RTCEN_Pos  = ( 15U );
  static constexpr uint32_t BDCR_RTCEN_Msk  = ( 0x1U << BDCR_RTCEN_Pos );
  static constexpr uint32_t BDCR_RTCEN      = BDCR_RTCEN_Msk;
  static constexpr uint32_t BDCR_BDRST_Pos  = ( 16U );
  static constexpr uint32_t BDCR_BDRST_Msk  = ( 0x1U << BDCR_BDRST_Pos );
  static constexpr uint32_t BDCR_BDRST      = BDCR_BDRST_Msk;

  /*------------------------------------------------
  CSR Register
  ------------------------------------------------*/
  static constexpr uint32_t CSR_LSION_Pos    = ( 0U );
  static constexpr uint32_t CSR_LSION_Msk    = ( 0x1U << CSR_LSION_Pos );
  static constexpr uint32_t CSR_LSION        = CSR_LSION_Msk;
  static constexpr uint32_t CSR_LSIRDY_Pos   = ( 1U );
  static constexpr uint32_t CSR_LSIRDY_Msk   = ( 0x1U << CSR_LSIRDY_Pos );
  static constexpr uint32_t CSR_LSIRDY       = CSR_LSIRDY_Msk;
  static constexpr uint32_t CSR_RMVF_Pos     = ( 24U );
  static constexpr uint32_t CSR_RMVF_Msk     = ( 0x1U << CSR_RMVF_Pos );
  static constexpr uint32_t CSR_RMVF         = CSR_RMVF_Msk;
  static constexpr uint32_t CSR_BORRSTF_Pos  = ( 25U );
  static constexpr uint32_t CSR_BORRSTF_Msk  = ( 0x1U << CSR_BORRSTF_Pos );
  static constexpr uint32_t CSR_BORRSTF      = CSR_BORRSTF_Msk;
  static constexpr uint32_t CSR_PINRSTF_Pos  = ( 26U );
  static constexpr uint32_t CSR_PINRSTF_Msk  = ( 0x1U << CSR_PINRSTF_Pos );
  static constexpr uint32_t CSR_PINRSTF      = CSR_PINRSTF_Msk;
  static constexpr uint32_t CSR_PORRSTF_Pos  = ( 27U );
  static constexpr uint32_t CSR_PORRSTF_Msk  = ( 0x1U << CSR_PORRSTF_Pos );
  static constexpr uint32_t CSR_PORRSTF      = CSR_PORRSTF_Msk;
  static constexpr uint32_t CSR_SFTRSTF_Pos  = ( 28U );
  static constexpr uint32_t CSR_SFTRSTF_Msk  = ( 0x1U << CSR_SFTRSTF_Pos );
  static constexpr uint32_t CSR_SFTRSTF      = CSR_SFTRSTF_Msk;
  static constexpr uint32_t CSR_IWDGRSTF_Pos = ( 29U );
  static constexpr uint32_t CSR_IWDGRSTF_Msk = ( 0x1U << CSR_IWDGRSTF_Pos );
  static constexpr uint32_t CSR_IWDGRSTF     = CSR_IWDGRSTF_Msk;
  static constexpr uint32_t CSR_WWDGRSTF_Pos = ( 30U );
  static constexpr uint32_t CSR_WWDGRSTF_Msk = ( 0x1U << CSR_WWDGRSTF_Pos );
  static constexpr uint32_t CSR_WWDGRSTF     = CSR_WWDGRSTF_Msk;
  static constexpr uint32_t CSR_LPWRRSTF_Pos = ( 31U );
  static constexpr uint32_t CSR_LPWRRSTF_Msk = ( 0x1U << CSR_LPWRRSTF_Pos );
  static constexpr uint32_t CSR_LPWRRSTF     = CSR_LPWRRSTF_Msk;

  /*------------------------------------------------
  SSCGR Register
  ------------------------------------------------*/
  static constexpr uint32_t SSCGR_MODPER_Pos    = ( 0U );
  static constexpr uint32_t SSCGR_MODPER_Msk    = ( 0x1FFFU << SSCGR_MODPER_Pos );
  static constexpr uint32_t SSCGR_MODPER        = SSCGR_MODPER_Msk;
  static constexpr uint32_t SSCGR_INCSTEP_Pos   = ( 13U );
  static constexpr uint32_t SSCGR_INCSTEP_Msk   = ( 0x7FFFU << SSCGR_INCSTEP_Pos );
  static constexpr uint32_t SSCGR_INCSTEP       = SSCGR_INCSTEP_Msk;
  static constexpr uint32_t SSCGR_SPREADSEL_Pos = ( 30U );
  static constexpr uint32_t SSCGR_SPREADSEL_Msk = ( 0x1U << SSCGR_SPREADSEL_Pos );
  static constexpr uint32_t SSCGR_SPREADSEL     = SSCGR_SPREADSEL_Msk;
  static constexpr uint32_t SSCGR_SSCGEN_Pos    = ( 31U );
  static constexpr uint32_t SSCGR_SSCGEN_Msk    = ( 0x1U << SSCGR_SSCGEN_Pos );
  static constexpr uint32_t SSCGR_SSCGEN        = SSCGR_SSCGEN_Msk;

  /*------------------------------------------------
  PLLI2SCFGR Register
  ------------------------------------------------*/
  static constexpr uint32_t PLLI2SCFGR_PLLI2SM_Pos = ( 0U );
  static constexpr uint32_t PLLI2SCFGR_PLLI2SM_Msk = ( 0x3FU << PLLI2SCFGR_PLLI2SM_Pos );
  static constexpr uint32_t PLLI2SCFGR_PLLI2SM     = PLLI2SCFGR_PLLI2SM_Msk;
  static constexpr uint32_t PLLI2SCFGR_PLLI2SM_0   = ( 0x01U << PLLI2SCFGR_PLLI2SM_Pos );
  static constexpr uint32_t PLLI2SCFGR_PLLI2SM_1   = ( 0x02U << PLLI2SCFGR_PLLI2SM_Pos );
  static constexpr uint32_t PLLI2SCFGR_PLLI2SM_2   = ( 0x04U << PLLI2SCFGR_PLLI2SM_Pos );
  static constexpr uint32_t PLLI2SCFGR_PLLI2SM_3   = ( 0x08U << PLLI2SCFGR_PLLI2SM_Pos );
  static constexpr uint32_t PLLI2SCFGR_PLLI2SM_4   = ( 0x10U << PLLI2SCFGR_PLLI2SM_Pos );
  static constexpr uint32_t PLLI2SCFGR_PLLI2SM_5   = ( 0x20U << PLLI2SCFGR_PLLI2SM_Pos );
  static constexpr uint32_t PLLI2SCFGR_PLLI2SN_Pos = ( 6U );
  static constexpr uint32_t PLLI2SCFGR_PLLI2SN_Msk = ( 0x1FFU << PLLI2SCFGR_PLLI2SN_Pos );
  static constexpr uint32_t PLLI2SCFGR_PLLI2SN     = PLLI2SCFGR_PLLI2SN_Msk;
  static constexpr uint32_t PLLI2SCFGR_PLLI2SN_0   = ( 0x001U << PLLI2SCFGR_PLLI2SN_Pos );
  static constexpr uint32_t PLLI2SCFGR_PLLI2SN_1   = ( 0x002U << PLLI2SCFGR_PLLI2SN_Pos );
  static constexpr uint32_t PLLI2SCFGR_PLLI2SN_2   = ( 0x004U << PLLI2SCFGR_PLLI2SN_Pos );
  static constexpr uint32_t PLLI2SCFGR_PLLI2SN_3   = ( 0x008U << PLLI2SCFGR_PLLI2SN_Pos );
  static constexpr uint32_t PLLI2SCFGR_PLLI2SN_4   = ( 0x010U << PLLI2SCFGR_PLLI2SN_Pos );
  static constexpr uint32_t PLLI2SCFGR_PLLI2SN_5   = ( 0x020U << PLLI2SCFGR_PLLI2SN_Pos );
  static constexpr uint32_t PLLI2SCFGR_PLLI2SN_6   = ( 0x040U << PLLI2SCFGR_PLLI2SN_Pos );
  static constexpr uint32_t PLLI2SCFGR_PLLI2SN_7   = ( 0x080U << PLLI2SCFGR_PLLI2SN_Pos );
  static constexpr uint32_t PLLI2SCFGR_PLLI2SN_8   = ( 0x100U << PLLI2SCFGR_PLLI2SN_Pos );
  static constexpr uint32_t PLLI2SCFGR_PLLI2SP_Pos = ( 16U );
  static constexpr uint32_t PLLI2SCFGR_PLLI2SP_Msk = ( 0x3U << PLLI2SCFGR_PLLI2SP_Pos );
  static constexpr uint32_t PLLI2SCFGR_PLLI2SP     = PLLI2SCFGR_PLLI2SP_Msk;
  static constexpr uint32_t PLLI2SCFGR_PLLI2SP_0   = ( 0x1U << PLLI2SCFGR_PLLI2SP_Pos );
  static constexpr uint32_t PLLI2SCFGR_PLLI2SP_1   = ( 0x2U << PLLI2SCFGR_PLLI2SP_Pos );
  static constexpr uint32_t PLLI2SCFGR_PLLI2SQ_Pos = ( 24U );
  static constexpr uint32_t PLLI2SCFGR_PLLI2SQ_Msk = ( 0xFU << PLLI2SCFGR_PLLI2SQ_Pos );
  static constexpr uint32_t PLLI2SCFGR_PLLI2SQ     = PLLI2SCFGR_PLLI2SQ_Msk;
  static constexpr uint32_t PLLI2SCFGR_PLLI2SQ_0   = ( 0x1U << PLLI2SCFGR_PLLI2SQ_Pos );
  static constexpr uint32_t PLLI2SCFGR_PLLI2SQ_1   = ( 0x2U << PLLI2SCFGR_PLLI2SQ_Pos );
  static constexpr uint32_t PLLI2SCFGR_PLLI2SQ_2   = ( 0x4U << PLLI2SCFGR_PLLI2SQ_Pos );
  static constexpr uint32_t PLLI2SCFGR_PLLI2SQ_3   = ( 0x8U << PLLI2SCFGR_PLLI2SQ_Pos );
  static constexpr uint32_t PLLI2SCFGR_PLLI2SR_Pos = ( 28U );
  static constexpr uint32_t PLLI2SCFGR_PLLI2SR_Msk = ( 0x7U << PLLI2SCFGR_PLLI2SR_Pos );
  static constexpr uint32_t PLLI2SCFGR_PLLI2SR     = PLLI2SCFGR_PLLI2SR_Msk;
  static constexpr uint32_t PLLI2SCFGR_PLLI2SR_0   = ( 0x1U << PLLI2SCFGR_PLLI2SR_Pos );
  static constexpr uint32_t PLLI2SCFGR_PLLI2SR_1   = ( 0x2U << PLLI2SCFGR_PLLI2SR_Pos );
  static constexpr uint32_t PLLI2SCFGR_PLLI2SR_2   = ( 0x4U << PLLI2SCFGR_PLLI2SR_Pos );

  /*------------------------------------------------
  PLLSAICFGR Register
  ------------------------------------------------*/
  static constexpr uint32_t PLLSAICFGR_PLLSAIM_Pos = ( 0U );
  static constexpr uint32_t PLLSAICFGR_PLLSAIM_Msk = ( 0x3FU << PLLSAICFGR_PLLSAIM_Pos );
  static constexpr uint32_t PLLSAICFGR_PLLSAIM     = PLLSAICFGR_PLLSAIM_Msk;
  static constexpr uint32_t PLLSAICFGR_PLLSAIM_0   = ( 0x01U << PLLSAICFGR_PLLSAIM_Pos );
  static constexpr uint32_t PLLSAICFGR_PLLSAIM_1   = ( 0x02U << PLLSAICFGR_PLLSAIM_Pos );
  static constexpr uint32_t PLLSAICFGR_PLLSAIM_2   = ( 0x04U << PLLSAICFGR_PLLSAIM_Pos );
  static constexpr uint32_t PLLSAICFGR_PLLSAIM_3   = ( 0x08U << PLLSAICFGR_PLLSAIM_Pos );
  static constexpr uint32_t PLLSAICFGR_PLLSAIM_4   = ( 0x10U << PLLSAICFGR_PLLSAIM_Pos );
  static constexpr uint32_t PLLSAICFGR_PLLSAIM_5   = ( 0x20U << PLLSAICFGR_PLLSAIM_Pos );
  static constexpr uint32_t PLLSAICFGR_PLLSAIN_Pos = ( 6U );
  static constexpr uint32_t PLLSAICFGR_PLLSAIN_Msk = ( 0x1FFU << PLLSAICFGR_PLLSAIN_Pos );
  static constexpr uint32_t PLLSAICFGR_PLLSAIN     = PLLSAICFGR_PLLSAIN_Msk;
  static constexpr uint32_t PLLSAICFGR_PLLSAIN_0   = ( 0x001U << PLLSAICFGR_PLLSAIN_Pos );
  static constexpr uint32_t PLLSAICFGR_PLLSAIN_1   = ( 0x002U << PLLSAICFGR_PLLSAIN_Pos );
  static constexpr uint32_t PLLSAICFGR_PLLSAIN_2   = ( 0x004U << PLLSAICFGR_PLLSAIN_Pos );
  static constexpr uint32_t PLLSAICFGR_PLLSAIN_3   = ( 0x008U << PLLSAICFGR_PLLSAIN_Pos );
  static constexpr uint32_t PLLSAICFGR_PLLSAIN_4   = ( 0x010U << PLLSAICFGR_PLLSAIN_Pos );
  static constexpr uint32_t PLLSAICFGR_PLLSAIN_5   = ( 0x020U << PLLSAICFGR_PLLSAIN_Pos );
  static constexpr uint32_t PLLSAICFGR_PLLSAIN_6   = ( 0x040U << PLLSAICFGR_PLLSAIN_Pos );
  static constexpr uint32_t PLLSAICFGR_PLLSAIN_7   = ( 0x080U << PLLSAICFGR_PLLSAIN_Pos );
  static constexpr uint32_t PLLSAICFGR_PLLSAIN_8   = ( 0x100U << PLLSAICFGR_PLLSAIN_Pos );
  static constexpr uint32_t PLLSAICFGR_PLLSAIP_Pos = ( 16U );
  static constexpr uint32_t PLLSAICFGR_PLLSAIP_Msk = ( 0x3U << PLLSAICFGR_PLLSAIP_Pos );
  static constexpr uint32_t PLLSAICFGR_PLLSAIP     = PLLSAICFGR_PLLSAIP_Msk;
  static constexpr uint32_t PLLSAICFGR_PLLSAIP_0   = ( 0x1U << PLLSAICFGR_PLLSAIP_Pos );
  static constexpr uint32_t PLLSAICFGR_PLLSAIP_1   = ( 0x2U << PLLSAICFGR_PLLSAIP_Pos );
  static constexpr uint32_t PLLSAICFGR_PLLSAIQ_Pos = ( 24U );
  static constexpr uint32_t PLLSAICFGR_PLLSAIQ_Msk = ( 0xFU << PLLSAICFGR_PLLSAIQ_Pos );
  static constexpr uint32_t PLLSAICFGR_PLLSAIQ     = PLLSAICFGR_PLLSAIQ_Msk;
  static constexpr uint32_t PLLSAICFGR_PLLSAIQ_0   = ( 0x1U << PLLSAICFGR_PLLSAIQ_Pos );
  static constexpr uint32_t PLLSAICFGR_PLLSAIQ_1   = ( 0x2U << PLLSAICFGR_PLLSAIQ_Pos );
  static constexpr uint32_t PLLSAICFGR_PLLSAIQ_2   = ( 0x4U << PLLSAICFGR_PLLSAIQ_Pos );
  static constexpr uint32_t PLLSAICFGR_PLLSAIQ_3   = ( 0x8U << PLLSAICFGR_PLLSAIQ_Pos );

  /*------------------------------------------------
  DCKCFGR Register
  ------------------------------------------------*/
  static constexpr uint32_t DCKCFGR_PLLI2SDIVQ_Pos = ( 0U );
  static constexpr uint32_t DCKCFGR_PLLI2SDIVQ_Msk = ( 0x1FU << DCKCFGR_PLLI2SDIVQ_Pos );
  static constexpr uint32_t DCKCFGR_PLLI2SDIVQ     = DCKCFGR_PLLI2SDIVQ_Msk;
  static constexpr uint32_t DCKCFGR_PLLI2SDIVQ_0   = ( 0x01U << DCKCFGR_PLLI2SDIVQ_Pos );
  static constexpr uint32_t DCKCFGR_PLLI2SDIVQ_1   = ( 0x02U << DCKCFGR_PLLI2SDIVQ_Pos );
  static constexpr uint32_t DCKCFGR_PLLI2SDIVQ_2   = ( 0x04U << DCKCFGR_PLLI2SDIVQ_Pos );
  static constexpr uint32_t DCKCFGR_PLLI2SDIVQ_3   = ( 0x08U << DCKCFGR_PLLI2SDIVQ_Pos );
  static constexpr uint32_t DCKCFGR_PLLI2SDIVQ_4   = ( 0x10U << DCKCFGR_PLLI2SDIVQ_Pos );
  static constexpr uint32_t DCKCFGR_PLLSAIDIVQ_Pos = ( 8U );
  static constexpr uint32_t DCKCFGR_PLLSAIDIVQ_Msk = ( 0x1FU << DCKCFGR_PLLSAIDIVQ_Pos );
  static constexpr uint32_t DCKCFGR_PLLSAIDIVQ     = DCKCFGR_PLLSAIDIVQ_Msk;
  static constexpr uint32_t DCKCFGR_PLLSAIDIVQ_0   = ( 0x01U << DCKCFGR_PLLSAIDIVQ_Pos );
  static constexpr uint32_t DCKCFGR_PLLSAIDIVQ_1   = ( 0x02U << DCKCFGR_PLLSAIDIVQ_Pos );
  static constexpr uint32_t DCKCFGR_PLLSAIDIVQ_2   = ( 0x04U << DCKCFGR_PLLSAIDIVQ_Pos );
  static constexpr uint32_t DCKCFGR_PLLSAIDIVQ_3   = ( 0x08U << DCKCFGR_PLLSAIDIVQ_Pos );
  static constexpr uint32_t DCKCFGR_PLLSAIDIVQ_4   = ( 0x10U << DCKCFGR_PLLSAIDIVQ_Pos );
  static constexpr uint32_t DCKCFGR_SAI1SRC_Pos    = ( 20U );
  static constexpr uint32_t DCKCFGR_SAI1SRC_Msk    = ( 0x3U << DCKCFGR_SAI1SRC_Pos );
  static constexpr uint32_t DCKCFGR_SAI1SRC        = DCKCFGR_SAI1SRC_Msk;
  static constexpr uint32_t DCKCFGR_SAI1SRC_0      = ( 0x1U << DCKCFGR_SAI1SRC_Pos );
  static constexpr uint32_t DCKCFGR_SAI1SRC_1      = ( 0x2U << DCKCFGR_SAI1SRC_Pos );
  static constexpr uint32_t DCKCFGR_SAI2SRC_Pos    = ( 22U );
  static constexpr uint32_t DCKCFGR_SAI2SRC_Msk    = ( 0x3U << DCKCFGR_SAI2SRC_Pos );
  static constexpr uint32_t DCKCFGR_SAI2SRC        = DCKCFGR_SAI2SRC_Msk;
  static constexpr uint32_t DCKCFGR_SAI2SRC_0      = ( 0x1U << DCKCFGR_SAI2SRC_Pos );
  static constexpr uint32_t DCKCFGR_SAI2SRC_1      = ( 0x2U << DCKCFGR_SAI2SRC_Pos );
  static constexpr uint32_t DCKCFGR_TIMPRE_Pos     = ( 24U );
  static constexpr uint32_t DCKCFGR_TIMPRE_Msk     = ( 0x1U << DCKCFGR_TIMPRE_Pos );
  static constexpr uint32_t DCKCFGR_TIMPRE         = DCKCFGR_TIMPRE_Msk;
  static constexpr uint32_t DCKCFGR_I2S1SRC_Pos    = ( 25U );
  static constexpr uint32_t DCKCFGR_I2S1SRC_Msk    = ( 0x3U << DCKCFGR_I2S1SRC_Pos );
  static constexpr uint32_t DCKCFGR_I2S1SRC        = DCKCFGR_I2S1SRC_Msk;
  static constexpr uint32_t DCKCFGR_I2S1SRC_0      = ( 0x1U << DCKCFGR_I2S1SRC_Pos );
  static constexpr uint32_t DCKCFGR_I2S1SRC_1      = ( 0x2U << DCKCFGR_I2S1SRC_Pos );
  static constexpr uint32_t DCKCFGR_I2S2SRC_Pos    = ( 27U );
  static constexpr uint32_t DCKCFGR_I2S2SRC_Msk    = ( 0x3U << DCKCFGR_I2S2SRC_Pos );
  static constexpr uint32_t DCKCFGR_I2S2SRC        = DCKCFGR_I2S2SRC_Msk;
  static constexpr uint32_t DCKCFGR_I2S2SRC_0      = ( 0x1U << DCKCFGR_I2S2SRC_Pos );
  static constexpr uint32_t DCKCFGR_I2S2SRC_1      = ( 0x2U << DCKCFGR_I2S2SRC_Pos );

  /*------------------------------------------------
  CKGATENR Register
  ------------------------------------------------*/
  static constexpr uint32_t CKGATENR_AHB2APB1_CKEN_Pos = ( 0U );
  static constexpr uint32_t CKGATENR_AHB2APB1_CKEN_Msk = ( 0x1U << CKGATENR_AHB2APB1_CKEN_Pos );
  static constexpr uint32_t CKGATENR_AHB2APB1_CKEN     = CKGATENR_AHB2APB1_CKEN_Msk;
  static constexpr uint32_t CKGATENR_AHB2APB2_CKEN_Pos = ( 1U );
  static constexpr uint32_t CKGATENR_AHB2APB2_CKEN_Msk = ( 0x1U << CKGATENR_AHB2APB2_CKEN_Pos );
  static constexpr uint32_t CKGATENR_AHB2APB2_CKEN     = CKGATENR_AHB2APB2_CKEN_Msk;
  static constexpr uint32_t CKGATENR_CM4DBG_CKEN_Pos   = ( 2U );
  static constexpr uint32_t CKGATENR_CM4DBG_CKEN_Msk   = ( 0x1U << CKGATENR_CM4DBG_CKEN_Pos );
  static constexpr uint32_t CKGATENR_CM4DBG_CKEN       = CKGATENR_CM4DBG_CKEN_Msk;
  static constexpr uint32_t CKGATENR_SPARE_CKEN_Pos    = ( 3U );
  static constexpr uint32_t CKGATENR_SPARE_CKEN_Msk    = ( 0x1U << CKGATENR_SPARE_CKEN_Pos );
  static constexpr uint32_t CKGATENR_SPARE_CKEN        = CKGATENR_SPARE_CKEN_Msk;
  static constexpr uint32_t CKGATENR_SRAM_CKEN_Pos     = ( 4U );
  static constexpr uint32_t CKGATENR_SRAM_CKEN_Msk     = ( 0x1U << CKGATENR_SRAM_CKEN_Pos );
  static constexpr uint32_t CKGATENR_SRAM_CKEN         = CKGATENR_SRAM_CKEN_Msk;
  static constexpr uint32_t CKGATENR_FLITF_CKEN_Pos    = ( 5U );
  static constexpr uint32_t CKGATENR_FLITF_CKEN_Msk    = ( 0x1U << CKGATENR_FLITF_CKEN_Pos );
  static constexpr uint32_t CKGATENR_FLITF_CKEN        = CKGATENR_FLITF_CKEN_Msk;
  static constexpr uint32_t CKGATENR_CKEN_Pos      = ( 6U );
  static constexpr uint32_t CKGATENR_CKEN_Msk      = ( 0x1U << CKGATENR_CKEN_Pos );
  static constexpr uint32_t CKGATENR_CKEN          = CKGATENR_CKEN_Msk;

  /*------------------------------------------------
  DCKCFGR2 Register
  ------------------------------------------------*/
  static constexpr uint32_t DCKCFGR2_FMPI2C1SEL_Pos = ( 22U );
  static constexpr uint32_t DCKCFGR2_FMPI2C1SEL_Msk = ( 0x3U << DCKCFGR2_FMPI2C1SEL_Pos );
  static constexpr uint32_t DCKCFGR2_FMPI2C1SEL     = DCKCFGR2_FMPI2C1SEL_Msk;
  static constexpr uint32_t DCKCFGR2_FMPI2C1SEL_0   = ( 0x1U << DCKCFGR2_FMPI2C1SEL_Pos );
  static constexpr uint32_t DCKCFGR2_FMPI2C1SEL_1   = ( 0x2U << DCKCFGR2_FMPI2C1SEL_Pos );
  static constexpr uint32_t DCKCFGR2_CECSEL_Pos     = ( 26U );
  static constexpr uint32_t DCKCFGR2_CECSEL_Msk     = ( 0x1U << DCKCFGR2_CECSEL_Pos );
  static constexpr uint32_t DCKCFGR2_CECSEL         = DCKCFGR2_CECSEL_Msk;
  static constexpr uint32_t DCKCFGR2_CK48MSEL_Pos   = ( 27U );
  static constexpr uint32_t DCKCFGR2_CK48MSEL_Msk   = ( 0x1U << DCKCFGR2_CK48MSEL_Pos );
  static constexpr uint32_t DCKCFGR2_CK48MSEL       = DCKCFGR2_CK48MSEL_Msk;
  static constexpr uint32_t DCKCFGR2_SDIOSEL_Pos    = ( 28U );
  static constexpr uint32_t DCKCFGR2_SDIOSEL_Msk    = ( 0x1U << DCKCFGR2_SDIOSEL_Pos );
  static constexpr uint32_t DCKCFGR2_SDIOSEL        = DCKCFGR2_SDIOSEL_Msk;
  static constexpr uint32_t DCKCFGR2_SPDIFRXSEL_Pos = ( 29U );
  static constexpr uint32_t DCKCFGR2_SPDIFRXSEL_Msk = ( 0x1U << DCKCFGR2_SPDIFRXSEL_Pos );
  static constexpr uint32_t DCKCFGR2_SPDIFRXSEL     = DCKCFGR2_SPDIFRXSEL_Msk;

  static constexpr uint32_t PLLCFGR_RST_VALUE    = 0x24003010U;
  static constexpr uint32_t PLLI2SCFGR_RST_VALUE = 0x24003010U;
  static constexpr uint32_t PLLSAICFGR_RST_VALUE = 0x04003010U;
  static constexpr uint32_t MAX_FREQUENCY        = 180000000U; /*!< Max frequency of family in Hz*/
  static constexpr uint32_t MAX_FREQUENCY_SCALE1 =
      MAX_FREQUENCY;                                           /*!< Maximum frequency for system clock at power scale1, in Hz */
  static constexpr uint32_t MAX_FREQUENCY_SCALE2 = 168000000U; /*!< Maximum frequency for system clock at power scale2, in Hz */
  static constexpr uint32_t MAX_FREQUENCY_SCALE3 = 120000000U; /*!< Maximum frequency for system clock at power scale3, in Hz */
  static constexpr uint32_t PLLVCO_OUTPUT_MIN    = 100000000U; /*!< Frequency min for PLLVCO output, in Hz */
  static constexpr uint32_t PLLVCO_INPUT_MIN     = 950000U;    /*!< Frequency min for PLLVCO input, in Hz  */
  static constexpr uint32_t PLLVCO_INPUT_MAX     = 2100000U;   /*!< Frequency max for PLLVCO input, in Hz  */
  static constexpr uint32_t PLLVCO_OUTPUT_MAX    = 432000000U; /*!< Frequency max for PLLVCO output, in Hz */
  static constexpr uint32_t PLLN_MIN_VALUE       = 50U;
  static constexpr uint32_t PLLN_MAX_VALUE       = 432U;

}    // namespace Thor::Driver::RCC

#endif /* !THOR_HW_RCC_REGISTER_HPP */