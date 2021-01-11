/********************************************************************************
 *  File Name:
 *    hw_exti_register_stm32f4xxxx.hpp
 *
 *  Description:
 *    EXTI register definitions for the STM32F4xxxx series chips
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_HW_EXTI_REGISTER_STM32F4XXXX_HPP
#define THOR_HW_EXTI_REGISTER_STM32F4XXXX_HPP

/* Chimera Includes */
#include <Chimera/common>

/* Driver Includes */
#include <Thor/lld/stm32f4x/system/variant/sys_memory_map_stm32f446xx.hpp>


namespace Thor::LLD::EXTI
{
  /*-------------------------------------------------------------------------------
  Peripheral Instance Memory Map
  -------------------------------------------------------------------------------*/
  static constexpr uint32_t EXTI1_BASE_ADDR = Thor::System::MemoryMap::EXTI_PERIPH_START_ADDRESS;

  /*-------------------------------------------------------------------------------
  Peripheral Register Definitions
  -------------------------------------------------------------------------------*/
  /*******************  Bit definition for IMR1 register  ******************/
  static constexpr Reg32_t IMR1_IM0_Pos  = ( 0U );
  static constexpr Reg32_t IMR1_IM0_Msk  = ( 0x1UL << IMR1_IM0_Pos );
  static constexpr Reg32_t IMR1_IM0      = IMR1_IM0_Msk;
  static constexpr Reg32_t IMR1_IM1_Pos  = ( 1U );
  static constexpr Reg32_t IMR1_IM1_Msk  = ( 0x1UL << IMR1_IM1_Pos );
  static constexpr Reg32_t IMR1_IM1      = IMR1_IM1_Msk;
  static constexpr Reg32_t IMR1_IM2_Pos  = ( 2U );
  static constexpr Reg32_t IMR1_IM2_Msk  = ( 0x1UL << IMR1_IM2_Pos );
  static constexpr Reg32_t IMR1_IM2      = IMR1_IM2_Msk;
  static constexpr Reg32_t IMR1_IM3_Pos  = ( 3U );
  static constexpr Reg32_t IMR1_IM3_Msk  = ( 0x1UL << IMR1_IM3_Pos );
  static constexpr Reg32_t IMR1_IM3      = IMR1_IM3_Msk;
  static constexpr Reg32_t IMR1_IM4_Pos  = ( 4U );
  static constexpr Reg32_t IMR1_IM4_Msk  = ( 0x1UL << IMR1_IM4_Pos );
  static constexpr Reg32_t IMR1_IM4      = IMR1_IM4_Msk;
  static constexpr Reg32_t IMR1_IM5_Pos  = ( 5U );
  static constexpr Reg32_t IMR1_IM5_Msk  = ( 0x1UL << IMR1_IM5_Pos );
  static constexpr Reg32_t IMR1_IM5      = IMR1_IM5_Msk;
  static constexpr Reg32_t IMR1_IM6_Pos  = ( 6U );
  static constexpr Reg32_t IMR1_IM6_Msk  = ( 0x1UL << IMR1_IM6_Pos );
  static constexpr Reg32_t IMR1_IM6      = IMR1_IM6_Msk;
  static constexpr Reg32_t IMR1_IM7_Pos  = ( 7U );
  static constexpr Reg32_t IMR1_IM7_Msk  = ( 0x1UL << IMR1_IM7_Pos );
  static constexpr Reg32_t IMR1_IM7      = IMR1_IM7_Msk;
  static constexpr Reg32_t IMR1_IM8_Pos  = ( 8U );
  static constexpr Reg32_t IMR1_IM8_Msk  = ( 0x1UL << IMR1_IM8_Pos );
  static constexpr Reg32_t IMR1_IM8      = IMR1_IM8_Msk;
  static constexpr Reg32_t IMR1_IM9_Pos  = ( 9U );
  static constexpr Reg32_t IMR1_IM9_Msk  = ( 0x1UL << IMR1_IM9_Pos );
  static constexpr Reg32_t IMR1_IM9      = IMR1_IM9_Msk;
  static constexpr Reg32_t IMR1_IM10_Pos = ( 10U );
  static constexpr Reg32_t IMR1_IM10_Msk = ( 0x1UL << IMR1_IM10_Pos );
  static constexpr Reg32_t IMR1_IM10     = IMR1_IM10_Msk;
  static constexpr Reg32_t IMR1_IM11_Pos = ( 11U );
  static constexpr Reg32_t IMR1_IM11_Msk = ( 0x1UL << IMR1_IM11_Pos );
  static constexpr Reg32_t IMR1_IM11     = IMR1_IM11_Msk;
  static constexpr Reg32_t IMR1_IM12_Pos = ( 12U );
  static constexpr Reg32_t IMR1_IM12_Msk = ( 0x1UL << IMR1_IM12_Pos );
  static constexpr Reg32_t IMR1_IM12     = IMR1_IM12_Msk;
  static constexpr Reg32_t IMR1_IM13_Pos = ( 13U );
  static constexpr Reg32_t IMR1_IM13_Msk = ( 0x1UL << IMR1_IM13_Pos );
  static constexpr Reg32_t IMR1_IM13     = IMR1_IM13_Msk;
  static constexpr Reg32_t IMR1_IM14_Pos = ( 14U );
  static constexpr Reg32_t IMR1_IM14_Msk = ( 0x1UL << IMR1_IM14_Pos );
  static constexpr Reg32_t IMR1_IM14     = IMR1_IM14_Msk;
  static constexpr Reg32_t IMR1_IM15_Pos = ( 15U );
  static constexpr Reg32_t IMR1_IM15_Msk = ( 0x1UL << IMR1_IM15_Pos );
  static constexpr Reg32_t IMR1_IM15     = IMR1_IM15_Msk;
  static constexpr Reg32_t IMR1_IM16_Pos = ( 16U );
  static constexpr Reg32_t IMR1_IM16_Msk = ( 0x1UL << IMR1_IM16_Pos );
  static constexpr Reg32_t IMR1_IM16     = IMR1_IM16_Msk;
  static constexpr Reg32_t IMR1_IM17_Pos = ( 17U );
  static constexpr Reg32_t IMR1_IM17_Msk = ( 0x1UL << IMR1_IM17_Pos );
  static constexpr Reg32_t IMR1_IM17     = IMR1_IM17_Msk;
  static constexpr Reg32_t IMR1_IM18_Pos = ( 18U );
  static constexpr Reg32_t IMR1_IM18_Msk = ( 0x1UL << IMR1_IM18_Pos );
  static constexpr Reg32_t IMR1_IM18     = IMR1_IM18_Msk;
  static constexpr Reg32_t IMR1_IM20_Pos = ( 20U );
  static constexpr Reg32_t IMR1_IM20_Msk = ( 0x1UL << IMR1_IM20_Pos );
  static constexpr Reg32_t IMR1_IM20     = IMR1_IM20_Msk;
  static constexpr Reg32_t IMR1_IM21_Pos = ( 21U );
  static constexpr Reg32_t IMR1_IM21_Msk = ( 0x1UL << IMR1_IM21_Pos );
  static constexpr Reg32_t IMR1_IM21     = IMR1_IM21_Msk;
  static constexpr Reg32_t IMR1_IM22_Pos = ( 22U );
  static constexpr Reg32_t IMR1_IM22_Msk = ( 0x1UL << IMR1_IM22_Pos );
  static constexpr Reg32_t IMR1_IM22     = IMR1_IM22_Msk;

  static constexpr Reg32_t IMR1_IM_Pos = ( 0U );
  static constexpr Reg32_t IMR1_IM_Msk = ( 0x9FFFFFFFUL << IMR1_IM_Pos );
  static constexpr Reg32_t IMR1_IM     = IMR1_IM_Msk;

  /*******************  Bit definition for EMR1 register  ******************/
  static constexpr Reg32_t EMR1_EM0_Pos  = ( 0U );
  static constexpr Reg32_t EMR1_EM0_Msk  = ( 0x1UL << EMR1_EM0_Pos );
  static constexpr Reg32_t EMR1_EM0      = EMR1_EM0_Msk;
  static constexpr Reg32_t EMR1_EM1_Pos  = ( 1U );
  static constexpr Reg32_t EMR1_EM1_Msk  = ( 0x1UL << EMR1_EM1_Pos );
  static constexpr Reg32_t EMR1_EM1      = EMR1_EM1_Msk;
  static constexpr Reg32_t EMR1_EM2_Pos  = ( 2U );
  static constexpr Reg32_t EMR1_EM2_Msk  = ( 0x1UL << EMR1_EM2_Pos );
  static constexpr Reg32_t EMR1_EM2      = EMR1_EM2_Msk;
  static constexpr Reg32_t EMR1_EM3_Pos  = ( 3U );
  static constexpr Reg32_t EMR1_EM3_Msk  = ( 0x1UL << EMR1_EM3_Pos );
  static constexpr Reg32_t EMR1_EM3      = EMR1_EM3_Msk;
  static constexpr Reg32_t EMR1_EM4_Pos  = ( 4U );
  static constexpr Reg32_t EMR1_EM4_Msk  = ( 0x1UL << EMR1_EM4_Pos );
  static constexpr Reg32_t EMR1_EM4      = EMR1_EM4_Msk;
  static constexpr Reg32_t EMR1_EM5_Pos  = ( 5U );
  static constexpr Reg32_t EMR1_EM5_Msk  = ( 0x1UL << EMR1_EM5_Pos );
  static constexpr Reg32_t EMR1_EM5      = EMR1_EM5_Msk;
  static constexpr Reg32_t EMR1_EM6_Pos  = ( 6U );
  static constexpr Reg32_t EMR1_EM6_Msk  = ( 0x1UL << EMR1_EM6_Pos );
  static constexpr Reg32_t EMR1_EM6      = EMR1_EM6_Msk;
  static constexpr Reg32_t EMR1_EM7_Pos  = ( 7U );
  static constexpr Reg32_t EMR1_EM7_Msk  = ( 0x1UL << EMR1_EM7_Pos );
  static constexpr Reg32_t EMR1_EM7      = EMR1_EM7_Msk;
  static constexpr Reg32_t EMR1_EM8_Pos  = ( 8U );
  static constexpr Reg32_t EMR1_EM8_Msk  = ( 0x1UL << EMR1_EM8_Pos );
  static constexpr Reg32_t EMR1_EM8      = EMR1_EM8_Msk;
  static constexpr Reg32_t EMR1_EM9_Pos  = ( 9U );
  static constexpr Reg32_t EMR1_EM9_Msk  = ( 0x1UL << EMR1_EM9_Pos );
  static constexpr Reg32_t EMR1_EM9      = EMR1_EM9_Msk;
  static constexpr Reg32_t EMR1_EM10_Pos = ( 10U );
  static constexpr Reg32_t EMR1_EM10_Msk = ( 0x1UL << EMR1_EM10_Pos );
  static constexpr Reg32_t EMR1_EM10     = EMR1_EM10_Msk;
  static constexpr Reg32_t EMR1_EM11_Pos = ( 11U );
  static constexpr Reg32_t EMR1_EM11_Msk = ( 0x1UL << EMR1_EM11_Pos );
  static constexpr Reg32_t EMR1_EM11     = EMR1_EM11_Msk;
  static constexpr Reg32_t EMR1_EM12_Pos = ( 12U );
  static constexpr Reg32_t EMR1_EM12_Msk = ( 0x1UL << EMR1_EM12_Pos );
  static constexpr Reg32_t EMR1_EM12     = EMR1_EM12_Msk;
  static constexpr Reg32_t EMR1_EM13_Pos = ( 13U );
  static constexpr Reg32_t EMR1_EM13_Msk = ( 0x1UL << EMR1_EM13_Pos );
  static constexpr Reg32_t EMR1_EM13     = EMR1_EM13_Msk;
  static constexpr Reg32_t EMR1_EM14_Pos = ( 14U );
  static constexpr Reg32_t EMR1_EM14_Msk = ( 0x1UL << EMR1_EM14_Pos );
  static constexpr Reg32_t EMR1_EM14     = EMR1_EM14_Msk;
  static constexpr Reg32_t EMR1_EM15_Pos = ( 15U );
  static constexpr Reg32_t EMR1_EM15_Msk = ( 0x1UL << EMR1_EM15_Pos );
  static constexpr Reg32_t EMR1_EM15     = EMR1_EM15_Msk;
  static constexpr Reg32_t EMR1_EM16_Pos = ( 16U );
  static constexpr Reg32_t EMR1_EM16_Msk = ( 0x1UL << EMR1_EM16_Pos );
  static constexpr Reg32_t EMR1_EM16     = EMR1_EM16_Msk;
  static constexpr Reg32_t EMR1_EM17_Pos = ( 17U );
  static constexpr Reg32_t EMR1_EM17_Msk = ( 0x1UL << EMR1_EM17_Pos );
  static constexpr Reg32_t EMR1_EM17     = EMR1_EM17_Msk;
  static constexpr Reg32_t EMR1_EM18_Pos = ( 18U );
  static constexpr Reg32_t EMR1_EM18_Msk = ( 0x1UL << EMR1_EM18_Pos );
  static constexpr Reg32_t EMR1_EM18     = EMR1_EM18_Msk;
  static constexpr Reg32_t EMR1_EM20_Pos = ( 20U );
  static constexpr Reg32_t EMR1_EM20_Msk = ( 0x1UL << EMR1_EM20_Pos );
  static constexpr Reg32_t EMR1_EM20     = EMR1_EM20_Msk;
  static constexpr Reg32_t EMR1_EM21_Pos = ( 21U );
  static constexpr Reg32_t EMR1_EM21_Msk = ( 0x1UL << EMR1_EM21_Pos );
  static constexpr Reg32_t EMR1_EM21     = EMR1_EM21_Msk;
  static constexpr Reg32_t EMR1_EM22_Pos = ( 22U );
  static constexpr Reg32_t EMR1_EM22_Msk = ( 0x1UL << EMR1_EM22_Pos );
  static constexpr Reg32_t EMR1_EM22     = EMR1_EM22_Msk;

  /******************  Bit definition for RTSR1 register  ******************/
  static constexpr Reg32_t RTSR1_RT0_Pos  = ( 0U );
  static constexpr Reg32_t RTSR1_RT0_Msk  = ( 0x1UL << RTSR1_RT0_Pos );
  static constexpr Reg32_t RTSR1_RT0      = RTSR1_RT0_Msk;
  static constexpr Reg32_t RTSR1_RT1_Pos  = ( 1U );
  static constexpr Reg32_t RTSR1_RT1_Msk  = ( 0x1UL << RTSR1_RT1_Pos );
  static constexpr Reg32_t RTSR1_RT1      = RTSR1_RT1_Msk;
  static constexpr Reg32_t RTSR1_RT2_Pos  = ( 2U );
  static constexpr Reg32_t RTSR1_RT2_Msk  = ( 0x1UL << RTSR1_RT2_Pos );
  static constexpr Reg32_t RTSR1_RT2      = RTSR1_RT2_Msk;
  static constexpr Reg32_t RTSR1_RT3_Pos  = ( 3U );
  static constexpr Reg32_t RTSR1_RT3_Msk  = ( 0x1UL << RTSR1_RT3_Pos );
  static constexpr Reg32_t RTSR1_RT3      = RTSR1_RT3_Msk;
  static constexpr Reg32_t RTSR1_RT4_Pos  = ( 4U );
  static constexpr Reg32_t RTSR1_RT4_Msk  = ( 0x1UL << RTSR1_RT4_Pos );
  static constexpr Reg32_t RTSR1_RT4      = RTSR1_RT4_Msk;
  static constexpr Reg32_t RTSR1_RT5_Pos  = ( 5U );
  static constexpr Reg32_t RTSR1_RT5_Msk  = ( 0x1UL << RTSR1_RT5_Pos );
  static constexpr Reg32_t RTSR1_RT5      = RTSR1_RT5_Msk;
  static constexpr Reg32_t RTSR1_RT6_Pos  = ( 6U );
  static constexpr Reg32_t RTSR1_RT6_Msk  = ( 0x1UL << RTSR1_RT6_Pos );
  static constexpr Reg32_t RTSR1_RT6      = RTSR1_RT6_Msk;
  static constexpr Reg32_t RTSR1_RT7_Pos  = ( 7U );
  static constexpr Reg32_t RTSR1_RT7_Msk  = ( 0x1UL << RTSR1_RT7_Pos );
  static constexpr Reg32_t RTSR1_RT7      = RTSR1_RT7_Msk;
  static constexpr Reg32_t RTSR1_RT8_Pos  = ( 8U );
  static constexpr Reg32_t RTSR1_RT8_Msk  = ( 0x1UL << RTSR1_RT8_Pos );
  static constexpr Reg32_t RTSR1_RT8      = RTSR1_RT8_Msk;
  static constexpr Reg32_t RTSR1_RT9_Pos  = ( 9U );
  static constexpr Reg32_t RTSR1_RT9_Msk  = ( 0x1UL << RTSR1_RT9_Pos );
  static constexpr Reg32_t RTSR1_RT9      = RTSR1_RT9_Msk;
  static constexpr Reg32_t RTSR1_RT10_Pos = ( 10U );
  static constexpr Reg32_t RTSR1_RT10_Msk = ( 0x1UL << RTSR1_RT10_Pos );
  static constexpr Reg32_t RTSR1_RT10     = RTSR1_RT10_Msk;
  static constexpr Reg32_t RTSR1_RT11_Pos = ( 11U );
  static constexpr Reg32_t RTSR1_RT11_Msk = ( 0x1UL << RTSR1_RT11_Pos );
  static constexpr Reg32_t RTSR1_RT11     = RTSR1_RT11_Msk;
  static constexpr Reg32_t RTSR1_RT12_Pos = ( 12U );
  static constexpr Reg32_t RTSR1_RT12_Msk = ( 0x1UL << RTSR1_RT12_Pos );
  static constexpr Reg32_t RTSR1_RT12     = RTSR1_RT12_Msk;
  static constexpr Reg32_t RTSR1_RT13_Pos = ( 13U );
  static constexpr Reg32_t RTSR1_RT13_Msk = ( 0x1UL << RTSR1_RT13_Pos );
  static constexpr Reg32_t RTSR1_RT13     = RTSR1_RT13_Msk;
  static constexpr Reg32_t RTSR1_RT14_Pos = ( 14U );
  static constexpr Reg32_t RTSR1_RT14_Msk = ( 0x1UL << RTSR1_RT14_Pos );
  static constexpr Reg32_t RTSR1_RT14     = RTSR1_RT14_Msk;
  static constexpr Reg32_t RTSR1_RT15_Pos = ( 15U );
  static constexpr Reg32_t RTSR1_RT15_Msk = ( 0x1UL << RTSR1_RT15_Pos );
  static constexpr Reg32_t RTSR1_RT15     = RTSR1_RT15_Msk;
  static constexpr Reg32_t RTSR1_RT16_Pos = ( 16U );
  static constexpr Reg32_t RTSR1_RT16_Msk = ( 0x1UL << RTSR1_RT16_Pos );
  static constexpr Reg32_t RTSR1_RT16     = RTSR1_RT16_Msk;
  static constexpr Reg32_t RTSR1_RT18_Pos = ( 18U );
  static constexpr Reg32_t RTSR1_RT18_Msk = ( 0x1UL << RTSR1_RT18_Pos );
  static constexpr Reg32_t RTSR1_RT18     = RTSR1_RT18_Msk;
  static constexpr Reg32_t RTSR1_RT20_Pos = ( 20U );
  static constexpr Reg32_t RTSR1_RT20_Msk = ( 0x1UL << RTSR1_RT20_Pos );
  static constexpr Reg32_t RTSR1_RT20     = RTSR1_RT20_Msk;
  static constexpr Reg32_t RTSR1_RT21_Pos = ( 21U );
  static constexpr Reg32_t RTSR1_RT21_Msk = ( 0x1UL << RTSR1_RT21_Pos );
  static constexpr Reg32_t RTSR1_RT21     = RTSR1_RT21_Msk;
  static constexpr Reg32_t RTSR1_RT22_Pos = ( 22U );
  static constexpr Reg32_t RTSR1_RT22_Msk = ( 0x1UL << RTSR1_RT22_Pos );
  static constexpr Reg32_t RTSR1_RT22     = RTSR1_RT22_Msk;

  /******************  Bit definition for FTSR1 register  ******************/
  static constexpr Reg32_t FTSR1_FT0_Pos  = ( 0U );
  static constexpr Reg32_t FTSR1_FT0_Msk  = ( 0x1UL << FTSR1_FT0_Pos );
  static constexpr Reg32_t FTSR1_FT0      = FTSR1_FT0_Msk;
  static constexpr Reg32_t FTSR1_FT1_Pos  = ( 1U );
  static constexpr Reg32_t FTSR1_FT1_Msk  = ( 0x1UL << FTSR1_FT1_Pos );
  static constexpr Reg32_t FTSR1_FT1      = FTSR1_FT1_Msk;
  static constexpr Reg32_t FTSR1_FT2_Pos  = ( 2U );
  static constexpr Reg32_t FTSR1_FT2_Msk  = ( 0x1UL << FTSR1_FT2_Pos );
  static constexpr Reg32_t FTSR1_FT2      = FTSR1_FT2_Msk;
  static constexpr Reg32_t FTSR1_FT3_Pos  = ( 3U );
  static constexpr Reg32_t FTSR1_FT3_Msk  = ( 0x1UL << FTSR1_FT3_Pos );
  static constexpr Reg32_t FTSR1_FT3      = FTSR1_FT3_Msk;
  static constexpr Reg32_t FTSR1_FT4_Pos  = ( 4U );
  static constexpr Reg32_t FTSR1_FT4_Msk  = ( 0x1UL << FTSR1_FT4_Pos );
  static constexpr Reg32_t FTSR1_FT4      = FTSR1_FT4_Msk;
  static constexpr Reg32_t FTSR1_FT5_Pos  = ( 5U );
  static constexpr Reg32_t FTSR1_FT5_Msk  = ( 0x1UL << FTSR1_FT5_Pos );
  static constexpr Reg32_t FTSR1_FT5      = FTSR1_FT5_Msk;
  static constexpr Reg32_t FTSR1_FT6_Pos  = ( 6U );
  static constexpr Reg32_t FTSR1_FT6_Msk  = ( 0x1UL << FTSR1_FT6_Pos );
  static constexpr Reg32_t FTSR1_FT6      = FTSR1_FT6_Msk;
  static constexpr Reg32_t FTSR1_FT7_Pos  = ( 7U );
  static constexpr Reg32_t FTSR1_FT7_Msk  = ( 0x1UL << FTSR1_FT7_Pos );
  static constexpr Reg32_t FTSR1_FT7      = FTSR1_FT7_Msk;
  static constexpr Reg32_t FTSR1_FT8_Pos  = ( 8U );
  static constexpr Reg32_t FTSR1_FT8_Msk  = ( 0x1UL << FTSR1_FT8_Pos );
  static constexpr Reg32_t FTSR1_FT8      = FTSR1_FT8_Msk;
  static constexpr Reg32_t FTSR1_FT9_Pos  = ( 9U );
  static constexpr Reg32_t FTSR1_FT9_Msk  = ( 0x1UL << FTSR1_FT9_Pos );
  static constexpr Reg32_t FTSR1_FT9      = FTSR1_FT9_Msk;
  static constexpr Reg32_t FTSR1_FT10_Pos = ( 10U );
  static constexpr Reg32_t FTSR1_FT10_Msk = ( 0x1UL << FTSR1_FT10_Pos );
  static constexpr Reg32_t FTSR1_FT10     = FTSR1_FT10_Msk;
  static constexpr Reg32_t FTSR1_FT11_Pos = ( 11U );
  static constexpr Reg32_t FTSR1_FT11_Msk = ( 0x1UL << FTSR1_FT11_Pos );
  static constexpr Reg32_t FTSR1_FT11     = FTSR1_FT11_Msk;
  static constexpr Reg32_t FTSR1_FT12_Pos = ( 12U );
  static constexpr Reg32_t FTSR1_FT12_Msk = ( 0x1UL << FTSR1_FT12_Pos );
  static constexpr Reg32_t FTSR1_FT12     = FTSR1_FT12_Msk;
  static constexpr Reg32_t FTSR1_FT13_Pos = ( 13U );
  static constexpr Reg32_t FTSR1_FT13_Msk = ( 0x1UL << FTSR1_FT13_Pos );
  static constexpr Reg32_t FTSR1_FT13     = FTSR1_FT13_Msk;
  static constexpr Reg32_t FTSR1_FT14_Pos = ( 14U );
  static constexpr Reg32_t FTSR1_FT14_Msk = ( 0x1UL << FTSR1_FT14_Pos );
  static constexpr Reg32_t FTSR1_FT14     = FTSR1_FT14_Msk;
  static constexpr Reg32_t FTSR1_FT15_Pos = ( 15U );
  static constexpr Reg32_t FTSR1_FT15_Msk = ( 0x1UL << FTSR1_FT15_Pos );
  static constexpr Reg32_t FTSR1_FT15     = FTSR1_FT15_Msk;
  static constexpr Reg32_t FTSR1_FT16_Pos = ( 16U );
  static constexpr Reg32_t FTSR1_FT16_Msk = ( 0x1UL << FTSR1_FT16_Pos );
  static constexpr Reg32_t FTSR1_FT16     = FTSR1_FT16_Msk;
  static constexpr Reg32_t FTSR1_FT18_Pos = ( 18U );
  static constexpr Reg32_t FTSR1_FT18_Msk = ( 0x1UL << FTSR1_FT18_Pos );
  static constexpr Reg32_t FTSR1_FT18     = FTSR1_FT18_Msk;
  static constexpr Reg32_t FTSR1_FT20_Pos = ( 20U );
  static constexpr Reg32_t FTSR1_FT20_Msk = ( 0x1UL << FTSR1_FT20_Pos );
  static constexpr Reg32_t FTSR1_FT20     = FTSR1_FT20_Msk;
  static constexpr Reg32_t FTSR1_FT21_Pos = ( 21U );
  static constexpr Reg32_t FTSR1_FT21_Msk = ( 0x1UL << FTSR1_FT21_Pos );
  static constexpr Reg32_t FTSR1_FT21     = FTSR1_FT21_Msk;
  static constexpr Reg32_t FTSR1_FT22_Pos = ( 22U );
  static constexpr Reg32_t FTSR1_FT22_Msk = ( 0x1UL << FTSR1_FT22_Pos );
  static constexpr Reg32_t FTSR1_FT22     = FTSR1_FT22_Msk;

  /******************  Bit definition for SWIER1 register  *****************/
  static constexpr Reg32_t SWIER1_SWI0_Pos  = ( 0U );
  static constexpr Reg32_t SWIER1_SWI0_Msk  = ( 0x1UL << SWIER1_SWI0_Pos );
  static constexpr Reg32_t SWIER1_SWI0      = SWIER1_SWI0_Msk;
  static constexpr Reg32_t SWIER1_SWI1_Pos  = ( 1U );
  static constexpr Reg32_t SWIER1_SWI1_Msk  = ( 0x1UL << SWIER1_SWI1_Pos );
  static constexpr Reg32_t SWIER1_SWI1      = SWIER1_SWI1_Msk;
  static constexpr Reg32_t SWIER1_SWI2_Pos  = ( 2U );
  static constexpr Reg32_t SWIER1_SWI2_Msk  = ( 0x1UL << SWIER1_SWI2_Pos );
  static constexpr Reg32_t SWIER1_SWI2      = SWIER1_SWI2_Msk;
  static constexpr Reg32_t SWIER1_SWI3_Pos  = ( 3U );
  static constexpr Reg32_t SWIER1_SWI3_Msk  = ( 0x1UL << SWIER1_SWI3_Pos );
  static constexpr Reg32_t SWIER1_SWI3      = SWIER1_SWI3_Msk;
  static constexpr Reg32_t SWIER1_SWI4_Pos  = ( 4U );
  static constexpr Reg32_t SWIER1_SWI4_Msk  = ( 0x1UL << SWIER1_SWI4_Pos );
  static constexpr Reg32_t SWIER1_SWI4      = SWIER1_SWI4_Msk;
  static constexpr Reg32_t SWIER1_SWI5_Pos  = ( 5U );
  static constexpr Reg32_t SWIER1_SWI5_Msk  = ( 0x1UL << SWIER1_SWI5_Pos );
  static constexpr Reg32_t SWIER1_SWI5      = SWIER1_SWI5_Msk;
  static constexpr Reg32_t SWIER1_SWI6_Pos  = ( 6U );
  static constexpr Reg32_t SWIER1_SWI6_Msk  = ( 0x1UL << SWIER1_SWI6_Pos );
  static constexpr Reg32_t SWIER1_SWI6      = SWIER1_SWI6_Msk;
  static constexpr Reg32_t SWIER1_SWI7_Pos  = ( 7U );
  static constexpr Reg32_t SWIER1_SWI7_Msk  = ( 0x1UL << SWIER1_SWI7_Pos );
  static constexpr Reg32_t SWIER1_SWI7      = SWIER1_SWI7_Msk;
  static constexpr Reg32_t SWIER1_SWI8_Pos  = ( 8U );
  static constexpr Reg32_t SWIER1_SWI8_Msk  = ( 0x1UL << SWIER1_SWI8_Pos );
  static constexpr Reg32_t SWIER1_SWI8      = SWIER1_SWI8_Msk;
  static constexpr Reg32_t SWIER1_SWI9_Pos  = ( 9U );
  static constexpr Reg32_t SWIER1_SWI9_Msk  = ( 0x1UL << SWIER1_SWI9_Pos );
  static constexpr Reg32_t SWIER1_SWI9      = SWIER1_SWI9_Msk;
  static constexpr Reg32_t SWIER1_SWI10_Pos = ( 10U );
  static constexpr Reg32_t SWIER1_SWI10_Msk = ( 0x1UL << SWIER1_SWI10_Pos );
  static constexpr Reg32_t SWIER1_SWI10     = SWIER1_SWI10_Msk;
  static constexpr Reg32_t SWIER1_SWI11_Pos = ( 11U );
  static constexpr Reg32_t SWIER1_SWI11_Msk = ( 0x1UL << SWIER1_SWI11_Pos );
  static constexpr Reg32_t SWIER1_SWI11     = SWIER1_SWI11_Msk;
  static constexpr Reg32_t SWIER1_SWI12_Pos = ( 12U );
  static constexpr Reg32_t SWIER1_SWI12_Msk = ( 0x1UL << SWIER1_SWI12_Pos );
  static constexpr Reg32_t SWIER1_SWI12     = SWIER1_SWI12_Msk;
  static constexpr Reg32_t SWIER1_SWI13_Pos = ( 13U );
  static constexpr Reg32_t SWIER1_SWI13_Msk = ( 0x1UL << SWIER1_SWI13_Pos );
  static constexpr Reg32_t SWIER1_SWI13     = SWIER1_SWI13_Msk;
  static constexpr Reg32_t SWIER1_SWI14_Pos = ( 14U );
  static constexpr Reg32_t SWIER1_SWI14_Msk = ( 0x1UL << SWIER1_SWI14_Pos );
  static constexpr Reg32_t SWIER1_SWI14     = SWIER1_SWI14_Msk;
  static constexpr Reg32_t SWIER1_SWI15_Pos = ( 15U );
  static constexpr Reg32_t SWIER1_SWI15_Msk = ( 0x1UL << SWIER1_SWI15_Pos );
  static constexpr Reg32_t SWIER1_SWI15     = SWIER1_SWI15_Msk;
  static constexpr Reg32_t SWIER1_SWI16_Pos = ( 16U );
  static constexpr Reg32_t SWIER1_SWI16_Msk = ( 0x1UL << SWIER1_SWI16_Pos );
  static constexpr Reg32_t SWIER1_SWI16     = SWIER1_SWI16_Msk;
  static constexpr Reg32_t SWIER1_SWI18_Pos = ( 18U );
  static constexpr Reg32_t SWIER1_SWI18_Msk = ( 0x1UL << SWIER1_SWI18_Pos );
  static constexpr Reg32_t SWIER1_SWI18     = SWIER1_SWI18_Msk;
  static constexpr Reg32_t SWIER1_SWI20_Pos = ( 20U );
  static constexpr Reg32_t SWIER1_SWI20_Msk = ( 0x1UL << SWIER1_SWI20_Pos );
  static constexpr Reg32_t SWIER1_SWI20     = SWIER1_SWI20_Msk;
  static constexpr Reg32_t SWIER1_SWI21_Pos = ( 21U );
  static constexpr Reg32_t SWIER1_SWI21_Msk = ( 0x1UL << SWIER1_SWI21_Pos );
  static constexpr Reg32_t SWIER1_SWI21     = SWIER1_SWI21_Msk;
  static constexpr Reg32_t SWIER1_SWI22_Pos = ( 22U );
  static constexpr Reg32_t SWIER1_SWI22_Msk = ( 0x1UL << SWIER1_SWI22_Pos );
  static constexpr Reg32_t SWIER1_SWI22     = SWIER1_SWI22_Msk;

  /*******************  Bit definition for PR1 register  *******************/
  static constexpr Reg32_t PR1_PIF0_Pos  = ( 0U );
  static constexpr Reg32_t PR1_PIF0_Msk  = ( 0x1UL << PR1_PIF0_Pos );
  static constexpr Reg32_t PR1_PIF0      = PR1_PIF0_Msk;
  static constexpr Reg32_t PR1_PIF1_Pos  = ( 1U );
  static constexpr Reg32_t PR1_PIF1_Msk  = ( 0x1UL << PR1_PIF1_Pos );
  static constexpr Reg32_t PR1_PIF1      = PR1_PIF1_Msk;
  static constexpr Reg32_t PR1_PIF2_Pos  = ( 2U );
  static constexpr Reg32_t PR1_PIF2_Msk  = ( 0x1UL << PR1_PIF2_Pos );
  static constexpr Reg32_t PR1_PIF2      = PR1_PIF2_Msk;
  static constexpr Reg32_t PR1_PIF3_Pos  = ( 3U );
  static constexpr Reg32_t PR1_PIF3_Msk  = ( 0x1UL << PR1_PIF3_Pos );
  static constexpr Reg32_t PR1_PIF3      = PR1_PIF3_Msk;
  static constexpr Reg32_t PR1_PIF4_Pos  = ( 4U );
  static constexpr Reg32_t PR1_PIF4_Msk  = ( 0x1UL << PR1_PIF4_Pos );
  static constexpr Reg32_t PR1_PIF4      = PR1_PIF4_Msk;
  static constexpr Reg32_t PR1_PIF5_Pos  = ( 5U );
  static constexpr Reg32_t PR1_PIF5_Msk  = ( 0x1UL << PR1_PIF5_Pos );
  static constexpr Reg32_t PR1_PIF5      = PR1_PIF5_Msk;
  static constexpr Reg32_t PR1_PIF6_Pos  = ( 6U );
  static constexpr Reg32_t PR1_PIF6_Msk  = ( 0x1UL << PR1_PIF6_Pos );
  static constexpr Reg32_t PR1_PIF6      = PR1_PIF6_Msk;
  static constexpr Reg32_t PR1_PIF7_Pos  = ( 7U );
  static constexpr Reg32_t PR1_PIF7_Msk  = ( 0x1UL << PR1_PIF7_Pos );
  static constexpr Reg32_t PR1_PIF7      = PR1_PIF7_Msk;
  static constexpr Reg32_t PR1_PIF8_Pos  = ( 8U );
  static constexpr Reg32_t PR1_PIF8_Msk  = ( 0x1UL << PR1_PIF8_Pos );
  static constexpr Reg32_t PR1_PIF8      = PR1_PIF8_Msk;
  static constexpr Reg32_t PR1_PIF9_Pos  = ( 9U );
  static constexpr Reg32_t PR1_PIF9_Msk  = ( 0x1UL << PR1_PIF9_Pos );
  static constexpr Reg32_t PR1_PIF9      = PR1_PIF9_Msk;
  static constexpr Reg32_t PR1_PIF10_Pos = ( 10U );
  static constexpr Reg32_t PR1_PIF10_Msk = ( 0x1UL << PR1_PIF10_Pos );
  static constexpr Reg32_t PR1_PIF10     = PR1_PIF10_Msk;
  static constexpr Reg32_t PR1_PIF11_Pos = ( 11U );
  static constexpr Reg32_t PR1_PIF11_Msk = ( 0x1UL << PR1_PIF11_Pos );
  static constexpr Reg32_t PR1_PIF11     = PR1_PIF11_Msk;
  static constexpr Reg32_t PR1_PIF12_Pos = ( 12U );
  static constexpr Reg32_t PR1_PIF12_Msk = ( 0x1UL << PR1_PIF12_Pos );
  static constexpr Reg32_t PR1_PIF12     = PR1_PIF12_Msk;
  static constexpr Reg32_t PR1_PIF13_Pos = ( 13U );
  static constexpr Reg32_t PR1_PIF13_Msk = ( 0x1UL << PR1_PIF13_Pos );
  static constexpr Reg32_t PR1_PIF13     = PR1_PIF13_Msk;
  static constexpr Reg32_t PR1_PIF14_Pos = ( 14U );
  static constexpr Reg32_t PR1_PIF14_Msk = ( 0x1UL << PR1_PIF14_Pos );
  static constexpr Reg32_t PR1_PIF14     = PR1_PIF14_Msk;
  static constexpr Reg32_t PR1_PIF15_Pos = ( 15U );
  static constexpr Reg32_t PR1_PIF15_Msk = ( 0x1UL << PR1_PIF15_Pos );
  static constexpr Reg32_t PR1_PIF15     = PR1_PIF15_Msk;
  static constexpr Reg32_t PR1_PIF16_Pos = ( 16U );
  static constexpr Reg32_t PR1_PIF16_Msk = ( 0x1UL << PR1_PIF16_Pos );
  static constexpr Reg32_t PR1_PIF16     = PR1_PIF16_Msk;
  static constexpr Reg32_t PR1_PIF18_Pos = ( 18U );
  static constexpr Reg32_t PR1_PIF18_Msk = ( 0x1UL << PR1_PIF18_Pos );
  static constexpr Reg32_t PR1_PIF18     = PR1_PIF18_Msk;
  static constexpr Reg32_t PR1_PIF20_Pos = ( 20U );
  static constexpr Reg32_t PR1_PIF20_Msk = ( 0x1UL << PR1_PIF20_Pos );
  static constexpr Reg32_t PR1_PIF20     = PR1_PIF20_Msk;
  static constexpr Reg32_t PR1_PIF21_Pos = ( 21U );
  static constexpr Reg32_t PR1_PIF21_Msk = ( 0x1UL << PR1_PIF21_Pos );
  static constexpr Reg32_t PR1_PIF21     = PR1_PIF21_Msk;
  static constexpr Reg32_t PR1_PIF22_Pos = ( 22U );
  static constexpr Reg32_t PR1_PIF22_Msk = ( 0x1UL << PR1_PIF22_Pos );
  static constexpr Reg32_t PR1_PIF22     = PR1_PIF22_Msk;

}    // namespace Thor::LLD::EXTI

#endif /* !THOR_HW_EXTI_REGISTER_STM32F4XXXX_HPP */
