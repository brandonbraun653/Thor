/******************************************************************************
 *  File Name:
 *    hw_exti_register_stm32l4xxxx.hpp
 *
 *  Description:
 *    EXTI register definitions for the STM32L4xxxx series chips
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_HW_EXTI_REGISTER_STM32L4XXXX_HPP
#define THOR_HW_EXTI_REGISTER_STM32L4XXXX_HPP

/* Chimera Includes */
#include <Chimera/common>

/* Driver Includes */
#include <Thor/lld/stm32l4x/system/variant/sys_memory_map_stm32l432kc.hpp>


namespace Thor::LLD::EXTI
{
  /*---------------------------------------------------------------------------
  Peripheral Instance Memory Map
  ---------------------------------------------------------------------------*/
  static constexpr uint32_t EXTI1_BASE_ADDR = Thor::System::MemoryMap::EXTI_PERIPH_START_ADDRESS;

  /*---------------------------------------------------------------------------
  Peripheral Register Definitions
  ---------------------------------------------------------------------------*/
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
  static constexpr Reg32_t IMR1_IM19_Pos = ( 19U );
  static constexpr Reg32_t IMR1_IM19_Msk = ( 0x1UL << IMR1_IM19_Pos );
  static constexpr Reg32_t IMR1_IM19     = IMR1_IM19_Msk;
  static constexpr Reg32_t IMR1_IM20_Pos = ( 20U );
  static constexpr Reg32_t IMR1_IM20_Msk = ( 0x1UL << IMR1_IM20_Pos );
  static constexpr Reg32_t IMR1_IM20     = IMR1_IM20_Msk;
  static constexpr Reg32_t IMR1_IM21_Pos = ( 21U );
  static constexpr Reg32_t IMR1_IM21_Msk = ( 0x1UL << IMR1_IM21_Pos );
  static constexpr Reg32_t IMR1_IM21     = IMR1_IM21_Msk;
  static constexpr Reg32_t IMR1_IM22_Pos = ( 22U );
  static constexpr Reg32_t IMR1_IM22_Msk = ( 0x1UL << IMR1_IM22_Pos );
  static constexpr Reg32_t IMR1_IM22     = IMR1_IM22_Msk;
  static constexpr Reg32_t IMR1_IM23_Pos = ( 23U );
  static constexpr Reg32_t IMR1_IM23_Msk = ( 0x1UL << IMR1_IM23_Pos );
  static constexpr Reg32_t IMR1_IM23     = IMR1_IM23_Msk;
  static constexpr Reg32_t IMR1_IM24_Pos = ( 24U );
  static constexpr Reg32_t IMR1_IM24_Msk = ( 0x1UL << IMR1_IM24_Pos );
  static constexpr Reg32_t IMR1_IM24     = IMR1_IM24_Msk;
  static constexpr Reg32_t IMR1_IM25_Pos = ( 25U );
  static constexpr Reg32_t IMR1_IM25_Msk = ( 0x1UL << IMR1_IM25_Pos );
  static constexpr Reg32_t IMR1_IM25     = IMR1_IM25_Msk;
  static constexpr Reg32_t IMR1_IM26_Pos = ( 26U );
  static constexpr Reg32_t IMR1_IM26_Msk = ( 0x1UL << IMR1_IM26_Pos );
  static constexpr Reg32_t IMR1_IM26     = IMR1_IM26_Msk;
  static constexpr Reg32_t IMR1_IM27_Pos = ( 27U );
  static constexpr Reg32_t IMR1_IM27_Msk = ( 0x1UL << IMR1_IM27_Pos );
  static constexpr Reg32_t IMR1_IM27     = IMR1_IM27_Msk;
  static constexpr Reg32_t IMR1_IM28_Pos = ( 28U );
  static constexpr Reg32_t IMR1_IM28_Msk = ( 0x1UL << IMR1_IM28_Pos );
  static constexpr Reg32_t IMR1_IM28     = IMR1_IM28_Msk;
  static constexpr Reg32_t IMR1_IM31_Pos = ( 31U );
  static constexpr Reg32_t IMR1_IM31_Msk = ( 0x1UL << IMR1_IM31_Pos );
  static constexpr Reg32_t IMR1_IM31     = IMR1_IM31_Msk;
  static constexpr Reg32_t IMR1_IM_Pos   = ( 0U );
  static constexpr Reg32_t IMR1_IM_Msk   = ( 0x9FFFFFFFUL << IMR1_IM_Pos );
  static constexpr Reg32_t IMR1_IM       = IMR1_IM_Msk;

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
  static constexpr Reg32_t EMR1_EM19_Pos = ( 19U );
  static constexpr Reg32_t EMR1_EM19_Msk = ( 0x1UL << EMR1_EM19_Pos );
  static constexpr Reg32_t EMR1_EM19     = EMR1_EM19_Msk;
  static constexpr Reg32_t EMR1_EM20_Pos = ( 20U );
  static constexpr Reg32_t EMR1_EM20_Msk = ( 0x1UL << EMR1_EM20_Pos );
  static constexpr Reg32_t EMR1_EM20     = EMR1_EM20_Msk;
  static constexpr Reg32_t EMR1_EM21_Pos = ( 21U );
  static constexpr Reg32_t EMR1_EM21_Msk = ( 0x1UL << EMR1_EM21_Pos );
  static constexpr Reg32_t EMR1_EM21     = EMR1_EM21_Msk;
  static constexpr Reg32_t EMR1_EM22_Pos = ( 22U );
  static constexpr Reg32_t EMR1_EM22_Msk = ( 0x1UL << EMR1_EM22_Pos );
  static constexpr Reg32_t EMR1_EM22     = EMR1_EM22_Msk;
  static constexpr Reg32_t EMR1_EM23_Pos = ( 23U );
  static constexpr Reg32_t EMR1_EM23_Msk = ( 0x1UL << EMR1_EM23_Pos );
  static constexpr Reg32_t EMR1_EM23     = EMR1_EM23_Msk;
  static constexpr Reg32_t EMR1_EM24_Pos = ( 24U );
  static constexpr Reg32_t EMR1_EM24_Msk = ( 0x1UL << EMR1_EM24_Pos );
  static constexpr Reg32_t EMR1_EM24     = EMR1_EM24_Msk;
  static constexpr Reg32_t EMR1_EM25_Pos = ( 25U );
  static constexpr Reg32_t EMR1_EM25_Msk = ( 0x1UL << EMR1_EM25_Pos );
  static constexpr Reg32_t EMR1_EM25     = EMR1_EM25_Msk;
  static constexpr Reg32_t EMR1_EM26_Pos = ( 26U );
  static constexpr Reg32_t EMR1_EM26_Msk = ( 0x1UL << EMR1_EM26_Pos );
  static constexpr Reg32_t EMR1_EM26     = EMR1_EM26_Msk;
  static constexpr Reg32_t EMR1_EM27_Pos = ( 27U );
  static constexpr Reg32_t EMR1_EM27_Msk = ( 0x1UL << EMR1_EM27_Pos );
  static constexpr Reg32_t EMR1_EM27     = EMR1_EM27_Msk;
  static constexpr Reg32_t EMR1_EM28_Pos = ( 28U );
  static constexpr Reg32_t EMR1_EM28_Msk = ( 0x1UL << EMR1_EM28_Pos );
  static constexpr Reg32_t EMR1_EM28     = EMR1_EM28_Msk;
  static constexpr Reg32_t EMR1_EM31_Pos = ( 31U );
  static constexpr Reg32_t EMR1_EM31_Msk = ( 0x1UL << EMR1_EM31_Pos );
  static constexpr Reg32_t EMR1_EM31     = EMR1_EM31_Msk;

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
  static constexpr Reg32_t RTSR1_RT19_Pos = ( 19U );
  static constexpr Reg32_t RTSR1_RT19_Msk = ( 0x1UL << RTSR1_RT19_Pos );
  static constexpr Reg32_t RTSR1_RT19     = RTSR1_RT19_Msk;
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
  static constexpr Reg32_t FTSR1_FT19_Pos = ( 19U );
  static constexpr Reg32_t FTSR1_FT19_Msk = ( 0x1UL << FTSR1_FT19_Pos );
  static constexpr Reg32_t FTSR1_FT19     = FTSR1_FT19_Msk;
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
  static constexpr Reg32_t SWIER1_SWI19_Pos = ( 19U );
  static constexpr Reg32_t SWIER1_SWI19_Msk = ( 0x1UL << SWIER1_SWI19_Pos );
  static constexpr Reg32_t SWIER1_SWI19     = SWIER1_SWI19_Msk;
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
  static constexpr Reg32_t PR1_PIF19_Pos = ( 19U );
  static constexpr Reg32_t PR1_PIF19_Msk = ( 0x1UL << PR1_PIF19_Pos );
  static constexpr Reg32_t PR1_PIF19     = PR1_PIF19_Msk;
  static constexpr Reg32_t PR1_PIF20_Pos = ( 20U );
  static constexpr Reg32_t PR1_PIF20_Msk = ( 0x1UL << PR1_PIF20_Pos );
  static constexpr Reg32_t PR1_PIF20     = PR1_PIF20_Msk;
  static constexpr Reg32_t PR1_PIF21_Pos = ( 21U );
  static constexpr Reg32_t PR1_PIF21_Msk = ( 0x1UL << PR1_PIF21_Pos );
  static constexpr Reg32_t PR1_PIF21     = PR1_PIF21_Msk;
  static constexpr Reg32_t PR1_PIF22_Pos = ( 22U );
  static constexpr Reg32_t PR1_PIF22_Msk = ( 0x1UL << PR1_PIF22_Pos );
  static constexpr Reg32_t PR1_PIF22     = PR1_PIF22_Msk;

  /*******************  Bit definition for IMR2 register  ******************/
  static constexpr Reg32_t IMR2_IM32_Pos = ( 0U );
  static constexpr Reg32_t IMR2_IM32_Msk = ( 0x1UL << IMR2_IM32_Pos );
  static constexpr Reg32_t IMR2_IM32     = IMR2_IM32_Msk;
  static constexpr Reg32_t IMR2_IM33_Pos = ( 1U );
  static constexpr Reg32_t IMR2_IM33_Msk = ( 0x1UL << IMR2_IM33_Pos );
  static constexpr Reg32_t IMR2_IM33     = IMR2_IM33_Msk;
  static constexpr Reg32_t IMR2_IM34_Pos = ( 2U );
  static constexpr Reg32_t IMR2_IM34_Msk = ( 0x1UL << IMR2_IM34_Pos );
  static constexpr Reg32_t IMR2_IM34     = IMR2_IM34_Msk;
  static constexpr Reg32_t IMR2_IM35_Pos = ( 3U );
  static constexpr Reg32_t IMR2_IM35_Msk = ( 0x1UL << IMR2_IM35_Pos );
  static constexpr Reg32_t IMR2_IM35     = IMR2_IM35_Msk;
  static constexpr Reg32_t IMR2_IM37_Pos = ( 5U );
  static constexpr Reg32_t IMR2_IM37_Msk = ( 0x1UL << IMR2_IM37_Pos );
  static constexpr Reg32_t IMR2_IM37     = IMR2_IM37_Msk;
  static constexpr Reg32_t IMR2_IM38_Pos = ( 6U );
  static constexpr Reg32_t IMR2_IM38_Msk = ( 0x1UL << IMR2_IM38_Pos );
  static constexpr Reg32_t IMR2_IM38     = IMR2_IM38_Msk;
  static constexpr Reg32_t IMR2_IM_Pos   = ( 0U );
  static constexpr Reg32_t IMR2_IM_Msk   = ( 0x6FUL << IMR2_IM_Pos );
  static constexpr Reg32_t IMR2_IM       = IMR2_IM_Msk;

  /*******************  Bit definition for EMR2 register  ******************/
  static constexpr Reg32_t EMR2_EM32_Pos = ( 0U );
  static constexpr Reg32_t EMR2_EM32_Msk = ( 0x1UL << EMR2_EM32_Pos );
  static constexpr Reg32_t EMR2_EM32     = EMR2_EM32_Msk;
  static constexpr Reg32_t EMR2_EM33_Pos = ( 1U );
  static constexpr Reg32_t EMR2_EM33_Msk = ( 0x1UL << EMR2_EM33_Pos );
  static constexpr Reg32_t EMR2_EM33     = EMR2_EM33_Msk;
  static constexpr Reg32_t EMR2_EM34_Pos = ( 2U );
  static constexpr Reg32_t EMR2_EM34_Msk = ( 0x1UL << EMR2_EM34_Pos );
  static constexpr Reg32_t EMR2_EM34     = EMR2_EM34_Msk;
  static constexpr Reg32_t EMR2_EM35_Pos = ( 3U );
  static constexpr Reg32_t EMR2_EM35_Msk = ( 0x1UL << EMR2_EM35_Pos );
  static constexpr Reg32_t EMR2_EM35     = EMR2_EM35_Msk;
  static constexpr Reg32_t EMR2_EM37_Pos = ( 5U );
  static constexpr Reg32_t EMR2_EM37_Msk = ( 0x1UL << EMR2_EM37_Pos );
  static constexpr Reg32_t EMR2_EM37     = EMR2_EM37_Msk;
  static constexpr Reg32_t EMR2_EM38_Pos = ( 6U );
  static constexpr Reg32_t EMR2_EM38_Msk = ( 0x1UL << EMR2_EM38_Pos );
  static constexpr Reg32_t EMR2_EM38     = EMR2_EM38_Msk;
  static constexpr Reg32_t EMR2_EM_Pos   = ( 0U );
  static constexpr Reg32_t EMR2_EM_Msk   = ( 0x6FUL << EMR2_EM_Pos );
  static constexpr Reg32_t EMR2_EM       = EMR2_EM_Msk;

  /******************  Bit definition for RTSR2 register  ******************/
  static constexpr Reg32_t RTSR2_RT35_Pos = ( 3U );
  static constexpr Reg32_t RTSR2_RT35_Msk = ( 0x1UL << RTSR2_RT35_Pos );
  static constexpr Reg32_t RTSR2_RT35     = RTSR2_RT35_Msk;
  static constexpr Reg32_t RTSR2_RT37_Pos = ( 5U );
  static constexpr Reg32_t RTSR2_RT37_Msk = ( 0x1UL << RTSR2_RT37_Pos );
  static constexpr Reg32_t RTSR2_RT37     = RTSR2_RT37_Msk;
  static constexpr Reg32_t RTSR2_RT38_Pos = ( 6U );
  static constexpr Reg32_t RTSR2_RT38_Msk = ( 0x1UL << RTSR2_RT38_Pos );
  static constexpr Reg32_t RTSR2_RT38     = RTSR2_RT38_Msk;

  /******************  Bit definition for FTSR2 register  ******************/
  static constexpr Reg32_t FTSR2_FT35_Pos = ( 3U );
  static constexpr Reg32_t FTSR2_FT35_Msk = ( 0x1UL << FTSR2_FT35_Pos );
  static constexpr Reg32_t FTSR2_FT35     = FTSR2_FT35_Msk;
  static constexpr Reg32_t FTSR2_FT37_Pos = ( 5U );
  static constexpr Reg32_t FTSR2_FT37_Msk = ( 0x1UL << FTSR2_FT37_Pos );
  static constexpr Reg32_t FTSR2_FT37     = FTSR2_FT37_Msk;
  static constexpr Reg32_t FTSR2_FT38_Pos = ( 6U );
  static constexpr Reg32_t FTSR2_FT38_Msk = ( 0x1UL << FTSR2_FT38_Pos );
  static constexpr Reg32_t FTSR2_FT38     = FTSR2_FT38_Msk;

  /******************  Bit definition for SWIER2 register  *****************/
  static constexpr Reg32_t SWIER2_SWI35_Pos = ( 3U );
  static constexpr Reg32_t SWIER2_SWI35_Msk = ( 0x1UL << SWIER2_SWI35_Pos );
  static constexpr Reg32_t SWIER2_SWI35     = SWIER2_SWI35_Msk;
  static constexpr Reg32_t SWIER2_SWI37_Pos = ( 5U );
  static constexpr Reg32_t SWIER2_SWI37_Msk = ( 0x1UL << SWIER2_SWI37_Pos );
  static constexpr Reg32_t SWIER2_SWI37     = SWIER2_SWI37_Msk;
  static constexpr Reg32_t SWIER2_SWI38_Pos = ( 6U );
  static constexpr Reg32_t SWIER2_SWI38_Msk = ( 0x1UL << SWIER2_SWI38_Pos );
  static constexpr Reg32_t SWIER2_SWI38     = SWIER2_SWI38_Msk;

  /*******************  Bit definition for PR2 register  *******************/
  static constexpr Reg32_t PR2_PIF35_Pos = ( 3U );
  static constexpr Reg32_t PR2_PIF35_Msk = ( 0x1UL << PR2_PIF35_Pos );
  static constexpr Reg32_t PR2_PIF35     = PR2_PIF35_Msk;
  static constexpr Reg32_t PR2_PIF37_Pos = ( 5U );
  static constexpr Reg32_t PR2_PIF37_Msk = ( 0x1UL << PR2_PIF37_Pos );
  static constexpr Reg32_t PR2_PIF37     = PR2_PIF37_Msk;
  static constexpr Reg32_t PR2_PIF38_Pos = ( 6U );
  static constexpr Reg32_t PR2_PIF38_Msk = ( 0x1UL << PR2_PIF38_Pos );
  static constexpr Reg32_t PR2_PIF38     = PR2_PIF38_Msk;

}    // namespace Thor::LLD::EXTI

#endif /* !THOR_HW_EXTI_REGISTER_STM32L4XXXX_HPP */
