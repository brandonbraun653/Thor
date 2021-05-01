/********************************************************************************
 *  File Name:
 *    hw_adc_register_stm32f4xxxx.hpp
 *
 *  Description:
 *    ADC register definitions for the STM32F4xxxx series chips.
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_ADC_REGISTER_STM32F4XXXX_HPP
#define THOR_HW_ADC_REGISTER_STM32F4XXXX_HPP

/* C++ Includes */
#include <cstdint>

/* Chimera Includes */
#include <Chimera/common>

/* Driver Includes */
#include <Thor/lld/stm32f4x/system/variant/sys_memory_map_stm32f446xx.hpp>


namespace Thor::LLD::ADC
{
  /*-------------------------------------------------
  Peripheral Instance Memory Map Base
  -------------------------------------------------*/
  static constexpr uint32_t ADC1_BASE_ADDR = Thor::System::MemoryMap::ADC1_BASE;
  static constexpr uint32_t ADC2_BASE_ADDR = Thor::System::MemoryMap::ADC2_BASE;
  static constexpr uint32_t ADC3_BASE_ADDR = Thor::System::MemoryMap::ADC3_BASE;

  /*-------------------------------------------------
  Common Register Base Address: RM 13.13.15
  -------------------------------------------------*/
  static constexpr uint32_t ADC_CMN_BASE_ADDR = ADC1_BASE_ADDR + 0x300;

  /*-------------------------------------------------
  Peripheral Register Definitions
  -------------------------------------------------*/

  /********************  Bit definition for SR register  ********************/
  static constexpr uint32_t SR_AWD_Pos   = ( 0U );
  static constexpr uint32_t SR_AWD_Msk   = ( 0x1UL << SR_AWD_Pos );
  static constexpr uint32_t SR_AWD       = SR_AWD_Msk;
  static constexpr uint32_t SR_EOC_Pos   = ( 1U );
  static constexpr uint32_t SR_EOC_Msk   = ( 0x1UL << SR_EOC_Pos );
  static constexpr uint32_t SR_EOC       = SR_EOC_Msk;
  static constexpr uint32_t SR_JEOC_Pos  = ( 2U );
  static constexpr uint32_t SR_JEOC_Msk  = ( 0x1UL << SR_JEOC_Pos );
  static constexpr uint32_t SR_JEOC      = SR_JEOC_Msk;
  static constexpr uint32_t SR_JSTRT_Pos = ( 3U );
  static constexpr uint32_t SR_JSTRT_Msk = ( 0x1UL << SR_JSTRT_Pos );
  static constexpr uint32_t SR_JSTRT     = SR_JSTRT_Msk;
  static constexpr uint32_t SR_STRT_Pos  = ( 4U );
  static constexpr uint32_t SR_STRT_Msk  = ( 0x1UL << SR_STRT_Pos );
  static constexpr uint32_t SR_STRT      = SR_STRT_Msk;
  static constexpr uint32_t SR_OVR_Pos   = ( 5U );
  static constexpr uint32_t SR_OVR_Msk   = ( 0x1UL << SR_OVR_Pos );
  static constexpr uint32_t SR_OVR       = SR_OVR_Msk;

  /*******************  Bit definition for CR1 register  ********************/
  static constexpr uint32_t CR1_AWDCH_Pos   = ( 0U );
  static constexpr uint32_t CR1_AWDCH_Msk   = ( 0x1FUL << CR1_AWDCH_Pos );
  static constexpr uint32_t CR1_AWDCH       = CR1_AWDCH_Msk;
  static constexpr uint32_t CR1_AWDCH_0     = ( 0x01UL << CR1_AWDCH_Pos );
  static constexpr uint32_t CR1_AWDCH_1     = ( 0x02UL << CR1_AWDCH_Pos );
  static constexpr uint32_t CR1_AWDCH_2     = ( 0x04UL << CR1_AWDCH_Pos );
  static constexpr uint32_t CR1_AWDCH_3     = ( 0x08UL << CR1_AWDCH_Pos );
  static constexpr uint32_t CR1_AWDCH_4     = ( 0x10UL << CR1_AWDCH_Pos );
  static constexpr uint32_t CR1_EOCIE_Pos   = ( 5U );
  static constexpr uint32_t CR1_EOCIE_Msk   = ( 0x1UL << CR1_EOCIE_Pos );
  static constexpr uint32_t CR1_EOCIE       = CR1_EOCIE_Msk;
  static constexpr uint32_t CR1_AWDIE_Pos   = ( 6U );
  static constexpr uint32_t CR1_AWDIE_Msk   = ( 0x1UL << CR1_AWDIE_Pos );
  static constexpr uint32_t CR1_AWDIE       = CR1_AWDIE_Msk;
  static constexpr uint32_t CR1_JEOCIE_Pos  = ( 7U );
  static constexpr uint32_t CR1_JEOCIE_Msk  = ( 0x1UL << CR1_JEOCIE_Pos );
  static constexpr uint32_t CR1_JEOCIE      = CR1_JEOCIE_Msk;
  static constexpr uint32_t CR1_SCAN_Pos    = ( 8U );
  static constexpr uint32_t CR1_SCAN_Msk    = ( 0x1UL << CR1_SCAN_Pos );
  static constexpr uint32_t CR1_SCAN        = CR1_SCAN_Msk;
  static constexpr uint32_t CR1_AWDSGL_Pos  = ( 9U );
  static constexpr uint32_t CR1_AWDSGL_Msk  = ( 0x1UL << CR1_AWDSGL_Pos );
  static constexpr uint32_t CR1_AWDSGL      = CR1_AWDSGL_Msk;
  static constexpr uint32_t CR1_JAUTO_Pos   = ( 10U );
  static constexpr uint32_t CR1_JAUTO_Msk   = ( 0x1UL << CR1_JAUTO_Pos );
  static constexpr uint32_t CR1_JAUTO       = CR1_JAUTO_Msk;
  static constexpr uint32_t CR1_DISCEN_Pos  = ( 11U );
  static constexpr uint32_t CR1_DISCEN_Msk  = ( 0x1UL << CR1_DISCEN_Pos );
  static constexpr uint32_t CR1_DISCEN      = CR1_DISCEN_Msk;
  static constexpr uint32_t CR1_JDISCEN_Pos = ( 12U );
  static constexpr uint32_t CR1_JDISCEN_Msk = ( 0x1UL << CR1_JDISCEN_Pos );
  static constexpr uint32_t CR1_JDISCEN     = CR1_JDISCEN_Msk;
  static constexpr uint32_t CR1_DISCNUM_Pos = ( 13U );
  static constexpr uint32_t CR1_DISCNUM_Msk = ( 0x7UL << CR1_DISCNUM_Pos );
  static constexpr uint32_t CR1_DISCNUM     = CR1_DISCNUM_Msk;
  static constexpr uint32_t CR1_DISCNUM_0   = ( 0x1UL << CR1_DISCNUM_Pos );
  static constexpr uint32_t CR1_DISCNUM_1   = ( 0x2UL << CR1_DISCNUM_Pos );
  static constexpr uint32_t CR1_DISCNUM_2   = ( 0x4UL << CR1_DISCNUM_Pos );
  static constexpr uint32_t CR1_JAWDEN_Pos  = ( 22U );
  static constexpr uint32_t CR1_JAWDEN_Msk  = ( 0x1UL << CR1_JAWDEN_Pos );
  static constexpr uint32_t CR1_JAWDEN      = CR1_JAWDEN_Msk;
  static constexpr uint32_t CR1_AWDEN_Pos   = ( 23U );
  static constexpr uint32_t CR1_AWDEN_Msk   = ( 0x1UL << CR1_AWDEN_Pos );
  static constexpr uint32_t CR1_AWDEN       = CR1_AWDEN_Msk;
  static constexpr uint32_t CR1_RES_Pos     = ( 24U );
  static constexpr uint32_t CR1_RES_Msk     = ( 0x3UL << CR1_RES_Pos );
  static constexpr uint32_t CR1_RES         = CR1_RES_Msk;
  static constexpr uint32_t CR1_RES_0       = ( 0x1UL << CR1_RES_Pos );
  static constexpr uint32_t CR1_RES_1       = ( 0x2UL << CR1_RES_Pos );
  static constexpr uint32_t CR1_OVRIE_Pos   = ( 26U );
  static constexpr uint32_t CR1_OVRIE_Msk   = ( 0x1UL << CR1_OVRIE_Pos );
  static constexpr uint32_t CR1_OVRIE       = CR1_OVRIE_Msk;

  /*******************  Bit definition for CR2 register  ********************/
  static constexpr uint32_t CR2_ADON_Pos     = ( 0U );
  static constexpr uint32_t CR2_ADON_Msk     = ( 0x1UL << CR2_ADON_Pos );
  static constexpr uint32_t CR2_ADON         = CR2_ADON_Msk;
  static constexpr uint32_t CR2_CONT_Pos     = ( 1U );
  static constexpr uint32_t CR2_CONT_Msk     = ( 0x1UL << CR2_CONT_Pos );
  static constexpr uint32_t CR2_CONT         = CR2_CONT_Msk;
  static constexpr uint32_t CR2_DMA_Pos      = ( 8U );
  static constexpr uint32_t CR2_DMA_Msk      = ( 0x1UL << CR2_DMA_Pos );
  static constexpr uint32_t CR2_DMA          = CR2_DMA_Msk;
  static constexpr uint32_t CR2_DDS_Pos      = ( 9U );
  static constexpr uint32_t CR2_DDS_Msk      = ( 0x1UL << CR2_DDS_Pos );
  static constexpr uint32_t CR2_DDS          = CR2_DDS_Msk;
  static constexpr uint32_t CR2_EOCS_Pos     = ( 10U );
  static constexpr uint32_t CR2_EOCS_Msk     = ( 0x1UL << CR2_EOCS_Pos );
  static constexpr uint32_t CR2_EOCS         = CR2_EOCS_Msk;
  static constexpr uint32_t CR2_ALIGN_Pos    = ( 11U );
  static constexpr uint32_t CR2_ALIGN_Msk    = ( 0x1UL << CR2_ALIGN_Pos );
  static constexpr uint32_t CR2_ALIGN        = CR2_ALIGN_Msk;
  static constexpr uint32_t CR2_JEXTSEL_Pos  = ( 16U );
  static constexpr uint32_t CR2_JEXTSEL_Msk  = ( 0xFUL << CR2_JEXTSEL_Pos );
  static constexpr uint32_t CR2_JEXTSEL      = CR2_JEXTSEL_Msk;
  static constexpr uint32_t CR2_JEXTSEL_0    = ( 0x1UL << CR2_JEXTSEL_Pos );
  static constexpr uint32_t CR2_JEXTSEL_1    = ( 0x2UL << CR2_JEXTSEL_Pos );
  static constexpr uint32_t CR2_JEXTSEL_2    = ( 0x4UL << CR2_JEXTSEL_Pos );
  static constexpr uint32_t CR2_JEXTSEL_3    = ( 0x8UL << CR2_JEXTSEL_Pos );
  static constexpr uint32_t CR2_JEXTEN_Pos   = ( 20U );
  static constexpr uint32_t CR2_JEXTEN_Msk   = ( 0x3UL << CR2_JEXTEN_Pos );
  static constexpr uint32_t CR2_JEXTEN       = CR2_JEXTEN_Msk;
  static constexpr uint32_t CR2_JEXTEN_0     = ( 0x1UL << CR2_JEXTEN_Pos );
  static constexpr uint32_t CR2_JEXTEN_1     = ( 0x2UL << CR2_JEXTEN_Pos );
  static constexpr uint32_t CR2_JSWSTART_Pos = ( 22U );
  static constexpr uint32_t CR2_JSWSTART_Msk = ( 0x1UL << CR2_JSWSTART_Pos );
  static constexpr uint32_t CR2_JSWSTART     = CR2_JSWSTART_Msk;
  static constexpr uint32_t CR2_EXTSEL_Pos   = ( 24U );
  static constexpr uint32_t CR2_EXTSEL_Msk   = ( 0xFUL << CR2_EXTSEL_Pos );
  static constexpr uint32_t CR2_EXTSEL       = CR2_EXTSEL_Msk;
  static constexpr uint32_t CR2_EXTSEL_0     = ( 0x1UL << CR2_EXTSEL_Pos );
  static constexpr uint32_t CR2_EXTSEL_1     = ( 0x2UL << CR2_EXTSEL_Pos );
  static constexpr uint32_t CR2_EXTSEL_2     = ( 0x4UL << CR2_EXTSEL_Pos );
  static constexpr uint32_t CR2_EXTSEL_3     = ( 0x8UL << CR2_EXTSEL_Pos );
  static constexpr uint32_t CR2_EXTEN_Pos    = ( 28U );
  static constexpr uint32_t CR2_EXTEN_Msk    = ( 0x3UL << CR2_EXTEN_Pos );
  static constexpr uint32_t CR2_EXTEN        = CR2_EXTEN_Msk;
  static constexpr uint32_t CR2_EXTEN_0      = ( 0x1UL << CR2_EXTEN_Pos );
  static constexpr uint32_t CR2_EXTEN_1      = ( 0x2UL << CR2_EXTEN_Pos );
  static constexpr uint32_t CR2_SWSTART_Pos  = ( 30U );
  static constexpr uint32_t CR2_SWSTART_Msk  = ( 0x1UL << CR2_SWSTART_Pos );
  static constexpr uint32_t CR2_SWSTART      = CR2_SWSTART_Msk;

  /******************  Bit definition for SMPR1 register  *******************/
  static constexpr Reg32_t SMPRx_BIT_Wid = 3;
  static constexpr Reg32_t SMPRx_BIT_Msk = 0x7;
  static constexpr Reg32_t SMPR1_ALL_Msk = 0x3FFFFFFF;

  static constexpr uint32_t SMPR1_SMP10_Pos = ( 0U );
  static constexpr uint32_t SMPR1_SMP10_Msk = ( 0x7UL << SMPR1_SMP10_Pos );
  static constexpr uint32_t SMPR1_SMP10     = SMPR1_SMP10_Msk;
  static constexpr uint32_t SMPR1_SMP10_0   = ( 0x1UL << SMPR1_SMP10_Pos );
  static constexpr uint32_t SMPR1_SMP10_1   = ( 0x2UL << SMPR1_SMP10_Pos );
  static constexpr uint32_t SMPR1_SMP10_2   = ( 0x4UL << SMPR1_SMP10_Pos );
  static constexpr uint32_t SMPR1_SMP11_Pos = ( 3U );
  static constexpr uint32_t SMPR1_SMP11_Msk = ( 0x7UL << SMPR1_SMP11_Pos );
  static constexpr uint32_t SMPR1_SMP11     = SMPR1_SMP11_Msk;
  static constexpr uint32_t SMPR1_SMP11_0   = ( 0x1UL << SMPR1_SMP11_Pos );
  static constexpr uint32_t SMPR1_SMP11_1   = ( 0x2UL << SMPR1_SMP11_Pos );
  static constexpr uint32_t SMPR1_SMP11_2   = ( 0x4UL << SMPR1_SMP11_Pos );
  static constexpr uint32_t SMPR1_SMP12_Pos = ( 6U );
  static constexpr uint32_t SMPR1_SMP12_Msk = ( 0x7UL << SMPR1_SMP12_Pos );
  static constexpr uint32_t SMPR1_SMP12     = SMPR1_SMP12_Msk;
  static constexpr uint32_t SMPR1_SMP12_0   = ( 0x1UL << SMPR1_SMP12_Pos );
  static constexpr uint32_t SMPR1_SMP12_1   = ( 0x2UL << SMPR1_SMP12_Pos );
  static constexpr uint32_t SMPR1_SMP12_2   = ( 0x4UL << SMPR1_SMP12_Pos );
  static constexpr uint32_t SMPR1_SMP13_Pos = ( 9U );
  static constexpr uint32_t SMPR1_SMP13_Msk = ( 0x7UL << SMPR1_SMP13_Pos );
  static constexpr uint32_t SMPR1_SMP13     = SMPR1_SMP13_Msk;
  static constexpr uint32_t SMPR1_SMP13_0   = ( 0x1UL << SMPR1_SMP13_Pos );
  static constexpr uint32_t SMPR1_SMP13_1   = ( 0x2UL << SMPR1_SMP13_Pos );
  static constexpr uint32_t SMPR1_SMP13_2   = ( 0x4UL << SMPR1_SMP13_Pos );
  static constexpr uint32_t SMPR1_SMP14_Pos = ( 12U );
  static constexpr uint32_t SMPR1_SMP14_Msk = ( 0x7UL << SMPR1_SMP14_Pos );
  static constexpr uint32_t SMPR1_SMP14     = SMPR1_SMP14_Msk;
  static constexpr uint32_t SMPR1_SMP14_0   = ( 0x1UL << SMPR1_SMP14_Pos );
  static constexpr uint32_t SMPR1_SMP14_1   = ( 0x2UL << SMPR1_SMP14_Pos );
  static constexpr uint32_t SMPR1_SMP14_2   = ( 0x4UL << SMPR1_SMP14_Pos );
  static constexpr uint32_t SMPR1_SMP15_Pos = ( 15U );
  static constexpr uint32_t SMPR1_SMP15_Msk = ( 0x7UL << SMPR1_SMP15_Pos );
  static constexpr uint32_t SMPR1_SMP15     = SMPR1_SMP15_Msk;
  static constexpr uint32_t SMPR1_SMP15_0   = ( 0x1UL << SMPR1_SMP15_Pos );
  static constexpr uint32_t SMPR1_SMP15_1   = ( 0x2UL << SMPR1_SMP15_Pos );
  static constexpr uint32_t SMPR1_SMP15_2   = ( 0x4UL << SMPR1_SMP15_Pos );
  static constexpr uint32_t SMPR1_SMP16_Pos = ( 18U );
  static constexpr uint32_t SMPR1_SMP16_Msk = ( 0x7UL << SMPR1_SMP16_Pos );
  static constexpr uint32_t SMPR1_SMP16     = SMPR1_SMP16_Msk;
  static constexpr uint32_t SMPR1_SMP16_0   = ( 0x1UL << SMPR1_SMP16_Pos );
  static constexpr uint32_t SMPR1_SMP16_1   = ( 0x2UL << SMPR1_SMP16_Pos );
  static constexpr uint32_t SMPR1_SMP16_2   = ( 0x4UL << SMPR1_SMP16_Pos );
  static constexpr uint32_t SMPR1_SMP17_Pos = ( 21U );
  static constexpr uint32_t SMPR1_SMP17_Msk = ( 0x7UL << SMPR1_SMP17_Pos );
  static constexpr uint32_t SMPR1_SMP17     = SMPR1_SMP17_Msk;
  static constexpr uint32_t SMPR1_SMP17_0   = ( 0x1UL << SMPR1_SMP17_Pos );
  static constexpr uint32_t SMPR1_SMP17_1   = ( 0x2UL << SMPR1_SMP17_Pos );
  static constexpr uint32_t SMPR1_SMP17_2   = ( 0x4UL << SMPR1_SMP17_Pos );
  static constexpr uint32_t SMPR1_SMP18_Pos = ( 24U );
  static constexpr uint32_t SMPR1_SMP18_Msk = ( 0x7UL << SMPR1_SMP18_Pos );
  static constexpr uint32_t SMPR1_SMP18     = SMPR1_SMP18_Msk;
  static constexpr uint32_t SMPR1_SMP18_0   = ( 0x1UL << SMPR1_SMP18_Pos );
  static constexpr uint32_t SMPR1_SMP18_1   = ( 0x2UL << SMPR1_SMP18_Pos );
  static constexpr uint32_t SMPR1_SMP18_2   = ( 0x4UL << SMPR1_SMP18_Pos );

  /******************  Bit definition for SMPR2 register  *******************/
  static constexpr Reg32_t SMPR2_ALL_Msk = 0x07FFFFFF;

  static constexpr uint32_t SMPR2_SMP0_Pos = ( 0U );
  static constexpr uint32_t SMPR2_SMP0_Msk = ( 0x7UL << SMPR2_SMP0_Pos );
  static constexpr uint32_t SMPR2_SMP0     = SMPR2_SMP0_Msk;
  static constexpr uint32_t SMPR2_SMP0_0   = ( 0x1UL << SMPR2_SMP0_Pos );
  static constexpr uint32_t SMPR2_SMP0_1   = ( 0x2UL << SMPR2_SMP0_Pos );
  static constexpr uint32_t SMPR2_SMP0_2   = ( 0x4UL << SMPR2_SMP0_Pos );
  static constexpr uint32_t SMPR2_SMP1_Pos = ( 3U );
  static constexpr uint32_t SMPR2_SMP1_Msk = ( 0x7UL << SMPR2_SMP1_Pos );
  static constexpr uint32_t SMPR2_SMP1     = SMPR2_SMP1_Msk;
  static constexpr uint32_t SMPR2_SMP1_0   = ( 0x1UL << SMPR2_SMP1_Pos );
  static constexpr uint32_t SMPR2_SMP1_1   = ( 0x2UL << SMPR2_SMP1_Pos );
  static constexpr uint32_t SMPR2_SMP1_2   = ( 0x4UL << SMPR2_SMP1_Pos );
  static constexpr uint32_t SMPR2_SMP2_Pos = ( 6U );
  static constexpr uint32_t SMPR2_SMP2_Msk = ( 0x7UL << SMPR2_SMP2_Pos );
  static constexpr uint32_t SMPR2_SMP2     = SMPR2_SMP2_Msk;
  static constexpr uint32_t SMPR2_SMP2_0   = ( 0x1UL << SMPR2_SMP2_Pos );
  static constexpr uint32_t SMPR2_SMP2_1   = ( 0x2UL << SMPR2_SMP2_Pos );
  static constexpr uint32_t SMPR2_SMP2_2   = ( 0x4UL << SMPR2_SMP2_Pos );
  static constexpr uint32_t SMPR2_SMP3_Pos = ( 9U );
  static constexpr uint32_t SMPR2_SMP3_Msk = ( 0x7UL << SMPR2_SMP3_Pos );
  static constexpr uint32_t SMPR2_SMP3     = SMPR2_SMP3_Msk;
  static constexpr uint32_t SMPR2_SMP3_0   = ( 0x1UL << SMPR2_SMP3_Pos );
  static constexpr uint32_t SMPR2_SMP3_1   = ( 0x2UL << SMPR2_SMP3_Pos );
  static constexpr uint32_t SMPR2_SMP3_2   = ( 0x4UL << SMPR2_SMP3_Pos );
  static constexpr uint32_t SMPR2_SMP4_Pos = ( 12U );
  static constexpr uint32_t SMPR2_SMP4_Msk = ( 0x7UL << SMPR2_SMP4_Pos );
  static constexpr uint32_t SMPR2_SMP4     = SMPR2_SMP4_Msk;
  static constexpr uint32_t SMPR2_SMP4_0   = ( 0x1UL << SMPR2_SMP4_Pos );
  static constexpr uint32_t SMPR2_SMP4_1   = ( 0x2UL << SMPR2_SMP4_Pos );
  static constexpr uint32_t SMPR2_SMP4_2   = ( 0x4UL << SMPR2_SMP4_Pos );
  static constexpr uint32_t SMPR2_SMP5_Pos = ( 15U );
  static constexpr uint32_t SMPR2_SMP5_Msk = ( 0x7UL << SMPR2_SMP5_Pos );
  static constexpr uint32_t SMPR2_SMP5     = SMPR2_SMP5_Msk;
  static constexpr uint32_t SMPR2_SMP5_0   = ( 0x1UL << SMPR2_SMP5_Pos );
  static constexpr uint32_t SMPR2_SMP5_1   = ( 0x2UL << SMPR2_SMP5_Pos );
  static constexpr uint32_t SMPR2_SMP5_2   = ( 0x4UL << SMPR2_SMP5_Pos );
  static constexpr uint32_t SMPR2_SMP6_Pos = ( 18U );
  static constexpr uint32_t SMPR2_SMP6_Msk = ( 0x7UL << SMPR2_SMP6_Pos );
  static constexpr uint32_t SMPR2_SMP6     = SMPR2_SMP6_Msk;
  static constexpr uint32_t SMPR2_SMP6_0   = ( 0x1UL << SMPR2_SMP6_Pos );
  static constexpr uint32_t SMPR2_SMP6_1   = ( 0x2UL << SMPR2_SMP6_Pos );
  static constexpr uint32_t SMPR2_SMP6_2   = ( 0x4UL << SMPR2_SMP6_Pos );
  static constexpr uint32_t SMPR2_SMP7_Pos = ( 21U );
  static constexpr uint32_t SMPR2_SMP7_Msk = ( 0x7UL << SMPR2_SMP7_Pos );
  static constexpr uint32_t SMPR2_SMP7     = SMPR2_SMP7_Msk;
  static constexpr uint32_t SMPR2_SMP7_0   = ( 0x1UL << SMPR2_SMP7_Pos );
  static constexpr uint32_t SMPR2_SMP7_1   = ( 0x2UL << SMPR2_SMP7_Pos );
  static constexpr uint32_t SMPR2_SMP7_2   = ( 0x4UL << SMPR2_SMP7_Pos );
  static constexpr uint32_t SMPR2_SMP8_Pos = ( 24U );
  static constexpr uint32_t SMPR2_SMP8_Msk = ( 0x7UL << SMPR2_SMP8_Pos );
  static constexpr uint32_t SMPR2_SMP8     = SMPR2_SMP8_Msk;
  static constexpr uint32_t SMPR2_SMP8_0   = ( 0x1UL << SMPR2_SMP8_Pos );
  static constexpr uint32_t SMPR2_SMP8_1   = ( 0x2UL << SMPR2_SMP8_Pos );
  static constexpr uint32_t SMPR2_SMP8_2   = ( 0x4UL << SMPR2_SMP8_Pos );
  static constexpr uint32_t SMPR2_SMP9_Pos = ( 27U );
  static constexpr uint32_t SMPR2_SMP9_Msk = ( 0x7UL << SMPR2_SMP9_Pos );
  static constexpr uint32_t SMPR2_SMP9     = SMPR2_SMP9_Msk;
  static constexpr uint32_t SMPR2_SMP9_0   = ( 0x1UL << SMPR2_SMP9_Pos );
  static constexpr uint32_t SMPR2_SMP9_1   = ( 0x2UL << SMPR2_SMP9_Pos );
  static constexpr uint32_t SMPR2_SMP9_2   = ( 0x4UL << SMPR2_SMP9_Pos );

  /******************  Bit definition for JOFR1 register  *******************/
  static constexpr uint32_t JOFR1_JOFFSET1_Pos = ( 0U );
  static constexpr uint32_t JOFR1_JOFFSET1_Msk = ( 0xFFFUL << JOFR1_JOFFSET1_Pos );
  static constexpr uint32_t JOFR1_JOFFSET1     = JOFR1_JOFFSET1_Msk;

  /******************  Bit definition for JOFR2 register  *******************/
  static constexpr uint32_t JOFR2_JOFFSET2_Pos = ( 0U );
  static constexpr uint32_t JOFR2_JOFFSET2_Msk = ( 0xFFFUL << JOFR2_JOFFSET2_Pos );
  static constexpr uint32_t JOFR2_JOFFSET2     = JOFR2_JOFFSET2_Msk;

  /******************  Bit definition for JOFR3 register  *******************/
  static constexpr uint32_t JOFR3_JOFFSET3_Pos = ( 0U );
  static constexpr uint32_t JOFR3_JOFFSET3_Msk = ( 0xFFFUL << JOFR3_JOFFSET3_Pos );
  static constexpr uint32_t JOFR3_JOFFSET3     = JOFR3_JOFFSET3_Msk;

  /******************  Bit definition for JOFR4 register  *******************/
  static constexpr uint32_t JOFR4_JOFFSET4_Pos = ( 0U );
  static constexpr uint32_t JOFR4_JOFFSET4_Msk = ( 0xFFFUL << JOFR4_JOFFSET4_Pos );
  static constexpr uint32_t JOFR4_JOFFSET4     = JOFR4_JOFFSET4_Msk;

  /*******************  Bit definition for HTR register  ********************/
  static constexpr uint32_t HTR_HT_Pos = ( 0U );
  static constexpr uint32_t HTR_HT_Msk = ( 0xFFFUL << HTR_HT_Pos );
  static constexpr uint32_t HTR_HT     = HTR_HT_Msk;

  /*******************  Bit definition for LTR register  ********************/
  static constexpr uint32_t LTR_LT_Pos = ( 0U );
  static constexpr uint32_t LTR_LT_Msk = ( 0xFFFUL << LTR_LT_Pos );
  static constexpr uint32_t LTR_LT     = LTR_LT_Msk;

  /*******************  Bit definition for SQR1 register  *******************/
  static constexpr uint32_t SQR1_SQ13_Pos = ( 0U );
  static constexpr uint32_t SQR1_SQ13_Msk = ( 0x1FUL << SQR1_SQ13_Pos );
  static constexpr uint32_t SQR1_SQ13     = SQR1_SQ13_Msk;
  static constexpr uint32_t SQR1_SQ13_0   = ( 0x01UL << SQR1_SQ13_Pos );
  static constexpr uint32_t SQR1_SQ13_1   = ( 0x02UL << SQR1_SQ13_Pos );
  static constexpr uint32_t SQR1_SQ13_2   = ( 0x04UL << SQR1_SQ13_Pos );
  static constexpr uint32_t SQR1_SQ13_3   = ( 0x08UL << SQR1_SQ13_Pos );
  static constexpr uint32_t SQR1_SQ13_4   = ( 0x10UL << SQR1_SQ13_Pos );
  static constexpr uint32_t SQR1_SQ14_Pos = ( 5U );
  static constexpr uint32_t SQR1_SQ14_Msk = ( 0x1FUL << SQR1_SQ14_Pos );
  static constexpr uint32_t SQR1_SQ14     = SQR1_SQ14_Msk;
  static constexpr uint32_t SQR1_SQ14_0   = ( 0x01UL << SQR1_SQ14_Pos );
  static constexpr uint32_t SQR1_SQ14_1   = ( 0x02UL << SQR1_SQ14_Pos );
  static constexpr uint32_t SQR1_SQ14_2   = ( 0x04UL << SQR1_SQ14_Pos );
  static constexpr uint32_t SQR1_SQ14_3   = ( 0x08UL << SQR1_SQ14_Pos );
  static constexpr uint32_t SQR1_SQ14_4   = ( 0x10UL << SQR1_SQ14_Pos );
  static constexpr uint32_t SQR1_SQ15_Pos = ( 10U );
  static constexpr uint32_t SQR1_SQ15_Msk = ( 0x1FUL << SQR1_SQ15_Pos );
  static constexpr uint32_t SQR1_SQ15     = SQR1_SQ15_Msk;
  static constexpr uint32_t SQR1_SQ15_0   = ( 0x01UL << SQR1_SQ15_Pos );
  static constexpr uint32_t SQR1_SQ15_1   = ( 0x02UL << SQR1_SQ15_Pos );
  static constexpr uint32_t SQR1_SQ15_2   = ( 0x04UL << SQR1_SQ15_Pos );
  static constexpr uint32_t SQR1_SQ15_3   = ( 0x08UL << SQR1_SQ15_Pos );
  static constexpr uint32_t SQR1_SQ15_4   = ( 0x10UL << SQR1_SQ15_Pos );
  static constexpr uint32_t SQR1_SQ16_Pos = ( 15U );
  static constexpr uint32_t SQR1_SQ16_Msk = ( 0x1FUL << SQR1_SQ16_Pos );
  static constexpr uint32_t SQR1_SQ16     = SQR1_SQ16_Msk;
  static constexpr uint32_t SQR1_SQ16_0   = ( 0x01UL << SQR1_SQ16_Pos );
  static constexpr uint32_t SQR1_SQ16_1   = ( 0x02UL << SQR1_SQ16_Pos );
  static constexpr uint32_t SQR1_SQ16_2   = ( 0x04UL << SQR1_SQ16_Pos );
  static constexpr uint32_t SQR1_SQ16_3   = ( 0x08UL << SQR1_SQ16_Pos );
  static constexpr uint32_t SQR1_SQ16_4   = ( 0x10UL << SQR1_SQ16_Pos );
  static constexpr uint32_t SQR1_L_Pos    = ( 20U );
  static constexpr uint32_t SQR1_L_Msk    = ( 0xFUL << SQR1_L_Pos );
  static constexpr uint32_t SQR1_L        = SQR1_L_Msk;
  static constexpr uint32_t SQR1_L_0      = ( 0x1UL << SQR1_L_Pos );
  static constexpr uint32_t SQR1_L_1      = ( 0x2UL << SQR1_L_Pos );
  static constexpr uint32_t SQR1_L_2      = ( 0x4UL << SQR1_L_Pos );
  static constexpr uint32_t SQR1_L_3      = ( 0x8UL << SQR1_L_Pos );

  /*******************  Bit definition for SQR2 register  *******************/
  static constexpr uint32_t SQR2_SQ7_Pos  = ( 0U );
  static constexpr uint32_t SQR2_SQ7_Msk  = ( 0x1FUL << SQR2_SQ7_Pos );
  static constexpr uint32_t SQR2_SQ7      = SQR2_SQ7_Msk;
  static constexpr uint32_t SQR2_SQ7_0    = ( 0x01UL << SQR2_SQ7_Pos );
  static constexpr uint32_t SQR2_SQ7_1    = ( 0x02UL << SQR2_SQ7_Pos );
  static constexpr uint32_t SQR2_SQ7_2    = ( 0x04UL << SQR2_SQ7_Pos );
  static constexpr uint32_t SQR2_SQ7_3    = ( 0x08UL << SQR2_SQ7_Pos );
  static constexpr uint32_t SQR2_SQ7_4    = ( 0x10UL << SQR2_SQ7_Pos );
  static constexpr uint32_t SQR2_SQ8_Pos  = ( 5U );
  static constexpr uint32_t SQR2_SQ8_Msk  = ( 0x1FUL << SQR2_SQ8_Pos );
  static constexpr uint32_t SQR2_SQ8      = SQR2_SQ8_Msk;
  static constexpr uint32_t SQR2_SQ8_0    = ( 0x01UL << SQR2_SQ8_Pos );
  static constexpr uint32_t SQR2_SQ8_1    = ( 0x02UL << SQR2_SQ8_Pos );
  static constexpr uint32_t SQR2_SQ8_2    = ( 0x04UL << SQR2_SQ8_Pos );
  static constexpr uint32_t SQR2_SQ8_3    = ( 0x08UL << SQR2_SQ8_Pos );
  static constexpr uint32_t SQR2_SQ8_4    = ( 0x10UL << SQR2_SQ8_Pos );
  static constexpr uint32_t SQR2_SQ9_Pos  = ( 10U );
  static constexpr uint32_t SQR2_SQ9_Msk  = ( 0x1FUL << SQR2_SQ9_Pos );
  static constexpr uint32_t SQR2_SQ9      = SQR2_SQ9_Msk;
  static constexpr uint32_t SQR2_SQ9_0    = ( 0x01UL << SQR2_SQ9_Pos );
  static constexpr uint32_t SQR2_SQ9_1    = ( 0x02UL << SQR2_SQ9_Pos );
  static constexpr uint32_t SQR2_SQ9_2    = ( 0x04UL << SQR2_SQ9_Pos );
  static constexpr uint32_t SQR2_SQ9_3    = ( 0x08UL << SQR2_SQ9_Pos );
  static constexpr uint32_t SQR2_SQ9_4    = ( 0x10UL << SQR2_SQ9_Pos );
  static constexpr uint32_t SQR2_SQ10_Pos = ( 15U );
  static constexpr uint32_t SQR2_SQ10_Msk = ( 0x1FUL << SQR2_SQ10_Pos );
  static constexpr uint32_t SQR2_SQ10     = SQR2_SQ10_Msk;
  static constexpr uint32_t SQR2_SQ10_0   = ( 0x01UL << SQR2_SQ10_Pos );
  static constexpr uint32_t SQR2_SQ10_1   = ( 0x02UL << SQR2_SQ10_Pos );
  static constexpr uint32_t SQR2_SQ10_2   = ( 0x04UL << SQR2_SQ10_Pos );
  static constexpr uint32_t SQR2_SQ10_3   = ( 0x08UL << SQR2_SQ10_Pos );
  static constexpr uint32_t SQR2_SQ10_4   = ( 0x10UL << SQR2_SQ10_Pos );
  static constexpr uint32_t SQR2_SQ11_Pos = ( 20U );
  static constexpr uint32_t SQR2_SQ11_Msk = ( 0x1FUL << SQR2_SQ11_Pos );
  static constexpr uint32_t SQR2_SQ11     = SQR2_SQ11_Msk;
  static constexpr uint32_t SQR2_SQ11_0   = ( 0x01UL << SQR2_SQ11_Pos );
  static constexpr uint32_t SQR2_SQ11_1   = ( 0x02UL << SQR2_SQ11_Pos );
  static constexpr uint32_t SQR2_SQ11_2   = ( 0x04UL << SQR2_SQ11_Pos );
  static constexpr uint32_t SQR2_SQ11_3   = ( 0x08UL << SQR2_SQ11_Pos );
  static constexpr uint32_t SQR2_SQ11_4   = ( 0x10UL << SQR2_SQ11_Pos );
  static constexpr uint32_t SQR2_SQ12_Pos = ( 25U );
  static constexpr uint32_t SQR2_SQ12_Msk = ( 0x1FUL << SQR2_SQ12_Pos );
  static constexpr uint32_t SQR2_SQ12     = SQR2_SQ12_Msk;
  static constexpr uint32_t SQR2_SQ12_0   = ( 0x01UL << SQR2_SQ12_Pos );
  static constexpr uint32_t SQR2_SQ12_1   = ( 0x02UL << SQR2_SQ12_Pos );
  static constexpr uint32_t SQR2_SQ12_2   = ( 0x04UL << SQR2_SQ12_Pos );
  static constexpr uint32_t SQR2_SQ12_3   = ( 0x08UL << SQR2_SQ12_Pos );
  static constexpr uint32_t SQR2_SQ12_4   = ( 0x10UL << SQR2_SQ12_Pos );

  /*******************  Bit definition for SQR3 register  *******************/
  static constexpr uint32_t SQR3_SQ1_Pos = ( 0U );
  static constexpr uint32_t SQR3_SQ1_Msk = ( 0x1FUL << SQR3_SQ1_Pos );
  static constexpr uint32_t SQR3_SQ1     = SQR3_SQ1_Msk;
  static constexpr uint32_t SQR3_SQ1_0   = ( 0x01UL << SQR3_SQ1_Pos );
  static constexpr uint32_t SQR3_SQ1_1   = ( 0x02UL << SQR3_SQ1_Pos );
  static constexpr uint32_t SQR3_SQ1_2   = ( 0x04UL << SQR3_SQ1_Pos );
  static constexpr uint32_t SQR3_SQ1_3   = ( 0x08UL << SQR3_SQ1_Pos );
  static constexpr uint32_t SQR3_SQ1_4   = ( 0x10UL << SQR3_SQ1_Pos );
  static constexpr uint32_t SQR3_SQ2_Pos = ( 5U );
  static constexpr uint32_t SQR3_SQ2_Msk = ( 0x1FUL << SQR3_SQ2_Pos );
  static constexpr uint32_t SQR3_SQ2     = SQR3_SQ2_Msk;
  static constexpr uint32_t SQR3_SQ2_0   = ( 0x01UL << SQR3_SQ2_Pos );
  static constexpr uint32_t SQR3_SQ2_1   = ( 0x02UL << SQR3_SQ2_Pos );
  static constexpr uint32_t SQR3_SQ2_2   = ( 0x04UL << SQR3_SQ2_Pos );
  static constexpr uint32_t SQR3_SQ2_3   = ( 0x08UL << SQR3_SQ2_Pos );
  static constexpr uint32_t SQR3_SQ2_4   = ( 0x10UL << SQR3_SQ2_Pos );
  static constexpr uint32_t SQR3_SQ3_Pos = ( 10U );
  static constexpr uint32_t SQR3_SQ3_Msk = ( 0x1FUL << SQR3_SQ3_Pos );
  static constexpr uint32_t SQR3_SQ3     = SQR3_SQ3_Msk;
  static constexpr uint32_t SQR3_SQ3_0   = ( 0x01UL << SQR3_SQ3_Pos );
  static constexpr uint32_t SQR3_SQ3_1   = ( 0x02UL << SQR3_SQ3_Pos );
  static constexpr uint32_t SQR3_SQ3_2   = ( 0x04UL << SQR3_SQ3_Pos );
  static constexpr uint32_t SQR3_SQ3_3   = ( 0x08UL << SQR3_SQ3_Pos );
  static constexpr uint32_t SQR3_SQ3_4   = ( 0x10UL << SQR3_SQ3_Pos );
  static constexpr uint32_t SQR3_SQ4_Pos = ( 15U );
  static constexpr uint32_t SQR3_SQ4_Msk = ( 0x1FUL << SQR3_SQ4_Pos );
  static constexpr uint32_t SQR3_SQ4     = SQR3_SQ4_Msk;
  static constexpr uint32_t SQR3_SQ4_0   = ( 0x01UL << SQR3_SQ4_Pos );
  static constexpr uint32_t SQR3_SQ4_1   = ( 0x02UL << SQR3_SQ4_Pos );
  static constexpr uint32_t SQR3_SQ4_2   = ( 0x04UL << SQR3_SQ4_Pos );
  static constexpr uint32_t SQR3_SQ4_3   = ( 0x08UL << SQR3_SQ4_Pos );
  static constexpr uint32_t SQR3_SQ4_4   = ( 0x10UL << SQR3_SQ4_Pos );
  static constexpr uint32_t SQR3_SQ5_Pos = ( 20U );
  static constexpr uint32_t SQR3_SQ5_Msk = ( 0x1FUL << SQR3_SQ5_Pos );
  static constexpr uint32_t SQR3_SQ5     = SQR3_SQ5_Msk;
  static constexpr uint32_t SQR3_SQ5_0   = ( 0x01UL << SQR3_SQ5_Pos );
  static constexpr uint32_t SQR3_SQ5_1   = ( 0x02UL << SQR3_SQ5_Pos );
  static constexpr uint32_t SQR3_SQ5_2   = ( 0x04UL << SQR3_SQ5_Pos );
  static constexpr uint32_t SQR3_SQ5_3   = ( 0x08UL << SQR3_SQ5_Pos );
  static constexpr uint32_t SQR3_SQ5_4   = ( 0x10UL << SQR3_SQ5_Pos );
  static constexpr uint32_t SQR3_SQ6_Pos = ( 25U );
  static constexpr uint32_t SQR3_SQ6_Msk = ( 0x1FUL << SQR3_SQ6_Pos );
  static constexpr uint32_t SQR3_SQ6     = SQR3_SQ6_Msk;
  static constexpr uint32_t SQR3_SQ6_0   = ( 0x01UL << SQR3_SQ6_Pos );
  static constexpr uint32_t SQR3_SQ6_1   = ( 0x02UL << SQR3_SQ6_Pos );
  static constexpr uint32_t SQR3_SQ6_2   = ( 0x04UL << SQR3_SQ6_Pos );
  static constexpr uint32_t SQR3_SQ6_3   = ( 0x08UL << SQR3_SQ6_Pos );
  static constexpr uint32_t SQR3_SQ6_4   = ( 0x10UL << SQR3_SQ6_Pos );

  /*******************  Bit definition for JSQR register  *******************/
  static constexpr uint32_t JSQR_JSQ1_Pos = ( 0U );
  static constexpr uint32_t JSQR_JSQ1_Msk = ( 0x1FUL << JSQR_JSQ1_Pos );
  static constexpr uint32_t JSQR_JSQ1     = JSQR_JSQ1_Msk;
  static constexpr uint32_t JSQR_JSQ1_0   = ( 0x01UL << JSQR_JSQ1_Pos );
  static constexpr uint32_t JSQR_JSQ1_1   = ( 0x02UL << JSQR_JSQ1_Pos );
  static constexpr uint32_t JSQR_JSQ1_2   = ( 0x04UL << JSQR_JSQ1_Pos );
  static constexpr uint32_t JSQR_JSQ1_3   = ( 0x08UL << JSQR_JSQ1_Pos );
  static constexpr uint32_t JSQR_JSQ1_4   = ( 0x10UL << JSQR_JSQ1_Pos );
  static constexpr uint32_t JSQR_JSQ2_Pos = ( 5U );
  static constexpr uint32_t JSQR_JSQ2_Msk = ( 0x1FUL << JSQR_JSQ2_Pos );
  static constexpr uint32_t JSQR_JSQ2     = JSQR_JSQ2_Msk;
  static constexpr uint32_t JSQR_JSQ2_0   = ( 0x01UL << JSQR_JSQ2_Pos );
  static constexpr uint32_t JSQR_JSQ2_1   = ( 0x02UL << JSQR_JSQ2_Pos );
  static constexpr uint32_t JSQR_JSQ2_2   = ( 0x04UL << JSQR_JSQ2_Pos );
  static constexpr uint32_t JSQR_JSQ2_3   = ( 0x08UL << JSQR_JSQ2_Pos );
  static constexpr uint32_t JSQR_JSQ2_4   = ( 0x10UL << JSQR_JSQ2_Pos );
  static constexpr uint32_t JSQR_JSQ3_Pos = ( 10U );
  static constexpr uint32_t JSQR_JSQ3_Msk = ( 0x1FUL << JSQR_JSQ3_Pos );
  static constexpr uint32_t JSQR_JSQ3     = JSQR_JSQ3_Msk;
  static constexpr uint32_t JSQR_JSQ3_0   = ( 0x01UL << JSQR_JSQ3_Pos );
  static constexpr uint32_t JSQR_JSQ3_1   = ( 0x02UL << JSQR_JSQ3_Pos );
  static constexpr uint32_t JSQR_JSQ3_2   = ( 0x04UL << JSQR_JSQ3_Pos );
  static constexpr uint32_t JSQR_JSQ3_3   = ( 0x08UL << JSQR_JSQ3_Pos );
  static constexpr uint32_t JSQR_JSQ3_4   = ( 0x10UL << JSQR_JSQ3_Pos );
  static constexpr uint32_t JSQR_JSQ4_Pos = ( 15U );
  static constexpr uint32_t JSQR_JSQ4_Msk = ( 0x1FUL << JSQR_JSQ4_Pos );
  static constexpr uint32_t JSQR_JSQ4     = JSQR_JSQ4_Msk;
  static constexpr uint32_t JSQR_JSQ4_0   = ( 0x01UL << JSQR_JSQ4_Pos );
  static constexpr uint32_t JSQR_JSQ4_1   = ( 0x02UL << JSQR_JSQ4_Pos );
  static constexpr uint32_t JSQR_JSQ4_2   = ( 0x04UL << JSQR_JSQ4_Pos );
  static constexpr uint32_t JSQR_JSQ4_3   = ( 0x08UL << JSQR_JSQ4_Pos );
  static constexpr uint32_t JSQR_JSQ4_4   = ( 0x10UL << JSQR_JSQ4_Pos );
  static constexpr uint32_t JSQR_JL_Pos   = ( 20U );
  static constexpr uint32_t JSQR_JL_Msk   = ( 0x3UL << JSQR_JL_Pos );
  static constexpr uint32_t JSQR_JL       = JSQR_JL_Msk;
  static constexpr uint32_t JSQR_JL_0     = ( 0x1UL << JSQR_JL_Pos );
  static constexpr uint32_t JSQR_JL_1     = ( 0x2UL << JSQR_JL_Pos );

  /*******************  Bit definition for JDR1 register  *******************/
  static constexpr uint32_t JDR1_JDATA_Pos = ( 0U );
  static constexpr uint32_t JDR1_JDATA_Msk = ( 0xFFFFUL << JDR1_JDATA_Pos );
  static constexpr uint32_t JDR1_JDATA     = JDR1_JDATA_Msk;

  /*******************  Bit definition for JDR2 register  *******************/
  static constexpr uint32_t JDR2_JDATA_Pos = ( 0U );
  static constexpr uint32_t JDR2_JDATA_Msk = ( 0xFFFFUL << JDR2_JDATA_Pos );
  static constexpr uint32_t JDR2_JDATA     = JDR2_JDATA_Msk;

  /*******************  Bit definition for JDR3 register  *******************/
  static constexpr uint32_t JDR3_JDATA_Pos = ( 0U );
  static constexpr uint32_t JDR3_JDATA_Msk = ( 0xFFFFUL << JDR3_JDATA_Pos );
  static constexpr uint32_t JDR3_JDATA     = JDR3_JDATA_Msk;

  /*******************  Bit definition for JDR4 register  *******************/
  static constexpr uint32_t JDR4_JDATA_Pos = ( 0U );
  static constexpr uint32_t JDR4_JDATA_Msk = ( 0xFFFFUL << JDR4_JDATA_Pos );
  static constexpr uint32_t JDR4_JDATA     = JDR4_JDATA_Msk;

  /********************  Bit definition for DR register  ********************/
  static constexpr uint32_t DR_DATA_Pos     = ( 0U );
  static constexpr uint32_t DR_DATA_Msk     = ( 0xFFFFUL << DR_DATA_Pos );
  static constexpr uint32_t DR_DATA         = DR_DATA_Msk;
  static constexpr uint32_t DR_ADC2DATA_Pos = ( 16U );
  static constexpr uint32_t DR_ADC2DATA_Msk = ( 0xFFFFUL << DR_ADC2DATA_Pos );
  static constexpr uint32_t DR_ADC2DATA     = DR_ADC2DATA_Msk;

  /*******************  Bit definition for CSR register  ********************/
  static constexpr uint32_t CSR_AWD1_Pos   = ( 0U );
  static constexpr uint32_t CSR_AWD1_Msk   = ( 0x1UL << CSR_AWD1_Pos );
  static constexpr uint32_t CSR_AWD1       = CSR_AWD1_Msk;
  static constexpr uint32_t CSR_EOC1_Pos   = ( 1U );
  static constexpr uint32_t CSR_EOC1_Msk   = ( 0x1UL << CSR_EOC1_Pos );
  static constexpr uint32_t CSR_EOC1       = CSR_EOC1_Msk;
  static constexpr uint32_t CSR_JEOC1_Pos  = ( 2U );
  static constexpr uint32_t CSR_JEOC1_Msk  = ( 0x1UL << CSR_JEOC1_Pos );
  static constexpr uint32_t CSR_JEOC1      = CSR_JEOC1_Msk;
  static constexpr uint32_t CSR_JSTRT1_Pos = ( 3U );
  static constexpr uint32_t CSR_JSTRT1_Msk = ( 0x1UL << CSR_JSTRT1_Pos );
  static constexpr uint32_t CSR_JSTRT1     = CSR_JSTRT1_Msk;
  static constexpr uint32_t CSR_STRT1_Pos  = ( 4U );
  static constexpr uint32_t CSR_STRT1_Msk  = ( 0x1UL << CSR_STRT1_Pos );
  static constexpr uint32_t CSR_STRT1      = CSR_STRT1_Msk;
  static constexpr uint32_t CSR_OVR1_Pos   = ( 5U );
  static constexpr uint32_t CSR_OVR1_Msk   = ( 0x1UL << CSR_OVR1_Pos );
  static constexpr uint32_t CSR_OVR1       = CSR_OVR1_Msk;
  static constexpr uint32_t CSR_AWD2_Pos   = ( 8U );
  static constexpr uint32_t CSR_AWD2_Msk   = ( 0x1UL << CSR_AWD2_Pos );
  static constexpr uint32_t CSR_AWD2       = CSR_AWD2_Msk;
  static constexpr uint32_t CSR_EOC2_Pos   = ( 9U );
  static constexpr uint32_t CSR_EOC2_Msk   = ( 0x1UL << CSR_EOC2_Pos );
  static constexpr uint32_t CSR_EOC2       = CSR_EOC2_Msk;
  static constexpr uint32_t CSR_JEOC2_Pos  = ( 10U );
  static constexpr uint32_t CSR_JEOC2_Msk  = ( 0x1UL << CSR_JEOC2_Pos );
  static constexpr uint32_t CSR_JEOC2      = CSR_JEOC2_Msk;
  static constexpr uint32_t CSR_JSTRT2_Pos = ( 11U );
  static constexpr uint32_t CSR_JSTRT2_Msk = ( 0x1UL << CSR_JSTRT2_Pos );
  static constexpr uint32_t CSR_JSTRT2     = CSR_JSTRT2_Msk;
  static constexpr uint32_t CSR_STRT2_Pos  = ( 12U );
  static constexpr uint32_t CSR_STRT2_Msk  = ( 0x1UL << CSR_STRT2_Pos );
  static constexpr uint32_t CSR_STRT2      = CSR_STRT2_Msk;
  static constexpr uint32_t CSR_OVR2_Pos   = ( 13U );
  static constexpr uint32_t CSR_OVR2_Msk   = ( 0x1UL << CSR_OVR2_Pos );
  static constexpr uint32_t CSR_OVR2       = CSR_OVR2_Msk;
  static constexpr uint32_t CSR_AWD3_Pos   = ( 16U );
  static constexpr uint32_t CSR_AWD3_Msk   = ( 0x1UL << CSR_AWD3_Pos );
  static constexpr uint32_t CSR_AWD3       = CSR_AWD3_Msk;
  static constexpr uint32_t CSR_EOC3_Pos   = ( 17U );
  static constexpr uint32_t CSR_EOC3_Msk   = ( 0x1UL << CSR_EOC3_Pos );
  static constexpr uint32_t CSR_EOC3       = CSR_EOC3_Msk;
  static constexpr uint32_t CSR_JEOC3_Pos  = ( 18U );
  static constexpr uint32_t CSR_JEOC3_Msk  = ( 0x1UL << CSR_JEOC3_Pos );
  static constexpr uint32_t CSR_JEOC3      = CSR_JEOC3_Msk;
  static constexpr uint32_t CSR_JSTRT3_Pos = ( 19U );
  static constexpr uint32_t CSR_JSTRT3_Msk = ( 0x1UL << CSR_JSTRT3_Pos );
  static constexpr uint32_t CSR_JSTRT3     = CSR_JSTRT3_Msk;
  static constexpr uint32_t CSR_STRT3_Pos  = ( 20U );
  static constexpr uint32_t CSR_STRT3_Msk  = ( 0x1UL << CSR_STRT3_Pos );
  static constexpr uint32_t CSR_STRT3      = CSR_STRT3_Msk;
  static constexpr uint32_t CSR_OVR3_Pos   = ( 21U );
  static constexpr uint32_t CSR_OVR3_Msk   = ( 0x1UL << CSR_OVR3_Pos );
  static constexpr uint32_t CSR_OVR3       = CSR_OVR3_Msk;

  /*******************  Bit definition for CCR register  ********************/
  static constexpr uint32_t CCR_MULTI_Pos   = ( 0U );
  static constexpr uint32_t CCR_MULTI_Msk   = ( 0x1FUL << CCR_MULTI_Pos );
  static constexpr uint32_t CCR_MULTI       = CCR_MULTI_Msk;
  static constexpr uint32_t CCR_MULTI_0     = ( 0x01UL << CCR_MULTI_Pos );
  static constexpr uint32_t CCR_MULTI_1     = ( 0x02UL << CCR_MULTI_Pos );
  static constexpr uint32_t CCR_MULTI_2     = ( 0x04UL << CCR_MULTI_Pos );
  static constexpr uint32_t CCR_MULTI_3     = ( 0x08UL << CCR_MULTI_Pos );
  static constexpr uint32_t CCR_MULTI_4     = ( 0x10UL << CCR_MULTI_Pos );
  static constexpr uint32_t CCR_DELAY_Pos   = ( 8U );
  static constexpr uint32_t CCR_DELAY_Msk   = ( 0xFUL << CCR_DELAY_Pos );
  static constexpr uint32_t CCR_DELAY       = CCR_DELAY_Msk;
  static constexpr uint32_t CCR_DELAY_0     = ( 0x1UL << CCR_DELAY_Pos );
  static constexpr uint32_t CCR_DELAY_1     = ( 0x2UL << CCR_DELAY_Pos );
  static constexpr uint32_t CCR_DELAY_2     = ( 0x4UL << CCR_DELAY_Pos );
  static constexpr uint32_t CCR_DELAY_3     = ( 0x8UL << CCR_DELAY_Pos );
  static constexpr uint32_t CCR_DDS_Pos     = ( 13U );
  static constexpr uint32_t CCR_DDS_Msk     = ( 0x1UL << CCR_DDS_Pos );
  static constexpr uint32_t CCR_DDS         = CCR_DDS_Msk;
  static constexpr uint32_t CCR_DMA_Pos     = ( 14U );
  static constexpr uint32_t CCR_DMA_Msk     = ( 0x3UL << CCR_DMA_Pos );
  static constexpr uint32_t CCR_DMA         = CCR_DMA_Msk;
  static constexpr uint32_t CCR_DMA_0       = ( 0x1UL << CCR_DMA_Pos );
  static constexpr uint32_t CCR_DMA_1       = ( 0x2UL << CCR_DMA_Pos );
  static constexpr uint32_t CCR_ADCPRE_Pos  = ( 16U );
  static constexpr uint32_t CCR_ADCPRE_Msk  = ( 0x3UL << CCR_ADCPRE_Pos );
  static constexpr uint32_t CCR_ADCPRE      = CCR_ADCPRE_Msk;
  static constexpr uint32_t CCR_ADCPRE_0    = ( 0x1UL << CCR_ADCPRE_Pos );
  static constexpr uint32_t CCR_ADCPRE_1    = ( 0x2UL << CCR_ADCPRE_Pos );
  static constexpr uint32_t CCR_VBATE_Pos   = ( 22U );
  static constexpr uint32_t CCR_VBATE_Msk   = ( 0x1UL << CCR_VBATE_Pos );
  static constexpr uint32_t CCR_VBATE       = CCR_VBATE_Msk;
  static constexpr uint32_t CCR_TSVREFE_Pos = ( 23U );
  static constexpr uint32_t CCR_TSVREFE_Msk = ( 0x1UL << CCR_TSVREFE_Pos );
  static constexpr uint32_t CCR_TSVREFE     = CCR_TSVREFE_Msk;

  /*******************  Bit definition for CDR register  ********************/
  static constexpr uint32_t CDR_DATA1_Pos = ( 0U );
  static constexpr uint32_t CDR_DATA1_Msk = ( 0xFFFFUL << CDR_DATA1_Pos );
  static constexpr uint32_t CDR_DATA1     = CDR_DATA1_Msk;
  static constexpr uint32_t CDR_DATA2_Pos = ( 16U );
  static constexpr uint32_t CDR_DATA2_Msk = ( 0xFFFFUL << CDR_DATA2_Pos );
  static constexpr uint32_t CDR_DATA2     = CDR_DATA2_Msk;

}    // namespace Thor::LLD::ADC

#endif /* !THOR_HW_ADC_REGISTER_STM32F4XXXX_HPP */
