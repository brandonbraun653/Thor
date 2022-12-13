/******************************************************************************
 *  File Name:
 *    hw_adc_types.hpp
 *
 *  Description:
 *    LLD types for the ADC Peripheral
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_HW_ADC_TYPES_HPP
#define THOR_HW_ADC_TYPES_HPP

/* C++ Includes */
#include <cstdint>

/* Driver Includes */
#include <Thor/lld/common/registers/field_accessor.hpp>
#include <Thor/lld/stm32f4x/adc/hw_adc_prj.hpp>

namespace Thor::LLD::ADC
{
  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct RegisterMap
  {
    volatile uint32_t SR;    /**< ADC status register,                         Address offset: 0x00 */
    volatile uint32_t CR1;   /**< ADC control register 1,                      Address offset: 0x04 */
    volatile uint32_t CR2;   /**< ADC control register 2,                      Address offset: 0x08 */
    volatile uint32_t SMPR1; /**< ADC sample time register 1,                  Address offset: 0x0C */
    volatile uint32_t SMPR2; /**< ADC sample time register 2,                  Address offset: 0x10 */
    volatile uint32_t JOFR1; /**< ADC injected channel data offset register 1, Address offset: 0x14 */
    volatile uint32_t JOFR2; /**< ADC injected channel data offset register 2, Address offset: 0x18 */
    volatile uint32_t JOFR3; /**< ADC injected channel data offset register 3, Address offset: 0x1C */
    volatile uint32_t JOFR4; /**< ADC injected channel data offset register 4, Address offset: 0x20 */
    volatile uint32_t HTR;   /**< ADC watchdog higher threshold register,      Address offset: 0x24 */
    volatile uint32_t LTR;   /**< ADC watchdog lower threshold register,       Address offset: 0x28 */
    volatile uint32_t SQR1;  /**< ADC regular sequence register 1,             Address offset: 0x2C */
    volatile uint32_t SQR2;  /**< ADC regular sequence register 2,             Address offset: 0x30 */
    volatile uint32_t SQR3;  /**< ADC regular sequence register 3,             Address offset: 0x34 */
    volatile uint32_t JSQR;  /**< ADC injected sequence register,              Address offset: 0x38*/
    volatile uint32_t JDR1;  /**< ADC injected data register 1,                Address offset: 0x3C */
    volatile uint32_t JDR2;  /**< ADC injected data register 2,                Address offset: 0x40 */
    volatile uint32_t JDR3;  /**< ADC injected data register 3,                Address offset: 0x44 */
    volatile uint32_t JDR4;  /**< ADC injected data register 4,                Address offset: 0x48 */
    volatile uint32_t DR;    /**< ADC regular data register,                   Address offset: 0x4C */
  };

  struct CommonRegisterMap
  {
    volatile uint32_t
        CSR; /**< ADC Common status register,                                  Address offset: ADC1 base address + 0x300 */
    volatile uint32_t
        CCR; /**< ADC common control register,                                 Address offset: ADC1 base address + 0x304 */
    volatile uint32_t
        CDR; /**< ADC common regular data register for dual AND triple modes, Address offset: ADC1 base address + 0x308 */
  };

  /*---------------------------------------------------------------------------
  Register Classes
  ---------------------------------------------------------------------------*/
  /*-------------------------------------------------
  Status Registers
  -------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, SR, SR_OVR_Msk, OVR, BIT_ACCESS_RCW0 );
  REG_ACCESSOR( RegisterMap, SR, SR_STRT_Msk, STRT, BIT_ACCESS_RCW0 );
  REG_ACCESSOR( RegisterMap, SR, SR_JSTRT_Msk, JSTRT, BIT_ACCESS_RCW0 );
  REG_ACCESSOR( RegisterMap, SR, SR_JEOC_Msk, JEOC, BIT_ACCESS_RCW0 );
  REG_ACCESSOR( RegisterMap, SR, SR_EOC_Msk, EOC, BIT_ACCESS_RCW0 );
  REG_ACCESSOR( RegisterMap, SR, SR_AWD_Msk, AWD, BIT_ACCESS_RCW0 );

  /*-------------------------------------------------
  Control Register 1
  -------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CR1, CR1_OVRIE_Msk, OVRIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_RES_Msk, RES, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_AWDEN_Msk, AWDEN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_JAWDEN_Msk, JAWDEN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_DISCNUM_Msk, DISCNUM, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_JDISCEN_Msk, JDISCEN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_DISCEN_Msk, DISCEN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_JAUTO_Msk, JAUTO, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_AWDSGL_Msk, AWDSGL, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_SCAN_Msk, SCAN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_JEOCIE_Msk, JEOCIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_AWDIE_Msk, AWDIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_EOCIE_Msk, EOCIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_AWDCH_Msk, AWDCH, BIT_ACCESS_RW );

  /*-------------------------------------------------
  Control Register 2
  -------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CR2, CR2_SWSTART_Msk, SWSTART, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_EXTEN_Msk, EXTEN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_EXTSEL_Msk, EXTSEL, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_JSWSTART_Msk, JSWSTART, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_JEXTEN_Msk, JEXTEN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_JEXTSEL_Msk, JEXTSEL, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_ALIGN_Msk, ALIGN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_EOCS_Msk, EOCS, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_DDS_Msk, DDS, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_DMA_Msk, DMA, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_CONT_Msk, CONT, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_ADON_Msk, ADON, BIT_ACCESS_RW );

  /*-------------------------------------------------
  Sample Time Registers (1/2)
  -------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, SMPR1, SMPR1_ALL_Msk, SMPR1_ALL, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, SMPR2, SMPR2_ALL_Msk, SMPR2_ALL, BIT_ACCESS_RW );

  /*-------------------------------------------------
  Analog Watchdog Higher Threshold
  -------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, HTR, HTR_HT_Msk, HTHRESH, BIT_ACCESS_RW );

  /*-------------------------------------------------
  Analog Watchdog Lower Threshold
  -------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, LTR, LTR_LT_Msk, LTHRESH, BIT_ACCESS_RW );

  /*-------------------------------------------------
  Regular Sequence Registers
  -------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, SQR1, SQR1_ALL_Msk, SQR1_ALL, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, SQR2, SQR2_ALL_Msk, SQR2_ALL, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, SQR3, SQR3_ALL_Msk, SQR3_ALL, BIT_ACCESS_RW );

  REG_ACCESSOR( RegisterMap, SQR1, SQR1_L_Msk, L, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, SQR1, SQR1_SQ16_Msk, SQ16, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, SQR1, SQR1_SQ15_Msk, SQ15, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, SQR1, SQR1_SQ14_Msk, SQ14, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, SQR1, SQR1_SQ13_Msk, SQ13, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, SQR2, SQR2_SQ12_Msk, SQ12, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, SQR2, SQR2_SQ11_Msk, SQ11, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, SQR2, SQR2_SQ10_Msk, SQ10, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, SQR2, SQR2_SQ9_Msk, SQ9, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, SQR2, SQR2_SQ8_Msk, SQ8, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, SQR2, SQR2_SQ7_Msk, SQ7, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, SQR3, SQR3_SQ6_Msk, SQ6, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, SQR3, SQR3_SQ5_Msk, SQ5, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, SQR3, SQR3_SQ4_Msk, SQ4, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, SQR3, SQR3_SQ3_Msk, SQ3, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, SQR3, SQR3_SQ2_Msk, SQ2, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, SQR3, SQR3_SQ1_Msk, SQ1, BIT_ACCESS_RW );

  /*-------------------------------------------------
  Injected Sequence Register
  -------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, JSQR, JSQR_JL_Msk, JL, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, JSQR, JSQR_JSQ4_Msk, JSQ4, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, JSQR, JSQR_JSQ3_Msk, JSQ3, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, JSQR, JSQR_JSQ2_Msk, JSQ2, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, JSQR, JSQR_JSQ1_Msk, JSQ1, BIT_ACCESS_RW );

  /*-------------------------------------------------
  Injected Data Register
  -------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, JDR1, JDR1_JDATA_Msk, JDATA1, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, JDR2, JDR2_JDATA_Msk, JDATA2, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, JDR3, JDR3_JDATA_Msk, JDATA3, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, JDR4, JDR4_JDATA_Msk, JDATA4, BIT_ACCESS_R );

  /*-------------------------------------------------
  Regular Data Register
  -------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, DR, DR_DATA_Msk, DATA, BIT_ACCESS_RW );

  /*-------------------------------------------------
  Common Status Register
  -------------------------------------------------*/
  REG_ACCESSOR( CommonRegisterMap, CSR, CSR_OVR3_Msk, OVR3, BIT_ACCESS_R );
  REG_ACCESSOR( CommonRegisterMap, CSR, CSR_STRT3_Msk, STRT3, BIT_ACCESS_R );
  REG_ACCESSOR( CommonRegisterMap, CSR, CSR_JSTRT3_Msk, JSTRT3, BIT_ACCESS_R );
  REG_ACCESSOR( CommonRegisterMap, CSR, CSR_JEOC3_Msk, JEOC3, BIT_ACCESS_R );
  REG_ACCESSOR( CommonRegisterMap, CSR, CSR_EOC3_Msk, EOC3, BIT_ACCESS_R );
  REG_ACCESSOR( CommonRegisterMap, CSR, CSR_AWD3_Msk, AWD3, BIT_ACCESS_R );

  REG_ACCESSOR( CommonRegisterMap, CSR, CSR_OVR2_Msk, OVR2, BIT_ACCESS_R );
  REG_ACCESSOR( CommonRegisterMap, CSR, CSR_STRT2_Msk, STRT2, BIT_ACCESS_R );
  REG_ACCESSOR( CommonRegisterMap, CSR, CSR_JSTRT2_Msk, JSTRT2, BIT_ACCESS_R );
  REG_ACCESSOR( CommonRegisterMap, CSR, CSR_JEOC2_Msk, JEOC2, BIT_ACCESS_R );
  REG_ACCESSOR( CommonRegisterMap, CSR, CSR_EOC2_Msk, EOC2, BIT_ACCESS_R );
  REG_ACCESSOR( CommonRegisterMap, CSR, CSR_AWD2_Msk, AWD2, BIT_ACCESS_R );

  REG_ACCESSOR( CommonRegisterMap, CSR, CSR_OVR1_Msk, OVR1, BIT_ACCESS_R );
  REG_ACCESSOR( CommonRegisterMap, CSR, CSR_STRT1_Msk, STRT1, BIT_ACCESS_R );
  REG_ACCESSOR( CommonRegisterMap, CSR, CSR_JSTRT1_Msk, JSTRT1, BIT_ACCESS_R );
  REG_ACCESSOR( CommonRegisterMap, CSR, CSR_JEOC1_Msk, JEOC1, BIT_ACCESS_R );
  REG_ACCESSOR( CommonRegisterMap, CSR, CSR_EOC1_Msk, EOC1, BIT_ACCESS_R );
  REG_ACCESSOR( CommonRegisterMap, CSR, CSR_AWD1_Msk, AWD1, BIT_ACCESS_R );

  /*-------------------------------------------------
  Common Control Register
  -------------------------------------------------*/
  REG_ACCESSOR( CommonRegisterMap, CCR, CCR_TSVREFE_Msk, TSVREFE, BIT_ACCESS_RW );
  REG_ACCESSOR( CommonRegisterMap, CCR, CCR_VBATE_Msk, VBATE, BIT_ACCESS_RW );
  REG_ACCESSOR( CommonRegisterMap, CCR, CCR_ADCPRE_Msk, ADCPRE, BIT_ACCESS_RW );
  REG_ACCESSOR( CommonRegisterMap, CCR, CCR_DMA_Msk, CCR_DMA, BIT_ACCESS_RW );
  REG_ACCESSOR( CommonRegisterMap, CCR, CCR_DDS_Msk, CCR_DDS, BIT_ACCESS_RW );
  REG_ACCESSOR( CommonRegisterMap, CCR, CCR_DELAY_Msk, DELAY, BIT_ACCESS_RW );
  REG_ACCESSOR( CommonRegisterMap, CCR, CCR_MULTI_Msk, MULTI, BIT_ACCESS_RW );

  /*-------------------------------------------------
  Common Regular Data Register (Dual/Triple Modes)
  -------------------------------------------------*/
  REG_ACCESSOR( CommonRegisterMap, CDR, CDR_DATA2_Msk, DATA2, BIT_ACCESS_R );
  REG_ACCESSOR( CommonRegisterMap, CDR, CDR_DATA1_Msk, DATA1, BIT_ACCESS_R );

}    // namespace Thor::LLD::ADC

#endif /* !THOR_HW_ADC_TYPES_HPP*/
