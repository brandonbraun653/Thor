/******************************************************************************
 *  File Name:
 *    hw_adc_types.hpp
 *
 *  Description:
 *    LLD types for the ADC Peripheral
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_HW_ADC_TYPES_HPP
#define THOR_HW_ADC_TYPES_HPP

/* C++ Includes */
#include <cstdint>

/* Driver Includes */
#include <Thor/lld/common/registers/field_accessor.hpp>
#include <Thor/lld/stm32l4x/adc/hw_adc_prj.hpp>

namespace Thor::LLD::ADC
{
  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/


  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct RegisterMap
  {
    volatile uint32_t ISR;            /**< ADC interrupt and status register,             Address offset: 0x00 */
    volatile uint32_t IER;            /**< ADC interrupt enable register,                 Address offset: 0x04 */
    volatile uint32_t CR;             /**< ADC control register,                          Address offset: 0x08 */
    volatile uint32_t CFGR;           /**< ADC configuration register 1,                  Address offset: 0x0C */
    volatile uint32_t CFGR2;          /**< ADC configuration register 2,                  Address offset: 0x10 */
    volatile uint32_t SMPR1;          /**< ADC sampling time register 1,                  Address offset: 0x14 */
    volatile uint32_t SMPR2;          /**< ADC sampling time register 2,                  Address offset: 0x18 */
    volatile uint32_t RESERVED1;      /**< Reserved,                                                      0x1C */
    volatile uint32_t TR1;            /**< ADC analog watchdog 1 threshold register,      Address offset: 0x20 */
    volatile uint32_t TR2;            /**< ADC analog watchdog 2 threshold register,      Address offset: 0x24 */
    volatile uint32_t TR3;            /**< ADC analog watchdog 3 threshold register,      Address offset: 0x28 */
    volatile uint32_t RESERVED2;      /**< Reserved,                                                      0x2C */
    volatile uint32_t SQR1;           /**< ADC group regular sequencer register 1,        Address offset: 0x30 */
    volatile uint32_t SQR2;           /**< ADC group regular sequencer register 2,        Address offset: 0x34 */
    volatile uint32_t SQR3;           /**< ADC group regular sequencer register 3,        Address offset: 0x38 */
    volatile uint32_t SQR4;           /**< ADC group regular sequencer register 4,        Address offset: 0x3C */
    volatile uint32_t DR;             /**< ADC group regular data register,               Address offset: 0x40 */
    volatile uint32_t RESERVED3;      /**< Reserved,                                                      0x44 */
    volatile uint32_t RESERVED4;      /**< Reserved,                                                      0x48 */
    volatile uint32_t JSQR;           /**< ADC group injected sequencer register,         Address offset: 0x4C */
    volatile uint32_t RESERVED5[ 4 ]; /**< Reserved,                                               0x50 - 0x5C */
    volatile uint32_t OFR1;           /**< ADC offset register 1,                         Address offset: 0x60 */
    volatile uint32_t OFR2;           /**< ADC offset register 2,                         Address offset: 0x64 */
    volatile uint32_t OFR3;           /**< ADC offset register 3,                         Address offset: 0x68 */
    volatile uint32_t OFR4;           /**< ADC offset register 4,                         Address offset: 0x6C */
    volatile uint32_t RESERVED6[ 4 ]; /**< Reserved,                                               0x70 - 0x7C */
    volatile uint32_t JDR1;           /**< ADC group injected rank 1 data register,       Address offset: 0x80 */
    volatile uint32_t JDR2;           /**< ADC group injected rank 2 data register,       Address offset: 0x84 */
    volatile uint32_t JDR3;           /**< ADC group injected rank 3 data register,       Address offset: 0x88 */
    volatile uint32_t JDR4;           /**< ADC group injected rank 4 data register,       Address offset: 0x8C */
    volatile uint32_t RESERVED7[ 4 ]; /**< Reserved,                                             0x090 - 0x09C */
    volatile uint32_t AWD2CR;         /**< ADC analog watchdog 1 configuration register,  Address offset: 0xA0 */
    volatile uint32_t AWD3CR;         /**< ADC analog watchdog 3 Configuration Register,  Address offset: 0xA4 */
    volatile uint32_t RESERVED8;      /**< Reserved,                                                     0x0A8 */
    volatile uint32_t RESERVED9;      /**< Reserved,                                                     0x0AC */
    volatile uint32_t DIFSEL;         /**< ADC differential mode selection register,      Address offset: 0xB0 */
    volatile uint32_t CALFACT;        /**< ADC calibration factors,                       Address offset: 0xB4 */
  };


  struct CommonRegisterMap
  {
    volatile uint32_t RESERVED1; /**< Reserved,                          Address offset: ADC1 base address + 0x300 */
    volatile uint32_t RESERVED2; /**< Reserved,                          Address offset: ADC1 base address + 0x304 */
    volatile uint32_t CCR;       /**< ADC common configuration register, Address offset: ADC1 base address + 0x308 */
    volatile uint32_t RESERVED3; /**< Reserved,                          Address offset: ADC1 base address + 0x30C */
  };

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  /*-------------------------------------------------
  Common Control Register (CCR)
  -------------------------------------------------*/
  REG_ACCESSOR( CommonRegisterMap, CCR, CCR_VBATEN_Msk, VBATEN, BIT_ACCESS_RW );
  REG_ACCESSOR( CommonRegisterMap, CCR, CCR_TSEN_Msk, TSEN, BIT_ACCESS_RW );
  REG_ACCESSOR( CommonRegisterMap, CCR, CCR_VREFEN_Msk, VREFEN, BIT_ACCESS_RW );
  REG_ACCESSOR( CommonRegisterMap, CCR, CCR_PRESC_Msk, PRESC, BIT_ACCESS_RW );
  REG_ACCESSOR( CommonRegisterMap, CCR, CCR_CKMODE_Msk, CKMODE, BIT_ACCESS_RW );

  /*-------------------------------------------------
  Interrupt Status Register (ISR)
  -------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, ISR, ISR_ALL_Msk, ISR_ALL, BIT_ACCESS_RCW1 );
  REG_ACCESSOR( RegisterMap, ISR, ISR_JQOVF_Msk, JQOVF, BIT_ACCESS_RCW1 );
  REG_ACCESSOR( RegisterMap, ISR, ISR_AWD3_Msk, AWD3, BIT_ACCESS_RCW1 );
  REG_ACCESSOR( RegisterMap, ISR, ISR_AWD2_Msk, AWD2, BIT_ACCESS_RCW1 );
  REG_ACCESSOR( RegisterMap, ISR, ISR_AWD1_Msk, AWD1, BIT_ACCESS_RCW1 );
  REG_ACCESSOR( RegisterMap, ISR, ISR_JEOS_Msk, JEOS, BIT_ACCESS_RCW1 );
  REG_ACCESSOR( RegisterMap, ISR, ISR_JEOC_Msk, JEOC, BIT_ACCESS_RCW1 );
  REG_ACCESSOR( RegisterMap, ISR, ISR_OVR_Msk, OVR, BIT_ACCESS_RCW1 );
  REG_ACCESSOR( RegisterMap, ISR, ISR_EOS_Msk, EOS, BIT_ACCESS_RCW1 );
  REG_ACCESSOR( RegisterMap, ISR, ISR_EOC_Msk, EOC, BIT_ACCESS_RCW1 );
  REG_ACCESSOR( RegisterMap, ISR, ISR_EOSMP_Msk, EOSMP, BIT_ACCESS_RCW1 );
  REG_ACCESSOR( RegisterMap, ISR, ISR_ADRDY_Msk, ADRDY, BIT_ACCESS_RCW1 );

  /*-------------------------------------------------
  Interrupt Enable Register (IER)
  -------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, IER, IER_ALL_Msk, IER_ALL, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, IER, IER_JQOVFIE_Msk, JQOVFIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, IER, IER_AWD3IE_Msk, AWD3IE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, IER, IER_AWD2IE_Msk, AWD2IE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, IER, IER_AWD1IE_Msk, AWD1IE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, IER, IER_JEOSIE_Msk, JEOSIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, IER, IER_JEOCIE_Msk, JEOCIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, IER, IER_OVRIE_Msk, OVRIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, IER, IER_EOSIE_Msk, EOSIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, IER, IER_EOCIE_Msk, EOCIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, IER, IER_EOSMPIE_Msk, EOSMPIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, IER, IER_ADRDYIE_Msk, ADRDYIE, BIT_ACCESS_RW );

  /*-------------------------------------------------
  Control Register (CR)
  -------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CR, CR_ADCAL_Msk, ADCAL, BIT_ACCESS_RS );
  REG_ACCESSOR( RegisterMap, CR, CR_ADCALDIF_Msk, ADCALDIF, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR, CR_DEEPPWD_Msk, DEEPPWD, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR, CR_ADVREGEN_Msk, ADVREGEN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR, CR_JADSTP_Msk, JADSTP, BIT_ACCESS_RS );
  REG_ACCESSOR( RegisterMap, CR, CR_ADSTP_Msk, ADSTP, BIT_ACCESS_RS );
  REG_ACCESSOR( RegisterMap, CR, CR_JADSTART_Msk, JADSTART, BIT_ACCESS_RS );
  REG_ACCESSOR( RegisterMap, CR, CR_ADSTART_Msk, ADSTART, BIT_ACCESS_RS );
  REG_ACCESSOR( RegisterMap, CR, CR_ADDIS_Msk, ADDIS, BIT_ACCESS_RS );
  REG_ACCESSOR( RegisterMap, CR, CR_ADEN_Msk, ADEN, BIT_ACCESS_RS );

  /*-------------------------------------------------
  Configuration Register (CFGR)
  -------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CFGR, CFGR_JQDIS_Msk, JQDIS, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CFGR, CFGR_AWD1CH_Msk, AWD1CH, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CFGR, CFGR_JAUTO_Msk, JAUTO, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CFGR, CFGR_JAWD1EN_Msk, JAWD1EN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CFGR, CFGR_AWD1EN_Msk, AWD1EN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CFGR, CFGR_AWD1SGL_Msk, AWD1SGL, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CFGR, CFGR_JQM_Msk, JQM, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CFGR, CFGR_JDISCEN_Msk, JDISCEN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CFGR, CFGR_DISCNUM_Msk, DISCNUM, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CFGR, CFGR_DISCEN_Msk, DISCEN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CFGR, CFGR_AUTDLY_Msk, AUTDLY, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CFGR, CFGR_CONT_Msk, CONT, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CFGR, CFGR_OVRMOD_Msk, OVRMOD, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CFGR, CFGR_EXTEN_Msk, EXTEN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CFGR, CFGR_EXTSEL_Msk, EXTSEL, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CFGR, CFGR_ALIGN_Msk, ALIGN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CFGR, CFGR_RES_Msk, RES, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CFGR, CFGR_DMACFG_Msk, DMACFG, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CFGR, CFGR_DMAEN_Msk, DMAEN, BIT_ACCESS_RW );

  /*-------------------------------------------------
  Configuration Register 2 (CFGR2)
  -------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CFGR2, CFGR2_ROVSM_Msk, ROVSM, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CFGR2, CFGR2_TROVS_Msk, TROVS, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CFGR2, CFGR2_OVSS_Msk, OVSS, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CFGR2, CFGR2_OVSR_Msk, OVSR, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CFGR2, CFGR2_JOVSE_Msk, JOVSE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CFGR2, CFGR2_ROVSE_Msk, ROVSE, BIT_ACCESS_RW );

  /*-------------------------------------------------
  Sample Time Register 1/2 (SMPRx)
  -------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, SMPR1, SMPR1_ALL_Msk, SMPR1_ALL, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, SMPR2, SMPR2_ALL_Msk, SMPR2_ALL, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  Watchdog Threshold Registers
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, TR1, TR1_ALL_Msk, TR1_ALL, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, TR2, TR2_ALL_Msk, TR2_ALL, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, TR3, TR3_ALL_Msk, TR3_ALL, BIT_ACCESS_RW );

  /*-------------------------------------------------
  Regular Sequence Register (SQR)
  -------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, SQR1, SQR1_ALL_Msk, SQR1_ALL, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, SQR2, SQR2_ALL_Msk, SQR2_ALL, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, SQR3, SQR3_ALL_Msk, SQR3_ALL, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, SQR4, SQR4_ALL_Msk, SQR4_ALL, BIT_ACCESS_RW );

  REG_ACCESSOR( RegisterMap, SQR1, SQR1_L_Msk, L, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, SQR1, SQR1_SQ1_Msk, SQ1, BIT_ACCESS_RW );

  /*-------------------------------------------------
  Regular Data Register
  -------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, DR, DR_RDATA_Msk, DATA, BIT_ACCESS_R );

  /*---------------------------------------------------------------------------
  Injected sequence Register
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, JSQR, JSQR_ALL_Msk, JSQR_ALL, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, JSQR, JSQR_JL_Msk, JL, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, JSQR, JSQR_JSQ1_Msk, JSQ1, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  Injected Sequence Data Registers
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, JDR1, JDR1_JDATA_Msk, JDATA1, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, JDR2, JDR2_JDATA_Msk, JDATA2, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, JDR3, JDR3_JDATA_Msk, JDATA3, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, JDR4, JDR4_JDATA_Msk, JDATA4, BIT_ACCESS_R );

  /*---------------------------------------------------------------------------
  Analog Watchdog Configuration Registers
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, AWD2CR, AWD2CR_AWD2CH_Msk, AWD2CH, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, AWD3CR, AWD3CR_AWD3CH_Msk, AWD3CH, BIT_ACCESS_RW );

  /*-------------------------------------------------
  Differential Mode Selection Register (DIFSEL)
  -------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, DIFSEL, DIFSEL_DIFSEL_Msk, DIFSEL, BIT_ACCESS_RW );

}    // namespace Thor::LLD::ADC

#endif /* !THOR_HW_ADC_TYPES_HPP*/
