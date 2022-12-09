/********************************************************************************
 *  File Name:
 *    hw_adc_register_stm32l4xxxx.hpp
 *
 *  Description:
 *    ADC register definitions for the STM32L4xxxx series chips.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_ADC_REGISTER_STM32L4XXXX_HPP
#define THOR_HW_ADC_REGISTER_STM32L4XXXX_HPP

/* C++ Includes */
#include <cstdint>

/* Chimera Includes */
#include <Chimera/common>

/* Driver Includes */
#include <Thor/lld/stm32l4x/system/variant/sys_memory_map_stm32l432kc.hpp>


namespace Thor::LLD::ADC
{
  /*-------------------------------------------------
  Peripheral Instance Memory Map Base
  -------------------------------------------------*/
  static constexpr uint32_t ADC1_BASE_ADDR = Thor::System::MemoryMap::ADC1_PERIPH_START_ADDRESS;

  /* See RM 16.7.4 for this offset definition */
  static constexpr uint32_t ADC_CMN_BASE_ADDR = ADC1_BASE_ADDR + 0x300;

  /*-------------------------------------------------
  Peripheral Register Definitions
  -------------------------------------------------*/
  /********************  Bit definition for ISR register  *******************/
  static constexpr Reg32_t ISR_ALL_Msk = 0x000007FF;

  static constexpr Reg32_t ISR_ADRDY_Pos = ( 0U );
  static constexpr Reg32_t ISR_ADRDY_Msk = ( 0x1UL << ISR_ADRDY_Pos );
  static constexpr Reg32_t ISR_ADRDY     = ISR_ADRDY_Msk;
  static constexpr Reg32_t ISR_EOSMP_Pos = ( 1U );
  static constexpr Reg32_t ISR_EOSMP_Msk = ( 0x1UL << ISR_EOSMP_Pos );
  static constexpr Reg32_t ISR_EOSMP     = ISR_EOSMP_Msk;
  static constexpr Reg32_t ISR_EOC_Pos   = ( 2U );
  static constexpr Reg32_t ISR_EOC_Msk   = ( 0x1UL << ISR_EOC_Pos );
  static constexpr Reg32_t ISR_EOC       = ISR_EOC_Msk;
  static constexpr Reg32_t ISR_EOS_Pos   = ( 3U );
  static constexpr Reg32_t ISR_EOS_Msk   = ( 0x1UL << ISR_EOS_Pos );
  static constexpr Reg32_t ISR_EOS       = ISR_EOS_Msk;
  static constexpr Reg32_t ISR_OVR_Pos   = ( 4U );
  static constexpr Reg32_t ISR_OVR_Msk   = ( 0x1UL << ISR_OVR_Pos );
  static constexpr Reg32_t ISR_OVR       = ISR_OVR_Msk;
  static constexpr Reg32_t ISR_JEOC_Pos  = ( 5U );
  static constexpr Reg32_t ISR_JEOC_Msk  = ( 0x1UL << ISR_JEOC_Pos );
  static constexpr Reg32_t ISR_JEOC      = ISR_JEOC_Msk;
  static constexpr Reg32_t ISR_JEOS_Pos  = ( 6U );
  static constexpr Reg32_t ISR_JEOS_Msk  = ( 0x1UL << ISR_JEOS_Pos );
  static constexpr Reg32_t ISR_JEOS      = ISR_JEOS_Msk;
  static constexpr Reg32_t ISR_AWD1_Pos  = ( 7U );
  static constexpr Reg32_t ISR_AWD1_Msk  = ( 0x1UL << ISR_AWD1_Pos );
  static constexpr Reg32_t ISR_AWD1      = ISR_AWD1_Msk;
  static constexpr Reg32_t ISR_AWD2_Pos  = ( 8U );
  static constexpr Reg32_t ISR_AWD2_Msk  = ( 0x1UL << ISR_AWD2_Pos );
  static constexpr Reg32_t ISR_AWD2      = ISR_AWD2_Msk;
  static constexpr Reg32_t ISR_AWD3_Pos  = ( 9U );
  static constexpr Reg32_t ISR_AWD3_Msk  = ( 0x1UL << ISR_AWD3_Pos );
  static constexpr Reg32_t ISR_AWD3      = ISR_AWD3_Msk;
  static constexpr Reg32_t ISR_JQOVF_Pos = ( 10U );
  static constexpr Reg32_t ISR_JQOVF_Msk = ( 0x1UL << ISR_JQOVF_Pos );
  static constexpr Reg32_t ISR_JQOVF     = ISR_JQOVF_Msk;

  /********************  Bit definition for IER register  *******************/
  static constexpr Reg32_t IER_ALL_Msk     = 0x7FF;
  static constexpr Reg32_t IER_ADRDYIE_Pos = ( 0U );
  static constexpr Reg32_t IER_ADRDYIE_Msk = ( 0x1UL << IER_ADRDYIE_Pos );
  static constexpr Reg32_t IER_ADRDYIE     = IER_ADRDYIE_Msk;
  static constexpr Reg32_t IER_EOSMPIE_Pos = ( 1U );
  static constexpr Reg32_t IER_EOSMPIE_Msk = ( 0x1UL << IER_EOSMPIE_Pos );
  static constexpr Reg32_t IER_EOSMPIE     = IER_EOSMPIE_Msk;
  static constexpr Reg32_t IER_EOCIE_Pos   = ( 2U );
  static constexpr Reg32_t IER_EOCIE_Msk   = ( 0x1UL << IER_EOCIE_Pos );
  static constexpr Reg32_t IER_EOCIE       = IER_EOCIE_Msk;
  static constexpr Reg32_t IER_EOSIE_Pos   = ( 3U );
  static constexpr Reg32_t IER_EOSIE_Msk   = ( 0x1UL << IER_EOSIE_Pos );
  static constexpr Reg32_t IER_EOSIE       = IER_EOSIE_Msk;
  static constexpr Reg32_t IER_OVRIE_Pos   = ( 4U );
  static constexpr Reg32_t IER_OVRIE_Msk   = ( 0x1UL << IER_OVRIE_Pos );
  static constexpr Reg32_t IER_OVRIE       = IER_OVRIE_Msk;
  static constexpr Reg32_t IER_JEOCIE_Pos  = ( 5U );
  static constexpr Reg32_t IER_JEOCIE_Msk  = ( 0x1UL << IER_JEOCIE_Pos );
  static constexpr Reg32_t IER_JEOCIE      = IER_JEOCIE_Msk;
  static constexpr Reg32_t IER_JEOSIE_Pos  = ( 6U );
  static constexpr Reg32_t IER_JEOSIE_Msk  = ( 0x1UL << IER_JEOSIE_Pos );
  static constexpr Reg32_t IER_JEOSIE      = IER_JEOSIE_Msk;
  static constexpr Reg32_t IER_AWD1IE_Pos  = ( 7U );
  static constexpr Reg32_t IER_AWD1IE_Msk  = ( 0x1UL << IER_AWD1IE_Pos );
  static constexpr Reg32_t IER_AWD1IE      = IER_AWD1IE_Msk;
  static constexpr Reg32_t IER_AWD2IE_Pos  = ( 8U );
  static constexpr Reg32_t IER_AWD2IE_Msk  = ( 0x1UL << IER_AWD2IE_Pos );
  static constexpr Reg32_t IER_AWD2IE      = IER_AWD2IE_Msk;
  static constexpr Reg32_t IER_AWD3IE_Pos  = ( 9U );
  static constexpr Reg32_t IER_AWD3IE_Msk  = ( 0x1UL << IER_AWD3IE_Pos );
  static constexpr Reg32_t IER_AWD3IE      = IER_AWD3IE_Msk;
  static constexpr Reg32_t IER_JQOVFIE_Pos = ( 10U );
  static constexpr Reg32_t IER_JQOVFIE_Msk = ( 0x1UL << IER_JQOVFIE_Pos );
  static constexpr Reg32_t IER_JQOVFIE     = IER_JQOVFIE_Msk;

  /********************  Bit definition for CR register  ********************/
  static constexpr Reg32_t CR_ADEN_Pos     = ( 0U );
  static constexpr Reg32_t CR_ADEN_Msk     = ( 0x1UL << CR_ADEN_Pos );
  static constexpr Reg32_t CR_ADEN         = CR_ADEN_Msk;
  static constexpr Reg32_t CR_ADDIS_Pos    = ( 1U );
  static constexpr Reg32_t CR_ADDIS_Msk    = ( 0x1UL << CR_ADDIS_Pos );
  static constexpr Reg32_t CR_ADDIS        = CR_ADDIS_Msk;
  static constexpr Reg32_t CR_ADSTART_Pos  = ( 2U );
  static constexpr Reg32_t CR_ADSTART_Msk  = ( 0x1UL << CR_ADSTART_Pos );
  static constexpr Reg32_t CR_ADSTART      = CR_ADSTART_Msk;
  static constexpr Reg32_t CR_JADSTART_Pos = ( 3U );
  static constexpr Reg32_t CR_JADSTART_Msk = ( 0x1UL << CR_JADSTART_Pos );
  static constexpr Reg32_t CR_JADSTART     = CR_JADSTART_Msk;
  static constexpr Reg32_t CR_ADSTP_Pos    = ( 4U );
  static constexpr Reg32_t CR_ADSTP_Msk    = ( 0x1UL << CR_ADSTP_Pos );
  static constexpr Reg32_t CR_ADSTP        = CR_ADSTP_Msk;
  static constexpr Reg32_t CR_JADSTP_Pos   = ( 5U );
  static constexpr Reg32_t CR_JADSTP_Msk   = ( 0x1UL << CR_JADSTP_Pos );
  static constexpr Reg32_t CR_JADSTP       = CR_JADSTP_Msk;
  static constexpr Reg32_t CR_ADVREGEN_Pos = ( 28U );
  static constexpr Reg32_t CR_ADVREGEN_Msk = ( 0x1UL << CR_ADVREGEN_Pos );
  static constexpr Reg32_t CR_ADVREGEN     = CR_ADVREGEN_Msk;
  static constexpr Reg32_t CR_DEEPPWD_Pos  = ( 29U );
  static constexpr Reg32_t CR_DEEPPWD_Msk  = ( 0x1UL << CR_DEEPPWD_Pos );
  static constexpr Reg32_t CR_DEEPPWD      = CR_DEEPPWD_Msk;
  static constexpr Reg32_t CR_ADCALDIF_Pos = ( 30U );
  static constexpr Reg32_t CR_ADCALDIF_Msk = ( 0x1UL << CR_ADCALDIF_Pos );
  static constexpr Reg32_t CR_ADCALDIF     = CR_ADCALDIF_Msk;
  static constexpr Reg32_t CR_ADCAL_Pos    = ( 31U );
  static constexpr Reg32_t CR_ADCAL_Msk    = ( 0x1UL << CR_ADCAL_Pos );
  static constexpr Reg32_t CR_ADCAL        = CR_ADCAL_Msk;

  /********************  Bit definition for CFGR register  ******************/
  static constexpr Reg32_t CFGR_DMAEN_Pos  = ( 0U );
  static constexpr Reg32_t CFGR_DMAEN_Msk  = ( 0x1UL << CFGR_DMAEN_Pos );
  static constexpr Reg32_t CFGR_DMAEN      = CFGR_DMAEN_Msk;
  static constexpr Reg32_t CFGR_DMACFG_Pos = ( 1U );
  static constexpr Reg32_t CFGR_DMACFG_Msk = ( 0x1UL << CFGR_DMACFG_Pos );
  static constexpr Reg32_t CFGR_DMACFG     = CFGR_DMACFG_Msk;

  static constexpr Reg32_t CFGR_RES_Pos = ( 3U );
  static constexpr Reg32_t CFGR_RES_Msk = ( 0x3UL << CFGR_RES_Pos );
  static constexpr Reg32_t CFGR_RES     = CFGR_RES_Msk;
  static constexpr Reg32_t CFGR_RES_0   = ( 0x1UL << CFGR_RES_Pos );
  static constexpr Reg32_t CFGR_RES_1   = ( 0x2UL << CFGR_RES_Pos );

  static constexpr Reg32_t CFGR_ALIGN_Pos = ( 5U );
  static constexpr Reg32_t CFGR_ALIGN_Msk = ( 0x1UL << CFGR_ALIGN_Pos );
  static constexpr Reg32_t CFGR_ALIGN     = CFGR_ALIGN_Msk;

  static constexpr Reg32_t CFGR_EXTSEL_Pos = ( 6U );
  static constexpr Reg32_t CFGR_EXTSEL_Msk = ( 0xFUL << CFGR_EXTSEL_Pos );
  static constexpr Reg32_t CFGR_EXTSEL     = CFGR_EXTSEL_Msk;
  static constexpr Reg32_t CFGR_EXTSEL_0   = ( 0x1UL << CFGR_EXTSEL_Pos );
  static constexpr Reg32_t CFGR_EXTSEL_1   = ( 0x2UL << CFGR_EXTSEL_Pos );
  static constexpr Reg32_t CFGR_EXTSEL_2   = ( 0x4UL << CFGR_EXTSEL_Pos );
  static constexpr Reg32_t CFGR_EXTSEL_3   = ( 0x8UL << CFGR_EXTSEL_Pos );

  static constexpr Reg32_t CFGR_EXTEN_Pos = ( 10U );
  static constexpr Reg32_t CFGR_EXTEN_Msk = ( 0x3UL << CFGR_EXTEN_Pos );
  static constexpr Reg32_t CFGR_EXTEN     = CFGR_EXTEN_Msk;
  static constexpr Reg32_t CFGR_EXTEN_0   = ( 0x1UL << CFGR_EXTEN_Pos );
  static constexpr Reg32_t CFGR_EXTEN_1   = ( 0x2UL << CFGR_EXTEN_Pos );

  static constexpr Reg32_t CFGR_OVRMOD_Pos = ( 12U );
  static constexpr Reg32_t CFGR_OVRMOD_Msk = ( 0x1UL << CFGR_OVRMOD_Pos );
  static constexpr Reg32_t CFGR_OVRMOD     = CFGR_OVRMOD_Msk;
  static constexpr Reg32_t CFGR_CONT_Pos   = ( 13U );
  static constexpr Reg32_t CFGR_CONT_Msk   = ( 0x1UL << CFGR_CONT_Pos );
  static constexpr Reg32_t CFGR_CONT       = CFGR_CONT_Msk;
  static constexpr Reg32_t CFGR_AUTDLY_Pos = ( 14U );
  static constexpr Reg32_t CFGR_AUTDLY_Msk = ( 0x1UL << CFGR_AUTDLY_Pos );
  static constexpr Reg32_t CFGR_AUTDLY     = CFGR_AUTDLY_Msk;

  static constexpr Reg32_t CFGR_DISCEN_Pos = ( 16U );
  static constexpr Reg32_t CFGR_DISCEN_Msk = ( 0x1UL << CFGR_DISCEN_Pos );
  static constexpr Reg32_t CFGR_DISCEN     = CFGR_DISCEN_Msk;

  static constexpr Reg32_t CFGR_DISCNUM_Pos = ( 17U );
  static constexpr Reg32_t CFGR_DISCNUM_Msk = ( 0x7UL << CFGR_DISCNUM_Pos );
  static constexpr Reg32_t CFGR_DISCNUM     = CFGR_DISCNUM_Msk;
  static constexpr Reg32_t CFGR_DISCNUM_0   = ( 0x1UL << CFGR_DISCNUM_Pos );
  static constexpr Reg32_t CFGR_DISCNUM_1   = ( 0x2UL << CFGR_DISCNUM_Pos );
  static constexpr Reg32_t CFGR_DISCNUM_2   = ( 0x4UL << CFGR_DISCNUM_Pos );

  static constexpr Reg32_t CFGR_JDISCEN_Pos = ( 20U );
  static constexpr Reg32_t CFGR_JDISCEN_Msk = ( 0x1UL << CFGR_JDISCEN_Pos );
  static constexpr Reg32_t CFGR_JDISCEN     = CFGR_JDISCEN_Msk;
  static constexpr Reg32_t CFGR_JQM_Pos     = ( 21U );
  static constexpr Reg32_t CFGR_JQM_Msk     = ( 0x1UL << CFGR_JQM_Pos );
  static constexpr Reg32_t CFGR_JQM         = CFGR_JQM_Msk;
  static constexpr Reg32_t CFGR_AWD1SGL_Pos = ( 22U );
  static constexpr Reg32_t CFGR_AWD1SGL_Msk = ( 0x1UL << CFGR_AWD1SGL_Pos );
  static constexpr Reg32_t CFGR_AWD1SGL     = CFGR_AWD1SGL_Msk;
  static constexpr Reg32_t CFGR_AWD1EN_Pos  = ( 23U );
  static constexpr Reg32_t CFGR_AWD1EN_Msk  = ( 0x1UL << CFGR_AWD1EN_Pos );
  static constexpr Reg32_t CFGR_AWD1EN      = CFGR_AWD1EN_Msk;
  static constexpr Reg32_t CFGR_JAWD1EN_Pos = ( 24U );
  static constexpr Reg32_t CFGR_JAWD1EN_Msk = ( 0x1UL << CFGR_JAWD1EN_Pos );
  static constexpr Reg32_t CFGR_JAWD1EN     = CFGR_JAWD1EN_Msk;
  static constexpr Reg32_t CFGR_JAUTO_Pos   = ( 25U );
  static constexpr Reg32_t CFGR_JAUTO_Msk   = ( 0x1UL << CFGR_JAUTO_Pos );
  static constexpr Reg32_t CFGR_JAUTO       = CFGR_JAUTO_Msk;

  static constexpr Reg32_t CFGR_AWD1CH_Pos = ( 26U );
  static constexpr Reg32_t CFGR_AWD1CH_Msk = ( 0x1FUL << CFGR_AWD1CH_Pos );
  static constexpr Reg32_t CFGR_AWD1CH     = CFGR_AWD1CH_Msk;
  static constexpr Reg32_t CFGR_AWD1CH_0   = ( 0x01UL << CFGR_AWD1CH_Pos );
  static constexpr Reg32_t CFGR_AWD1CH_1   = ( 0x02UL << CFGR_AWD1CH_Pos );
  static constexpr Reg32_t CFGR_AWD1CH_2   = ( 0x04UL << CFGR_AWD1CH_Pos );
  static constexpr Reg32_t CFGR_AWD1CH_3   = ( 0x08UL << CFGR_AWD1CH_Pos );
  static constexpr Reg32_t CFGR_AWD1CH_4   = ( 0x10UL << CFGR_AWD1CH_Pos );

  static constexpr Reg32_t CFGR_JQDIS_Pos = ( 31U );
  static constexpr Reg32_t CFGR_JQDIS_Msk = ( 0x1UL << CFGR_JQDIS_Pos );
  static constexpr Reg32_t CFGR_JQDIS     = CFGR_JQDIS_Msk;

  /********************  Bit definition for CFGR2 register  *****************/
  static constexpr Reg32_t CFGR2_ROVSE_Pos = ( 0U );
  static constexpr Reg32_t CFGR2_ROVSE_Msk = ( 0x1UL << CFGR2_ROVSE_Pos );
  static constexpr Reg32_t CFGR2_ROVSE     = CFGR2_ROVSE_Msk;
  static constexpr Reg32_t CFGR2_JOVSE_Pos = ( 1U );
  static constexpr Reg32_t CFGR2_JOVSE_Msk = ( 0x1UL << CFGR2_JOVSE_Pos );
  static constexpr Reg32_t CFGR2_JOVSE     = CFGR2_JOVSE_Msk;

  static constexpr Reg32_t CFGR2_OVSR_Pos = ( 2U );
  static constexpr Reg32_t CFGR2_OVSR_Msk = ( 0x7UL << CFGR2_OVSR_Pos );
  static constexpr Reg32_t CFGR2_OVSR     = CFGR2_OVSR_Msk;
  static constexpr Reg32_t CFGR2_OVSR_0   = ( 0x1UL << CFGR2_OVSR_Pos );
  static constexpr Reg32_t CFGR2_OVSR_1   = ( 0x2UL << CFGR2_OVSR_Pos );
  static constexpr Reg32_t CFGR2_OVSR_2   = ( 0x4UL << CFGR2_OVSR_Pos );

  static constexpr Reg32_t CFGR2_OVSS_Pos = ( 5U );
  static constexpr Reg32_t CFGR2_OVSS_Msk = ( 0xFUL << CFGR2_OVSS_Pos );
  static constexpr Reg32_t CFGR2_OVSS     = CFGR2_OVSS_Msk;
  static constexpr Reg32_t CFGR2_OVSS_0   = ( 0x1UL << CFGR2_OVSS_Pos );
  static constexpr Reg32_t CFGR2_OVSS_1   = ( 0x2UL << CFGR2_OVSS_Pos );
  static constexpr Reg32_t CFGR2_OVSS_2   = ( 0x4UL << CFGR2_OVSS_Pos );
  static constexpr Reg32_t CFGR2_OVSS_3   = ( 0x8UL << CFGR2_OVSS_Pos );

  static constexpr Reg32_t CFGR2_TROVS_Pos = ( 9U );
  static constexpr Reg32_t CFGR2_TROVS_Msk = ( 0x1UL << CFGR2_TROVS_Pos );
  static constexpr Reg32_t CFGR2_TROVS     = CFGR2_TROVS_Msk;
  static constexpr Reg32_t CFGR2_ROVSM_Pos = ( 10U );
  static constexpr Reg32_t CFGR2_ROVSM_Msk = ( 0x1UL << CFGR2_ROVSM_Pos );
  static constexpr Reg32_t CFGR2_ROVSM     = CFGR2_ROVSM_Msk;

  /********************  Bit definition for SMPR1 register  *****************/
  static constexpr Reg32_t SMPRx_BIT_Wid = 3;
  static constexpr Reg32_t SMPRx_BIT_Msk = 0x7;
  static constexpr Reg32_t SMPR1_ALL_Msk = 0x3FFFFFFF;

  static constexpr Reg32_t SMPR1_SMP0_Pos = ( 0U );
  static constexpr Reg32_t SMPR1_SMP0_Msk = ( 0x7UL << SMPR1_SMP0_Pos );
  static constexpr Reg32_t SMPR1_SMP0     = SMPR1_SMP0_Msk;
  static constexpr Reg32_t SMPR1_SMP0_0   = ( 0x1UL << SMPR1_SMP0_Pos );
  static constexpr Reg32_t SMPR1_SMP0_1   = ( 0x2UL << SMPR1_SMP0_Pos );
  static constexpr Reg32_t SMPR1_SMP0_2   = ( 0x4UL << SMPR1_SMP0_Pos );

  static constexpr Reg32_t SMPR1_SMP1_Pos = ( 3U );
  static constexpr Reg32_t SMPR1_SMP1_Msk = ( 0x7UL << SMPR1_SMP1_Pos );
  static constexpr Reg32_t SMPR1_SMP1     = SMPR1_SMP1_Msk;
  static constexpr Reg32_t SMPR1_SMP1_0   = ( 0x1UL << SMPR1_SMP1_Pos );
  static constexpr Reg32_t SMPR1_SMP1_1   = ( 0x2UL << SMPR1_SMP1_Pos );
  static constexpr Reg32_t SMPR1_SMP1_2   = ( 0x4UL << SMPR1_SMP1_Pos );

  static constexpr Reg32_t SMPR1_SMP2_Pos = ( 6U );
  static constexpr Reg32_t SMPR1_SMP2_Msk = ( 0x7UL << SMPR1_SMP2_Pos );
  static constexpr Reg32_t SMPR1_SMP2     = SMPR1_SMP2_Msk;
  static constexpr Reg32_t SMPR1_SMP2_0   = ( 0x1UL << SMPR1_SMP2_Pos );
  static constexpr Reg32_t SMPR1_SMP2_1   = ( 0x2UL << SMPR1_SMP2_Pos );
  static constexpr Reg32_t SMPR1_SMP2_2   = ( 0x4UL << SMPR1_SMP2_Pos );

  static constexpr Reg32_t SMPR1_SMP3_Pos = ( 9U );
  static constexpr Reg32_t SMPR1_SMP3_Msk = ( 0x7UL << SMPR1_SMP3_Pos );
  static constexpr Reg32_t SMPR1_SMP3     = SMPR1_SMP3_Msk;
  static constexpr Reg32_t SMPR1_SMP3_0   = ( 0x1UL << SMPR1_SMP3_Pos );
  static constexpr Reg32_t SMPR1_SMP3_1   = ( 0x2UL << SMPR1_SMP3_Pos );
  static constexpr Reg32_t SMPR1_SMP3_2   = ( 0x4UL << SMPR1_SMP3_Pos );

  static constexpr Reg32_t SMPR1_SMP4_Pos = ( 12U );
  static constexpr Reg32_t SMPR1_SMP4_Msk = ( 0x7UL << SMPR1_SMP4_Pos );
  static constexpr Reg32_t SMPR1_SMP4     = SMPR1_SMP4_Msk;
  static constexpr Reg32_t SMPR1_SMP4_0   = ( 0x1UL << SMPR1_SMP4_Pos );
  static constexpr Reg32_t SMPR1_SMP4_1   = ( 0x2UL << SMPR1_SMP4_Pos );
  static constexpr Reg32_t SMPR1_SMP4_2   = ( 0x4UL << SMPR1_SMP4_Pos );

  static constexpr Reg32_t SMPR1_SMP5_Pos = ( 15U );
  static constexpr Reg32_t SMPR1_SMP5_Msk = ( 0x7UL << SMPR1_SMP5_Pos );
  static constexpr Reg32_t SMPR1_SMP5     = SMPR1_SMP5_Msk;
  static constexpr Reg32_t SMPR1_SMP5_0   = ( 0x1UL << SMPR1_SMP5_Pos );
  static constexpr Reg32_t SMPR1_SMP5_1   = ( 0x2UL << SMPR1_SMP5_Pos );
  static constexpr Reg32_t SMPR1_SMP5_2   = ( 0x4UL << SMPR1_SMP5_Pos );

  static constexpr Reg32_t SMPR1_SMP6_Pos = ( 18U );
  static constexpr Reg32_t SMPR1_SMP6_Msk = ( 0x7UL << SMPR1_SMP6_Pos );
  static constexpr Reg32_t SMPR1_SMP6     = SMPR1_SMP6_Msk;
  static constexpr Reg32_t SMPR1_SMP6_0   = ( 0x1UL << SMPR1_SMP6_Pos );
  static constexpr Reg32_t SMPR1_SMP6_1   = ( 0x2UL << SMPR1_SMP6_Pos );
  static constexpr Reg32_t SMPR1_SMP6_2   = ( 0x4UL << SMPR1_SMP6_Pos );

  static constexpr Reg32_t SMPR1_SMP7_Pos = ( 21U );
  static constexpr Reg32_t SMPR1_SMP7_Msk = ( 0x7UL << SMPR1_SMP7_Pos );
  static constexpr Reg32_t SMPR1_SMP7     = SMPR1_SMP7_Msk;
  static constexpr Reg32_t SMPR1_SMP7_0   = ( 0x1UL << SMPR1_SMP7_Pos );
  static constexpr Reg32_t SMPR1_SMP7_1   = ( 0x2UL << SMPR1_SMP7_Pos );
  static constexpr Reg32_t SMPR1_SMP7_2   = ( 0x4UL << SMPR1_SMP7_Pos );

  static constexpr Reg32_t SMPR1_SMP8_Pos = ( 24U );
  static constexpr Reg32_t SMPR1_SMP8_Msk = ( 0x7UL << SMPR1_SMP8_Pos );
  static constexpr Reg32_t SMPR1_SMP8     = SMPR1_SMP8_Msk;
  static constexpr Reg32_t SMPR1_SMP8_0   = ( 0x1UL << SMPR1_SMP8_Pos );
  static constexpr Reg32_t SMPR1_SMP8_1   = ( 0x2UL << SMPR1_SMP8_Pos );
  static constexpr Reg32_t SMPR1_SMP8_2   = ( 0x4UL << SMPR1_SMP8_Pos );

  static constexpr Reg32_t SMPR1_SMP9_Pos = ( 27U );
  static constexpr Reg32_t SMPR1_SMP9_Msk = ( 0x7UL << SMPR1_SMP9_Pos );
  static constexpr Reg32_t SMPR1_SMP9     = SMPR1_SMP9_Msk;
  static constexpr Reg32_t SMPR1_SMP9_0   = ( 0x1UL << SMPR1_SMP9_Pos );
  static constexpr Reg32_t SMPR1_SMP9_1   = ( 0x2UL << SMPR1_SMP9_Pos );
  static constexpr Reg32_t SMPR1_SMP9_2   = ( 0x4UL << SMPR1_SMP9_Pos );

  /********************  Bit definition for SMPR2 register  *****************/
  static constexpr Reg32_t SMPR2_ALL_Msk = 0x07FFFFFF;

  static constexpr Reg32_t SMPR2_SMP10_Pos = ( 0U );
  static constexpr Reg32_t SMPR2_SMP10_Msk = ( 0x7UL << SMPR2_SMP10_Pos );
  static constexpr Reg32_t SMPR2_SMP10     = SMPR2_SMP10_Msk;
  static constexpr Reg32_t SMPR2_SMP10_0   = ( 0x1UL << SMPR2_SMP10_Pos );
  static constexpr Reg32_t SMPR2_SMP10_1   = ( 0x2UL << SMPR2_SMP10_Pos );
  static constexpr Reg32_t SMPR2_SMP10_2   = ( 0x4UL << SMPR2_SMP10_Pos );

  static constexpr Reg32_t SMPR2_SMP11_Pos = ( 3U );
  static constexpr Reg32_t SMPR2_SMP11_Msk = ( 0x7UL << SMPR2_SMP11_Pos );
  static constexpr Reg32_t SMPR2_SMP11     = SMPR2_SMP11_Msk;
  static constexpr Reg32_t SMPR2_SMP11_0   = ( 0x1UL << SMPR2_SMP11_Pos );
  static constexpr Reg32_t SMPR2_SMP11_1   = ( 0x2UL << SMPR2_SMP11_Pos );
  static constexpr Reg32_t SMPR2_SMP11_2   = ( 0x4UL << SMPR2_SMP11_Pos );

  static constexpr Reg32_t SMPR2_SMP12_Pos = ( 6U );
  static constexpr Reg32_t SMPR2_SMP12_Msk = ( 0x7UL << SMPR2_SMP12_Pos );
  static constexpr Reg32_t SMPR2_SMP12     = SMPR2_SMP12_Msk;
  static constexpr Reg32_t SMPR2_SMP12_0   = ( 0x1UL << SMPR2_SMP12_Pos );
  static constexpr Reg32_t SMPR2_SMP12_1   = ( 0x2UL << SMPR2_SMP12_Pos );
  static constexpr Reg32_t SMPR2_SMP12_2   = ( 0x4UL << SMPR2_SMP12_Pos );

  static constexpr Reg32_t SMPR2_SMP13_Pos = ( 9U );
  static constexpr Reg32_t SMPR2_SMP13_Msk = ( 0x7UL << SMPR2_SMP13_Pos );
  static constexpr Reg32_t SMPR2_SMP13     = SMPR2_SMP13_Msk;
  static constexpr Reg32_t SMPR2_SMP13_0   = ( 0x1UL << SMPR2_SMP13_Pos );
  static constexpr Reg32_t SMPR2_SMP13_1   = ( 0x2UL << SMPR2_SMP13_Pos );
  static constexpr Reg32_t SMPR2_SMP13_2   = ( 0x4UL << SMPR2_SMP13_Pos );

  static constexpr Reg32_t SMPR2_SMP14_Pos = ( 12U );
  static constexpr Reg32_t SMPR2_SMP14_Msk = ( 0x7UL << SMPR2_SMP14_Pos );
  static constexpr Reg32_t SMPR2_SMP14     = SMPR2_SMP14_Msk;
  static constexpr Reg32_t SMPR2_SMP14_0   = ( 0x1UL << SMPR2_SMP14_Pos );
  static constexpr Reg32_t SMPR2_SMP14_1   = ( 0x2UL << SMPR2_SMP14_Pos );
  static constexpr Reg32_t SMPR2_SMP14_2   = ( 0x4UL << SMPR2_SMP14_Pos );

  static constexpr Reg32_t SMPR2_SMP15_Pos = ( 15U );
  static constexpr Reg32_t SMPR2_SMP15_Msk = ( 0x7UL << SMPR2_SMP15_Pos );
  static constexpr Reg32_t SMPR2_SMP15     = SMPR2_SMP15_Msk;
  static constexpr Reg32_t SMPR2_SMP15_0   = ( 0x1UL << SMPR2_SMP15_Pos );
  static constexpr Reg32_t SMPR2_SMP15_1   = ( 0x2UL << SMPR2_SMP15_Pos );
  static constexpr Reg32_t SMPR2_SMP15_2   = ( 0x4UL << SMPR2_SMP15_Pos );

  static constexpr Reg32_t SMPR2_SMP16_Pos = ( 18U );
  static constexpr Reg32_t SMPR2_SMP16_Msk = ( 0x7UL << SMPR2_SMP16_Pos );
  static constexpr Reg32_t SMPR2_SMP16     = SMPR2_SMP16_Msk;
  static constexpr Reg32_t SMPR2_SMP16_0   = ( 0x1UL << SMPR2_SMP16_Pos );
  static constexpr Reg32_t SMPR2_SMP16_1   = ( 0x2UL << SMPR2_SMP16_Pos );
  static constexpr Reg32_t SMPR2_SMP16_2   = ( 0x4UL << SMPR2_SMP16_Pos );

  static constexpr Reg32_t SMPR2_SMP17_Pos = ( 21U );
  static constexpr Reg32_t SMPR2_SMP17_Msk = ( 0x7UL << SMPR2_SMP17_Pos );
  static constexpr Reg32_t SMPR2_SMP17     = SMPR2_SMP17_Msk;
  static constexpr Reg32_t SMPR2_SMP17_0   = ( 0x1UL << SMPR2_SMP17_Pos );
  static constexpr Reg32_t SMPR2_SMP17_1   = ( 0x2UL << SMPR2_SMP17_Pos );
  static constexpr Reg32_t SMPR2_SMP17_2   = ( 0x4UL << SMPR2_SMP17_Pos );

  static constexpr Reg32_t SMPR2_SMP18_Pos = ( 24U );
  static constexpr Reg32_t SMPR2_SMP18_Msk = ( 0x7UL << SMPR2_SMP18_Pos );
  static constexpr Reg32_t SMPR2_SMP18     = SMPR2_SMP18_Msk;
  static constexpr Reg32_t SMPR2_SMP18_0   = ( 0x1UL << SMPR2_SMP18_Pos );
  static constexpr Reg32_t SMPR2_SMP18_1   = ( 0x2UL << SMPR2_SMP18_Pos );
  static constexpr Reg32_t SMPR2_SMP18_2   = ( 0x4UL << SMPR2_SMP18_Pos );

  /********************  Bit definition for TR1 register  *******************/
  static constexpr Reg32_t TR1_ALL_Msk = 0x0FFF0FFF;

  static constexpr Reg32_t TR1_LT1_Pos = ( 0U );
  static constexpr Reg32_t TR1_LT1_Msk = ( 0xFFFUL << TR1_LT1_Pos );
  static constexpr Reg32_t TR1_LT1     = TR1_LT1_Msk;
  static constexpr Reg32_t TR1_LT1_0   = ( 0x001UL << TR1_LT1_Pos );
  static constexpr Reg32_t TR1_LT1_1   = ( 0x002UL << TR1_LT1_Pos );
  static constexpr Reg32_t TR1_LT1_2   = ( 0x004UL << TR1_LT1_Pos );
  static constexpr Reg32_t TR1_LT1_3   = ( 0x008UL << TR1_LT1_Pos );
  static constexpr Reg32_t TR1_LT1_4   = ( 0x010UL << TR1_LT1_Pos );
  static constexpr Reg32_t TR1_LT1_5   = ( 0x020UL << TR1_LT1_Pos );
  static constexpr Reg32_t TR1_LT1_6   = ( 0x040UL << TR1_LT1_Pos );
  static constexpr Reg32_t TR1_LT1_7   = ( 0x080UL << TR1_LT1_Pos );
  static constexpr Reg32_t TR1_LT1_8   = ( 0x100UL << TR1_LT1_Pos );
  static constexpr Reg32_t TR1_LT1_9   = ( 0x200UL << TR1_LT1_Pos );
  static constexpr Reg32_t TR1_LT1_10  = ( 0x400UL << TR1_LT1_Pos );
  static constexpr Reg32_t TR1_LT1_11  = ( 0x800UL << TR1_LT1_Pos );

  static constexpr Reg32_t TR1_HT1_Pos = ( 16U );
  static constexpr Reg32_t TR1_HT1_Msk = ( 0xFFFUL << TR1_HT1_Pos );
  static constexpr Reg32_t TR1_HT1     = TR1_HT1_Msk;
  static constexpr Reg32_t TR1_HT1_0   = ( 0x001UL << TR1_HT1_Pos );
  static constexpr Reg32_t TR1_HT1_1   = ( 0x002UL << TR1_HT1_Pos );
  static constexpr Reg32_t TR1_HT1_2   = ( 0x004UL << TR1_HT1_Pos );
  static constexpr Reg32_t TR1_HT1_3   = ( 0x008UL << TR1_HT1_Pos );
  static constexpr Reg32_t TR1_HT1_4   = ( 0x010UL << TR1_HT1_Pos );
  static constexpr Reg32_t TR1_HT1_5   = ( 0x020UL << TR1_HT1_Pos );
  static constexpr Reg32_t TR1_HT1_6   = ( 0x040UL << TR1_HT1_Pos );
  static constexpr Reg32_t TR1_HT1_7   = ( 0x080UL << TR1_HT1_Pos );
  static constexpr Reg32_t TR1_HT1_8   = ( 0x100UL << TR1_HT1_Pos );
  static constexpr Reg32_t TR1_HT1_9   = ( 0x200UL << TR1_HT1_Pos );
  static constexpr Reg32_t TR1_HT1_10  = ( 0x400UL << TR1_HT1_Pos );
  static constexpr Reg32_t TR1_HT1_11  = ( 0x800UL << TR1_HT1_Pos );

  /********************  Bit definition for TR2 register  *******************/
  static constexpr Reg32_t TR2_ALL_Msk = 0x00FF00FF;

  static constexpr Reg32_t TR2_LT2_Pos = ( 0U );
  static constexpr Reg32_t TR2_LT2_Msk = ( 0xFFUL << TR2_LT2_Pos );
  static constexpr Reg32_t TR2_LT2     = TR2_LT2_Msk;
  static constexpr Reg32_t TR2_LT2_0   = ( 0x01UL << TR2_LT2_Pos );
  static constexpr Reg32_t TR2_LT2_1   = ( 0x02UL << TR2_LT2_Pos );
  static constexpr Reg32_t TR2_LT2_2   = ( 0x04UL << TR2_LT2_Pos );
  static constexpr Reg32_t TR2_LT2_3   = ( 0x08UL << TR2_LT2_Pos );
  static constexpr Reg32_t TR2_LT2_4   = ( 0x10UL << TR2_LT2_Pos );
  static constexpr Reg32_t TR2_LT2_5   = ( 0x20UL << TR2_LT2_Pos );
  static constexpr Reg32_t TR2_LT2_6   = ( 0x40UL << TR2_LT2_Pos );
  static constexpr Reg32_t TR2_LT2_7   = ( 0x80UL << TR2_LT2_Pos );

  static constexpr Reg32_t TR2_HT2_Pos = ( 16U );
  static constexpr Reg32_t TR2_HT2_Msk = ( 0xFFUL << TR2_HT2_Pos );
  static constexpr Reg32_t TR2_HT2     = TR2_HT2_Msk;
  static constexpr Reg32_t TR2_HT2_0   = ( 0x01UL << TR2_HT2_Pos );
  static constexpr Reg32_t TR2_HT2_1   = ( 0x02UL << TR2_HT2_Pos );
  static constexpr Reg32_t TR2_HT2_2   = ( 0x04UL << TR2_HT2_Pos );
  static constexpr Reg32_t TR2_HT2_3   = ( 0x08UL << TR2_HT2_Pos );
  static constexpr Reg32_t TR2_HT2_4   = ( 0x10UL << TR2_HT2_Pos );
  static constexpr Reg32_t TR2_HT2_5   = ( 0x20UL << TR2_HT2_Pos );
  static constexpr Reg32_t TR2_HT2_6   = ( 0x40UL << TR2_HT2_Pos );
  static constexpr Reg32_t TR2_HT2_7   = ( 0x80UL << TR2_HT2_Pos );

  /********************  Bit definition for TR3 register  *******************/
  static constexpr Reg32_t TR3_ALL_Msk = 0x00FF00FF;

  static constexpr Reg32_t TR3_LT3_Pos = ( 0U );
  static constexpr Reg32_t TR3_LT3_Msk = ( 0xFFUL << TR3_LT3_Pos );
  static constexpr Reg32_t TR3_LT3     = TR3_LT3_Msk;
  static constexpr Reg32_t TR3_LT3_0   = ( 0x01UL << TR3_LT3_Pos );
  static constexpr Reg32_t TR3_LT3_1   = ( 0x02UL << TR3_LT3_Pos );
  static constexpr Reg32_t TR3_LT3_2   = ( 0x04UL << TR3_LT3_Pos );
  static constexpr Reg32_t TR3_LT3_3   = ( 0x08UL << TR3_LT3_Pos );
  static constexpr Reg32_t TR3_LT3_4   = ( 0x10UL << TR3_LT3_Pos );
  static constexpr Reg32_t TR3_LT3_5   = ( 0x20UL << TR3_LT3_Pos );
  static constexpr Reg32_t TR3_LT3_6   = ( 0x40UL << TR3_LT3_Pos );
  static constexpr Reg32_t TR3_LT3_7   = ( 0x80UL << TR3_LT3_Pos );

  static constexpr Reg32_t TR3_HT3_Pos = ( 16U );
  static constexpr Reg32_t TR3_HT3_Msk = ( 0xFFUL << TR3_HT3_Pos );
  static constexpr Reg32_t TR3_HT3     = TR3_HT3_Msk;
  static constexpr Reg32_t TR3_HT3_0   = ( 0x01UL << TR3_HT3_Pos );
  static constexpr Reg32_t TR3_HT3_1   = ( 0x02UL << TR3_HT3_Pos );
  static constexpr Reg32_t TR3_HT3_2   = ( 0x04UL << TR3_HT3_Pos );
  static constexpr Reg32_t TR3_HT3_3   = ( 0x08UL << TR3_HT3_Pos );
  static constexpr Reg32_t TR3_HT3_4   = ( 0x10UL << TR3_HT3_Pos );
  static constexpr Reg32_t TR3_HT3_5   = ( 0x20UL << TR3_HT3_Pos );
  static constexpr Reg32_t TR3_HT3_6   = ( 0x40UL << TR3_HT3_Pos );
  static constexpr Reg32_t TR3_HT3_7   = ( 0x80UL << TR3_HT3_Pos );

  /********************  Bit definition for SQR1 register  ******************/
  static constexpr Reg32_t SQR1_BIT_Wid = 5;
  static constexpr Reg32_t SQR1_BIT_Msk = 0x1F;
  static constexpr Reg32_t SQR1_ALL_Msk = 0x1F7DF7CF;

  static constexpr Reg32_t SQR1_L_Pos = ( 0U );
  static constexpr Reg32_t SQR1_L_Msk = ( 0xFUL << SQR1_L_Pos );
  static constexpr Reg32_t SQR1_L     = SQR1_L_Msk;
  static constexpr Reg32_t SQR1_L_0   = ( 0x1UL << SQR1_L_Pos );
  static constexpr Reg32_t SQR1_L_1   = ( 0x2UL << SQR1_L_Pos );
  static constexpr Reg32_t SQR1_L_2   = ( 0x4UL << SQR1_L_Pos );
  static constexpr Reg32_t SQR1_L_3   = ( 0x8UL << SQR1_L_Pos );

  static constexpr Reg32_t SQR1_SQ1_Pos = ( 6U );
  static constexpr Reg32_t SQR1_SQ1_Msk = ( 0x1FUL << SQR1_SQ1_Pos );
  static constexpr Reg32_t SQR1_SQ1     = SQR1_SQ1_Msk;
  static constexpr Reg32_t SQR1_SQ1_0   = ( 0x01UL << SQR1_SQ1_Pos );
  static constexpr Reg32_t SQR1_SQ1_1   = ( 0x02UL << SQR1_SQ1_Pos );
  static constexpr Reg32_t SQR1_SQ1_2   = ( 0x04UL << SQR1_SQ1_Pos );
  static constexpr Reg32_t SQR1_SQ1_3   = ( 0x08UL << SQR1_SQ1_Pos );
  static constexpr Reg32_t SQR1_SQ1_4   = ( 0x10UL << SQR1_SQ1_Pos );

  static constexpr Reg32_t SQR1_SQ2_Pos = ( 12U );
  static constexpr Reg32_t SQR1_SQ2_Msk = ( 0x1FUL << SQR1_SQ2_Pos );
  static constexpr Reg32_t SQR1_SQ2     = SQR1_SQ2_Msk;
  static constexpr Reg32_t SQR1_SQ2_0   = ( 0x01UL << SQR1_SQ2_Pos );
  static constexpr Reg32_t SQR1_SQ2_1   = ( 0x02UL << SQR1_SQ2_Pos );
  static constexpr Reg32_t SQR1_SQ2_2   = ( 0x04UL << SQR1_SQ2_Pos );
  static constexpr Reg32_t SQR1_SQ2_3   = ( 0x08UL << SQR1_SQ2_Pos );
  static constexpr Reg32_t SQR1_SQ2_4   = ( 0x10UL << SQR1_SQ2_Pos );

  static constexpr Reg32_t SQR1_SQ3_Pos = ( 18U );
  static constexpr Reg32_t SQR1_SQ3_Msk = ( 0x1FUL << SQR1_SQ3_Pos );
  static constexpr Reg32_t SQR1_SQ3     = SQR1_SQ3_Msk;
  static constexpr Reg32_t SQR1_SQ3_0   = ( 0x01UL << SQR1_SQ3_Pos );
  static constexpr Reg32_t SQR1_SQ3_1   = ( 0x02UL << SQR1_SQ3_Pos );
  static constexpr Reg32_t SQR1_SQ3_2   = ( 0x04UL << SQR1_SQ3_Pos );
  static constexpr Reg32_t SQR1_SQ3_3   = ( 0x08UL << SQR1_SQ3_Pos );
  static constexpr Reg32_t SQR1_SQ3_4   = ( 0x10UL << SQR1_SQ3_Pos );

  static constexpr Reg32_t SQR1_SQ4_Pos = ( 24U );
  static constexpr Reg32_t SQR1_SQ4_Msk = ( 0x1FUL << SQR1_SQ4_Pos );
  static constexpr Reg32_t SQR1_SQ4     = SQR1_SQ4_Msk;
  static constexpr Reg32_t SQR1_SQ4_0   = ( 0x01UL << SQR1_SQ4_Pos );
  static constexpr Reg32_t SQR1_SQ4_1   = ( 0x02UL << SQR1_SQ4_Pos );
  static constexpr Reg32_t SQR1_SQ4_2   = ( 0x04UL << SQR1_SQ4_Pos );
  static constexpr Reg32_t SQR1_SQ4_3   = ( 0x08UL << SQR1_SQ4_Pos );
  static constexpr Reg32_t SQR1_SQ4_4   = ( 0x10UL << SQR1_SQ4_Pos );

  /********************  Bit definition for SQR2 register  ******************/
  static constexpr Reg32_t SQR2_BIT_Wid = 5;
  static constexpr Reg32_t SQR2_BIT_Msk = 0x1F;
  static constexpr Reg32_t SQR2_ALL_Msk = 0x1F7DF7DF;

  static constexpr Reg32_t SQR2_SQ5_Pos = ( 0U );
  static constexpr Reg32_t SQR2_SQ5_Msk = ( 0x1FUL << SQR2_SQ5_Pos );
  static constexpr Reg32_t SQR2_SQ5     = SQR2_SQ5_Msk;
  static constexpr Reg32_t SQR2_SQ5_0   = ( 0x01UL << SQR2_SQ5_Pos );
  static constexpr Reg32_t SQR2_SQ5_1   = ( 0x02UL << SQR2_SQ5_Pos );
  static constexpr Reg32_t SQR2_SQ5_2   = ( 0x04UL << SQR2_SQ5_Pos );
  static constexpr Reg32_t SQR2_SQ5_3   = ( 0x08UL << SQR2_SQ5_Pos );
  static constexpr Reg32_t SQR2_SQ5_4   = ( 0x10UL << SQR2_SQ5_Pos );

  static constexpr Reg32_t SQR2_SQ6_Pos = ( 6U );
  static constexpr Reg32_t SQR2_SQ6_Msk = ( 0x1FUL << SQR2_SQ6_Pos );
  static constexpr Reg32_t SQR2_SQ6     = SQR2_SQ6_Msk;
  static constexpr Reg32_t SQR2_SQ6_0   = ( 0x01UL << SQR2_SQ6_Pos );
  static constexpr Reg32_t SQR2_SQ6_1   = ( 0x02UL << SQR2_SQ6_Pos );
  static constexpr Reg32_t SQR2_SQ6_2   = ( 0x04UL << SQR2_SQ6_Pos );
  static constexpr Reg32_t SQR2_SQ6_3   = ( 0x08UL << SQR2_SQ6_Pos );
  static constexpr Reg32_t SQR2_SQ6_4   = ( 0x10UL << SQR2_SQ6_Pos );

  static constexpr Reg32_t SQR2_SQ7_Pos = ( 12U );
  static constexpr Reg32_t SQR2_SQ7_Msk = ( 0x1FUL << SQR2_SQ7_Pos );
  static constexpr Reg32_t SQR2_SQ7     = SQR2_SQ7_Msk;
  static constexpr Reg32_t SQR2_SQ7_0   = ( 0x01UL << SQR2_SQ7_Pos );
  static constexpr Reg32_t SQR2_SQ7_1   = ( 0x02UL << SQR2_SQ7_Pos );
  static constexpr Reg32_t SQR2_SQ7_2   = ( 0x04UL << SQR2_SQ7_Pos );
  static constexpr Reg32_t SQR2_SQ7_3   = ( 0x08UL << SQR2_SQ7_Pos );
  static constexpr Reg32_t SQR2_SQ7_4   = ( 0x10UL << SQR2_SQ7_Pos );

  static constexpr Reg32_t SQR2_SQ8_Pos = ( 18U );
  static constexpr Reg32_t SQR2_SQ8_Msk = ( 0x1FUL << SQR2_SQ8_Pos );
  static constexpr Reg32_t SQR2_SQ8     = SQR2_SQ8_Msk;
  static constexpr Reg32_t SQR2_SQ8_0   = ( 0x01UL << SQR2_SQ8_Pos );
  static constexpr Reg32_t SQR2_SQ8_1   = ( 0x02UL << SQR2_SQ8_Pos );
  static constexpr Reg32_t SQR2_SQ8_2   = ( 0x04UL << SQR2_SQ8_Pos );
  static constexpr Reg32_t SQR2_SQ8_3   = ( 0x08UL << SQR2_SQ8_Pos );
  static constexpr Reg32_t SQR2_SQ8_4   = ( 0x10UL << SQR2_SQ8_Pos );

  static constexpr Reg32_t SQR2_SQ9_Pos = ( 24U );
  static constexpr Reg32_t SQR2_SQ9_Msk = ( 0x1FUL << SQR2_SQ9_Pos );
  static constexpr Reg32_t SQR2_SQ9     = SQR2_SQ9_Msk;
  static constexpr Reg32_t SQR2_SQ9_0   = ( 0x01UL << SQR2_SQ9_Pos );
  static constexpr Reg32_t SQR2_SQ9_1   = ( 0x02UL << SQR2_SQ9_Pos );
  static constexpr Reg32_t SQR2_SQ9_2   = ( 0x04UL << SQR2_SQ9_Pos );
  static constexpr Reg32_t SQR2_SQ9_3   = ( 0x08UL << SQR2_SQ9_Pos );
  static constexpr Reg32_t SQR2_SQ9_4   = ( 0x10UL << SQR2_SQ9_Pos );

  /********************  Bit definition for SQR3 register  ******************/
  static constexpr Reg32_t SQR3_BIT_Wid = 5;
  static constexpr Reg32_t SQR3_BIT_Msk = 0x1F;
  static constexpr Reg32_t SQR3_ALL_Msk = 0x1F7DF7DF;

  static constexpr Reg32_t SQR3_SQ10_Pos = ( 0U );
  static constexpr Reg32_t SQR3_SQ10_Msk = ( 0x1FUL << SQR3_SQ10_Pos );
  static constexpr Reg32_t SQR3_SQ10     = SQR3_SQ10_Msk;
  static constexpr Reg32_t SQR3_SQ10_0   = ( 0x01UL << SQR3_SQ10_Pos );
  static constexpr Reg32_t SQR3_SQ10_1   = ( 0x02UL << SQR3_SQ10_Pos );
  static constexpr Reg32_t SQR3_SQ10_2   = ( 0x04UL << SQR3_SQ10_Pos );
  static constexpr Reg32_t SQR3_SQ10_3   = ( 0x08UL << SQR3_SQ10_Pos );
  static constexpr Reg32_t SQR3_SQ10_4   = ( 0x10UL << SQR3_SQ10_Pos );

  static constexpr Reg32_t SQR3_SQ11_Pos = ( 6U );
  static constexpr Reg32_t SQR3_SQ11_Msk = ( 0x1FUL << SQR3_SQ11_Pos );
  static constexpr Reg32_t SQR3_SQ11     = SQR3_SQ11_Msk;
  static constexpr Reg32_t SQR3_SQ11_0   = ( 0x01UL << SQR3_SQ11_Pos );
  static constexpr Reg32_t SQR3_SQ11_1   = ( 0x02UL << SQR3_SQ11_Pos );
  static constexpr Reg32_t SQR3_SQ11_2   = ( 0x04UL << SQR3_SQ11_Pos );
  static constexpr Reg32_t SQR3_SQ11_3   = ( 0x08UL << SQR3_SQ11_Pos );
  static constexpr Reg32_t SQR3_SQ11_4   = ( 0x10UL << SQR3_SQ11_Pos );

  static constexpr Reg32_t SQR3_SQ12_Pos = ( 12U );
  static constexpr Reg32_t SQR3_SQ12_Msk = ( 0x1FUL << SQR3_SQ12_Pos );
  static constexpr Reg32_t SQR3_SQ12     = SQR3_SQ12_Msk;
  static constexpr Reg32_t SQR3_SQ12_0   = ( 0x01UL << SQR3_SQ12_Pos );
  static constexpr Reg32_t SQR3_SQ12_1   = ( 0x02UL << SQR3_SQ12_Pos );
  static constexpr Reg32_t SQR3_SQ12_2   = ( 0x04UL << SQR3_SQ12_Pos );
  static constexpr Reg32_t SQR3_SQ12_3   = ( 0x08UL << SQR3_SQ12_Pos );
  static constexpr Reg32_t SQR3_SQ12_4   = ( 0x10UL << SQR3_SQ12_Pos );

  static constexpr Reg32_t SQR3_SQ13_Pos = ( 18U );
  static constexpr Reg32_t SQR3_SQ13_Msk = ( 0x1FUL << SQR3_SQ13_Pos );
  static constexpr Reg32_t SQR3_SQ13     = SQR3_SQ13_Msk;
  static constexpr Reg32_t SQR3_SQ13_0   = ( 0x01UL << SQR3_SQ13_Pos );
  static constexpr Reg32_t SQR3_SQ13_1   = ( 0x02UL << SQR3_SQ13_Pos );
  static constexpr Reg32_t SQR3_SQ13_2   = ( 0x04UL << SQR3_SQ13_Pos );
  static constexpr Reg32_t SQR3_SQ13_3   = ( 0x08UL << SQR3_SQ13_Pos );
  static constexpr Reg32_t SQR3_SQ13_4   = ( 0x10UL << SQR3_SQ13_Pos );

  static constexpr Reg32_t SQR3_SQ14_Pos = ( 24U );
  static constexpr Reg32_t SQR3_SQ14_Msk = ( 0x1FUL << SQR3_SQ14_Pos );
  static constexpr Reg32_t SQR3_SQ14     = SQR3_SQ14_Msk;
  static constexpr Reg32_t SQR3_SQ14_0   = ( 0x01UL << SQR3_SQ14_Pos );
  static constexpr Reg32_t SQR3_SQ14_1   = ( 0x02UL << SQR3_SQ14_Pos );
  static constexpr Reg32_t SQR3_SQ14_2   = ( 0x04UL << SQR3_SQ14_Pos );
  static constexpr Reg32_t SQR3_SQ14_3   = ( 0x08UL << SQR3_SQ14_Pos );
  static constexpr Reg32_t SQR3_SQ14_4   = ( 0x10UL << SQR3_SQ14_Pos );

  /********************  Bit definition for SQR4 register  ******************/
  static constexpr Reg32_t SQR4_BIT_Wid = 5;
  static constexpr Reg32_t SQR4_BIT_Msk = 0x1F;
  static constexpr Reg32_t SQR4_ALL_Msk = 0x000007DF;

  static constexpr Reg32_t SQR4_SQ15_Pos = ( 0U );
  static constexpr Reg32_t SQR4_SQ15_Msk = ( 0x1FUL << SQR4_SQ15_Pos );
  static constexpr Reg32_t SQR4_SQ15     = SQR4_SQ15_Msk;
  static constexpr Reg32_t SQR4_SQ15_0   = ( 0x01UL << SQR4_SQ15_Pos );
  static constexpr Reg32_t SQR4_SQ15_1   = ( 0x02UL << SQR4_SQ15_Pos );
  static constexpr Reg32_t SQR4_SQ15_2   = ( 0x04UL << SQR4_SQ15_Pos );
  static constexpr Reg32_t SQR4_SQ15_3   = ( 0x08UL << SQR4_SQ15_Pos );
  static constexpr Reg32_t SQR4_SQ15_4   = ( 0x10UL << SQR4_SQ15_Pos );

  static constexpr Reg32_t SQR4_SQ16_Pos = ( 6U );
  static constexpr Reg32_t SQR4_SQ16_Msk = ( 0x1FUL << SQR4_SQ16_Pos );
  static constexpr Reg32_t SQR4_SQ16     = SQR4_SQ16_Msk;
  static constexpr Reg32_t SQR4_SQ16_0   = ( 0x01UL << SQR4_SQ16_Pos );
  static constexpr Reg32_t SQR4_SQ16_1   = ( 0x02UL << SQR4_SQ16_Pos );
  static constexpr Reg32_t SQR4_SQ16_2   = ( 0x04UL << SQR4_SQ16_Pos );
  static constexpr Reg32_t SQR4_SQ16_3   = ( 0x08UL << SQR4_SQ16_Pos );
  static constexpr Reg32_t SQR4_SQ16_4   = ( 0x10UL << SQR4_SQ16_Pos );

  /********************  Bit definition for DR register  ********************/
  static constexpr Reg32_t DR_RDATA_Pos = ( 0U );
  static constexpr Reg32_t DR_RDATA_Msk = ( 0xFFFFUL << DR_RDATA_Pos );
  static constexpr Reg32_t DR_RDATA     = DR_RDATA_Msk;
  static constexpr Reg32_t DR_RDATA_0   = ( 0x0001UL << DR_RDATA_Pos );
  static constexpr Reg32_t DR_RDATA_1   = ( 0x0002UL << DR_RDATA_Pos );
  static constexpr Reg32_t DR_RDATA_2   = ( 0x0004UL << DR_RDATA_Pos );
  static constexpr Reg32_t DR_RDATA_3   = ( 0x0008UL << DR_RDATA_Pos );
  static constexpr Reg32_t DR_RDATA_4   = ( 0x0010UL << DR_RDATA_Pos );
  static constexpr Reg32_t DR_RDATA_5   = ( 0x0020UL << DR_RDATA_Pos );
  static constexpr Reg32_t DR_RDATA_6   = ( 0x0040UL << DR_RDATA_Pos );
  static constexpr Reg32_t DR_RDATA_7   = ( 0x0080UL << DR_RDATA_Pos );
  static constexpr Reg32_t DR_RDATA_8   = ( 0x0100UL << DR_RDATA_Pos );
  static constexpr Reg32_t DR_RDATA_9   = ( 0x0200UL << DR_RDATA_Pos );
  static constexpr Reg32_t DR_RDATA_10  = ( 0x0400UL << DR_RDATA_Pos );
  static constexpr Reg32_t DR_RDATA_11  = ( 0x0800UL << DR_RDATA_Pos );
  static constexpr Reg32_t DR_RDATA_12  = ( 0x1000UL << DR_RDATA_Pos );
  static constexpr Reg32_t DR_RDATA_13  = ( 0x2000UL << DR_RDATA_Pos );
  static constexpr Reg32_t DR_RDATA_14  = ( 0x4000UL << DR_RDATA_Pos );
  static constexpr Reg32_t DR_RDATA_15  = ( 0x8000UL << DR_RDATA_Pos );

  /********************  Bit definition for JSQR register  ******************/
  static constexpr Reg32_t JSQR_ALL_Msk = 0x7DF7DFFF;

  static constexpr Reg32_t JSQR_JL_Pos = ( 0U );
  static constexpr Reg32_t JSQR_JL_Msk = ( 0x3UL << JSQR_JL_Pos );
  static constexpr Reg32_t JSQR_JL     = JSQR_JL_Msk;
  static constexpr Reg32_t JSQR_JL_0   = ( 0x1UL << JSQR_JL_Pos );
  static constexpr Reg32_t JSQR_JL_1   = ( 0x2UL << JSQR_JL_Pos );

  static constexpr Reg32_t JSQR_JEXTSEL_Pos = ( 2U );
  static constexpr Reg32_t JSQR_JEXTSEL_Msk = ( 0xFUL << JSQR_JEXTSEL_Pos );
  static constexpr Reg32_t JSQR_JEXTSEL     = JSQR_JEXTSEL_Msk;
  static constexpr Reg32_t JSQR_JEXTSEL_0   = ( 0x1UL << JSQR_JEXTSEL_Pos );
  static constexpr Reg32_t JSQR_JEXTSEL_1   = ( 0x2UL << JSQR_JEXTSEL_Pos );
  static constexpr Reg32_t JSQR_JEXTSEL_2   = ( 0x4UL << JSQR_JEXTSEL_Pos );
  static constexpr Reg32_t JSQR_JEXTSEL_3   = ( 0x8UL << JSQR_JEXTSEL_Pos );

  static constexpr Reg32_t JSQR_JEXTEN_Pos = ( 6U );
  static constexpr Reg32_t JSQR_JEXTEN_Msk = ( 0x3UL << JSQR_JEXTEN_Pos );
  static constexpr Reg32_t JSQR_JEXTEN     = JSQR_JEXTEN_Msk;
  static constexpr Reg32_t JSQR_JEXTEN_0   = ( 0x1UL << JSQR_JEXTEN_Pos );
  static constexpr Reg32_t JSQR_JEXTEN_1   = ( 0x2UL << JSQR_JEXTEN_Pos );

  static constexpr Reg32_t JSQR_JSQ1_Pos = ( 8U );
  static constexpr Reg32_t JSQR_JSQ1_Msk = ( 0x1FUL << JSQR_JSQ1_Pos );
  static constexpr Reg32_t JSQR_JSQ1     = JSQR_JSQ1_Msk;
  static constexpr Reg32_t JSQR_JSQ1_0   = ( 0x01UL << JSQR_JSQ1_Pos );
  static constexpr Reg32_t JSQR_JSQ1_1   = ( 0x02UL << JSQR_JSQ1_Pos );
  static constexpr Reg32_t JSQR_JSQ1_2   = ( 0x04UL << JSQR_JSQ1_Pos );
  static constexpr Reg32_t JSQR_JSQ1_3   = ( 0x08UL << JSQR_JSQ1_Pos );
  static constexpr Reg32_t JSQR_JSQ1_4   = ( 0x10UL << JSQR_JSQ1_Pos );

  static constexpr Reg32_t JSQR_JSQ2_Pos = ( 14U );
  static constexpr Reg32_t JSQR_JSQ2_Msk = ( 0x1FUL << JSQR_JSQ2_Pos );
  static constexpr Reg32_t JSQR_JSQ2     = JSQR_JSQ2_Msk;
  static constexpr Reg32_t JSQR_JSQ2_0   = ( 0x01UL << JSQR_JSQ2_Pos );
  static constexpr Reg32_t JSQR_JSQ2_1   = ( 0x02UL << JSQR_JSQ2_Pos );
  static constexpr Reg32_t JSQR_JSQ2_2   = ( 0x04UL << JSQR_JSQ2_Pos );
  static constexpr Reg32_t JSQR_JSQ2_3   = ( 0x08UL << JSQR_JSQ2_Pos );
  static constexpr Reg32_t JSQR_JSQ2_4   = ( 0x10UL << JSQR_JSQ2_Pos );

  static constexpr Reg32_t JSQR_JSQ3_Pos = ( 20U );
  static constexpr Reg32_t JSQR_JSQ3_Msk = ( 0x1FUL << JSQR_JSQ3_Pos );
  static constexpr Reg32_t JSQR_JSQ3     = JSQR_JSQ3_Msk;
  static constexpr Reg32_t JSQR_JSQ3_0   = ( 0x01UL << JSQR_JSQ3_Pos );
  static constexpr Reg32_t JSQR_JSQ3_1   = ( 0x02UL << JSQR_JSQ3_Pos );
  static constexpr Reg32_t JSQR_JSQ3_2   = ( 0x04UL << JSQR_JSQ3_Pos );
  static constexpr Reg32_t JSQR_JSQ3_3   = ( 0x08UL << JSQR_JSQ3_Pos );
  static constexpr Reg32_t JSQR_JSQ3_4   = ( 0x10UL << JSQR_JSQ3_Pos );

  static constexpr Reg32_t JSQR_JSQ4_Pos = ( 26U );
  static constexpr Reg32_t JSQR_JSQ4_Msk = ( 0x1FUL << JSQR_JSQ4_Pos );
  static constexpr Reg32_t JSQR_JSQ4     = JSQR_JSQ4_Msk;
  static constexpr Reg32_t JSQR_JSQ4_0   = ( 0x01UL << JSQR_JSQ4_Pos );
  static constexpr Reg32_t JSQR_JSQ4_1   = ( 0x02UL << JSQR_JSQ4_Pos );
  static constexpr Reg32_t JSQR_JSQ4_2   = ( 0x04UL << JSQR_JSQ4_Pos );
  static constexpr Reg32_t JSQR_JSQ4_3   = ( 0x08UL << JSQR_JSQ4_Pos );
  static constexpr Reg32_t JSQR_JSQ4_4   = ( 0x10UL << JSQR_JSQ4_Pos );

  /********************  Bit definition for OFR1 register  ******************/
  static constexpr Reg32_t OFR1_OFFSET1_Pos = ( 0U );
  static constexpr Reg32_t OFR1_OFFSET1_Msk = ( 0xFFFUL << OFR1_OFFSET1_Pos );
  static constexpr Reg32_t OFR1_OFFSET1     = OFR1_OFFSET1_Msk;
  static constexpr Reg32_t OFR1_OFFSET1_0   = ( 0x001UL << OFR1_OFFSET1_Pos );
  static constexpr Reg32_t OFR1_OFFSET1_1   = ( 0x002UL << OFR1_OFFSET1_Pos );
  static constexpr Reg32_t OFR1_OFFSET1_2   = ( 0x004UL << OFR1_OFFSET1_Pos );
  static constexpr Reg32_t OFR1_OFFSET1_3   = ( 0x008UL << OFR1_OFFSET1_Pos );
  static constexpr Reg32_t OFR1_OFFSET1_4   = ( 0x010UL << OFR1_OFFSET1_Pos );
  static constexpr Reg32_t OFR1_OFFSET1_5   = ( 0x020UL << OFR1_OFFSET1_Pos );
  static constexpr Reg32_t OFR1_OFFSET1_6   = ( 0x040UL << OFR1_OFFSET1_Pos );
  static constexpr Reg32_t OFR1_OFFSET1_7   = ( 0x080UL << OFR1_OFFSET1_Pos );
  static constexpr Reg32_t OFR1_OFFSET1_8   = ( 0x100UL << OFR1_OFFSET1_Pos );
  static constexpr Reg32_t OFR1_OFFSET1_9   = ( 0x200UL << OFR1_OFFSET1_Pos );
  static constexpr Reg32_t OFR1_OFFSET1_10  = ( 0x400UL << OFR1_OFFSET1_Pos );
  static constexpr Reg32_t OFR1_OFFSET1_11  = ( 0x800UL << OFR1_OFFSET1_Pos );

  static constexpr Reg32_t OFR1_OFFSET1_CH_Pos = ( 26U );
  static constexpr Reg32_t OFR1_OFFSET1_CH_Msk = ( 0x1FUL << OFR1_OFFSET1_CH_Pos );
  static constexpr Reg32_t OFR1_OFFSET1_CH     = OFR1_OFFSET1_CH_Msk;
  static constexpr Reg32_t OFR1_OFFSET1_CH_0   = ( 0x01UL << OFR1_OFFSET1_CH_Pos );
  static constexpr Reg32_t OFR1_OFFSET1_CH_1   = ( 0x02UL << OFR1_OFFSET1_CH_Pos );
  static constexpr Reg32_t OFR1_OFFSET1_CH_2   = ( 0x04UL << OFR1_OFFSET1_CH_Pos );
  static constexpr Reg32_t OFR1_OFFSET1_CH_3   = ( 0x08UL << OFR1_OFFSET1_CH_Pos );
  static constexpr Reg32_t OFR1_OFFSET1_CH_4   = ( 0x10UL << OFR1_OFFSET1_CH_Pos );

  static constexpr Reg32_t OFR1_OFFSET1_EN_Pos = ( 31U );
  static constexpr Reg32_t OFR1_OFFSET1_EN_Msk = ( 0x1UL << OFR1_OFFSET1_EN_Pos );
  static constexpr Reg32_t OFR1_OFFSET1_EN     = OFR1_OFFSET1_EN_Msk;

  /********************  Bit definition for OFR2 register  ******************/
  static constexpr Reg32_t OFR2_OFFSET2_Pos = ( 0U );
  static constexpr Reg32_t OFR2_OFFSET2_Msk = ( 0xFFFUL << OFR2_OFFSET2_Pos );
  static constexpr Reg32_t OFR2_OFFSET2     = OFR2_OFFSET2_Msk;
  static constexpr Reg32_t OFR2_OFFSET2_0   = ( 0x001UL << OFR2_OFFSET2_Pos );
  static constexpr Reg32_t OFR2_OFFSET2_1   = ( 0x002UL << OFR2_OFFSET2_Pos );
  static constexpr Reg32_t OFR2_OFFSET2_2   = ( 0x004UL << OFR2_OFFSET2_Pos );
  static constexpr Reg32_t OFR2_OFFSET2_3   = ( 0x008UL << OFR2_OFFSET2_Pos );
  static constexpr Reg32_t OFR2_OFFSET2_4   = ( 0x010UL << OFR2_OFFSET2_Pos );
  static constexpr Reg32_t OFR2_OFFSET2_5   = ( 0x020UL << OFR2_OFFSET2_Pos );
  static constexpr Reg32_t OFR2_OFFSET2_6   = ( 0x040UL << OFR2_OFFSET2_Pos );
  static constexpr Reg32_t OFR2_OFFSET2_7   = ( 0x080UL << OFR2_OFFSET2_Pos );
  static constexpr Reg32_t OFR2_OFFSET2_8   = ( 0x100UL << OFR2_OFFSET2_Pos );
  static constexpr Reg32_t OFR2_OFFSET2_9   = ( 0x200UL << OFR2_OFFSET2_Pos );
  static constexpr Reg32_t OFR2_OFFSET2_10  = ( 0x400UL << OFR2_OFFSET2_Pos );
  static constexpr Reg32_t OFR2_OFFSET2_11  = ( 0x800UL << OFR2_OFFSET2_Pos );

  static constexpr Reg32_t OFR2_OFFSET2_CH_Pos = ( 26U );
  static constexpr Reg32_t OFR2_OFFSET2_CH_Msk = ( 0x1FUL << OFR2_OFFSET2_CH_Pos );
  static constexpr Reg32_t OFR2_OFFSET2_CH     = OFR2_OFFSET2_CH_Msk;
  static constexpr Reg32_t OFR2_OFFSET2_CH_0   = ( 0x01UL << OFR2_OFFSET2_CH_Pos );
  static constexpr Reg32_t OFR2_OFFSET2_CH_1   = ( 0x02UL << OFR2_OFFSET2_CH_Pos );
  static constexpr Reg32_t OFR2_OFFSET2_CH_2   = ( 0x04UL << OFR2_OFFSET2_CH_Pos );
  static constexpr Reg32_t OFR2_OFFSET2_CH_3   = ( 0x08UL << OFR2_OFFSET2_CH_Pos );
  static constexpr Reg32_t OFR2_OFFSET2_CH_4   = ( 0x10UL << OFR2_OFFSET2_CH_Pos );

  static constexpr Reg32_t OFR2_OFFSET2_EN_Pos = ( 31U );
  static constexpr Reg32_t OFR2_OFFSET2_EN_Msk = ( 0x1UL << OFR2_OFFSET2_EN_Pos );
  static constexpr Reg32_t OFR2_OFFSET2_EN     = OFR2_OFFSET2_EN_Msk;

  /********************  Bit definition for OFR3 register  ******************/
  static constexpr Reg32_t OFR3_OFFSET3_Pos = ( 0U );
  static constexpr Reg32_t OFR3_OFFSET3_Msk = ( 0xFFFUL << OFR3_OFFSET3_Pos );
  static constexpr Reg32_t OFR3_OFFSET3     = OFR3_OFFSET3_Msk;
  static constexpr Reg32_t OFR3_OFFSET3_0   = ( 0x001UL << OFR3_OFFSET3_Pos );
  static constexpr Reg32_t OFR3_OFFSET3_1   = ( 0x002UL << OFR3_OFFSET3_Pos );
  static constexpr Reg32_t OFR3_OFFSET3_2   = ( 0x004UL << OFR3_OFFSET3_Pos );
  static constexpr Reg32_t OFR3_OFFSET3_3   = ( 0x008UL << OFR3_OFFSET3_Pos );
  static constexpr Reg32_t OFR3_OFFSET3_4   = ( 0x010UL << OFR3_OFFSET3_Pos );
  static constexpr Reg32_t OFR3_OFFSET3_5   = ( 0x020UL << OFR3_OFFSET3_Pos );
  static constexpr Reg32_t OFR3_OFFSET3_6   = ( 0x040UL << OFR3_OFFSET3_Pos );
  static constexpr Reg32_t OFR3_OFFSET3_7   = ( 0x080UL << OFR3_OFFSET3_Pos );
  static constexpr Reg32_t OFR3_OFFSET3_8   = ( 0x100UL << OFR3_OFFSET3_Pos );
  static constexpr Reg32_t OFR3_OFFSET3_9   = ( 0x200UL << OFR3_OFFSET3_Pos );
  static constexpr Reg32_t OFR3_OFFSET3_10  = ( 0x400UL << OFR3_OFFSET3_Pos );
  static constexpr Reg32_t OFR3_OFFSET3_11  = ( 0x800UL << OFR3_OFFSET3_Pos );

  static constexpr Reg32_t OFR3_OFFSET3_CH_Pos = ( 26U );
  static constexpr Reg32_t OFR3_OFFSET3_CH_Msk = ( 0x1FUL << OFR3_OFFSET3_CH_Pos );
  static constexpr Reg32_t OFR3_OFFSET3_CH     = OFR3_OFFSET3_CH_Msk;
  static constexpr Reg32_t OFR3_OFFSET3_CH_0   = ( 0x01UL << OFR3_OFFSET3_CH_Pos );
  static constexpr Reg32_t OFR3_OFFSET3_CH_1   = ( 0x02UL << OFR3_OFFSET3_CH_Pos );
  static constexpr Reg32_t OFR3_OFFSET3_CH_2   = ( 0x04UL << OFR3_OFFSET3_CH_Pos );
  static constexpr Reg32_t OFR3_OFFSET3_CH_3   = ( 0x08UL << OFR3_OFFSET3_CH_Pos );
  static constexpr Reg32_t OFR3_OFFSET3_CH_4   = ( 0x10UL << OFR3_OFFSET3_CH_Pos );

  static constexpr Reg32_t OFR3_OFFSET3_EN_Pos = ( 31U );
  static constexpr Reg32_t OFR3_OFFSET3_EN_Msk = ( 0x1UL << OFR3_OFFSET3_EN_Pos );
  static constexpr Reg32_t OFR3_OFFSET3_EN     = OFR3_OFFSET3_EN_Msk;

  /********************  Bit definition for OFR4 register  ******************/
  static constexpr Reg32_t OFR4_OFFSET4_Pos = ( 0U );
  static constexpr Reg32_t OFR4_OFFSET4_Msk = ( 0xFFFUL << OFR4_OFFSET4_Pos );
  static constexpr Reg32_t OFR4_OFFSET4     = OFR4_OFFSET4_Msk;
  static constexpr Reg32_t OFR4_OFFSET4_0   = ( 0x001UL << OFR4_OFFSET4_Pos );
  static constexpr Reg32_t OFR4_OFFSET4_1   = ( 0x002UL << OFR4_OFFSET4_Pos );
  static constexpr Reg32_t OFR4_OFFSET4_2   = ( 0x004UL << OFR4_OFFSET4_Pos );
  static constexpr Reg32_t OFR4_OFFSET4_3   = ( 0x008UL << OFR4_OFFSET4_Pos );
  static constexpr Reg32_t OFR4_OFFSET4_4   = ( 0x010UL << OFR4_OFFSET4_Pos );
  static constexpr Reg32_t OFR4_OFFSET4_5   = ( 0x020UL << OFR4_OFFSET4_Pos );
  static constexpr Reg32_t OFR4_OFFSET4_6   = ( 0x040UL << OFR4_OFFSET4_Pos );
  static constexpr Reg32_t OFR4_OFFSET4_7   = ( 0x080UL << OFR4_OFFSET4_Pos );
  static constexpr Reg32_t OFR4_OFFSET4_8   = ( 0x100UL << OFR4_OFFSET4_Pos );
  static constexpr Reg32_t OFR4_OFFSET4_9   = ( 0x200UL << OFR4_OFFSET4_Pos );
  static constexpr Reg32_t OFR4_OFFSET4_10  = ( 0x400UL << OFR4_OFFSET4_Pos );
  static constexpr Reg32_t OFR4_OFFSET4_11  = ( 0x800UL << OFR4_OFFSET4_Pos );

  static constexpr Reg32_t OFR4_OFFSET4_CH_Pos = ( 26U );
  static constexpr Reg32_t OFR4_OFFSET4_CH_Msk = ( 0x1FUL << OFR4_OFFSET4_CH_Pos );
  static constexpr Reg32_t OFR4_OFFSET4_CH     = OFR4_OFFSET4_CH_Msk;
  static constexpr Reg32_t OFR4_OFFSET4_CH_0   = ( 0x01UL << OFR4_OFFSET4_CH_Pos );
  static constexpr Reg32_t OFR4_OFFSET4_CH_1   = ( 0x02UL << OFR4_OFFSET4_CH_Pos );
  static constexpr Reg32_t OFR4_OFFSET4_CH_2   = ( 0x04UL << OFR4_OFFSET4_CH_Pos );
  static constexpr Reg32_t OFR4_OFFSET4_CH_3   = ( 0x08UL << OFR4_OFFSET4_CH_Pos );
  static constexpr Reg32_t OFR4_OFFSET4_CH_4   = ( 0x10UL << OFR4_OFFSET4_CH_Pos );

  static constexpr Reg32_t OFR4_OFFSET4_EN_Pos = ( 31U );
  static constexpr Reg32_t OFR4_OFFSET4_EN_Msk = ( 0x1UL << OFR4_OFFSET4_EN_Pos );
  static constexpr Reg32_t OFR4_OFFSET4_EN     = OFR4_OFFSET4_EN_Msk;

  /********************  Bit definition for JDR1 register  ******************/
  static constexpr Reg32_t JDR1_JDATA_Pos = ( 0U );
  static constexpr Reg32_t JDR1_JDATA_Msk = ( 0xFFFFUL << JDR1_JDATA_Pos );
  static constexpr Reg32_t JDR1_JDATA     = JDR1_JDATA_Msk;
  static constexpr Reg32_t JDR1_JDATA_0   = ( 0x0001UL << JDR1_JDATA_Pos );
  static constexpr Reg32_t JDR1_JDATA_1   = ( 0x0002UL << JDR1_JDATA_Pos );
  static constexpr Reg32_t JDR1_JDATA_2   = ( 0x0004UL << JDR1_JDATA_Pos );
  static constexpr Reg32_t JDR1_JDATA_3   = ( 0x0008UL << JDR1_JDATA_Pos );
  static constexpr Reg32_t JDR1_JDATA_4   = ( 0x0010UL << JDR1_JDATA_Pos );
  static constexpr Reg32_t JDR1_JDATA_5   = ( 0x0020UL << JDR1_JDATA_Pos );
  static constexpr Reg32_t JDR1_JDATA_6   = ( 0x0040UL << JDR1_JDATA_Pos );
  static constexpr Reg32_t JDR1_JDATA_7   = ( 0x0080UL << JDR1_JDATA_Pos );
  static constexpr Reg32_t JDR1_JDATA_8   = ( 0x0100UL << JDR1_JDATA_Pos );
  static constexpr Reg32_t JDR1_JDATA_9   = ( 0x0200UL << JDR1_JDATA_Pos );
  static constexpr Reg32_t JDR1_JDATA_10  = ( 0x0400UL << JDR1_JDATA_Pos );
  static constexpr Reg32_t JDR1_JDATA_11  = ( 0x0800UL << JDR1_JDATA_Pos );
  static constexpr Reg32_t JDR1_JDATA_12  = ( 0x1000UL << JDR1_JDATA_Pos );
  static constexpr Reg32_t JDR1_JDATA_13  = ( 0x2000UL << JDR1_JDATA_Pos );
  static constexpr Reg32_t JDR1_JDATA_14  = ( 0x4000UL << JDR1_JDATA_Pos );
  static constexpr Reg32_t JDR1_JDATA_15  = ( 0x8000UL << JDR1_JDATA_Pos );

  /********************  Bit definition for JDR2 register  ******************/
  static constexpr Reg32_t JDR2_JDATA_Pos = ( 0U );
  static constexpr Reg32_t JDR2_JDATA_Msk = ( 0xFFFFUL << JDR2_JDATA_Pos );
  static constexpr Reg32_t JDR2_JDATA     = JDR2_JDATA_Msk;
  static constexpr Reg32_t JDR2_JDATA_0   = ( 0x0001UL << JDR2_JDATA_Pos );
  static constexpr Reg32_t JDR2_JDATA_1   = ( 0x0002UL << JDR2_JDATA_Pos );
  static constexpr Reg32_t JDR2_JDATA_2   = ( 0x0004UL << JDR2_JDATA_Pos );
  static constexpr Reg32_t JDR2_JDATA_3   = ( 0x0008UL << JDR2_JDATA_Pos );
  static constexpr Reg32_t JDR2_JDATA_4   = ( 0x0010UL << JDR2_JDATA_Pos );
  static constexpr Reg32_t JDR2_JDATA_5   = ( 0x0020UL << JDR2_JDATA_Pos );
  static constexpr Reg32_t JDR2_JDATA_6   = ( 0x0040UL << JDR2_JDATA_Pos );
  static constexpr Reg32_t JDR2_JDATA_7   = ( 0x0080UL << JDR2_JDATA_Pos );
  static constexpr Reg32_t JDR2_JDATA_8   = ( 0x0100UL << JDR2_JDATA_Pos );
  static constexpr Reg32_t JDR2_JDATA_9   = ( 0x0200UL << JDR2_JDATA_Pos );
  static constexpr Reg32_t JDR2_JDATA_10  = ( 0x0400UL << JDR2_JDATA_Pos );
  static constexpr Reg32_t JDR2_JDATA_11  = ( 0x0800UL << JDR2_JDATA_Pos );
  static constexpr Reg32_t JDR2_JDATA_12  = ( 0x1000UL << JDR2_JDATA_Pos );
  static constexpr Reg32_t JDR2_JDATA_13  = ( 0x2000UL << JDR2_JDATA_Pos );
  static constexpr Reg32_t JDR2_JDATA_14  = ( 0x4000UL << JDR2_JDATA_Pos );
  static constexpr Reg32_t JDR2_JDATA_15  = ( 0x8000UL << JDR2_JDATA_Pos );

  /********************  Bit definition for JDR3 register  ******************/
  static constexpr Reg32_t JDR3_JDATA_Pos = ( 0U );
  static constexpr Reg32_t JDR3_JDATA_Msk = ( 0xFFFFUL << JDR3_JDATA_Pos );
  static constexpr Reg32_t JDR3_JDATA     = JDR3_JDATA_Msk;
  static constexpr Reg32_t JDR3_JDATA_0   = ( 0x0001UL << JDR3_JDATA_Pos );
  static constexpr Reg32_t JDR3_JDATA_1   = ( 0x0002UL << JDR3_JDATA_Pos );
  static constexpr Reg32_t JDR3_JDATA_2   = ( 0x0004UL << JDR3_JDATA_Pos );
  static constexpr Reg32_t JDR3_JDATA_3   = ( 0x0008UL << JDR3_JDATA_Pos );
  static constexpr Reg32_t JDR3_JDATA_4   = ( 0x0010UL << JDR3_JDATA_Pos );
  static constexpr Reg32_t JDR3_JDATA_5   = ( 0x0020UL << JDR3_JDATA_Pos );
  static constexpr Reg32_t JDR3_JDATA_6   = ( 0x0040UL << JDR3_JDATA_Pos );
  static constexpr Reg32_t JDR3_JDATA_7   = ( 0x0080UL << JDR3_JDATA_Pos );
  static constexpr Reg32_t JDR3_JDATA_8   = ( 0x0100UL << JDR3_JDATA_Pos );
  static constexpr Reg32_t JDR3_JDATA_9   = ( 0x0200UL << JDR3_JDATA_Pos );
  static constexpr Reg32_t JDR3_JDATA_10  = ( 0x0400UL << JDR3_JDATA_Pos );
  static constexpr Reg32_t JDR3_JDATA_11  = ( 0x0800UL << JDR3_JDATA_Pos );
  static constexpr Reg32_t JDR3_JDATA_12  = ( 0x1000UL << JDR3_JDATA_Pos );
  static constexpr Reg32_t JDR3_JDATA_13  = ( 0x2000UL << JDR3_JDATA_Pos );
  static constexpr Reg32_t JDR3_JDATA_14  = ( 0x4000UL << JDR3_JDATA_Pos );
  static constexpr Reg32_t JDR3_JDATA_15  = ( 0x8000UL << JDR3_JDATA_Pos );

  /********************  Bit definition for JDR4 register  ******************/
  static constexpr Reg32_t JDR4_JDATA_Pos = ( 0U );
  static constexpr Reg32_t JDR4_JDATA_Msk = ( 0xFFFFUL << JDR4_JDATA_Pos );
  static constexpr Reg32_t JDR4_JDATA     = JDR4_JDATA_Msk;
  static constexpr Reg32_t JDR4_JDATA_0   = ( 0x0001UL << JDR4_JDATA_Pos );
  static constexpr Reg32_t JDR4_JDATA_1   = ( 0x0002UL << JDR4_JDATA_Pos );
  static constexpr Reg32_t JDR4_JDATA_2   = ( 0x0004UL << JDR4_JDATA_Pos );
  static constexpr Reg32_t JDR4_JDATA_3   = ( 0x0008UL << JDR4_JDATA_Pos );
  static constexpr Reg32_t JDR4_JDATA_4   = ( 0x0010UL << JDR4_JDATA_Pos );
  static constexpr Reg32_t JDR4_JDATA_5   = ( 0x0020UL << JDR4_JDATA_Pos );
  static constexpr Reg32_t JDR4_JDATA_6   = ( 0x0040UL << JDR4_JDATA_Pos );
  static constexpr Reg32_t JDR4_JDATA_7   = ( 0x0080UL << JDR4_JDATA_Pos );
  static constexpr Reg32_t JDR4_JDATA_8   = ( 0x0100UL << JDR4_JDATA_Pos );
  static constexpr Reg32_t JDR4_JDATA_9   = ( 0x0200UL << JDR4_JDATA_Pos );
  static constexpr Reg32_t JDR4_JDATA_10  = ( 0x0400UL << JDR4_JDATA_Pos );
  static constexpr Reg32_t JDR4_JDATA_11  = ( 0x0800UL << JDR4_JDATA_Pos );
  static constexpr Reg32_t JDR4_JDATA_12  = ( 0x1000UL << JDR4_JDATA_Pos );
  static constexpr Reg32_t JDR4_JDATA_13  = ( 0x2000UL << JDR4_JDATA_Pos );
  static constexpr Reg32_t JDR4_JDATA_14  = ( 0x4000UL << JDR4_JDATA_Pos );
  static constexpr Reg32_t JDR4_JDATA_15  = ( 0x8000UL << JDR4_JDATA_Pos );

  /********************  Bit definition for AWD2CR register  ****************/
  static constexpr Reg32_t AWD2CR_AWD2CH_Pos = ( 0U );
  static constexpr Reg32_t AWD2CR_AWD2CH_Msk = ( 0x7FFFFUL << AWD2CR_AWD2CH_Pos );
  static constexpr Reg32_t AWD2CR_AWD2CH     = AWD2CR_AWD2CH_Msk;
  static constexpr Reg32_t AWD2CR_AWD2CH_0   = ( 0x00001UL << AWD2CR_AWD2CH_Pos );
  static constexpr Reg32_t AWD2CR_AWD2CH_1   = ( 0x00002UL << AWD2CR_AWD2CH_Pos );
  static constexpr Reg32_t AWD2CR_AWD2CH_2   = ( 0x00004UL << AWD2CR_AWD2CH_Pos );
  static constexpr Reg32_t AWD2CR_AWD2CH_3   = ( 0x00008UL << AWD2CR_AWD2CH_Pos );
  static constexpr Reg32_t AWD2CR_AWD2CH_4   = ( 0x00010UL << AWD2CR_AWD2CH_Pos );
  static constexpr Reg32_t AWD2CR_AWD2CH_5   = ( 0x00020UL << AWD2CR_AWD2CH_Pos );
  static constexpr Reg32_t AWD2CR_AWD2CH_6   = ( 0x00040UL << AWD2CR_AWD2CH_Pos );
  static constexpr Reg32_t AWD2CR_AWD2CH_7   = ( 0x00080UL << AWD2CR_AWD2CH_Pos );
  static constexpr Reg32_t AWD2CR_AWD2CH_8   = ( 0x00100UL << AWD2CR_AWD2CH_Pos );
  static constexpr Reg32_t AWD2CR_AWD2CH_9   = ( 0x00200UL << AWD2CR_AWD2CH_Pos );
  static constexpr Reg32_t AWD2CR_AWD2CH_10  = ( 0x00400UL << AWD2CR_AWD2CH_Pos );
  static constexpr Reg32_t AWD2CR_AWD2CH_11  = ( 0x00800UL << AWD2CR_AWD2CH_Pos );
  static constexpr Reg32_t AWD2CR_AWD2CH_12  = ( 0x01000UL << AWD2CR_AWD2CH_Pos );
  static constexpr Reg32_t AWD2CR_AWD2CH_13  = ( 0x02000UL << AWD2CR_AWD2CH_Pos );
  static constexpr Reg32_t AWD2CR_AWD2CH_14  = ( 0x04000UL << AWD2CR_AWD2CH_Pos );
  static constexpr Reg32_t AWD2CR_AWD2CH_15  = ( 0x08000UL << AWD2CR_AWD2CH_Pos );
  static constexpr Reg32_t AWD2CR_AWD2CH_16  = ( 0x10000UL << AWD2CR_AWD2CH_Pos );
  static constexpr Reg32_t AWD2CR_AWD2CH_17  = ( 0x20000UL << AWD2CR_AWD2CH_Pos );
  static constexpr Reg32_t AWD2CR_AWD2CH_18  = ( 0x40000UL << AWD2CR_AWD2CH_Pos );

  /********************  Bit definition for AWD3CR register  ****************/
  static constexpr Reg32_t AWD3CR_AWD3CH_Pos = ( 0U );
  static constexpr Reg32_t AWD3CR_AWD3CH_Msk = ( 0x7FFFFUL << AWD3CR_AWD3CH_Pos );
  static constexpr Reg32_t AWD3CR_AWD3CH     = AWD3CR_AWD3CH_Msk;
  static constexpr Reg32_t AWD3CR_AWD3CH_0   = ( 0x00001UL << AWD3CR_AWD3CH_Pos );
  static constexpr Reg32_t AWD3CR_AWD3CH_1   = ( 0x00002UL << AWD3CR_AWD3CH_Pos );
  static constexpr Reg32_t AWD3CR_AWD3CH_2   = ( 0x00004UL << AWD3CR_AWD3CH_Pos );
  static constexpr Reg32_t AWD3CR_AWD3CH_3   = ( 0x00008UL << AWD3CR_AWD3CH_Pos );
  static constexpr Reg32_t AWD3CR_AWD3CH_4   = ( 0x00010UL << AWD3CR_AWD3CH_Pos );
  static constexpr Reg32_t AWD3CR_AWD3CH_5   = ( 0x00020UL << AWD3CR_AWD3CH_Pos );
  static constexpr Reg32_t AWD3CR_AWD3CH_6   = ( 0x00040UL << AWD3CR_AWD3CH_Pos );
  static constexpr Reg32_t AWD3CR_AWD3CH_7   = ( 0x00080UL << AWD3CR_AWD3CH_Pos );
  static constexpr Reg32_t AWD3CR_AWD3CH_8   = ( 0x00100UL << AWD3CR_AWD3CH_Pos );
  static constexpr Reg32_t AWD3CR_AWD3CH_9   = ( 0x00200UL << AWD3CR_AWD3CH_Pos );
  static constexpr Reg32_t AWD3CR_AWD3CH_10  = ( 0x00400UL << AWD3CR_AWD3CH_Pos );
  static constexpr Reg32_t AWD3CR_AWD3CH_11  = ( 0x00800UL << AWD3CR_AWD3CH_Pos );
  static constexpr Reg32_t AWD3CR_AWD3CH_12  = ( 0x01000UL << AWD3CR_AWD3CH_Pos );
  static constexpr Reg32_t AWD3CR_AWD3CH_13  = ( 0x02000UL << AWD3CR_AWD3CH_Pos );
  static constexpr Reg32_t AWD3CR_AWD3CH_14  = ( 0x04000UL << AWD3CR_AWD3CH_Pos );
  static constexpr Reg32_t AWD3CR_AWD3CH_15  = ( 0x08000UL << AWD3CR_AWD3CH_Pos );
  static constexpr Reg32_t AWD3CR_AWD3CH_16  = ( 0x10000UL << AWD3CR_AWD3CH_Pos );
  static constexpr Reg32_t AWD3CR_AWD3CH_17  = ( 0x20000UL << AWD3CR_AWD3CH_Pos );
  static constexpr Reg32_t AWD3CR_AWD3CH_18  = ( 0x40000UL << AWD3CR_AWD3CH_Pos );

  /********************  Bit definition for DIFSEL register  ****************/
  static constexpr Reg32_t DIFSEL_DIFSEL_Pos = ( 0U );
  static constexpr Reg32_t DIFSEL_DIFSEL_Msk = ( 0x7FFFFUL << DIFSEL_DIFSEL_Pos );
  static constexpr Reg32_t DIFSEL_DIFSEL     = DIFSEL_DIFSEL_Msk;
  static constexpr Reg32_t DIFSEL_DIFSEL_0   = ( 0x00001UL << DIFSEL_DIFSEL_Pos );
  static constexpr Reg32_t DIFSEL_DIFSEL_1   = ( 0x00002UL << DIFSEL_DIFSEL_Pos );
  static constexpr Reg32_t DIFSEL_DIFSEL_2   = ( 0x00004UL << DIFSEL_DIFSEL_Pos );
  static constexpr Reg32_t DIFSEL_DIFSEL_3   = ( 0x00008UL << DIFSEL_DIFSEL_Pos );
  static constexpr Reg32_t DIFSEL_DIFSEL_4   = ( 0x00010UL << DIFSEL_DIFSEL_Pos );
  static constexpr Reg32_t DIFSEL_DIFSEL_5   = ( 0x00020UL << DIFSEL_DIFSEL_Pos );
  static constexpr Reg32_t DIFSEL_DIFSEL_6   = ( 0x00040UL << DIFSEL_DIFSEL_Pos );
  static constexpr Reg32_t DIFSEL_DIFSEL_7   = ( 0x00080UL << DIFSEL_DIFSEL_Pos );
  static constexpr Reg32_t DIFSEL_DIFSEL_8   = ( 0x00100UL << DIFSEL_DIFSEL_Pos );
  static constexpr Reg32_t DIFSEL_DIFSEL_9   = ( 0x00200UL << DIFSEL_DIFSEL_Pos );
  static constexpr Reg32_t DIFSEL_DIFSEL_10  = ( 0x00400UL << DIFSEL_DIFSEL_Pos );
  static constexpr Reg32_t DIFSEL_DIFSEL_11  = ( 0x00800UL << DIFSEL_DIFSEL_Pos );
  static constexpr Reg32_t DIFSEL_DIFSEL_12  = ( 0x01000UL << DIFSEL_DIFSEL_Pos );
  static constexpr Reg32_t DIFSEL_DIFSEL_13  = ( 0x02000UL << DIFSEL_DIFSEL_Pos );
  static constexpr Reg32_t DIFSEL_DIFSEL_14  = ( 0x04000UL << DIFSEL_DIFSEL_Pos );
  static constexpr Reg32_t DIFSEL_DIFSEL_15  = ( 0x08000UL << DIFSEL_DIFSEL_Pos );
  static constexpr Reg32_t DIFSEL_DIFSEL_16  = ( 0x10000UL << DIFSEL_DIFSEL_Pos );
  static constexpr Reg32_t DIFSEL_DIFSEL_17  = ( 0x20000UL << DIFSEL_DIFSEL_Pos );
  static constexpr Reg32_t DIFSEL_DIFSEL_18  = ( 0x40000UL << DIFSEL_DIFSEL_Pos );

  /********************  Bit definition for CALFACT register  ***************/
  static constexpr Reg32_t CALFACT_CALFACT_S_Pos = ( 0U );
  static constexpr Reg32_t CALFACT_CALFACT_S_Msk = ( 0x7FUL << CALFACT_CALFACT_S_Pos );
  static constexpr Reg32_t CALFACT_CALFACT_S     = CALFACT_CALFACT_S_Msk;
  static constexpr Reg32_t CALFACT_CALFACT_S_0   = ( 0x01UL << CALFACT_CALFACT_S_Pos );
  static constexpr Reg32_t CALFACT_CALFACT_S_1   = ( 0x02UL << CALFACT_CALFACT_S_Pos );
  static constexpr Reg32_t CALFACT_CALFACT_S_2   = ( 0x04UL << CALFACT_CALFACT_S_Pos );
  static constexpr Reg32_t CALFACT_CALFACT_S_3   = ( 0x08UL << CALFACT_CALFACT_S_Pos );
  static constexpr Reg32_t CALFACT_CALFACT_S_4   = ( 0x10UL << CALFACT_CALFACT_S_Pos );
  static constexpr Reg32_t CALFACT_CALFACT_S_5   = ( 0x20UL << CALFACT_CALFACT_S_Pos );
  static constexpr Reg32_t CALFACT_CALFACT_S_6   = ( 0x40UL << CALFACT_CALFACT_S_Pos );

  static constexpr Reg32_t CALFACT_CALFACT_D_Pos = ( 16U );
  static constexpr Reg32_t CALFACT_CALFACT_D_Msk = ( 0x7FUL << CALFACT_CALFACT_D_Pos );
  static constexpr Reg32_t CALFACT_CALFACT_D     = CALFACT_CALFACT_D_Msk;
  static constexpr Reg32_t CALFACT_CALFACT_D_0   = ( 0x01UL << CALFACT_CALFACT_D_Pos );
  static constexpr Reg32_t CALFACT_CALFACT_D_1   = ( 0x02UL << CALFACT_CALFACT_D_Pos );
  static constexpr Reg32_t CALFACT_CALFACT_D_2   = ( 0x04UL << CALFACT_CALFACT_D_Pos );
  static constexpr Reg32_t CALFACT_CALFACT_D_3   = ( 0x08UL << CALFACT_CALFACT_D_Pos );
  static constexpr Reg32_t CALFACT_CALFACT_D_4   = ( 0x10UL << CALFACT_CALFACT_D_Pos );
  static constexpr Reg32_t CALFACT_CALFACT_D_5   = ( 0x20UL << CALFACT_CALFACT_D_Pos );
  static constexpr Reg32_t CALFACT_CALFACT_D_6   = ( 0x40UL << CALFACT_CALFACT_D_Pos );

  /*************************  ADC Common registers  *****************************/
  /********************  Bit definition for CCR register  *******************/
  static constexpr Reg32_t CCR_CKMODE_Pos = ( 16U );
  static constexpr Reg32_t CCR_CKMODE_Msk = ( 0x3UL << CCR_CKMODE_Pos );
  static constexpr Reg32_t CCR_CKMODE     = CCR_CKMODE_Msk;
  static constexpr Reg32_t CCR_CKMODE_0   = ( 0x1UL << CCR_CKMODE_Pos );
  static constexpr Reg32_t CCR_CKMODE_1   = ( 0x2UL << CCR_CKMODE_Pos );

  static constexpr Reg32_t CCR_PRESC_Pos = ( 18U );
  static constexpr Reg32_t CCR_PRESC_Msk = ( 0xFUL << CCR_PRESC_Pos );
  static constexpr Reg32_t CCR_PRESC     = CCR_PRESC_Msk;
  static constexpr Reg32_t CCR_PRESC_0   = ( 0x1UL << CCR_PRESC_Pos );
  static constexpr Reg32_t CCR_PRESC_1   = ( 0x2UL << CCR_PRESC_Pos );
  static constexpr Reg32_t CCR_PRESC_2   = ( 0x4UL << CCR_PRESC_Pos );
  static constexpr Reg32_t CCR_PRESC_3   = ( 0x8UL << CCR_PRESC_Pos );

  static constexpr Reg32_t CCR_VREFEN_Pos = ( 22U );
  static constexpr Reg32_t CCR_VREFEN_Msk = ( 0x1UL << CCR_VREFEN_Pos );
  static constexpr Reg32_t CCR_VREFEN     = CCR_VREFEN_Msk;
  static constexpr Reg32_t CCR_TSEN_Pos   = ( 23U );
  static constexpr Reg32_t CCR_TSEN_Msk   = ( 0x1UL << CCR_TSEN_Pos );
  static constexpr Reg32_t CCR_TSEN       = CCR_TSEN_Msk;
  static constexpr Reg32_t CCR_VBATEN_Pos = ( 24U );
  static constexpr Reg32_t CCR_VBATEN_Msk = ( 0x1UL << CCR_VBATEN_Pos );
  static constexpr Reg32_t CCR_VBATEN     = CCR_VBATEN_Msk;

}    // namespace Thor::LLD::ADC

#endif /* !THOR_HW_ADC_REGISTER_STM32L4XXXX_HPP */
