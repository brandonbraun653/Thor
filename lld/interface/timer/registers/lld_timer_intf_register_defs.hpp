/******************************************************************************
 *  File Name:
 *    lld_timer_intf_register_defs.hpp
 *
 *  Description:
 *    Register descriptors for all supported Timers
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_TIMER_INTF_REGISTERS_HPP
#define THOR_LLD_TIMER_INTF_REGISTERS_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>
#include <Thor/lld/common/registers/field_accessor.hpp>
#include <Thor/lld/interface/timer/registers/lld_timer_intf_register_bits.hpp>

namespace Thor::LLD::TIMER
{
  /**
   * @brief Generalized register mapping that applies to Advanced, Basic, and General Purpose timers
   */
  struct RegisterMap
  {
    volatile uint32_t CR1;   /**< TIM control register 1,              Address offset: 0x00 */
    volatile uint32_t CR2;   /**< TIM control register 2,              Address offset: 0x04 */
    volatile uint32_t SMCR;  /**< TIM slave mode control register,     Address offset: 0x08 */
    volatile uint32_t DIER;  /**< TIM DMA/interrupt enable register,   Address offset: 0x0C */
    volatile uint32_t SR;    /**< TIM status register,                 Address offset: 0x10 */
    volatile uint32_t EGR;   /**< TIM event generation register,       Address offset: 0x14 */
    volatile uint32_t CCMR1; /**< TIM capture/compare mode register 1, Address offset: 0x18 */
    volatile uint32_t CCMR2; /**< TIM capture/compare mode register 2, Address offset: 0x1C */
    volatile uint32_t CCER;  /**< TIM capture/compare enable register, Address offset: 0x20 */
    volatile uint32_t CNT;   /**< TIM counter register,                Address offset: 0x24 */
    volatile uint32_t PSC;   /**< TIM prescaler,                       Address offset: 0x28 */
    volatile uint32_t ARR;   /**< TIM auto-reload register,            Address offset: 0x2C */
    volatile uint32_t RCR;   /**< TIM repetition counter register,     Address offset: 0x30 */
    volatile uint32_t CCR1;  /**< TIM capture/compare register 1,      Address offset: 0x34 */
    volatile uint32_t CCR2;  /**< TIM capture/compare register 2,      Address offset: 0x38 */
    volatile uint32_t CCR3;  /**< TIM capture/compare register 3,      Address offset: 0x3C */
    volatile uint32_t CCR4;  /**< TIM capture/compare register 4,      Address offset: 0x40 */
    volatile uint32_t BDTR;  /**< TIM break and dead-time register,    Address offset: 0x44 */
    volatile uint32_t DCR;   /**< TIM DMA control register,            Address offset: 0x48 */
    volatile uint32_t DMAR;  /**< TIM DMA address for full transfer,   Address offset: 0x4C */
    volatile uint32_t OR1;   /**< TIM option register 1,               Address offset: 0x50 */
    volatile uint32_t CCMR3; /**< TIM capture/compare mode register 3, Address offset: 0x54 */
    volatile uint32_t CCR5;  /**< TIM capture/compare register5,       Address offset: 0x58 */
    volatile uint32_t CCR6;  /**< TIM capture/compare register6,       Address offset: 0x5C */
    volatile uint32_t OR2;   /**< TIM option register 2,               Address offset: 0x60 */
    volatile uint32_t OR3;   /**< TIM option register 3,               Address offset: 0x64 */
  };

  /**
   * @brief Register mapping for Low Power timers
   */
  struct LPRegisterMap
  {
    volatile uint32_t ISR;  /**< LPTIM Interrupt and Status register, Address offset: 0x00 */
    volatile uint32_t ICR;  /**< LPTIM Interrupt Clear register,      Address offset: 0x04 */
    volatile uint32_t IER;  /**< LPTIM Interrupt Enable register,     Address offset: 0x08 */
    volatile uint32_t CFGR; /**< LPTIM Configuration register,        Address offset: 0x0C */
    volatile uint32_t CR;   /**< LPTIM Control register,              Address offset: 0x10 */
    volatile uint32_t CMP;  /**< LPTIM Compare register,              Address offset: 0x14 */
    volatile uint32_t ARR;  /**< LPTIM Autoreload register,           Address offset: 0x18 */
    volatile uint32_t CNT;  /**< LPTIM Counter register,              Address offset: 0x1C */
    volatile uint32_t OR;   /**< LPTIM Option register,               Address offset: 0x20 */
  };

  /*---------------------------------------------------------------------------
  TIMx_CR1: Control Register 1
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CR1, CR1_ARPE, ARPE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_CEN, CEN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_CKD, CKD, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_CMS, CMS, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_DIR, DIR, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_OPM, OPM, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_UDIS, UDIS, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_UIFREMAP, UIFREMAP, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_URS, URS, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  TIMx_CR2: Control Register 2
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CR2, CR2_ALL, CR2, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_CCDS, CCDS, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_CCPC, CCPC, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_CCUS, CCUS, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_MMS, MMS, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_MMS2, MMS2, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_OIS1, OIS1, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_OIS1N, OIS1N, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_OIS2, OIS2, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_OIS2N, OIS2N, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_OIS3, OIS3, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_OIS3N, OIS3N, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_OIS4, OIS4, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_OIS5, OIS5, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_OIS6, OIS6, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_TI1S, TI1S, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  TIMx_SMCR: Slave Mode Control Register
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, SMCR, SMCR_ECE, ECE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, SMCR, SMCR_ETF, ETF, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, SMCR, SMCR_ETP, ETP, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, SMCR, SMCR_ETPS, ETPS, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, SMCR, SMCR_MSM, MSM, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, SMCR, SMCR_OCCS, OCCS, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, SMCR, SMCR_SMS, SMS, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, SMCR, SMCR_TS, TS, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  TIMx_DIER:  DMA/interrupt enable register
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, DIER, DIER_BIE, BIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, DIER, DIER_CC1DE, CC1DE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, DIER, DIER_CC1IE, CC1IE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, DIER, DIER_CC2DE, CC2DE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, DIER, DIER_CC2IE, CC2IE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, DIER, DIER_CC3DE, CC3DE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, DIER, DIER_CC3IE, CC3IE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, DIER, DIER_CC4DE, CC4DE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, DIER, DIER_CC4IE, CC4IE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, DIER, DIER_COMDE, COMDE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, DIER, DIER_COMIE, COMIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, DIER, DIER_TDE, TDE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, DIER, DIER_TIE, TIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, DIER, DIER_UDE, UDE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, DIER, DIER_UIE, UIE, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  TIMx_SR:    status register
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, SR, SR_B2IF, B2IF, BIT_ACCESS_RCW0 );
  REG_ACCESSOR( RegisterMap, SR, SR_BIF, BIF, BIT_ACCESS_RCW0 );
  REG_ACCESSOR( RegisterMap, SR, SR_CC1IF, CC1IF, BIT_ACCESS_RCW0 );
  REG_ACCESSOR( RegisterMap, SR, SR_CC1OF, CC1OF, BIT_ACCESS_RCW0 );
  REG_ACCESSOR( RegisterMap, SR, SR_CC2IF, CC2IF, BIT_ACCESS_RCW0 );
  REG_ACCESSOR( RegisterMap, SR, SR_CC2OF, CC2OF, BIT_ACCESS_RCW0 );
  REG_ACCESSOR( RegisterMap, SR, SR_CC3IF, CC3IF, BIT_ACCESS_RCW0 );
  REG_ACCESSOR( RegisterMap, SR, SR_CC3OF, CC3OF, BIT_ACCESS_RCW0 );
  REG_ACCESSOR( RegisterMap, SR, SR_CC4IF, CC4IF, BIT_ACCESS_RCW0 );
  REG_ACCESSOR( RegisterMap, SR, SR_CC4OF, CC4OF, BIT_ACCESS_RCW0 );
  REG_ACCESSOR( RegisterMap, SR, SR_CC5IF, CC5IF, BIT_ACCESS_RCW0 );
  REG_ACCESSOR( RegisterMap, SR, SR_CC6IF, CC6IF, BIT_ACCESS_RCW0 );
  REG_ACCESSOR( RegisterMap, SR, SR_COMIF, COMIF, BIT_ACCESS_RCW0 );
  REG_ACCESSOR( RegisterMap, SR, SR_SBIF, SBIF, BIT_ACCESS_RCW0 );
  REG_ACCESSOR( RegisterMap, SR, SR_TIF, TIF, BIT_ACCESS_RCW0 );
  REG_ACCESSOR( RegisterMap, SR, SR_UIF, UIF, BIT_ACCESS_RCW0 );

  /*---------------------------------------------------------------------------
  TIMx_EGR:   event generation register
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, EGR, EGR_B2G, B2G, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, EGR, EGR_BG, BG, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, EGR, EGR_CC1G, CC1G, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, EGR, EGR_CC2G, CC2G, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, EGR, EGR_CC3G, CC3G, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, EGR, EGR_CC4G, CC4G, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, EGR, EGR_COMG, COMG, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, EGR, EGR_TG, TG, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, EGR, EGR_UG, UG, BIT_ACCESS_W );

  /*---------------------------------------------------------------------------
  TIMx_CCMR1: capture/compare mode register 1
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CCMR1, CCMR1_CC1S, CC1S, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR1, CCMR1_CC2S, CC2S, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR1, CCMR1_IC1F, IC1F, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR1, CCMR1_IC1PSC, IC1PSC, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR1, CCMR1_IC2F, IC2F, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR1, CCMR1_IC2PSC, IC2PSC, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR1, CCMR1_OC1CE, OC1CE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR1, CCMR1_OC1FE, OC1FE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR1, CCMR1_OC1M, OC1M, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR1, CCMR1_OC1PE, OC1PE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR1, CCMR1_OC2CE, OC2CE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR1, CCMR1_OC2FE, OC2FE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR1, CCMR1_OC2M, OC2M, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR1, CCMR1_OC2PE, OC2PE, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  TIMx_CCMR2: capture/compare mode register 2
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CCMR2, CCMR2_CC3S, CC3S, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR2, CCMR2_CC4S, CC4S, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR2, CCMR2_IC3F, IC3F, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR2, CCMR2_IC3PSC, IC3PSC, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR2, CCMR2_IC4F, IC4F, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR2, CCMR2_IC4PSC, IC4PSC, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR2, CCMR2_OC3CE, OC3CE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR2, CCMR2_OC3FE, OC3FE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR2, CCMR2_OC3M, OC3M, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR2, CCMR2_OC3PE, OC3PE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR2, CCMR2_OC4CE, OC4CE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR2, CCMR2_OC4FE, OC4FE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR2, CCMR2_OC4M, OC4M, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR2, CCMR2_OC4PE, OC4PE, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  TIMx_CCMR3: capture/compare mode register 3
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CCMR3, CCMR3_OC5CE, OC5CE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR3, CCMR3_OC5FE, OC5FE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR3, CCMR3_OC5M, OC5M, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR3, CCMR3_OC5PE, OC5PE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR3, CCMR3_OC6CE, OC6CE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR3, CCMR3_OC6FE, OC6FE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR3, CCMR3_OC6M, OC6M, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR3, CCMR3_OC6PE, OC6PE, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  TIMx_CCER:  capture/compare enable register
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CCER, CCER_ALL, CCER, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCER, CCER_CC1E, CC1E, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCER, CCER_CC1P, CC1P, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCER, CCER_CC1NE, CC1NE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCER, CCER_CC1NP, CC1NP, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCER, CCER_CC2E, CC2E, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCER, CCER_CC2P, CC2P, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCER, CCER_CC2NE, CC2NE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCER, CCER_CC2NP, CC2NP, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCER, CCER_CC3E, CC3E, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCER, CCER_CC3P, CC3P, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCER, CCER_CC3NE, CC3NE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCER, CCER_CC3NP, CC3NP, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCER, CCER_CC4E, CC4E, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCER, CCER_CC4P, CC4P, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCER, CCER_CC4NP, CC4NP, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCER, CCER_CC5E, CC5E, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCER, CCER_CC5P, CC5P, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCER, CCER_CC6E, CC6E, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCER, CCER_CC6P, CC6P, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  TIMx_CNT:   counter register
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CNT, CNT_CNT, COUNT, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CNT, CNT_UIFCPY, UIFCPY, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  TIMx_PSC:   prescaler
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, PSC, PSC_PSC, PRESCALE, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  TIMx_ARR:   auto-reload register
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, ARR, ARR_ARR, AUTO_RELOAD, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  TIMx_RCR:   repetition counter register
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, RCR, RCR_REP, REP, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  TIMx_CCR1:  capture/compare register 1-6
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CCR1, CCR1_CCR1, CC1, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCR2, CCR2_CCR2, CC2, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCR3, CCR3_CCR3, CC3, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCR4, CCR4_CCR4, CC4, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCR5, CCR5_CCR5, CC5, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCR5, CCR5_GC5C1, GC5C1, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCR5, CCR5_GC5C2, GC5C2, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCR5, CCR5_GC5C3, GC5C3, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCR6, CCR6_CCR6, CC6, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  TIMx_BDTR:  break and dead-time register
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, BDTR, BDTR_DTG, DTG, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, BDTR, BDTR_LOCK, LOCK, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, BDTR, BDTR_OSSI, OSSI, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, BDTR, BDTR_OSSR, OSSR, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, BDTR, BDTR_BKE, BKE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, BDTR, BDTR_BKP, BKP, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, BDTR, BDTR_AOE, AOE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, BDTR, BDTR_MOE, MOE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, BDTR, BDTR_BKF, BKF, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, BDTR, BDTR_BK2F, BK2F, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, BDTR, BDTR_BK2E, BK2E, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, BDTR, BDTR_BK2P, BK2P, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  TIMx_DCR:   DMA control register
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, DCR, DCR_DBA, DBA, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, DCR, DCR_DBL, DBL, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  TIMx_DMAR:  DMA address for full transfer
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, DMAR, DMAR_DMAB, DMAB, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  TIM1_OR1
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, OR1, TIM1_OR1_ETR_ADC1_RMP, TIM1_ETR_ADC1_RMP, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, OR1, TIM1_OR1_TI1_RMP, TIM1_TI1_RMP, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  TIM1_OR2
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, OR2, TIM1_OR2_BKINE, TIM1_BKINE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, OR2, TIM1_OR2_BKCMP1E, TIM1_BKCMP1E, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, OR2, TIM1_OR2_BKCMP2E, TIM1_BKCMP2E, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, OR2, TIM1_OR2_BKINP, TIM1_BKINP, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, OR2, TIM1_OR2_BKCMP1P, TIM1_BKCMP1P, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, OR2, TIM1_OR2_BKCMP2P, TIM1_BKCMP2P, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, OR2, TIM1_OR2_ETRSEL, TIM1_ETRSEL, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  TIM1_OR3
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, OR3, TIM1_OR3_BK2INE, TIM1_BK2INE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, OR3, TIM1_OR3_BK2CMP1E, TIM1_BK2CMP1E, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, OR3, TIM1_OR3_BK2CMP2E, TIM1_BK2CMP2E, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, OR3, TIM1_OR3_BK2INP, TIM1_BK2INP, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, OR3, TIM1_OR3_BK2CMP1P, TIM1_BK2CMP1P, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, OR3, TIM1_OR3_BK2CMP2P, TIM1_BK2CMP2P, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  TIM2_OR1
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, OR1, TIM2_OR1_ITR1_RMP, TIM2_ITR1_RMP, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, OR1, TIM2_OR1_ETR1_RMP, TIM2_ETR1_RMP, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, OR1, TIM2_OR1_TI4_RMP, TIM2_TI4_RMP, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  TIM2_OR2
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, OR2, TIM2_OR2_ETRSEL, TIM2_ETRSEL, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  TIM15_OR1
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, OR1, TIM15_OR1_TI1_RMP, TIM15_TI1_RMP, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, OR1, TIM15_OR1_ENCODER_MODE, TIM15_ENCODER_MODE, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  TIM15_OR2
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, OR2, TIM15_OR2_BKINE, TIM15_BKINE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, OR2, TIM15_OR2_BKCMP1E, TIM15_BKCMP1E, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, OR2, TIM15_OR2_BKCMP2E, TIM15_BKCMP2E, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, OR2, TIM15_OR2_BKINP, TIM15_BKINP, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, OR2, TIM15_OR2_BKCMP1P, TIM15_BKCMP1P, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, OR2, TIM15_OR2_BKCMP2P, TIM15_BKCMP2P, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  TIM16_OR1
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, OR1, TIM16_OR1_TI1_RMP, TIM16_TI1_RMP, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  TIM16_OR2
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, OR2, TIM16_OR2_BKINE, TIM16_BKINE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, OR2, TIM16_OR2_BKCMP1E, TIM16_BKCMP1E, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, OR2, TIM16_OR2_BKCMP2E, TIM16_BKCMP2E, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, OR2, TIM16_OR2_BKINP, TIM16_BKINP, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, OR2, TIM16_OR2_BKCMP1P, TIM16_BKCMP1P, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, OR2, TIM16_OR2_BKCMP2P, TIM16_BKCMP2P, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  LPISR
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( LPRegisterMap, ISR, LPISR_CMPM, CMPM, BIT_ACCESS_RW );
  REG_ACCESSOR( LPRegisterMap, ISR, LPISR_ARRM, ARRM, BIT_ACCESS_RW );
  REG_ACCESSOR( LPRegisterMap, ISR, LPISR_EXTTRIG, EXTTRIG, BIT_ACCESS_RW );
  REG_ACCESSOR( LPRegisterMap, ISR, LPISR_CMPOK, CMPOK, BIT_ACCESS_RW );
  REG_ACCESSOR( LPRegisterMap, ISR, LPISR_ARROK, ARROK, BIT_ACCESS_RW );
  REG_ACCESSOR( LPRegisterMap, ISR, LPISR_UP, UP, BIT_ACCESS_RW );
  REG_ACCESSOR( LPRegisterMap, ISR, LPISR_DOWN, DOWN, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  LPICR
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( LPRegisterMap, ICR, LPICR_CMPMCF, CMPMCF, BIT_ACCESS_RW );
  REG_ACCESSOR( LPRegisterMap, ICR, LPICR_ARRMCF, ARRMCF, BIT_ACCESS_RW );
  REG_ACCESSOR( LPRegisterMap, ICR, LPICR_EXTTRIGCF, EXTTRIGCF, BIT_ACCESS_RW );
  REG_ACCESSOR( LPRegisterMap, ICR, LPICR_CMPOKCF, CMPOKCF, BIT_ACCESS_RW );
  REG_ACCESSOR( LPRegisterMap, ICR, LPICR_ARROKCF, ARROKCF, BIT_ACCESS_RW );
  REG_ACCESSOR( LPRegisterMap, ICR, LPICR_UPCF, UPCF, BIT_ACCESS_RW );
  REG_ACCESSOR( LPRegisterMap, ICR, LPICR_DOWNCF, DOWNCF, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  LPIER
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( LPRegisterMap, IER, LPIER_CMPMIE, CMPMIE, BIT_ACCESS_RW );
  REG_ACCESSOR( LPRegisterMap, IER, LPIER_ARRMIE, ARRMIE, BIT_ACCESS_RW );
  REG_ACCESSOR( LPRegisterMap, IER, LPIER_EXTTRIGIE, EXTTRIGIE, BIT_ACCESS_RW );
  REG_ACCESSOR( LPRegisterMap, IER, LPIER_CMPOKIE, CMPOKIE, BIT_ACCESS_RW );
  REG_ACCESSOR( LPRegisterMap, IER, LPIER_ARROKIE, ARROKIE, BIT_ACCESS_RW );
  REG_ACCESSOR( LPRegisterMap, IER, LPIER_UPIE, UPIE, BIT_ACCESS_RW );
  REG_ACCESSOR( LPRegisterMap, IER, LPIER_DOWNIE, DOWNIE, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  LPCFGR
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( LPRegisterMap, CFGR, LPCFGR_CKSEL, CKSEL, BIT_ACCESS_RW );
  REG_ACCESSOR( LPRegisterMap, CFGR, LPCFGR_CKPOL, CKPOL, BIT_ACCESS_RW );
  REG_ACCESSOR( LPRegisterMap, CFGR, LPCFGR_CKFLT, CKFLT, BIT_ACCESS_RW );
  REG_ACCESSOR( LPRegisterMap, CFGR, LPCFGR_TRGFLT, TRGFLT, BIT_ACCESS_RW );
  REG_ACCESSOR( LPRegisterMap, CFGR, LPCFGR_PRESC, PRESC, BIT_ACCESS_RW );
  REG_ACCESSOR( LPRegisterMap, CFGR, LPCFGR_TRIGSEL, TRIGSEL, BIT_ACCESS_RW );
  REG_ACCESSOR( LPRegisterMap, CFGR, LPCFGR_TRIGEN, TRIGEN, BIT_ACCESS_RW );
  REG_ACCESSOR( LPRegisterMap, CFGR, LPCFGR_TIMOUT, TIMOUT, BIT_ACCESS_RW );
  REG_ACCESSOR( LPRegisterMap, CFGR, LPCFGR_WAVE, WAVE, BIT_ACCESS_RW );
  REG_ACCESSOR( LPRegisterMap, CFGR, LPCFGR_WAVPOL, WAVPOL, BIT_ACCESS_RW );
  REG_ACCESSOR( LPRegisterMap, CFGR, LPCFGR_PRELOAD, PRELOAD, BIT_ACCESS_RW );
  REG_ACCESSOR( LPRegisterMap, CFGR, LPCFGR_COUNTMODE, COUNTMODE, BIT_ACCESS_RW );
  REG_ACCESSOR( LPRegisterMap, CFGR, LPCFGR_ENC, ENC, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  LPCR
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( LPRegisterMap, CR, LPCR_ENABLE, ENABLE, BIT_ACCESS_RW );
  REG_ACCESSOR( LPRegisterMap, CR, LPCR_SNGSTRT, SNGSTRT, BIT_ACCESS_RW );
  REG_ACCESSOR( LPRegisterMap, CR, LPCR_CNTSTRT, CNTSTRT, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  LPCMP
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( LPRegisterMap, CMP, LPCMP_CMP, CMP, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  LPARR
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( LPRegisterMap, ARR, LPARR_ARR, ARR, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  LPCNT
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( LPRegisterMap, CNT, LPCNT_CNT, CNT, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  LPOR
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( LPRegisterMap, OR, LPOR_OR, OR, BIT_ACCESS_RW );

}    // namespace Thor::LLD::TIMER

#endif /* !THOR_LLD_TIMER_INTF_REGISTERS_HPP */
