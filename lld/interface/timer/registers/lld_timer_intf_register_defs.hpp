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
  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
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
  REG_ACCESSOR( RegisterMap, CR1, TIM_CR1_UIFREMAP, UIFREMAP, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, TIM_CR1_CKD, CKD, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, TIM_CR1_ARPE, ARPE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, TIM_CR1_CMS, CMS, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, TIM_CR1_DIR, DIR, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, TIM_CR1_OPM, OPM, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, TIM_CR1_URS, URS, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, TIM_CR1_UDIS, UDIS, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, TIM_CR1_CEN, CEN, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  TIMx_CR2: Control Register 2
  ---------------------------------------------------------------------------*/


  /*---------------------------------------------------------------------------
  TIMx_SMCR: Slave Mode Control Register
  ---------------------------------------------------------------------------*/


  /*---------------------------------------------------------------------------
  TIMx_DIER:  DMA/interrupt enable register
  ---------------------------------------------------------------------------*/


  /*---------------------------------------------------------------------------
  TIMx_SR:    status register
  ---------------------------------------------------------------------------*/


  /*---------------------------------------------------------------------------
  TIMx_EGR:   event generation register
  ---------------------------------------------------------------------------*/


  /*---------------------------------------------------------------------------
  TIMx_CCMR1: capture/compare mode register 1
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CCMR1, TIM_CCMR1_OC2M, OC2M, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR1, TIM_CCMR1_OC2PE, OC2PE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR1, TIM_CCMR1_CC2S, CC2S, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR1, TIM_CCMR1_OC1M, OC1M, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR1, TIM_CCMR1_OC1PE, OC1PE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR1, TIM_CCMR1_CC1S, CC1S, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  TIMx_CCMR2: capture/compare mode register 2
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CCMR2, TIM_CCMR2_OC4M, OC4M, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR2, TIM_CCMR2_OC4PE, OC4PE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR2, TIM_CCMR2_CC4S, CC4S, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR2, TIM_CCMR2_OC3M, OC3M, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR2, TIM_CCMR2_OC3PE, OC3PE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR2, TIM_CCMR2_CC3S, CC3S, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  TIMx_CCER:  capture/compare enable register
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CCER, TIM_CCER_CC1P, CC1P, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCER, TIM_CCER_CC1E, CC1E, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCER, TIM_CCER_CC2P, CC2P, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCER, TIM_CCER_CC2E, CC2E, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCER, TIM_CCER_CC3P, CC3P, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCER, TIM_CCER_CC3E, CC3E, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCER, TIM_CCER_CC4P, CC4P, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCER, TIM_CCER_CC4E, CC4E, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  TIMx_CNT:   counter register
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CNT, TIM_CNT_CNT, CNT, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  TIMx_PSC:   prescaler
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, PSC, TIM_PSC_PSC, PSC, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  TIMx_ARR:   auto-reload register
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, ARR, TIM_ARR_ARR, ARR, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  TIMx_RCR:   repetition counter register
  ---------------------------------------------------------------------------*/


  /*---------------------------------------------------------------------------
  TIMx_CCR1:  capture/compare register 1-4
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CCR1, TIM_CCR1_CCR1, CCR1, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCR2, TIM_CCR2_CCR2, CCR2, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCR3, TIM_CCR3_CCR3, CCR3, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCR4, TIM_CCR4_CCR4, CCR4, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  TIMx_BDTR:  break and dead-time register
  ---------------------------------------------------------------------------*/


  /*---------------------------------------------------------------------------
  TIMx_DCR:   DMA control register
  ---------------------------------------------------------------------------*/


  /*---------------------------------------------------------------------------
  TIMx_DMAR:  DMA address for full transfer
  ---------------------------------------------------------------------------*/


  /*---------------------------------------------------------------------------
  TIMx_OR1:   option register 1
  ---------------------------------------------------------------------------*/


  /*---------------------------------------------------------------------------
  TIMx_CCMR3: capture/compare mode register 3
  ---------------------------------------------------------------------------*/


  /*---------------------------------------------------------------------------
  TIMx_CCR5:  capture/compare register5
  ---------------------------------------------------------------------------*/


  /*---------------------------------------------------------------------------
  TIMx_CCR6:  capture/compare register6
  ---------------------------------------------------------------------------*/


  /*---------------------------------------------------------------------------
  TIMx_OR2:   option register 2
  ---------------------------------------------------------------------------*/


  /*---------------------------------------------------------------------------
  TIMx_OR3:   option register 3
  ---------------------------------------------------------------------------*/

}  // namespace Thor::LLD::TIMER

#endif  /* !THOR_LLD_TIMER_INTF_REGISTERS_HPP */
