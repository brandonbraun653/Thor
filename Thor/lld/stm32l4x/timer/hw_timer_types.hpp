/********************************************************************************
 *  File Name:
 *    hw_timer_types.hpp
 *
 *  Description:
 *    STM32L4 Types for the TIMER Peripheral
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_TIMER_TYPES_HPP
#define THOR_HW_TIMER_TYPES_HPP

/* C++ Includes */
#include <cstdint>

/* Driver Includes */
#include <Thor/lld/common/registers/field_accessor.hpp>
#include <Thor/lld/stm32l4x/timer/hw_timer_prj.hpp>

namespace Thor::LLD::TIMER
{
  /**
   *  Generalized register mapping that applies to Advanced, Basic, and General Purpose timers
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


  namespace Configuration
  {
    static constexpr Reg32_t ENABLED  = 1;
    static constexpr Reg32_t DISABLED = 0;

    namespace Direction
    {
      static constexpr Reg32_t COUNT_UP = 0;
      static constexpr Reg32_t COUNT_DN = TIM_CR1_DIR;
    }

    namespace OC
    {
      namespace Mode
      {
        static constexpr Reg32_t PWM_MODE_1 = 0x06;
        static constexpr Reg32_t PWM_MODE_2 = 0x07;
      }
    }

    namespace CC
    {
      namespace Direction
      {
        static constexpr Reg32_t OUTPUT = 0;
        static constexpr Reg32_t INPUT  = 1;
      }

      namespace Polarity
      {
        static constexpr Reg32_t ACTIVE_HIGH = 0;
        static constexpr Reg32_t ACTIVE_LOW  = 1;
      }
    }
  }

  /*------------------------------------------------
  TIMx_CR1
  ------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CR1, TIM_CR1_ARPE_Msk, ARPE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, TIM_CR1_DIR_Msk, DIR, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, TIM_CR1_CEN_Msk, CEN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, TIM_CR1_OPM_Msk, OPM, BIT_ACCESS_RW );
  
  /*------------------------------------------------
  TIMx_CCMR1
  ------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CCMR1, TIM_CCMR1_OC2M_Msk, OC2M, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR1, TIM_CCMR1_OC2PE_Msk, OC2PE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR1, TIM_CCMR1_CC2S_Msk, CC2S, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR1, TIM_CCMR1_OC1M_Msk, OC1M, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR1, TIM_CCMR1_OC1PE_Msk, OC1PE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR1, TIM_CCMR1_CC1S_Msk, CC1S, BIT_ACCESS_RW );

  /*------------------------------------------------
  TIMx_CCMR2
  ------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CCMR2, TIM_CCMR2_OC4M_Msk, OC4M, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR2, TIM_CCMR2_OC4PE_Msk, OC4PE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR2, TIM_CCMR2_CC4S_Msk, CC4S, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR2, TIM_CCMR2_OC3M_Msk, OC3M, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR2, TIM_CCMR2_OC3PE_Msk, OC3PE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCMR2, TIM_CCMR2_CC3S_Msk, CC3S, BIT_ACCESS_RW );

  
  /*------------------------------------------------
  TIMx_CCER
  ------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CCER, TIM_CCER_CC1P_Msk, CC1P, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCER, TIM_CCER_CC1E_Msk, CC1E, BIT_ACCESS_RW );

  REG_ACCESSOR( RegisterMap, CCER, TIM_CCER_CC2P_Msk, CC2P, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCER, TIM_CCER_CC2E_Msk, CC2E, BIT_ACCESS_RW );
  
  REG_ACCESSOR( RegisterMap, CCER, TIM_CCER_CC3P_Msk, CC3P, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCER, TIM_CCER_CC3E_Msk, CC3E, BIT_ACCESS_RW );

  REG_ACCESSOR( RegisterMap, CCER, TIM_CCER_CC4P_Msk, CC4P, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCER, TIM_CCER_CC4E_Msk, CC4E, BIT_ACCESS_RW );

  /*------------------------------------------------
  TIMx_CNT
  ------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CNT, TIM_CNT_CNT_Msk, CNT, BIT_ACCESS_RW );

  /*------------------------------------------------
  TIMx_PSC
  ------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, PSC, TIM_PSC_PSC_Msk, PSC, BIT_ACCESS_RW );

  /*------------------------------------------------
  TIMx_ARR
  ------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, ARR, TIM_ARR_ARR_Msk, ARR, BIT_ACCESS_RW );

  /*------------------------------------------------
  TIMx_CCR1-4
  ------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CCR1, TIM_CCR1_CCR1_Msk, CCR1, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCR2, TIM_CCR2_CCR2_Msk, CCR2, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCR3, TIM_CCR3_CCR3_Msk, CCR3, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCR4, TIM_CCR4_CCR4_Msk, CCR4, BIT_ACCESS_RW );

}    // namespace Thor::LLD::TIMER

#endif /* !THOR_HW_TIMER_TYPES_HPP*/
