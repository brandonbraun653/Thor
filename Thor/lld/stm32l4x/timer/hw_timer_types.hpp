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
#include <array>

/* Driver Includes */
#include <Thor/lld/stm32l4x/timer/hw_timer_prj.hpp>

namespace Thor::LLD::TIMER
{
  /**
   *  Generalized register mapping that applies to Advanced, Basic, and General Purpose timers
   */
  struct RegisterMap
  {
    volatile Reg32_t CR1;   /**< TIM control register 1,              Address offset: 0x00 */
    volatile Reg32_t CR2;   /**< TIM control register 2,              Address offset: 0x04 */
    volatile Reg32_t SMCR;  /**< TIM slave mode control register,     Address offset: 0x08 */
    volatile Reg32_t DIER;  /**< TIM DMA/interrupt enable register,   Address offset: 0x0C */
    volatile Reg32_t SR;    /**< TIM status register,                 Address offset: 0x10 */
    volatile Reg32_t EGR;   /**< TIM event generation register,       Address offset: 0x14 */
    volatile Reg32_t CCMR1; /**< TIM capture/compare mode register 1, Address offset: 0x18 */
    volatile Reg32_t CCMR2; /**< TIM capture/compare mode register 2, Address offset: 0x1C */
    volatile Reg32_t CCER;  /**< TIM capture/compare enable register, Address offset: 0x20 */
    volatile Reg32_t CNT;   /**< TIM counter register,                Address offset: 0x24 */
    volatile Reg32_t PSC;   /**< TIM prescaler,                       Address offset: 0x28 */
    volatile Reg32_t ARR;   /**< TIM auto-reload register,            Address offset: 0x2C */
    volatile Reg32_t RCR;   /**< TIM repetition counter register,     Address offset: 0x30 */
    volatile Reg32_t CCR1;  /**< TIM capture/compare register 1,      Address offset: 0x34 */
    volatile Reg32_t CCR2;  /**< TIM capture/compare register 2,      Address offset: 0x38 */
    volatile Reg32_t CCR3;  /**< TIM capture/compare register 3,      Address offset: 0x3C */
    volatile Reg32_t CCR4;  /**< TIM capture/compare register 4,      Address offset: 0x40 */
    volatile Reg32_t BDTR;  /**< TIM break and dead-time register,    Address offset: 0x44 */
    volatile Reg32_t DCR;   /**< TIM DMA control register,            Address offset: 0x48 */
    volatile Reg32_t DMAR;  /**< TIM DMA address for full transfer,   Address offset: 0x4C */
    volatile Reg32_t OR1;   /**< TIM option register 1,               Address offset: 0x50 */
    volatile Reg32_t CCMR3; /**< TIM capture/compare mode register 3, Address offset: 0x54 */
    volatile Reg32_t CCR5;  /**< TIM capture/compare register5,       Address offset: 0x58 */
    volatile Reg32_t CCR6;  /**< TIM capture/compare register6,       Address offset: 0x5C */
    volatile Reg32_t OR2;   /**< TIM option register 2,               Address offset: 0x60 */
    volatile Reg32_t OR3;   /**< TIM option register 3,               Address offset: 0x64 */
  };

  struct LPRegisterMap
  {
    volatile Reg32_t ISR;  /**< LPTIM Interrupt and Status register, Address offset: 0x00 */
    volatile Reg32_t ICR;  /**< LPTIM Interrupt Clear register,      Address offset: 0x04 */
    volatile Reg32_t IER;  /**< LPTIM Interrupt Enable register,     Address offset: 0x08 */
    volatile Reg32_t CFGR; /**< LPTIM Configuration register,        Address offset: 0x0C */
    volatile Reg32_t CR;   /**< LPTIM Control register,              Address offset: 0x10 */
    volatile Reg32_t CMP;  /**< LPTIM Compare register,              Address offset: 0x14 */
    volatile Reg32_t ARR;  /**< LPTIM Autoreload register,           Address offset: 0x18 */
    volatile Reg32_t CNT;  /**< LPTIM Counter register,              Address offset: 0x1C */
    volatile Reg32_t OR;   /**< LPTIM Option register,               Address offset: 0x20 */
  };

  using PeriphRegisterList = std::array<void *, NUM_TIMER_PERIPHS>;

}    // namespace Thor::LLD::TIMER

#endif /* !THOR_HW_TIMER_TYPES_HPP*/
