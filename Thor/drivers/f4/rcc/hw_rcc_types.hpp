/********************************************************************************
 *   File Name:
 *    hw_rcc_types.hpp
 *
 *   Description:
 *    Declares types specific to the RCC peripehral
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_RCC_TYPES_HPP
#define THOR_HW_RCC_TYPES_HPP

/* C++ Includes */
#include <cstdint>

/* Driver Includes */
#include <Thor/drivers/f4/rcc/hw_rcc_prj.hpp>

namespace Thor::Driver::RCC
{
  struct RegisterMap
  {
    volatile uint32_t CR;         /**< RCC clock control register,                                  Address offset: 0x00 */
    volatile uint32_t PLLCFGR;    /**< RCC PLL configuration register,                              Address offset: 0x04 */
    volatile uint32_t CFGR;       /**< RCC clock configuration register,                            Address offset: 0x08 */
    volatile uint32_t CIR;        /**< RCC clock interrupt register,                                Address offset: 0x0C */
    volatile uint32_t AHB1RSTR;   /**< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
    volatile uint32_t AHB2RSTR;   /**< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
    volatile uint32_t AHB3RSTR;   /**< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
    uint32_t RESERVED0;           /**< Reserved, 0x1C                                                                    */
    volatile uint32_t APB1RSTR;   /**< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
    volatile uint32_t APB2RSTR;   /**< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
    uint32_t RESERVED1[ 2 ];      /**< Reserved, 0x28-0x2C                                                               */
    volatile uint32_t AHB1ENR;    /**< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
    volatile uint32_t AHB2ENR;    /**< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
    volatile uint32_t AHB3ENR;    /**< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
    uint32_t RESERVED2;           /**< Reserved, 0x3C                                                                    */
    volatile uint32_t APB1ENR;    /**< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
    volatile uint32_t APB2ENR;    /**< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
    uint32_t RESERVED3[ 2 ];      /**< Reserved, 0x48-0x4C                                                               */
    volatile uint32_t AHB1LPENR;  /**< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
    volatile uint32_t AHB2LPENR;  /**< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
    volatile uint32_t AHB3LPENR;  /**< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
    uint32_t RESERVED4;           /**< Reserved, 0x5C                                                                    */
    volatile uint32_t APB1LPENR;  /**< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
    volatile uint32_t APB2LPENR;  /**< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
    uint32_t RESERVED5[ 2 ];      /**< Reserved, 0x68-0x6C                                                               */
    volatile uint32_t BDCR;       /**< RCC Backup domain control register,                          Address offset: 0x70 */
    volatile uint32_t CSR;        /**< RCC clock control & status register,                         Address offset: 0x74 */
    uint32_t RESERVED6[ 2 ];      /**< Reserved, 0x78-0x7C                                                               */
    volatile uint32_t SSCGR;      /**< RCC spread spectrum clock generation register,               Address offset: 0x80 */
    volatile uint32_t PLLI2SCFGR; /**< RCC PLLI2S configuration register,                           Address offset: 0x84 */
    volatile uint32_t PLLSAICFGR; /**< RCC PLLSAI configuration register,                           Address offset: 0x88 */
    volatile uint32_t DCKCFGR;    /**< RCC Dedicated Clocks configuration register,                 Address offset: 0x8C */
    volatile uint32_t CKGATENR;   /**< RCC Clocks Gated ENable Register,                            Address offset: 0x90 */
    volatile uint32_t DCKCFGR2;   /**< RCC Dedicated Clocks configuration register 2,               Address offset: 0x94 */
  };

  static RegisterMap *const RCC_PERIPH = reinterpret_cast<RegisterMap *const>( Thor::Driver::RCC::RCC_BASE_ADDR );

  /**
   *  Configuration struct for the clock enable register
   */
  struct CEConfig
  {
    volatile uint32_t *reg; /**< Clock enable register */
    uint8_t mask;           /**< Bit mask that will enable/disable the peripheral's clock */
  };

  /**
   *  Configuration struct for the clock enable low power register
   */
  struct CELPConfig
  {
    volatile uint32_t *reg; /**< Clock enable low power register */
    uint8_t mask;           /**< Bit mask that will enable/disable the peripheral's low power clock */
  };

  /**
   *  Configuration struct for the peripheral reset register
   */
  struct PRRConfig
  {
    volatile uint32_t *reg; /**< Peripheral Reset Register */
    uint8_t mask;           /**< Bit mask that will reset the peripheral */
  };
}    // namespace Thor::Driver::RCC

#endif /* !THOR_HW_RCC_TYPES_HPP */