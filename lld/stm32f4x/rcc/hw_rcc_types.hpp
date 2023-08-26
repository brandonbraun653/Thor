/******************************************************************************
 *  File Name:
 *    hw_rcc_types.hpp
 *
 *  Description:
 *    Declares types specific to the RCC peripehral
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_HW_RCC_TYPES_HPP
#define THOR_HW_RCC_TYPES_HPP

/* C++ Includes */
#include <cstdint>

/* Chimera/Includes */
#include <Chimera/common>

/* Driver Includes */
#include <Thor/lld/common/registers/field_accessor.hpp>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/rcc/rcc_types.hpp>
#include <Thor/lld/stm32f4x/rcc/hw_rcc_prj.hpp>

namespace Thor::LLD::RCC
{
  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/
  enum ResetFlags : Reg32_t
  {
    CLEARED     = 0,
    LOW_POWER   = CSR_LPWRRSTF,
    WWDG        = CSR_WWDGRSTF,
    IWDG        = CSR_IWDGRSTF,
    SOFTWARE    = CSR_SFTRSTF,
    BROWN_OUT   = CSR_BORRSTF,
    PIN_RESET   = CSR_PINRSTF
  };

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct RegisterMap
  {
    volatile Reg32_t CR;         /**< RCC clock control register,                                  Address offset: 0x00 */
    volatile Reg32_t PLLCFGR;    /**< RCC PLL configuration register,                              Address offset: 0x04 */
    volatile Reg32_t CFGR;       /**< RCC clock configuration register,                            Address offset: 0x08 */
    volatile Reg32_t CIR;        /**< RCC clock interrupt register,                                Address offset: 0x0C */
    volatile Reg32_t AHB1RSTR;   /**< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
    volatile Reg32_t AHB2RSTR;   /**< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
    volatile Reg32_t AHB3RSTR;   /**< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
    Reg32_t RESERVED0;           /**< Reserved, 0x1C                                                                    */
    volatile Reg32_t APB1RSTR;   /**< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
    volatile Reg32_t APB2RSTR;   /**< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
    Reg32_t RESERVED1[ 2 ];      /**< Reserved, 0x28-0x2C                                                               */
    volatile Reg32_t AHB1ENR;    /**< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
    volatile Reg32_t AHB2ENR;    /**< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
    volatile Reg32_t AHB3ENR;    /**< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
    Reg32_t RESERVED2;           /**< Reserved, 0x3C                                                                    */
    volatile Reg32_t APB1ENR;    /**< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
    volatile Reg32_t APB2ENR;    /**< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
    Reg32_t RESERVED3[ 2 ];      /**< Reserved, 0x48-0x4C                                                               */
    volatile Reg32_t AHB1LPENR;  /**< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
    volatile Reg32_t AHB2LPENR;  /**< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
    volatile Reg32_t AHB3LPENR;  /**< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
    Reg32_t RESERVED4;           /**< Reserved, 0x5C                                                                    */
    volatile Reg32_t APB1LPENR;  /**< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
    volatile Reg32_t APB2LPENR;  /**< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
    Reg32_t RESERVED5[ 2 ];      /**< Reserved, 0x68-0x6C                                                               */
    volatile Reg32_t BDCR;       /**< RCC Backup domain control register,                          Address offset: 0x70 */
    volatile Reg32_t CSR;        /**< RCC clock control & status register,                         Address offset: 0x74 */
    Reg32_t RESERVED6[ 2 ];      /**< Reserved, 0x78-0x7C                                                               */
    volatile Reg32_t SSCGR;      /**< RCC spread spectrum clock generation register,               Address offset: 0x80 */
    volatile Reg32_t PLLI2SCFGR; /**< RCC PLLI2S configuration register,                           Address offset: 0x84 */
    volatile Reg32_t PLLSAICFGR; /**< RCC PLLSAI configuration register,                           Address offset: 0x88 */
    volatile Reg32_t DCKCFGR;    /**< RCC Dedicated Clocks configuration register,                 Address offset: 0x8C */
    volatile Reg32_t CKGATENR;   /**< RCC Clocks Gated ENable Register,                            Address offset: 0x90 */
    volatile Reg32_t DCKCFGR2;   /**< RCC Dedicated Clocks configuration register 2,               Address offset: 0x94 */
  };


  /**
   *  Generic PLL configuration structure. Setting
   */
  struct PLLConfig
  {
    Reg32_t M;      /**< PLLM: Division factor for PLL VCO input clock. */
    Reg32_t N;      /**< PLLN: Multiplication factor for PLL VCO output clock. */
    Reg32_t P;      /**< PLLP: Division factor for main system clock (SYSCLK). */
    Reg32_t Q;      /**< PLLQ: Division factor for OTG FS, SDIO and RNG clocks.  */
    Reg32_t R;      /**< PLLR: PLL division factor for I2S, SAI, SYSTEM, SPDIFRX clocks. */
  };


  struct SourceMux
  {
    Chimera::Clock::Bus pll;   /**< Source for the PLL inputs */
    Chimera::Clock::Bus sys;   /**< Source for system clock */
    Chimera::Clock::Bus sdio;  /**< Source for SDIO peripheral */
    Chimera::Clock::Bus rtc;   /**< Source for RTC/AWU clock */
    Chimera::Clock::Bus usb48; /**< Source for USB 48 MHz clock */
    Chimera::Clock::Bus i2s1;  /**< Source for I2S1 clock */
    Chimera::Clock::Bus i2s2;  /**< Source for I2S2 clock */
    Chimera::Clock::Bus sai1;  /**< Source for SAI1 clock */
    Chimera::Clock::Bus sai2;  /**< Source for SAI2 clock */
  };


  struct BusPrescale
  {
    uint16_t ahb;
    uint16_t apb1;
    uint16_t apb2;
  };


  struct SourceEnabled
  {
    bool lsi;
    bool lse;
    bool hse;
    bool hsi;
    bool pll_core_clk;
    bool pll_core_q;
    bool pll_core_r;
    bool pll_sai_p;
    bool pll_sai_q;
    bool pll_i2s_p;
    bool pll_i2s_q;
    bool pll_i2s_r;
  };


  struct ClockTreeInit
  {
    uint32_t flashWaitStates;
    PLLConfig PLLCore;  /**< Core pll config */
    PLLConfig PLLSAI;   /**< SAI pll config */
    PLLConfig PLLI2S;   /**< I2S pll config */
    SourceMux mux;       /**< Clock mux selection options */
    BusPrescale prescaler;
    SourceEnabled enabled;

    void clear()
    {
      flashWaitStates = 10;
      memset( &PLLCore, 0, sizeof( PLLCore ) );
      memset( &PLLSAI, 0, sizeof( PLLSAI ) );
      memset( &PLLI2S, 0, sizeof( PLLI2S ) );
      memset( &mux, 0, sizeof( mux ) );
      memset( &prescaler, 0, sizeof( prescaler ) );
      memset( &enabled, 0, sizeof( enabled ) );
    }
  };


  /*---------------------------------------------------------------------------
  Register Accessors
  ---------------------------------------------------------------------------*/
  /*------------------------------------------------
  Clock Control Register
  ------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CR, CR_HSEON_Msk, HSEON, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR, CR_HSERDY_Msk, HSERDY, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, CR, CR_HSION_Msk, HSION, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR, CR_HSIRDY_Msk, HSIRDY, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, CR, CR_PLLON_Msk, PLLON, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR, CR_PLLRDY_Msk, PLLRDY, BIT_ACCESS_R );

  /*------------------------------------------------
  PLL Configuration Register
  ------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, PLLCFGR, PLLCFGR_PLLSRC_Msk, PLLSRC, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, PLLCFGR, PLLCFGR_PLLM_Msk, PLLM, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, PLLCFGR, PLLCFGR_PLLN_Msk, PLLN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, PLLCFGR, PLLCFGR_PLLP_Msk, PLLP, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, PLLCFGR, PLLCFGR_PLLQ_Msk, PLLQ, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, PLLCFGR, PLLCFGR_PLLR_Msk, PLLR, BIT_ACCESS_RW );

  /*------------------------------------------------
  Configuration Register
  ------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CFGR, CFGR_SW_Msk, SW, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CFGR, CFGR_SWS_Msk, SWS, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, CFGR, CFGR_HPRE_Msk, HPRE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CFGR, CFGR_PPRE1_Msk, PPRE1, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CFGR, CFGR_PPRE2_Msk, PPRE2, BIT_ACCESS_RW );

  /*------------------------------------------------
  Control Status Register
  ------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CSR, CSR_LPWRRSTF, LPWRRSTF, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, CSR, CSR_WWDGRSTF, WWDGRSTF, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, CSR, CSR_IWDGRSTF, IWDGRSTF, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, CSR, CSR_SFTRSTF, SFTRSTF, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, CSR, CSR_BORRSTF, BORRSTF, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, CSR, CSR_PINRSTF, PINRSTF, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, CSR, CSR_RMVF, RMVFRSTF, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CSR, CSR_LSIRDY, LSIRDY, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, CSR, CSR_LSION, LSION, BIT_ACCESS_RW );

  /*-------------------------------------------------
  Backup Domain Control Register
  -------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, BDCR, BDCR_LSEON_Msk, LSEON, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, BDCR, BDCR_LSERDY_Msk, LSERDY, BIT_ACCESS_RW );


  /*-------------------------------------------------
  Dedicated Clock Config Register
  -------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, DCKCFGR, DCKCFGR_TIMPRE_Msk, TIMPRE, BIT_ACCESS_RW );

  /*-------------------------------------------------
  Dedicated Clock Config Register 2
  -------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, DCKCFGR2, DCKCFGR2_CK48MSEL_Msk, CK48MSEL, BIT_ACCESS_RW );

}    // namespace Thor::LLD::RCC

#endif /* !THOR_HW_RCC_TYPES_HPP */
