/********************************************************************************
 *  File Name:
 *    hw_rcc_types.hpp
 *
 *  Description:
 *    Declares types specific to the RCC peripehral
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

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
  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  namespace Configuration
  {
    /**
     *  High level structure describing what kinds of clocks are available
     *  to be used as a source for the System Clock.
     */
    using OscillatorType_t = Reg32_t;
    namespace OscillatorType
    {
      static constexpr OscillatorType_t HSE     = 1u;
      static constexpr OscillatorType_t HSI     = 2u;
      static constexpr OscillatorType_t PLLCLK  = 4u;
      static constexpr OscillatorType_t PLLRCLK = 8u;
    }    // namespace OscillatorType

    /**
     *  High level structure describing what kinds of clocks are available
     *  to be configured by the code.
     */
    namespace ClockType
    {
      static constexpr ClockType_t SYSCLK = 1u;
      static constexpr ClockType_t HCLK   = 2u;
      static constexpr ClockType_t PCLK1  = 4u;
      static constexpr ClockType_t PCLK2  = 8u;
    }    // namespace ClockType
  }      // namespace Configuration


  /*-------------------------------------------------------------------------------
  Structures
  -------------------------------------------------------------------------------*/
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
   *  Initialization structure for the primary PLL
   *
   *  @note See reference manual Figure 13
   */
  struct PLLInit
  {
    Reg32_t State;  /**< The new state of the PLL.*/
    Reg32_t Source; /**< RCC_PLLSource: PLL entry clock source. */
    Reg32_t M;      /**< PLLM: Division factor for PLL VCO input clock. */
    Reg32_t N;      /**< PLLN: Multiplication factor for PLL VCO output clock. */
    Reg32_t P;      /**< PLLP: Division factor for main system clock (SYSCLK). */
    Reg32_t Q;      /**< PLLQ: Division factor for OTG FS, SDIO and RNG clocks.  */
    Reg32_t R;      /**< PLLR: PLL division factor for I2S, SAI, SYSTEM, SPDIFRX clocks. */
  };

  /**
   *  Initialization structure for the I2S PLL
   *
   *  @note See reference manual Figure 13
   */
  struct PLLI2SInit
  {
    Reg32_t M; /**< Specifies division factor for PLL VCO input clock. */
    Reg32_t N; /**< Specifies the multiplication factor for PLLI2S VCO output clock. */
    Reg32_t P; /**< Specifies division factor for SPDIFRX Clock. */
    Reg32_t Q; /**< Specifies the division factor for SAI clock. */
    Reg32_t R; /**< Specifies the division factor for I2S clock. */
  };

  /**
   *  Initialization structure for the SAI PLL
   *
   *  @note See reference manual Figure 13
   */
  struct PLLSAIInit
  {
    Reg32_t M; /**< Spcifies division factor for PLL VCO input clock. */
    Reg32_t N; /**< Specifies the multiplication factor for PLLI2S VCO output clock. */
    Reg32_t P; /**< Specifies division factor for OTG FS, SDIO and RNG clocks. */
    Reg32_t Q; /**< Specifies the division factor for SAI clock. */
  };

  /**
   *  Initialization structure that specifies how all the system oscillators should be configured.
   */
  struct OscillatorInit
  {
    Configuration::OscillatorType_t
        source;       /**< The oscillators to be configured. Can be multiple values of OscillatorType  OR'd together */
    Reg32_t HSEState; /**< The new state of the HSE. Can be value of CR::HSEConfig */
    Reg32_t LSEState; /**< The new state of the LSE. */
    Reg32_t HSIState; /**< The new state of the HSI. Can be value of CR::HSIConfig */
    Reg32_t LSIState; /**< The new state of the LSI. */
    Reg32_t HSICalibrationValue; /**< The HSI calibration trimming value */
    PLLInit PLL;                 /**< Main PLL initialization parameters */
  };

  /**
   *  Initialization structure that allows for specifying how multiple system clocks can be configured.
   */
  struct ClockInit
  {
    ClockType_t clock;                  /**< The clocks to be configured. Can be multiple values of ClockType OR'd together */
    CFGR::SW::SysOscSrc_t SYSCLKSource; /**< The system clock source (SYSCLKS)*/
    CFGR::HPRE::AHBPrescale_t AHBCLKDivider;    /**< The AHB clock (HCLK) divider */
    CFGR::PPRE1::APB1Prescale_t APB1CLKDivider; /**< The APB1 clock (PCLK1) divider */
    CFGR::PPRE2::APB2Prescale_t APB2CLKDivider; /**< The APB2 clock (PCLK2) divider */
    Reg32_t FlashLatency;                       /**< The new number of flash wait states given the updated system clock */
  };

  /*-------------------------------------------------------------------------------
  Register Accessors
  -------------------------------------------------------------------------------*/
  /*------------------------------------------------
  Clock Control Register
  ------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CR, CR_HSEON_Msk, HSEON, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR, CR_HSERDY_Msk, HSERDY, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, CR, CR_HSION_Msk, HSION, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR, CR_HSIRDY_Msk, HSIRDY, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, CR, CR_MSION_Msk, MSION, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR, CR_MSIRDY_Msk, MSIRDY, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, CR, CR_MSIPLLEN_Msk, MSIPLLEN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR, CR_MSIRGSEL_Msk, MSIRGSEL, BIT_ACCESS_RS );
  REG_ACCESSOR( RegisterMap, CR, CR_MSIRANGE_Msk, MSIRANGE1, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR, CR_PLLON_Msk, PLLON, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR, CR_PLLRDY_Msk, PLLRDY, BIT_ACCESS_R );

  /*------------------------------------------------
  PLL Configuration Register
  ------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, PLLCFGR, PLLCFGR_PLLSRC_Msk, PLLSRC, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, PLLCFGR, PLLCFGR_PLLM_Msk, PLLM, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, PLLCFGR, PLLCFGR_PLLN_Msk, PLLN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, PLLCFGR, PLLCFGR_PLLPEN_Msk, PLLPEN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, PLLCFGR, PLLCFGR_PLLP_Msk, PLLP, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, PLLCFGR, PLLCFGR_PLLQEN_Msk, PLLQEN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, PLLCFGR, PLLCFGR_PLLQ_Msk, PLLQ, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, PLLCFGR, PLLCFGR_PLLREN_Msk, PLLREN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, PLLCFGR, PLLCFGR_PLLR_Msk, PLLR, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, PLLCFGR, PLLCFGR_PLLPDIV_Msk, PLLPDIV, BIT_ACCESS_RW );

  /*------------------------------------------------
  Configuration Register
  ------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CFGR, CFGR_SW_Msk, SW, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CFGR, CFGR_SWS_Msk, SWS, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, CFGR, CFGR_HPRE_Msk, HPRE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CFGR, CFGR_PPRE1_Msk, PPRE1, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CFGR, CFGR_PPRE2_Msk, PPRE2, BIT_ACCESS_RW );

  /*------------------------------------------------
  Peripheral Clock Enable Register (APB1ENR1)
  ------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, APB1ENR1, APB1ENR1_PWREN_Msk, PWREN, BIT_ACCESS_RW );

  /*------------------------------------------------
  Control Status Register
  ------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CSR, CSR_MSISRANGE_Msk, MSIRANGE2, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CSR, CSR_LPWRRSTF, LPWRRSTF, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, CSR, CSR_WWDGRSTF, WWDGRSTF, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, CSR, CSR_IWDGRSTF, IWDGRSTF, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, CSR, CSR_SFTRSTF, SFTRSTF, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, CSR, CSR_BORRSTF, BORRSTF, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, CSR, CSR_PINRSTF, PINRSTF, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, CSR, CSR_OBLRSTF, OBLRSTF, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, CSR, CSR_FWRSTF, FWRSTF, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, CSR, CSR_RMVF, RMVFRSTF, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CSR, CSR_MSISRANGE, MSIRANGE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CSR, CSR_LSIRDY, LSIRDY, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, CSR, CSR_LSION, LSION, BIT_ACCESS_RW );

}    // namespace Thor::LLD::RCC

#endif /* !THOR_HW_RCC_TYPES_HPP */
