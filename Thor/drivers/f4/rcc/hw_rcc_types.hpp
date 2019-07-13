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
#include <Thor/headers.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_prj.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_RCC == 1 )

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

  static RegisterMap *const RCC_PERIPH = reinterpret_cast<RegisterMap *const>( RCC_BASE_ADDR );

  /**
   *  Describes a generic set of registers for a peripheral type that
   *  allow control of clocking and reset functionality.
   */
  struct PeriphCtrlRegisters
  {
    const void *clock;     /**< Standard clock configuration registers */
    const void *clockLP;   /**< Low power clock configuration registers */
    const void *reset;     /**< Peripheral reset registers*/
    size_t elements; /**< Number of elements in the tables */
  };

  /**
   *  High level structure describing what kinds of clocks are available
   *  to be used as a source for the System Clock.
   */
  using OscillatorType_t = uint32_t;
  struct OscillatorType
  {
    static constexpr OscillatorType_t HSE     = 1u;
    static constexpr OscillatorType_t HSI     = 2u;
    static constexpr OscillatorType_t PLLCLK  = 4u;
    static constexpr OscillatorType_t PLLRCLK = 8u;
  };

  /**
   *  High level structure describing what kinds of clocks are available
   *  to be configured by the code.
   */
  using ClockType_t = uint32_t;
  struct ClockType
  {
    static constexpr ClockType_t SYSCLK = 1u;
    static constexpr ClockType_t HCLK   = 2u;
    static constexpr ClockType_t PCLK1  = 4u;
    static constexpr ClockType_t PCLK2  = 8u;
  };

  /*------------------------------------------------
  RCC_CR Register Interaction Model
  ------------------------------------------------*/
  namespace CR
  {
    static constexpr uint32_t locked   = 1u;
    static constexpr uint32_t unlocked = 0u;

    static constexpr uint32_t enabled  = 1u;
    static constexpr uint32_t disabled = 0u;

    struct SAIRDY
    {
      static inline uint32_t get()
      {
        return RCC_PERIPH->CR & CR_PLLSAIRDY;
      }
    };

    struct SAION
    {
      static inline uint32_t get()
      {
        return RCC_PERIPH->CR & CR_PLLSAION;
      }

      static inline void set( const uint32_t val )
      {
      }
    };

    struct I2SRDY
    {
      static inline uint32_t get()
      {
        return RCC_PERIPH->CR & CR_PLLI2SRDY;
      }
    };

    struct I2SON
    {
      static inline uint32_t get()
      {
        return RCC_PERIPH->CR & CR_PLLI2SON;
      }

      static inline void set( const uint32_t val )
      {
      }
    };

    struct PLLRDY
    {
      static inline uint32_t get()
      {
        return RCC_PERIPH->CR & CR_PLLRDY;
      }
    };

    struct PLLON
    {
      static inline uint32_t get()
      {
        return RCC_PERIPH->CR & CR_PLLON;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = RCC_PERIPH->CR;
        tmp &= ~( CR_PLLON );
        tmp |= ( val & CR_PLLON );
        RCC_PERIPH->CR = tmp;
      }
    };

    struct PLLConfig
    {
      static constexpr uint32_t NONE = 0u;
      static constexpr uint32_t OFF  = 1u;
      static constexpr uint32_t ON   = CR_PLLON;
    };

    struct PLLDiv
    {
      static constexpr uint32_t DIV2 = 2u;
      static constexpr uint32_t DIV4 = 4u;
      static constexpr uint32_t DIV6 = 6u;
      static constexpr uint32_t DIV8 = 8u;
    };

    struct CSSON
    {
      static inline uint32_t get()
      {
        return RCC_PERIPH->CR & CR_CSSON;
      }

      static inline void set( const uint32_t val )
      {
      }
    };

    struct HSEBYP
    {
      static inline uint32_t get()
      {
        return RCC_PERIPH->CR & CR_HSEBYP;
      }

      static inline void set( const uint32_t val )
      {
      }
    };

    struct HSERDY
    {
      static inline uint32_t get()
      {
        return RCC_PERIPH->CR & CR_HSERDY;
      }
    };

    struct HSEON
    {
      static inline uint32_t get()
      {
        return RCC_PERIPH->CR & CR_HSEON;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = RCC_PERIPH->CR;
        tmp &= ~( CR_HSEON );
        tmp |= ( val & CR_HSEON );
        RCC_PERIPH->CR = tmp;
      }
    };

    struct HSEConfig
    {
      static constexpr uint32_t OFF    = 0u;
      static constexpr uint32_t ON     = CR_HSEON;
      static constexpr uint32_t BYPASS = CR_HSEBYP | CR_HSEON;
    };

    struct HSIRDY
    {
      static inline uint32_t get()
      {
        return RCC_PERIPH->CR & CR_HSIRDY;
      }
    };

    struct HSION
    {
      static inline uint32_t get()
      {
        return RCC_PERIPH->CR & CR_HSION;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = RCC_PERIPH->CR;
        tmp &= ~( CR_HSION );
        tmp |= ( val & CR_HSION );
        RCC_PERIPH->CR = tmp;
      }
    };

    struct HSICAL
    {
      static inline uint32_t get()
      {
        return RCC_PERIPH->CR & CR_HSICAL;
      }
    };

    struct HSITRIM
    {
      static inline uint32_t get()
      {
        return RCC_PERIPH->CR & CR_HSITRIM;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = RCC_PERIPH->CR;
        tmp &= ~( CR_HSITRIM );
        tmp |= ( val << CR_HSITRIM_Pos ) & CR_HSITRIM;
        RCC_PERIPH->CR = tmp;
      }
    };

    struct HSIConfig
    {
      static constexpr uint32_t OFF = 0u;
      static constexpr uint32_t ON  = 1u;
    };
  }    // namespace CR

  /*------------------------------------------------
  RCC_PLLCFGR Register Interaction Model
  ------------------------------------------------*/
  namespace PLLCFGR
  {
    struct SRC
    {
      static constexpr uint32_t HSI = PLLCFGR_PLLSRC_HSI;
      static constexpr uint32_t HSE = PLLCFGR_PLLSRC_HSE;

      static inline uint32_t get()
      {
        return RCC_PERIPH->PLLCFGR & PLLCFGR_PLLSRC;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = RCC_PERIPH->PLLCFGR;
        tmp &= ~( PLLCFGR_PLLSRC );
        tmp |= ( val & PLLCFGR_PLLSRC );
        RCC_PERIPH->PLLCFGR = tmp;
      }
    };

    struct M
    {
      static inline uint32_t get()
      {
        return RCC_PERIPH->PLLCFGR & PLLCFGR_PLLM;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = RCC_PERIPH->PLLCFGR;
        tmp &= ~( PLLCFGR_PLLM );
        tmp |= ( val & PLLCFGR_PLLM );
        RCC_PERIPH->PLLCFGR = tmp;
      }
    };

    struct N
    {
      static inline uint32_t get()
      {
        return RCC_PERIPH->PLLCFGR & PLLCFGR_PLLN;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = RCC_PERIPH->PLLCFGR;
        tmp &= ~( PLLCFGR_PLLN );
        tmp |= ( val & PLLCFGR_PLLN );
        RCC_PERIPH->PLLCFGR = tmp;
      }
    };

    struct P
    {
      static inline uint32_t get()
      {
        return RCC_PERIPH->PLLCFGR & PLLCFGR_PLLP;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = RCC_PERIPH->PLLCFGR;
        tmp &= ~( PLLCFGR_PLLP );
        tmp |= ( val & PLLCFGR_PLLP );
        RCC_PERIPH->PLLCFGR = tmp;
      }
    };

    struct Q
    {
      static inline uint32_t get()
      {
        return RCC_PERIPH->PLLCFGR & PLLCFGR_PLLQ;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = RCC_PERIPH->PLLCFGR;
        tmp &= ~( PLLCFGR_PLLQ );
        tmp |= ( val & PLLCFGR_PLLQ );
        RCC_PERIPH->PLLCFGR = tmp;
      }
    };

    struct R
    {
      static inline uint32_t get()
      {
        return RCC_PERIPH->PLLCFGR & PLLCFGR_PLLR;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = RCC_PERIPH->PLLCFGR;
        tmp &= ~( PLLCFGR_PLLR );
        tmp |= ( val & PLLCFGR_PLLR );
        RCC_PERIPH->PLLCFGR = tmp;
      }
    };

  }    // namespace PLLCFGR

  /*------------------------------------------------
  RCC_CFGR Register Interaction Model
  ------------------------------------------------*/
  namespace CFGR
  {
    struct SW
    {
      using SysOscSrc_t = uint32_t;

      static constexpr SysOscSrc_t HSI  = CFGR_SW_HSI;
      static constexpr SysOscSrc_t HSE  = CFGR_SW_HSE;
      static constexpr SysOscSrc_t PLLCLK  = CFGR_SW_PLL;
      static constexpr SysOscSrc_t PLLRCLK = CFGR_SW_PLLR;

      static inline uint32_t get()
      {
        return RCC_PERIPH->CFGR & CFGR_SW;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = RCC_PERIPH->CFGR;
        tmp &= ~( CFGR_SW );
        tmp |= ( val & CFGR_SW );
        RCC_PERIPH->CFGR = tmp;
      }
    };

    struct SWS
    {
      static constexpr uint32_t HSI  = CFGR_SWS_HSI;
      static constexpr uint32_t HSE  = CFGR_SWS_HSE;
      static constexpr uint32_t PLL  = CFGR_SWS_PLL;
      static constexpr uint32_t PLLR = CFGR_SWS_PLLR;

      static inline uint32_t get()
      {
        return RCC_PERIPH->CFGR & CFGR_SWS;
      }

      /**
       *  Gets the right shifted value instead of the masked value
       */
      static inline uint32_t getRS()
      {
        return ( RCC_PERIPH->CFGR & CFGR_SWS ) >> CFGR_SWS_Pos;
      }
    };

    struct HPRE
    {
      using AHBPrescale_t = uint32_t;

      static constexpr AHBPrescale_t DIV1   = CFGR_HPRE_DIV1;
      static constexpr AHBPrescale_t DIV2   = CFGR_HPRE_DIV2;
      static constexpr AHBPrescale_t DIV4   = CFGR_HPRE_DIV4;
      static constexpr AHBPrescale_t DIV8   = CFGR_HPRE_DIV8;
      static constexpr AHBPrescale_t DIV16  = CFGR_HPRE_DIV16;
      static constexpr AHBPrescale_t DIV64  = CFGR_HPRE_DIV64;
      static constexpr AHBPrescale_t DIV128 = CFGR_HPRE_DIV128;
      static constexpr AHBPrescale_t DIV256 = CFGR_HPRE_DIV256;
      static constexpr AHBPrescale_t DIV512 = CFGR_HPRE_DIV512;

      static inline uint32_t get()
      {
        return RCC_PERIPH->CFGR & CFGR_HPRE;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = RCC_PERIPH->CFGR;
        tmp &= ~( CFGR_HPRE );
        tmp |= ( val & CFGR_HPRE );
        RCC_PERIPH->CFGR = tmp;
      }
    };

    struct PPRE1
    {
      using APB1Prescale_t = uint32_t;

      static constexpr APB1Prescale_t DIV1  = CFGR_PPRE1_DIV1;
      static constexpr APB1Prescale_t DIV2  = CFGR_PPRE1_DIV2;
      static constexpr APB1Prescale_t DIV4  = CFGR_PPRE1_DIV4;
      static constexpr APB1Prescale_t DIV8  = CFGR_PPRE1_DIV8;
      static constexpr APB1Prescale_t DIV16 = CFGR_PPRE1_DIV16;

      static inline uint32_t get()
      {
        return RCC_PERIPH->CFGR & CFGR_PPRE1;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = RCC_PERIPH->CFGR;
        tmp &= ~( CFGR_PPRE1 );
        tmp |= ( val & CFGR_PPRE1 );
        RCC_PERIPH->CFGR = tmp;
      }
    };

    struct PPRE2
    {
      using APB2Prescale_t = uint32_t;

      static constexpr APB2Prescale_t DIV1  = CFGR_PPRE2_DIV1;
      static constexpr APB2Prescale_t DIV2  = CFGR_PPRE2_DIV2;
      static constexpr APB2Prescale_t DIV4  = CFGR_PPRE2_DIV4;
      static constexpr APB2Prescale_t DIV8  = CFGR_PPRE2_DIV8;
      static constexpr APB2Prescale_t DIV16 = CFGR_PPRE2_DIV16;

      static inline uint32_t get()
      {
        return RCC_PERIPH->CFGR & CFGR_PPRE2;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = RCC_PERIPH->CFGR;
        tmp &= ~( CFGR_PPRE2 );
        tmp |= ( val & CFGR_PPRE2 );
        RCC_PERIPH->CFGR = tmp;
      }
    };
  }    // namespace CFGR

  /*------------------------------------------------
  RCC_APB1ENR Register Interaction Model
  ------------------------------------------------*/
  namespace APB1ENR
  {
    struct PWREN
    {
      static inline uint32_t get()
      {
        return RCC_PERIPH->APB1ENR & APB1ENR_PWREN;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = RCC_PERIPH->APB1ENR;
        tmp &= ~( APB1ENR_PWREN );
        tmp |= ( val & APB1ENR_PWREN );
        RCC_PERIPH->APB1ENR = tmp;
      }
    };

    struct PWRENConfig
    {
      static constexpr uint32_t ON  = APB1ENR_PWREN;
      static constexpr uint32_t OFF = 0u;
    };

  }    // namespace APB1ENR

  /*------------------------------------------------
  RCC_BDCR Register Interaction Model
  ------------------------------------------------*/
  namespace BDCR
  {
    struct LSEON
    {
      static inline uint32_t get()
      {
        return RCC_PERIPH->BDCR & BDCR_LSEON;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = RCC_PERIPH->BDCR;
        tmp &= ~( BDCR_LSEON );
        tmp |= ( val & BDCR_LSEON );
        RCC_PERIPH->BDCR = tmp;
      }
    };

    struct LSERDY
    {
      static inline uint32_t get()
      {
        return RCC_PERIPH->BDCR & BDCR_LSERDY;
      }
    };

    struct LSEConfig
    {
      static constexpr uint32_t OFF    = 0u;
      static constexpr uint32_t ON     = BDCR_LSEON;
      static constexpr uint32_t BYPASS = BDCR_LSEBYP | BDCR_LSEON;
    };

    struct LSEBYP
    {
      static inline uint32_t get()
      {
        return RCC_PERIPH->BDCR & BDCR_LSEBYP;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = RCC_PERIPH->BDCR;
        tmp &= ~( BDCR_LSEBYP );
        tmp |= ( val & BDCR_LSEBYP );
        RCC_PERIPH->BDCR = tmp;
      }
    };

    struct LSEMOD
    {
      static inline uint32_t get()
      {
        return RCC_PERIPH->BDCR & BDCR_LSEMOD;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = RCC_PERIPH->BDCR;
        tmp &= ~( BDCR_LSEMOD );
        tmp |= ( val & BDCR_LSEMOD );
        RCC_PERIPH->BDCR = tmp;
      }
    };

    struct RTCSEL
    {
      static inline uint32_t get()
      {
        return RCC_PERIPH->BDCR & BDCR_RTCSEL;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = RCC_PERIPH->BDCR;
        tmp &= ~( BDCR_RTCSEL );
        tmp |= ( val & BDCR_RTCSEL );
        RCC_PERIPH->BDCR = tmp;
      }
    };

    struct RTCEN
    {
      static inline uint32_t get()
      {
        return RCC_PERIPH->BDCR & BDCR_RTCEN;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = RCC_PERIPH->BDCR;
        tmp &= ~( BDCR_RTCEN );
        tmp |= ( val & BDCR_RTCEN );
        RCC_PERIPH->BDCR = tmp;
      }
    };

    struct BDRST
    {
      static inline uint32_t get()
      {
        return RCC_PERIPH->BDCR & BDCR_BDRST;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = RCC_PERIPH->BDCR;
        tmp &= ~( BDCR_BDRST );
        tmp |= ( val & BDCR_BDRST );
        RCC_PERIPH->BDCR = tmp;
      }
    };
  }    // namespace BDCR

  /*------------------------------------------------
  RCC_CSR Register Interaction Model
  ------------------------------------------------*/
  namespace CSR
  {
    static constexpr uint32_t flagSet = 1u;
    static constexpr uint32_t flagClr = 0u;

    static constexpr uint32_t locked   = 1u;
    static constexpr uint32_t unlocked = 0u;

    struct LPWRRSTF
    {
      static inline uint32_t get()
      {
        return RCC_PERIPH->CSR & CSR_LPWRRSTF;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = RCC_PERIPH->CSR;
        tmp &= ~( CSR_LPWRRSTF );
        tmp |= ( val & CSR_LPWRRSTF );
        RCC_PERIPH->CSR = tmp;
      }
    };

    struct WWDGRSTF
    {
      static inline uint32_t get()
      {
        return RCC_PERIPH->CSR & CSR_WWDGRSTF;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = RCC_PERIPH->CSR;
        tmp &= ~( CSR_WWDGRSTF );
        tmp |= ( val & CSR_WWDGRSTF );
        RCC_PERIPH->CSR = tmp;
      }
    };

    struct IWDGRSTF
    {
      static inline uint32_t get()
      {
        return RCC_PERIPH->CSR & CSR_IWDGRSTF;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = RCC_PERIPH->CSR;
        tmp &= ~( CSR_IWDGRSTF );
        tmp |= ( val & CSR_IWDGRSTF );
        RCC_PERIPH->CSR = tmp;
      }
    };

    struct SFTRSTF
    {
      static inline uint32_t get()
      {
        return RCC_PERIPH->CSR & CSR_SFTRSTF;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = RCC_PERIPH->CSR;
        tmp &= ~( CSR_SFTRSTF );
        tmp |= ( val & CSR_SFTRSTF );
        RCC_PERIPH->CSR = tmp;
      }
    };

    struct PORRSTF
    {
      static inline uint32_t get()
      {
        return RCC_PERIPH->CSR & CSR_PORRSTF;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = RCC_PERIPH->CSR;
        tmp &= ~( CSR_PORRSTF );
        tmp |= ( val & CSR_PORRSTF );
        RCC_PERIPH->CSR = tmp;
      }
    };

    struct PINRSTF
    {
      static inline uint32_t get()
      {
        return RCC_PERIPH->CSR & CSR_PINRSTF;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = RCC_PERIPH->CSR;
        tmp &= ~( CSR_PINRSTF );
        tmp |= ( val & CSR_PINRSTF );
        RCC_PERIPH->CSR = tmp;
      }
    };

    struct BORRSTF
    {
      static inline uint32_t get()
      {
        return RCC_PERIPH->CSR & CSR_BORRSTF;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = RCC_PERIPH->CSR;
        tmp &= ~( CSR_BORRSTF );
        tmp |= ( val & CSR_BORRSTF );
        RCC_PERIPH->CSR = tmp;
      }
    };

    struct RMVF
    {
      static inline uint32_t get()
      {
        return RCC_PERIPH->CSR & CSR_RMVF;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = RCC_PERIPH->CSR;
        tmp &= ~( CSR_RMVF );
        tmp |= ( val & CSR_RMVF );
        RCC_PERIPH->CSR = tmp;
      }
    };

    struct LSIRDY
    {
      static inline uint32_t get()
      {
        return RCC_PERIPH->CSR & CSR_LSIRDY;
      }
    };

    struct LSION
    {
      static inline uint32_t get()
      {
        return RCC_PERIPH->CSR & CSR_LSION;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = RCC_PERIPH->CSR;
        tmp &= ~( CSR_LSION );
        tmp |= ( val & CSR_LSION );
        RCC_PERIPH->CSR = tmp;
      }
    };

    struct LSIConfig
    {
      static constexpr uint32_t OFF = 0u;
      static constexpr uint32_t ON  = 1u;
    };

  }    // namespace CSR

  /**
   *  Initialization structure for the primary PLL
   *
   *  @note See reference manual Figure 13
   */
  struct PLLInit
  {
    uint32_t State;  /**< The new state of the PLL.*/
    uint32_t Source; /**< RCC_PLLSource: PLL entry clock source. */
    uint32_t M;      /**< PLLM: Division factor for PLL VCO input clock. */
    uint32_t N;      /**< PLLN: Multiplication factor for PLL VCO output clock. */
    uint32_t P;      /**< PLLP: Division factor for main system clock (SYSCLK). */
    uint32_t Q;      /**< PLLQ: Division factor for OTG FS, SDIO and RNG clocks.  */
    uint32_t R;      /**< PLLR: PLL division factor for I2S, SAI, SYSTEM, SPDIFRX clocks. */
  };

  /**
   *  Initialization structure for the I2S PLL
   *
   *  @note See reference manual Figure 13
   */
  struct PLLI2SInit
  {
    uint32_t M; /**< Specifies division factor for PLL VCO input clock. */
    uint32_t N; /**< Specifies the multiplication factor for PLLI2S VCO output clock. */
    uint32_t P; /**< Specifies division factor for SPDIFRX Clock. */
    uint32_t Q; /**< Specifies the division factor for SAI clock. */
    uint32_t R; /**< Specifies the division factor for I2S clock. */
  };

  /**
   *  Initialization structure for the SAI PLL
   *
   *  @note See reference manual Figure 13
   */
  struct PLLSAIInit
  {
    uint32_t M; /**< Spcifies division factor for PLL VCO input clock. */
    uint32_t N; /**< Specifies the multiplication factor for PLLI2S VCO output clock. */
    uint32_t P; /**< Specifies division factor for OTG FS, SDIO and RNG clocks. */
    uint32_t Q; /**< Specifies the division factor for SAI clock. */
  };

  /**
   *  Initialization structure that specifies how all the system oscillators should be configured.
   */
  struct OscillatorInit
  {
    OscillatorType_t source;      /**< The oscillators to be configured. Can be multiple values of OscillatorType  OR'd together */
    uint32_t HSEState;            /**< The new state of the HSE. Can be value of CR::HSEConfig */
    uint32_t LSEState;            /**< The new state of the LSE. */
    uint32_t HSIState;            /**< The new state of the HSI. Can be value of CR::HSIConfig */
    uint32_t LSIState;            /**< The new state of the LSI. */
    uint32_t HSICalibrationValue; /**< The HSI calibration trimming value */
    PLLInit PLL;                  /**< Main PLL initialization parameters */
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
    uint32_t FlashLatency;                      /**< The new number of flash wait states given the updated system clock */
  };

  /**
   *  Configuration struct for the clock enable register
   */
  struct RegisterConfig
  {
    volatile uint32_t *reg; /**< Clock enable register */
    uint32_t mask;          /**< Bit mask that will enable/disable the peripheral's clock */
  };
}    // namespace Thor::Driver::RCC

#endif /* TARGET_STM32F4 && THOR_DRIVER_RCC */
#endif /* !THOR_HW_RCC_TYPES_HPP */