/********************************************************************************
 *   File Name:
 *    hw_power_types.hpp
 *
 *   Description:
 *    Declares types specific to the PWR peripheral
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_POWER_TYPES_HPP
#define THOR_HW_POWER_TYPES_HPP

/* C++ Includes */
#include <cstdint>

/* Driver Includes */
#include <Thor/drivers/f4/power/hw_power_prj.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_PWR == 1 )

namespace Thor::Driver::PWR
{
  struct RegisterMap
  {
    volatile uint32_t CR;  /**< PWR control register,         Address offset: 0x00 */
    volatile uint32_t CSR; /**< PWR control status register,  Address offset: 0x04 */
  };

  /*------------------------------------------------
  PWR_CR Register Interaction Model
  ------------------------------------------------*/
  namespace CR
  {
    struct LPDS
    {
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR & CR_LPDS;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR;
        tmp &= ~( CR_LPDS );
        tmp |= ( val & CR_LPDS );
        periph->CR = tmp;
      }
    };

    struct PDDS
    {
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR & CR_PDDS;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR;
        tmp &= ~( CR_PDDS );
        tmp |= ( val & CR_PDDS );
        periph->CR = tmp;
      }
    };

    struct CWUF
    {
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR & CR_CWUF;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR;
        tmp &= ~( CR_CWUF );
        tmp |= ( val & CR_CWUF );
        periph->CR = tmp;
      }
    };

    struct CSBF
    {
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR & CR_CSBF;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR;
        tmp &= ~( CR_CSBF );
        tmp |= ( val & CR_CSBF );
        periph->CR = tmp;
      }
    };

    struct PVDE
    {
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR & CR_PVDE;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR;
        tmp &= ~( CR_PVDE );
        tmp |= ( val & CR_PVDE );
        periph->CR = tmp;
      }
    };

    struct PLS
    {
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR & CR_PLS;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR;
        tmp &= ~( CR_PLS );
        tmp |= ( val & CR_PLS );
        periph->CR = tmp;
      }
    };

    struct DBP
    {
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR & CR_DBP;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR;
        tmp &= ~( CR_DBP );
        tmp |= ( val & CR_DBP );
        periph->CR = tmp;
      }
    };

    struct FPDS
    {
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR & CR_FPDS;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR;
        tmp &= ~( CR_FPDS );
        tmp |= ( val & CR_FPDS );
        periph->CR = tmp;
      }
    };

    struct LPUDS
    {
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR & CR_LPLVDS;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR;
        tmp &= ~( CR_LPLVDS );
        tmp |= ( val & CR_LPLVDS );
        periph->CR = tmp;
      }
    };

    struct MRUDS
    {
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR & CR_MRLVDS;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR;
        tmp &= ~( CR_MRLVDS );
        tmp |= ( val & CR_MRLVDS );
        periph->CR = tmp;
      }
    };

    struct ADCDC1
    {
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR & CR_ADCDC1;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR;
        tmp &= ~( CR_ADCDC1 );
        tmp |= ( val & CR_ADCDC1 );
        periph->CR = tmp;
      }
    };

    struct VOS
    {
      static constexpr uint32_t VOLTAGE_SCALE_1 = CR_VOS;
      static constexpr uint32_t VOLTAGE_SCALE_2 = CR_VOS_1;
      static constexpr uint32_t VOLTAGE_SCALE_3 = CR_VOS_0;

      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR & CR_VOS;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR;
        tmp &= ~( CR_VOS );
        tmp |= ( val & CR_VOS );
        periph->CR = tmp;
      }
    };

    struct ODEN
    {
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR & CR_ODEN;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR;
        tmp &= ~( CR_ODEN );
        tmp |= ( val & CR_ODEN );
        periph->CR = tmp;
      }
    };

    struct ODSWEN
    {
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR & CR_ODSWEN;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR;
        tmp &= ~( CR_ODSWEN );
        tmp |= ( val & CR_ODSWEN );
        periph->CR = tmp;
      }
    };

    struct UDEN
    {
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR & CR_UDEN;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR;
        tmp &= ~( CR_UDEN );
        tmp |= ( val & CR_UDEN );
        periph->CR = tmp;
      }
    };

    struct FMSSR
    {
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR & CR_FMSSR;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR;
        tmp &= ~( CR_FMSSR );
        tmp |= ( val & CR_FMSSR );
        periph->CR = tmp;
      }
    };

    struct FISSR
    {
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR & CR_FISSR;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR;
        tmp &= ~( CR_FISSR );
        tmp |= ( val & CR_FISSR );
        periph->CR = tmp;
      }
    };
  }    // namespace CR

  /*------------------------------------------------
  PWR_CSR Register Interaction Model
  ------------------------------------------------*/
  namespace CSR
  {
    struct WUF
    {
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CSR & CSR_WUF;
      }
    };

    struct SBF
    {
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CSR & CSR_SBF;
      }
    };

    struct PVDO
    {
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CSR & CSR_PVDO;
      }
    };

    struct BRR
    {
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CSR & CSR_BRR;
      }
    };

    struct EWUP2
    {
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CSR & CSR_EWUP2;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CSR;
        tmp &= ~( CSR_EWUP2 );
        tmp |= ( val & CSR_EWUP2 );
        periph->CSR = tmp;
      }
    };

    struct EWUP1
    {
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CSR & CSR_EWUP1;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CSR;
        tmp &= ~( CSR_EWUP1 );
        tmp |= ( val & CSR_EWUP1 );
        periph->CSR = tmp;
      }
    };

    struct BRE
    {
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CSR & CSR_BRE;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CSR;
        tmp &= ~( CSR_BRE );
        tmp |= ( val & CSR_BRE );
        periph->CSR = tmp;
      }
    };

    struct VOSRDY
    {
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CSR & CSR_VOSRDY;
      }
    };

    struct ODRDY
    {
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CSR & CSR_ODRDY;
      }
    };

    struct ODSWRDY
    {
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CSR & CSR_ODSWRDY;
      }
    };

    struct UDRDY
    {
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CSR & CSR_UDRDY;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CSR;
        tmp &= ~( CSR_UDRDY );
        tmp |= ( val & CSR_UDRDY );
        periph->CSR = tmp;
      }
    };

  }    // namespace CSR
}

#endif /* TARGET_STM32F4 && THOR_DRIVER_PWR */
#endif /* !THOR_HW_POWER_TYPES_HPP */