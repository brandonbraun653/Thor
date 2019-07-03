/********************************************************************************
 *   File Name:
 *    hw_power_types.hpp
 *
 *   Description:
 *    Declares types specific to the PWR peripehral
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

namespace Thor::Driver::PWR
{
  struct RegisterMap
  {
    volatile uint32_t CR;  /**< PWR control register,         Address offset: 0x00 */
    volatile uint32_t CSR; /**< PWR control status register,  Address offset: 0x04 */
  };

  static RegisterMap *const PWR_PERIPH = reinterpret_cast<RegisterMap *const>( PWR_BASE_ADDR );


  /*------------------------------------------------
  PWR_CR Register Interaction Model
  ------------------------------------------------*/
  namespace CR
  {
    struct LPDS
    {
      static inline uint32_t get()
      {
        return PWR_PERIPH->CR & CR_LPDS;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = get();
        tmp &= ~( CR_LPDS );
        tmp |= ( val & CR_LPDS );
        PWR_PERIPH->CR = tmp;
      }
    };

    struct PDDS
    {
      static inline uint32_t get()
      {
        return PWR_PERIPH->CR & CR_PDDS;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = get();
        tmp &= ~( CR_PDDS );
        tmp |= ( val & CR_PDDS );
        PWR_PERIPH->CR = tmp;
      }
    };

    struct CWUF
    {
      static inline uint32_t get()
      {
        return PWR_PERIPH->CR & CR_CWUF;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = get();
        tmp &= ~( CR_CWUF );
        tmp |= ( val & CR_CWUF );
        PWR_PERIPH->CR = tmp;
      }
    };

    struct CSBF
    {
      static inline uint32_t get()
      {
        return PWR_PERIPH->CR & CR_CSBF;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = get();
        tmp &= ~( CR_CSBF );
        tmp |= ( val & CR_CSBF );
        PWR_PERIPH->CR = tmp;
      }
    };

    struct PVDE
    {
      static inline uint32_t get()
      {
        return PWR_PERIPH->CR & CR_PVDE;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = get();
        tmp &= ~( CR_PVDE );
        tmp |= ( val & CR_PVDE );
        PWR_PERIPH->CR = tmp;
      }
    };

    struct PLS
    {
      static inline uint32_t get()
      {
        return PWR_PERIPH->CR & CR_PLS;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = get();
        tmp &= ~( CR_PLS );
        tmp |= ( val & CR_PLS );
        PWR_PERIPH->CR = tmp;
      }
    };

    struct DBP
    {
      static inline uint32_t get()
      {
        return PWR_PERIPH->CR & CR_DBP;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = get();
        tmp &= ~( CR_DBP );
        tmp |= ( val & CR_DBP );
        PWR_PERIPH->CR = tmp;
      }
    };

    struct FPDS
    {
      static inline uint32_t get()
      {
        return PWR_PERIPH->CR & CR_FPDS;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = get();
        tmp &= ~( CR_FPDS );
        tmp |= ( val & CR_FPDS );
        PWR_PERIPH->CR = tmp;
      }
    };

    struct LPUDS
    {
      static inline uint32_t get()
      {
        return PWR_PERIPH->CR & CR_LPLVDS;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = get();
        tmp &= ~( CR_LPLVDS );
        tmp |= ( val & CR_LPLVDS );
        PWR_PERIPH->CR = tmp;
      }
    };

    struct MRUDS
    {
      static inline uint32_t get()
      {
        return PWR_PERIPH->CR & CR_MRLVDS;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = get();
        tmp &= ~( CR_MRLVDS );
        tmp |= ( val & CR_MRLVDS );
        PWR_PERIPH->CR = tmp;
      }
    };

    struct ADCDC1
    {
      static inline uint32_t get()
      {
        return PWR_PERIPH->CR & CR_ADCDC1;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = get();
        tmp &= ~( CR_ADCDC1 );
        tmp |= ( val & CR_ADCDC1 );
        PWR_PERIPH->CR = tmp;
      }
    };

    struct VOS
    {
      static inline uint32_t get()
      {
        return PWR_PERIPH->CR & CR_VOS;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = get();
        tmp &= ~( CR_VOS );
        tmp |= ( val & CR_VOS );
        PWR_PERIPH->CR = tmp;
      }
    };

    struct ODEN
    {
      static inline uint32_t get()
      {
        return PWR_PERIPH->CR & CR_ODEN;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = get();
        tmp &= ~( CR_ODEN );
        tmp |= ( val & CR_ODEN );
        PWR_PERIPH->CR = tmp;
      }
    };

    struct ODSWEN
    {
      static inline uint32_t get()
      {
        return PWR_PERIPH->CR & CR_ODSWEN;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = get();
        tmp &= ~( CR_ODSWEN );
        tmp |= ( val & CR_ODSWEN );
        PWR_PERIPH->CR = tmp;
      }
    };

    struct UDEN
    {
      static inline uint32_t get()
      {
        return PWR_PERIPH->CR & CR_UDEN;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = get();
        tmp &= ~( CR_UDEN );
        tmp |= ( val & CR_UDEN );
        PWR_PERIPH->CR = tmp;
      }
    };

    struct FMSSR
    {
      static inline uint32_t get()
      {
        return PWR_PERIPH->CR & CR_FMSSR;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = get();
        tmp &= ~( CR_FMSSR );
        tmp |= ( val & CR_FMSSR );
        PWR_PERIPH->CR = tmp;
      }
    };

    struct FISSR
    {
      static inline uint32_t get()
      {
        return PWR_PERIPH->CR & CR_FISSR;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = get();
        tmp &= ~( CR_FISSR );
        tmp |= ( val & CR_FISSR );
        PWR_PERIPH->CR = tmp;
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
      static inline uint32_t get()
      {
        return PWR_PERIPH->CSR & CSR_WUF;
      }
    };

    struct SBF
    {
      static inline uint32_t get()
      {
        return PWR_PERIPH->CSR & CSR_SBF;
      }
    };

    struct PVDO
    {
      static inline uint32_t get()
      {
        return PWR_PERIPH->CSR & CSR_PVDO;
      }
    };

    struct BRR
    {
      static inline uint32_t get()
      {
        return PWR_PERIPH->CSR & CSR_BRR;
      }
    };

    struct EWUP2
    {
      static inline uint32_t get()
      {
        return PWR_PERIPH->CSR & CSR_EWUP2;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = get();
        tmp &= ~( CSR_EWUP2 );
        tmp |= ( val & CSR_EWUP2 );
        PWR_PERIPH->CSR = tmp;
      }
    };

    struct EWUP1
    {
      static inline uint32_t get()
      {
        return PWR_PERIPH->CSR & CSR_EWUP1;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = get();
        tmp &= ~( CSR_EWUP1 );
        tmp |= ( val & CSR_EWUP1 );
        PWR_PERIPH->CSR = tmp;
      }
    };

    struct BRE
    {
      static inline uint32_t get()
      {
        return PWR_PERIPH->CSR & CSR_BRE;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = get();
        tmp &= ~( CSR_BRE );
        tmp |= ( val & CSR_BRE );
        PWR_PERIPH->CSR = tmp;
      }
    };

    struct VOSRDY
    {
      static inline uint32_t get()
      {
        return PWR_PERIPH->CSR & CSR_VOSRDY;
      }
    };

    struct ODRDY
    {
      static inline uint32_t get()
      {
        return PWR_PERIPH->CSR & CSR_ODRDY;
      }
    };

    struct ODSWRDY
    {
      static inline uint32_t get()
      {
        return PWR_PERIPH->CSR & CSR_ODSWRDY;
      }
    };

    struct UDRDY
    {
      static inline uint32_t get()
      {
        return PWR_PERIPH->CSR & CSR_UDRDY;
      }

      static inline void set( const uint32_t val )
      {
        uint32_t tmp = get();
        tmp &= ~( CSR_UDRDY );
        tmp |= ( val & CSR_UDRDY );
        PWR_PERIPH->CSR = tmp;
      }
    };

  }    // namespace CSR
}

#endif /* !THOR_HW_POWER_TYPES_HPP */