/********************************************************************************
 *  File Name:
 *    hw_flash_types.hpp
 *
 *  Description:
 *    Implements Flash peripheral types
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_DRIVER_FLASH_TYPES_HPP
#define THOR_HW_DRIVER_FLASH_TYPES_HPP

/* C++ Includes */
#include <cstdint>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/stm32f4x/flash/hw_flash_prj.hpp>

namespace Thor::LLD::FLASH
{
  struct RegisterMap
  {
    volatile uint32_t ACR;     /**< FLASH access control register,   Address offset: 0x00 */
    volatile uint32_t KEYR;    /**< FLASH key register,              Address offset: 0x04 */
    volatile uint32_t OPTKEYR; /**< FLASH option key register,       Address offset: 0x08 */
    volatile uint32_t SR;      /**< FLASH status register,           Address offset: 0x0C */
    volatile uint32_t CR;      /**< FLASH control register,          Address offset: 0x10 */
    volatile uint32_t OPTCR;   /**< FLASH option control register ,  Address offset: 0x14 */
    volatile uint32_t OPTCR1;  /**< FLASH option control register 1, Address offset: 0x18 */
  };

  #if defined( THOR_LLD_FLASH )
  
  /*------------------------------------------------
  Access Control Register (ACR)
  ------------------------------------------------*/
  namespace ACR
  {
    struct DCRST
    {
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->ACR & ACR_DCRST;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->ACR;
        tmp &= ~( ACR_DCRST );
        tmp |= val;
        periph->ACR = tmp;
      }
    };

    struct ICRST
    {
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->ACR & ACR_ICRST;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->ACR;
        tmp &= ~( ACR_ICRST );
        tmp |= val & ACR_ICRST;
        periph->ACR = tmp;
      }
    };

    struct DCEN
    {
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->ACR & ACR_DCEN;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->ACR;
        tmp &= ~( ACR_DCEN );
        tmp |= val & ACR_DCEN;
        periph->ACR = tmp;
      }
    };

    struct ICEN
    {
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->ACR & ACR_ICEN;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->ACR;
        tmp &= ~( ACR_ICEN );
        tmp |= val & ACR_ICEN;
        periph->ACR = tmp;
      }
    };

    struct PRFTEN
    {
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->ACR & ACR_PRFTEN;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->ACR;
        tmp &= ~( ACR_PRFTEN );
        tmp |= val & ACR_PRFTEN;
        periph->ACR = tmp;
      }
    };

    struct LATENCY
    {
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->ACR & ACR_LATENCY;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->ACR;
        tmp &= ~( ACR_LATENCY );
        tmp |= val & ACR_LATENCY;
        periph->ACR = tmp;
      }
    };
  }    // namespace ACR

  /*------------------------------------------------
  Key Register (KEYR)
  ------------------------------------------------*/
  namespace KEYR
  {
  }

  /*------------------------------------------------
  Option Key Register (OPTKEYR)
  ------------------------------------------------*/
  namespace OPTKEYR
  {
  }

  /*------------------------------------------------
  Status Register (SR)
  ------------------------------------------------*/
  namespace SR
  {
  }

  /*------------------------------------------------
  Control Register (CR)
  ------------------------------------------------*/
  namespace CR
  {
  }

  /*------------------------------------------------
  Option Control Register (OPTCR)
  ------------------------------------------------*/
  namespace OPTCR
  {
  }

  /*------------------------------------------------
  Option Control Register 1 (OPTCR1)
  ------------------------------------------------*/
  namespace OPTCR1
  {
  }

  #endif /* THOR_LLD_FLASH */

}    // namespace Thor::LLD::FLASH

#endif /* !THOR_HW_DRIVER_FLASH_TYPES_HPP */