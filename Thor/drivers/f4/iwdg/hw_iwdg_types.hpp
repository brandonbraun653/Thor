/********************************************************************************
 *   File Name:
 *    hw_iwdg_types.hpp
 *
 *   Description:
 *
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#ifndef THOR_HW_IWDG_TYPES_HPP
#define THOR_HW_IWDG_TYPES_HPP

/* C++ Includes */
#include <cstdint>

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/f4/iwdg/hw_iwdg_prj.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_WATCHDOG == 1 )
namespace Thor::Driver::IWDG
{

  struct RegisterMap
  {
    volatile uint32_t KR;   /**< IWDG Key Register,       Address offset: 0x00 */
    volatile uint32_t PR;   /**< IWDG Prescale Register,  Address offset: 0x04 */
    volatile uint32_t RLR;  /**< IWDG Reload Register,    Address offset: 0x08 */
    volatile uint32_t SR;   /**< IWDG Status Register,    Address offset: 0x0C */
  };

  static RegisterMap *const IWDG_PERIPH = reinterpret_cast<RegisterMap *const>( IWDG_BASE_ADDR );

  /*------------------------------------------------
  Configuration Options
  ------------------------------------------------*/
  namespace Configuration
  {

  }

  /*------------------------------------------------
  Key Register
  ------------------------------------------------*/
  namespace KR
  {
    static inline uint32_t get( const RegisterMap *const periph )
    {
      return periph->KR & KR_Msk;
    }

    static inline uint32_t set( RegisterMap *const periph, const uint32_t val )
    {
      periph->KR = val & KR_Msk;
    }

    static inline void reset( RegisterMap *const periph )
    {
      periph->KR = KR_Rst & KR_Msk;
    }

    class KEY
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->KR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->KR;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->KR = tmp;
      }

    private:
      static constexpr uint32_t mask = KR_KEY_Msk;
    };
  }

  /*------------------------------------------------
  Prescale Register
  ------------------------------------------------*/
  namespace PR
  {
    static inline uint32_t get( const RegisterMap *const periph )
    {
      return periph->PR & PR_Msk;
    }

    static inline uint32_t set( RegisterMap *const periph, const uint32_t val )
    {
      periph->PR = val & PR_Msk;
    }

    static inline void reset( RegisterMap *const periph )
    {
      periph->PR = PR_Rst & PR_Msk;
    }
  }

  /*------------------------------------------------
  Reload Register
  ------------------------------------------------*/
  namespace RLR
  {
    static inline uint32_t get( const RegisterMap *const periph )
    {
      return periph->RLR & RLR_Msk;
    }

    static inline uint32_t set( RegisterMap *const periph, const uint32_t val )
    {
      periph->RLR = val & RLR_Msk;
    }

    static inline void reset( RegisterMap *const periph )
    {
      periph->RLR = RLR_Rst & RLR_Msk;
    }
  }

  /*------------------------------------------------
  Status Register
  ------------------------------------------------*/
  namespace SR
  {
    static inline uint32_t get( const RegisterMap *const periph )
    {
      return periph->SR & SR_Msk;
    }

    static inline uint32_t set( RegisterMap *const periph, const uint32_t val )
    {
      periph->SR = val & SR_Msk;
    }

    static inline void reset( RegisterMap *const periph )
    {
      periph->SR = SR_Rst & SR_Msk;
    }

    class RVU
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->SR & mask;
      }

    private:
      static constexpr uint32_t mask = SR_RVU_Msk;
    };

    class PVU
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->SR & mask;
      }

    private:
      static constexpr uint32_t mask = SR_PVU_Msk;
    };
  }

}    // namespace Thor::Driver::IWDG

#endif /* TARGET_STM32F4 && THOR_DRIVER_WATCHDOG */
#endif /* !THOR_HW_IWDG_TYPES_HPP */