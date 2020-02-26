/********************************************************************************
 *  File Name:
 *    hw_iwdg_types.hpp
 *
 *  Description:
 *    Types for the independent watchdog driver
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#ifndef THOR_HW_IWDG_TYPES_HPP
#define THOR_HW_IWDG_TYPES_HPP

/* C++ Includes */
#include <cstdint>
#include <array>

/* Driver Includes */
#include <Thor/lld/stm32f4x/iwdg/hw_iwdg_prj.hpp>

namespace Thor::LLD::IWDG
{
  struct RegisterMap
  {
    volatile uint32_t KR;   /**< IWDG Key Register,       Address offset: 0x00 */
    volatile uint32_t PR;   /**< IWDG Prescale Register,  Address offset: 0x04 */
    volatile uint32_t RLR;  /**< IWDG Reload Register,    Address offset: 0x08 */
    volatile uint32_t SR;   /**< IWDG Status Register,    Address offset: 0x0C */
  };
  class Driver;

  using DriverInstanceList = std::array<Driver *, NUM_IWDG_PERIPHS>;
  using PeriphRegisterList = std::array<RegisterMap *, NUM_IWDG_PERIPHS>;

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
    static constexpr uint32_t RefreshSequence = KR_REFRESH;
    static constexpr uint32_t StartSequence   = KR_START;
    static constexpr uint32_t UnlockSequence  = KR_UNLOCK;
    static constexpr uint32_t LockSequence    = KR_LOCK;

    static inline uint32_t get( const RegisterMap *const periph )
    {
      return periph->KR & KR_Msk;
    }

    static inline void set( RegisterMap *const periph, const uint32_t val )
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
    static constexpr uint32_t PRESCALE_4   = PR_PRESCALE_4;
    static constexpr uint32_t PRESCALE_8   = PR_PRESCALE_8;
    static constexpr uint32_t PRESCALE_16  = PR_PRESCALE_16;
    static constexpr uint32_t PRESCALE_32  = PR_PRESCALE_32;
    static constexpr uint32_t PRESCALE_64  = PR_PRESCALE_64;
    static constexpr uint32_t PRESCALE_128 = PR_PRESCALE_128;
    static constexpr uint32_t PRESCALE_256 = PR_PRESCALE_256;
    static constexpr uint32_t PRESCALE_MIN = PR_MIN_PRESCALE;
    static constexpr uint32_t PRESCALE_MAX = PR_MAX_PRESCALE;

    static constexpr uint32_t NUM_PRESCALE_OPTIONS = 7u;

    static inline uint32_t get( const RegisterMap *const periph )
    {
      return periph->PR & PR_Msk;
    }

    static inline void set( RegisterMap *const periph, const uint32_t val )
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

    static inline void set( RegisterMap *const periph, const uint32_t val )
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

    static inline void set( RegisterMap *const periph, const uint32_t val )
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

}    // namespace Thor::LLD::IWDG

#endif /* !THOR_HW_IWDG_TYPES_HPP */