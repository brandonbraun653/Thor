/********************************************************************************
 *   File Name:
 *    hw_wwdg_types.hpp
 *
 *   Description:
 *
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#ifndef THOR_HW_WATCHDOG_TYPES_HPP
#define THOR_HW_WATCHDOG_TYPES_HPP

/* C++ Includes */
#include <cstdint>

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/f4/wwdg/hw_wwdg_prj.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_WWDG == 1 )
namespace Thor::Driver::WWDG
{
  struct RegisterMap
  {
    volatile uint32_t CR;   /**< WWDG Control Register,         Address offset: 0x00 */
    volatile uint32_t CFR;  /**< WWDG Configuration Register,   Address offset: 0x04 */
    volatile uint32_t SR;   /**< WWDG Status Register,          Address offset: 0x08 */
  };

  /*------------------------------------------------
  Configuration Options
  ------------------------------------------------*/
  namespace Configuration
  {
  }

  /*------------------------------------------------
  Control Register
  ------------------------------------------------*/
  namespace CR
  {
    static inline uint32_t get( const RegisterMap *const periph )
    {
      return periph->CR & CR_Msk;
    }

    static inline void set( RegisterMap *const periph, const uint32_t val )
    {
      periph->CR = val & CR_Msk;
    }

    static inline void reset( RegisterMap *const periph )
    {
      periph->CR = CR_Rst & CR_Msk;
    }

    class WDGA
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR = tmp;
      }

    private:
      static constexpr uint32_t mask = CR_WDGA_Msk;
    };

    class T
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR;
        tmp &= ~mask;

        // Make sure the proper bit is set to prevent an instant reset
        tmp |= ( val & mask ) | CR_T_ACT_LOW_MANUAL_RESET;

        periph->CR = tmp;
      }

    private:
      static constexpr uint32_t mask = CR_T_Msk;
    };

  }    // namespace CR

  /*------------------------------------------------
  Configuration Register
  ------------------------------------------------*/
  namespace CFR
  {
    static constexpr uint32_t CLK_DIV_1  = CFR_CLK_DIV_1;
    static constexpr uint32_t CLK_DIV_2  = CFR_CLK_DIV_2;
    static constexpr uint32_t CLK_DIV_4  = CFR_CLK_DIV_4;
    static constexpr uint32_t CLK_DIV_8  = CFR_CLK_DIV_8;

    static constexpr uint32_t CLK_DIV_MIN = CLK_DIV_1;
    static constexpr uint32_t CLK_DIV_MAX = CLK_DIV_8;

    static constexpr uint32_t PCLK_1_DIV = CFR_PCLK_1_DIV;

    static inline uint32_t get( const RegisterMap *const periph )
    {
      return periph->CFR & CFR_Msk;
    }

    static inline void set( RegisterMap *const periph, const uint32_t val )
    {
      periph->CFR = val & CFR_Msk;
    }

    static inline void reset( RegisterMap *const periph )
    {
      periph->CFR = CFR_Rst & CFR_Msk;
    }

    class EWI
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CFR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CFR;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CFR = tmp;
      }

    private:
      static constexpr uint32_t mask = CFR_EWI_Msk;
    };

    class WDGTB
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CFR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CFR;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CFR = tmp;
      }

    private:
      static constexpr uint32_t mask = CFR_WDGTB_Msk;
    };

    class W
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CFR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CFR;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CFR = tmp;
      }

    private:
      static constexpr uint32_t mask = CFR_W_Msk;
    };
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

    class EWIF
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->SR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->SR;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->SR = tmp;
      }

    private:
      static constexpr uint32_t mask = SR_EWIF_Msk;
    };
  }

}    // namespace Thor::Driver::WWDG

#endif /* TARGET_STM32F4 && THOR_DRIVER_WATCHDOG */
#endif /* !THOR_HW_WATCHDOG_TYPES_HPP */