/********************************************************************************
 *   File Name:
 *    hw_usart_types.hpp
 *
 *   Description:
 *    STM32 Types for the USART Peripheral
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_USART_TYPES_HPP
#define THOR_HW_USART_TYPES_HPP

/* C++ Includes */
#include <cstdint>

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/f4/usart/hw_usart_prj.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_USART == 1 )

namespace Thor::Driver::USART
{
  struct RegisterMap
  {
    volatile uint32_t SR;   /**< USART Status register,                   Address offset: 0x00 */
    volatile uint32_t DR;   /**< USART Data register,                     Address offset: 0x04 */
    volatile uint32_t BRR;  /**< USART Baud rate register,                Address offset: 0x08 */
    volatile uint32_t CR1;  /**< USART Control register 1,                Address offset: 0x0C */
    volatile uint32_t CR2;  /**< USART Control register 2,                Address offset: 0x10 */
    volatile uint32_t CR3;  /**< USART Control register 3,                Address offset: 0x14 */
    volatile uint32_t GTPR; /**< USART Guard time and prescaler register, Address offset: 0x18 */
  };

  static RegisterMap *const USART1_PERIPH = reinterpret_cast<RegisterMap *const>( USART1_BASE_ADDR );
  static RegisterMap *const USART2_PERIPH = reinterpret_cast<RegisterMap *const>( USART2_BASE_ADDR );
  static RegisterMap *const USART3_PERIPH = reinterpret_cast<RegisterMap *const>( USART3_BASE_ADDR );
  static RegisterMap *const USART6_PERIPH = reinterpret_cast<RegisterMap *const>( USART6_BASE_ADDR );

  /*------------------------------------------------
  Status Register
  ------------------------------------------------*/
  namespace SR
  {
    static constexpr uint32_t resetValue = SR_Rst;

    static inline uint32_t get( const RegisterMap *const periph )
    {
      return periph->SR & SR_Msk;
    }

    static inline void set( RegisterMap *const periph, const uint32_t val )
    {
      periph->SR = val & SR_Msk;
    }

    class CTS
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
      static constexpr uint32_t mask = SR_CTS;
    };

    class LBD
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
      static constexpr uint32_t mask = SR_LBD;
    };

    class TXE
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->SR & mask;
      }

    private:
      static constexpr uint32_t mask = SR_TXE;
    };

    class TC
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
      static constexpr uint32_t mask = SR_TC;
    };

    class RXNE
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
      static constexpr uint32_t mask = SR_RXNE;
    };

    class IDLE
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->SR & mask;
      }

    private:
      static constexpr uint32_t mask = SR_IDLE;
    };

    class ORE
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->SR & mask;
      }

    private:
      static constexpr uint32_t mask = SR_ORE;
    };

    class NF
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->SR & mask;
      }

    private:
      static constexpr uint32_t mask = SR_NF;
    };

    class FE
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->SR & mask;
      }

    private:
      static constexpr uint32_t mask = SR_FE;
    };

    class PE
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->SR & mask;
      }

    private:
      static constexpr uint32_t mask = SR_PE;
    };
  }    // namespace SR

  /*------------------------------------------------
  Data Register
  ------------------------------------------------*/
  namespace DR
  {
    static inline uint32_t get( const RegisterMap *const periph )
    {
      return periph->DR & DR_DR;
    }

    static inline void set( RegisterMap *const periph, const uint32_t val )
    {
      periph->DR = val & DR_DR;
    }
  }    // namespace DR

  /*------------------------------------------------
  Baud Rate Register
  ------------------------------------------------*/
  namespace BRR
  {
    static constexpr uint32_t resetValue = BRR_Rst;

    static inline uint32_t get( const RegisterMap *const periph )
    {
      return periph->BRR & BRR_Msk;
    }

    static inline void set( RegisterMap *const periph, const uint32_t val )
    {
      periph->BRR = val & BRR_Msk;
    }

    class Mantissa
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->BRR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->BRR;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->BRR = tmp;
      }

    private:
      static constexpr uint32_t mask = BRR_DIV_Mantissa;
    };

    class Fraction
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->BRR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->BRR;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->BRR = tmp;
      }

    private:
      static constexpr uint32_t mask = BRR_DIV_Fraction;
    };
  }    // namespace BR

  /*------------------------------------------------
  Control Register 1
  ------------------------------------------------*/
  namespace CR1
  {
    static constexpr uint32_t resetValue = CR1_Rst;

    static inline uint32_t get( const RegisterMap *const periph )
    {
      return periph->CR1 & CR1_Msk;
    }

    static inline void set( RegisterMap *const periph, const uint32_t val )
    {
      periph->CR1 = val & CR1_Msk;
    }

    class OVER8
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR1 & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR1;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR1 = tmp;
      }

    private:
      static constexpr uint32_t mask = CR1_OVER8;
    };

    class UE
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR1 & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR1;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR1 = tmp;
      }

    private:
      static constexpr uint32_t mask = CR1_UE;
    };

    class M
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR1 & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR1;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR1 = tmp;
      }

    private:
      static constexpr uint32_t mask = CR1_M;
    };

    class WAKE
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR1 & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR1;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR1 = tmp;
      }

    private:
      static constexpr uint32_t mask = CR1_WAKE;
    };

    class PCE
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR1 & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR1;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR1 = tmp;
      }

    private:
      static constexpr uint32_t mask = CR1_PCE;
    };

    class PS
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR1 & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR1;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR1 = tmp;
      }

    private:
      static constexpr uint32_t mask = CR1_PS;
    };

    class PEIE
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR1 & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR1;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR1 = tmp;
      }

    private:
      static constexpr uint32_t mask = CR1_PEIE;
    };

    class TXEIE
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR1 & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR1;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR1 = tmp;
      }

    private:
      static constexpr uint32_t mask = CR1_TXEIE;
    };

    class TCIE
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR1 & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR1;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR1 = tmp;
      }

    private:
      static constexpr uint32_t mask = CR1_TCIE;
    };

    class RXNEIE
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR1 & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR1;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR1 = tmp;
      }

    private:
      static constexpr uint32_t mask = CR1_RXNEIE;
    };

    class IDLEIE
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR1 & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR1;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR1 = tmp;
      }

    private:
      static constexpr uint32_t mask = CR1_IDLEIE;
    };

    class TE
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR1 & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR1;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR1 = tmp;
      }

    private:
      static constexpr uint32_t mask = CR1_TE;
    };

    class RE
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR1 & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR1;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR1 = tmp;
      }

    private:
      static constexpr uint32_t mask = CR1_RE;
    };

    class RWU
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR1 & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR1;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR1 = tmp;
      }

    private:
      static constexpr uint32_t mask = CR1_RWU;
    };

    class SBK
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR1 & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR1;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR1 = tmp;
      }

    private:
      static constexpr uint32_t mask = CR1_SBK;
    };
  }    // namespace CR1

  /*------------------------------------------------
  Control Register 2
  ------------------------------------------------*/
  namespace CR2
  {
    static constexpr uint32_t resetValue = CR2_Rst;

    static inline uint32_t get( const RegisterMap *const periph )
    {
      return periph->CR2 & CR2_Msk;
    }

    static inline void set( RegisterMap *const periph, const uint32_t val )
    {
      periph->CR2 = val & CR2_Msk;
    }

    class LINEN
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR2 & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR2;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR2 = tmp;
      }

    private:
      static constexpr uint32_t mask = CR2_LINEN;
    };

    class STOP
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR2 & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR2;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR2 = tmp;
      }

    private:
      static constexpr uint32_t mask = CR2_STOP;
    };

    class CLKEN
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR2 & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR2;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR2 = tmp;
      }

    private:
      static constexpr uint32_t mask = CR2_CLKEN;
    };

    class CPOL
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR2 & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR2;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR2 = tmp;
      }

    private:
      static constexpr uint32_t mask = CR2_CPOL;
    };

    class CPHA
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR2 & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR2;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR2 = tmp;
      }

    private:
      static constexpr uint32_t mask = CR2_CPHA;
    };

    class LBCL
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR2 & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR2;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR2 = tmp;
      }

    private:
      static constexpr uint32_t mask = CR2_LBCL;
    };

    class LBDIE
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR2 & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR2;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR2 = tmp;
      }

    private:
      static constexpr uint32_t mask = CR2_LBDIE;
    };

    class LBDL
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR2 & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR2;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR2 = tmp;
      }

    private:
      static constexpr uint32_t mask = CR2_LBDL;
    };

    class ADD
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR2 & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR2;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR2 = tmp;
      }

    private:
      static constexpr uint32_t mask = CR2_ADD;
    };
  }

  /*------------------------------------------------
  Control Register 3
  ------------------------------------------------*/
  namespace CR3
  {
    static constexpr uint32_t resetValue = CR3_Rst;

    static inline uint32_t get( const RegisterMap *const periph )
    {
      return periph->CR3 & CR3_Msk;
    }

    static inline void set( RegisterMap *const periph, const uint32_t val )
    {
      periph->CR3 = val & CR3_Msk;
    }

    class ONEBIT
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR3 & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR3;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR3 = tmp;
      }

    private:
      static constexpr uint32_t mask = CR3_ONEBIT;
    };

    class CTSIE
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR3 & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR3;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR3 = tmp;
      }

    private:
      static constexpr uint32_t mask = CR3_CTSIE;
    };

    class CTSE
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR3 & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR3;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR3 = tmp;
      }

    private:
      static constexpr uint32_t mask = CR3_CTSE;
    };

    class RTSE
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR3 & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR3;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR3 = tmp;
      }

    private:
      static constexpr uint32_t mask = CR3_RTSE;
    };

    class DMAT
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR3 & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR3;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR3 = tmp;
      }

    private:
      static constexpr uint32_t mask = CR3_DMAT;
    };

    class DMAR
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR3 & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR3;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR3 = tmp;
      }

    private:
      static constexpr uint32_t mask = CR3_DMAR;
    };

    class SCEN
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR3 & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR3;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR3 = tmp;
      }

    private:
      static constexpr uint32_t mask = CR3_SCEN;
    };

    class NACK
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR3 & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR3;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR3 = tmp;
      }

    private:
      static constexpr uint32_t mask = CR3_NACK;
    };

    class HDSEL
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR3 & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR3;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR3 = tmp;
      }

    private:
      static constexpr uint32_t mask = CR3_HDSEL;
    };

    class IRLP
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR3 & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR3;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR3 = tmp;
      }

    private:
      static constexpr uint32_t mask = CR3_IRLP;
    };

    class IREN
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR3 & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR3;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR3 = tmp;
      }

    private:
      static constexpr uint32_t mask = CR3_IREN;
    };

    class EIE
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->CR3 & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->CR3;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR3 = tmp;
      }

    private:
      static constexpr uint32_t mask = CR3_EIE;
    };
  }    // namespace CR3

  /*------------------------------------------------
  Guard Time and Prescaler Register
  ------------------------------------------------*/
  namespace GTPR
  {
    static constexpr uint32_t resetValue = GTPR_Rst;

    static inline uint32_t get( const RegisterMap *const periph )
    {
      return periph->GTPR & GTPR_Msk;
    }

    static inline void set( RegisterMap *const periph, const uint32_t val )
    {
      periph->GTPR = val & GTPR_Msk;
    }

    class GT
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->GTPR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->GTPR;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->GTPR = tmp;
      }

    private:
      static constexpr uint32_t mask = GTPR_GT;
    };

    class PSC
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->GTPR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->GTPR;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->GTPR = tmp;
      }

    private:
      static constexpr uint32_t mask = GTPR_PSC;
    };
  }    // namespace GTPR
}    // namespace Thor::Driver::USART

#endif /* TARGET_STM32F4 && THOR_DRIVER_USART */
#endif /* !THOR_HW_USART_TYPES_HPP */