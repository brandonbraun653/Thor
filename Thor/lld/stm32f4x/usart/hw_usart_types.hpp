/********************************************************************************
 *  File Name:
 *    hw_usart_types.hpp
 *
 *  Description:
 *    STM32 Types for the USART Peripheral
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_USART_TYPES_HPP
#define THOR_HW_USART_TYPES_HPP

/* C++ Includes */
#include <cstdint>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/serial>

/* Driver Includes */
#include <Thor/lld/stm32f4x/usart/hw_usart_prj.hpp>

namespace Thor::Driver::USART
{
  struct RegisterMap
  {
    volatile Reg32_t SR;   /**< USART Status register,                   Address offset: 0x00 */
    volatile Reg32_t DR;   /**< USART Data register,                     Address offset: 0x04 */
    volatile Reg32_t BRR;  /**< USART Baud rate register,                Address offset: 0x08 */
    volatile Reg32_t CR1;  /**< USART Control register 1,                Address offset: 0x0C */
    volatile Reg32_t CR2;  /**< USART Control register 2,                Address offset: 0x10 */
    volatile Reg32_t CR3;  /**< USART Control register 3,                Address offset: 0x14 */
    volatile Reg32_t GTPR; /**< USART Guard time and prescaler register, Address offset: 0x18 */
  };

  class Driver;

  using DriverInstanceList = std::array<Driver *, NUM_USART_PERIPHS>;
  using PeriphRegisterList = std::array<RegisterMap *, NUM_USART_PERIPHS>;
  using DMASignalList      = std::array<Reg32_t, NUM_USART_PERIPHS>;

  /*------------------------------------------------
  Configuration Options
  ------------------------------------------------*/
  namespace Configuration
  {
    namespace WordLength
    {
      static constexpr Reg32_t LEN_8BIT = 0u;
      static constexpr Reg32_t LEN_9BIT = CR1_M;
    }    // namespace WordLength

    namespace Stop
    {
      static constexpr Reg32_t BIT_1   = 0u;
      static constexpr Reg32_t BIT_0_5 = CR2_STOP_0;
      static constexpr Reg32_t BIT_2   = CR2_STOP_1;
      static constexpr Reg32_t BIT_1_5 = CR2_STOP_0 | CR2_STOP_1;
    }    // namespace Stop

    namespace Parity
    {
      static constexpr Reg32_t NONE = 0u;
      static constexpr Reg32_t EVEN = CR1_PCE;
      static constexpr Reg32_t ODD  = CR1_PCE | CR1_PS;
    }    // namespace Parity

    namespace Modes
    {
      static constexpr Reg32_t RX    = CR1_RE;
      static constexpr Reg32_t TX    = CR1_TE;
      static constexpr Reg32_t TX_RX = RX | TX;
    }    // namespace Modes

    namespace Clock
    {
      static constexpr Reg32_t CLOCK_DISABLE = 0u;
      static constexpr Reg32_t CLOCK_ENABLE  = CR2_CLKEN;
    }    // namespace Clock

    namespace Polarity
    {
      static constexpr Reg32_t POLARITY_LOW  = 0u;
      static constexpr Reg32_t POLARITY_HIGH = CR2_CPOL;
    }    // namespace Polarity

    namespace Phase
    {
      static constexpr Reg32_t PHASE_1EDGE = 0u;
      static constexpr Reg32_t PHASE_2EDGE = CR2_CPHA;
    }    // namespace Phase

    namespace LastBit
    {
      static constexpr Reg32_t LASTBIT_DISABLE = 0u;
      static constexpr Reg32_t LASTBIT_ENABLE  = CR2_LBCL;
    }    // namespace LastBit

    namespace Nack
    {
      static constexpr Reg32_t NACK_ENABLE  = CR3_NACK;
      static constexpr Reg32_t NACK_DISABLE = 0u;
    }    // namespace Nack

    namespace Flags
    {
      static constexpr Reg32_t FLAG_CTS  = SR_CTS;
      static constexpr Reg32_t FLAG_LBD  = SR_LBD;
      static constexpr Reg32_t FLAG_TXE  = SR_TXE;
      static constexpr Reg32_t FLAG_TC   = SR_TC;
      static constexpr Reg32_t FLAG_RXNE = SR_RXNE;
      static constexpr Reg32_t FLAG_IDLE = SR_IDLE;
      static constexpr Reg32_t FLAG_ORE  = SR_ORE;
      static constexpr Reg32_t FLAG_NF   = SR_NF;
      static constexpr Reg32_t FLAG_FE   = SR_FE;
      static constexpr Reg32_t FLAG_PE   = SR_PE;
    }    // namespace Flags
  }

  /*------------------------------------------------
  Runtime
  ------------------------------------------------*/
  namespace Runtime
  {
    using Flag_t = Reg32_t;
    namespace Flag
    {
      /* Let the first 16 bits match the Status Register for consistency */
      static constexpr Flag_t RX_PARITY_ERROR    = Configuration::Flags::FLAG_PE;
      static constexpr Flag_t RX_FRAMING_ERROR   = Configuration::Flags::FLAG_FE;
      static constexpr Flag_t RX_NOISE_ERROR     = Configuration::Flags::FLAG_NF;
      static constexpr Flag_t RX_OVERRUN         = Configuration::Flags::FLAG_ORE;
      static constexpr Flag_t RX_IDLE_DETECTED   = Configuration::Flags::FLAG_IDLE;
      static constexpr Flag_t RX_BYTE_READY      = Configuration::Flags::FLAG_RXNE;
      static constexpr Flag_t TX_COMPLETE        = Configuration::Flags::FLAG_TC;
      static constexpr Flag_t TX_DR_EMPTY        = Configuration::Flags::FLAG_TXE;
      static constexpr Flag_t RX_LINE_IN_BREAK   = Configuration::Flags::FLAG_LBD;
      static constexpr Flag_t CTL_CLEAR_TO_SEND  = Configuration::Flags::FLAG_CTS;

      /* Use the remaining 16 bits for other signals */
      static constexpr Flag_t RX_LINE_IDLE_ABORT = ( 1u << 16 );
      static constexpr Flag_t RX_COMPLETE        = ( 1u << 17 );
    }
  }

  /*------------------------------------------------
  State Machine 
  ------------------------------------------------*/
  namespace StateMachine
  {
    enum TX : Chimera::Status_t
    {
      TX_READY    = Chimera::Serial::Status::TX_READY,
      TX_ONGOING  = Chimera::Serial::Status::TX_IN_PROGRESS,
      TX_ABORTED  = Chimera::Serial::Status::TX_ABORTED,
      TX_COMPLETE = Chimera::Serial::Status::TX_COMPLETE
    };

    enum RX : Chimera::Status_t
    {
      RX_READY    = Chimera::Serial::Status::RX_READY,
      RX_ONGOING  = Chimera::Serial::Status::RX_IN_PROGRESS,
      RX_COMPLETE = Chimera::Serial::Status::RX_COMPLETE,
      RX_ABORTED  = Chimera::Serial::Status::RX_ABORTED
    };
  }    // namespace StateMachine

  /*------------------------------------------------
  Status Register
  ------------------------------------------------*/
  namespace SR
  {
    static constexpr Reg32_t resetValue = SR_Rst;

    static inline Reg32_t get( const RegisterMap *const periph )
    {
      return periph->SR & SR_Msk;
    }

    static inline void set( RegisterMap *const periph, const Reg32_t val )
    {
      periph->SR = val & SR_Msk;
    }

    class CTS
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->SR & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->SR;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->SR = tmp;
      }

    private:
      static constexpr Reg32_t mask = SR_CTS;
    };

    class LBD
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->SR & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->SR;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->SR = tmp;
      }

    private:
      static constexpr Reg32_t mask = SR_LBD;
    };

    class TXE
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->SR & mask;
      }

    private:
      static constexpr Reg32_t mask = SR_TXE;
    };

    class TC
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->SR & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->SR;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->SR = tmp;
      }

    private:
      static constexpr Reg32_t mask = SR_TC;
    };

    class RXNE
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->SR & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->SR;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->SR = tmp;
      }

    private:
      static constexpr Reg32_t mask = SR_RXNE;
    };

    class IDLE
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->SR & mask;
      }

    private:
      static constexpr Reg32_t mask = SR_IDLE;
    };

    class ORE
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->SR & mask;
      }

    private:
      static constexpr Reg32_t mask = SR_ORE;
    };

    class NF
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->SR & mask;
      }

    private:
      static constexpr Reg32_t mask = SR_NF;
    };

    class FE
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->SR & mask;
      }

    private:
      static constexpr Reg32_t mask = SR_FE;
    };

    class PE
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->SR & mask;
      }

    private:
      static constexpr Reg32_t mask = SR_PE;
    };
  }    // namespace SR

  /*------------------------------------------------
  Data Register
  ------------------------------------------------*/
  namespace DR
  {
    static inline Reg32_t get( const RegisterMap *const periph )
    {
      return periph->DR & DR_DR;
    }

    static inline void set( RegisterMap *const periph, const Reg32_t val )
    {
      periph->DR = val & DR_DR;
    }
  }    // namespace DR

  /*------------------------------------------------
  Baud Rate Register
  ------------------------------------------------*/
  namespace BRR
  {
    static constexpr Reg32_t resetValue = BRR_Rst;

    static inline Reg32_t get( const RegisterMap *const periph )
    {
      return periph->BRR & BRR_Msk;
    }

    static inline void set( RegisterMap *const periph, const Reg32_t val )
    {
      periph->BRR = val & BRR_Msk;
    }

    class Mantissa
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->BRR & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->BRR;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->BRR = tmp;
      }

    private:
      static constexpr Reg32_t mask = BRR_DIV_Mantissa;
    };

    class Fraction
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->BRR & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->BRR;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->BRR = tmp;
      }

    private:
      static constexpr Reg32_t mask = BRR_DIV_Fraction;
    };
  }    // namespace BR

  /*------------------------------------------------
  Control Register 1
  ------------------------------------------------*/
  namespace CR1
  {
    static constexpr Reg32_t resetValue = CR1_Rst;
    static constexpr Reg32_t ITMask     = CR1_PEIE | CR1_TXEIE | CR1_TCIE | CR1_RXNEIE | CR1_IDLEIE;

    static inline Reg32_t get( const RegisterMap *const periph )
    {
      return periph->CR1 & CR1_Msk;
    }

    static inline void set( RegisterMap *const periph, const Reg32_t val )
    {
      periph->CR1 = val & CR1_Msk;
    }

    class OVER8
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->CR1 & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->CR1;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR1 = tmp;
      }

    private:
      static constexpr Reg32_t mask = CR1_OVER8;
    };

    class UE
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->CR1 & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->CR1;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR1 = tmp;
      }

    private:
      static constexpr Reg32_t mask = CR1_UE;
    };

    class M
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->CR1 & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->CR1;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR1 = tmp;
      }

    private:
      static constexpr Reg32_t mask = CR1_M;
    };

    class WAKE
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->CR1 & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->CR1;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR1 = tmp;
      }

    private:
      static constexpr Reg32_t mask = CR1_WAKE;
    };

    class PCE
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->CR1 & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->CR1;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR1 = tmp;
      }

    private:
      static constexpr Reg32_t mask = CR1_PCE;
    };

    class PS
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->CR1 & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->CR1;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR1 = tmp;
      }

    private:
      static constexpr Reg32_t mask = CR1_PS;
    };

    class PEIE
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->CR1 & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->CR1;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR1 = tmp;
      }

    private:
      static constexpr Reg32_t mask = CR1_PEIE;
    };

    class TXEIE
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->CR1 & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->CR1;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR1 = tmp;
      }

    private:
      static constexpr Reg32_t mask = CR1_TXEIE;
    };

    class TCIE
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->CR1 & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->CR1;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR1 = tmp;
      }

    private:
      static constexpr Reg32_t mask = CR1_TCIE;
    };

    class RXNEIE
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->CR1 & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->CR1;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR1 = tmp;
      }

    private:
      static constexpr Reg32_t mask = CR1_RXNEIE;
    };

    class IDLEIE
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->CR1 & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->CR1;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR1 = tmp;
      }

    private:
      static constexpr Reg32_t mask = CR1_IDLEIE;
    };

    class TE
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->CR1 & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->CR1;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR1 = tmp;
      }

    private:
      static constexpr Reg32_t mask = CR1_TE;
    };

    class RE
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->CR1 & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->CR1;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR1 = tmp;
      }

    private:
      static constexpr Reg32_t mask = CR1_RE;
    };

    class RWU
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->CR1 & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->CR1;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR1 = tmp;
      }

    private:
      static constexpr Reg32_t mask = CR1_RWU;
    };

    class SBK
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->CR1 & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->CR1;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR1 = tmp;
      }

    private:
      static constexpr Reg32_t mask = CR1_SBK;
    };
  }    // namespace CR1

  /*------------------------------------------------
  Control Register 2
  ------------------------------------------------*/
  namespace CR2
  {
    static constexpr Reg32_t resetValue = CR2_Rst;
    static constexpr Reg32_t ITMask     = CR2_LBDIE;

    static inline Reg32_t get( const RegisterMap *const periph )
    {
      return periph->CR2 & CR2_Msk;
    }

    static inline void set( RegisterMap *const periph, const Reg32_t val )
    {
      periph->CR2 = val & CR2_Msk;
    }

    class LINEN
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->CR2 & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->CR2;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR2 = tmp;
      }

    private:
      static constexpr Reg32_t mask = CR2_LINEN;
    };

    class STOP
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->CR2 & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->CR2;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR2 = tmp;
      }

    private:
      static constexpr Reg32_t mask = CR2_STOP;
    };

    class CLKEN
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->CR2 & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->CR2;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR2 = tmp;
      }

    private:
      static constexpr Reg32_t mask = CR2_CLKEN;
    };

    class CPOL
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->CR2 & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->CR2;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR2 = tmp;
      }

    private:
      static constexpr Reg32_t mask = CR2_CPOL;
    };

    class CPHA
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->CR2 & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->CR2;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR2 = tmp;
      }

    private:
      static constexpr Reg32_t mask = CR2_CPHA;
    };

    class LBCL
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->CR2 & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->CR2;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR2 = tmp;
      }

    private:
      static constexpr Reg32_t mask = CR2_LBCL;
    };

    class LBDIE
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->CR2 & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->CR2;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR2 = tmp;
      }

    private:
      static constexpr Reg32_t mask = CR2_LBDIE;
    };

    class LBDL
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->CR2 & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->CR2;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR2 = tmp;
      }

    private:
      static constexpr Reg32_t mask = CR2_LBDL;
    };

    class ADD
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->CR2 & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->CR2;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR2 = tmp;
      }

    private:
      static constexpr Reg32_t mask = CR2_ADD;
    };
  }

  /*------------------------------------------------
  Control Register 3
  ------------------------------------------------*/
  namespace CR3
  {
    static constexpr Reg32_t resetValue = CR3_Rst;
    static constexpr Reg32_t ITMask     = CR3_EIE;

    static inline Reg32_t get( const RegisterMap *const periph )
    {
      return periph->CR3 & CR3_Msk;
    }

    static inline void set( RegisterMap *const periph, const Reg32_t val )
    {
      periph->CR3 = val & CR3_Msk;
    }

    class ONEBIT
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->CR3 & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->CR3;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR3 = tmp;
      }

    private:
      static constexpr Reg32_t mask = CR3_ONEBIT;
    };

    class CTSIE
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->CR3 & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->CR3;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR3 = tmp;
      }

    private:
      static constexpr Reg32_t mask = CR3_CTSIE;
    };

    class CTSE
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->CR3 & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->CR3;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR3 = tmp;
      }

    private:
      static constexpr Reg32_t mask = CR3_CTSE;
    };

    class RTSE
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->CR3 & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->CR3;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR3 = tmp;
      }

    private:
      static constexpr Reg32_t mask = CR3_RTSE;
    };

    class DMAT
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->CR3 & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->CR3;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR3 = tmp;
      }

    private:
      static constexpr Reg32_t mask = CR3_DMAT;
    };

    class DMAR
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->CR3 & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->CR3;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR3 = tmp;
      }

    private:
      static constexpr Reg32_t mask = CR3_DMAR;
    };

    class SCEN
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->CR3 & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->CR3;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR3 = tmp;
      }

    private:
      static constexpr Reg32_t mask = CR3_SCEN;
    };

    class NACK
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->CR3 & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->CR3;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR3 = tmp;
      }

    private:
      static constexpr Reg32_t mask = CR3_NACK;
    };

    class HDSEL
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->CR3 & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->CR3;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR3 = tmp;
      }

    private:
      static constexpr Reg32_t mask = CR3_HDSEL;
    };

    class IRLP
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->CR3 & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->CR3;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR3 = tmp;
      }

    private:
      static constexpr Reg32_t mask = CR3_IRLP;
    };

    class IREN
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->CR3 & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->CR3;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR3 = tmp;
      }

    private:
      static constexpr Reg32_t mask = CR3_IREN;
    };

    class EIE
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->CR3 & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->CR3;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->CR3 = tmp;
      }

    private:
      static constexpr Reg32_t mask = CR3_EIE;
    };
  }    // namespace CR3

  /*------------------------------------------------
  Guard Time and Prescaler Register
  ------------------------------------------------*/
  namespace GTPR
  {
    static constexpr Reg32_t resetValue = GTPR_Rst;

    static inline Reg32_t get( const RegisterMap *const periph )
    {
      return periph->GTPR & GTPR_Msk;
    }

    static inline void set( RegisterMap *const periph, const Reg32_t val )
    {
      periph->GTPR = val & GTPR_Msk;
    }

    class GT
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->GTPR & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->GTPR;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->GTPR = tmp;
      }

    private:
      static constexpr Reg32_t mask = GTPR_GT;
    };

    class PSC
    {
    public:
      static inline Reg32_t get( const RegisterMap *const periph )
      {
        return periph->GTPR & mask;
      }

      static inline void set( RegisterMap *const periph, const Reg32_t val )
      {
        Reg32_t tmp = periph->GTPR;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->GTPR = tmp;
      }

    private:
      static constexpr Reg32_t mask = GTPR_PSC;
    };
  }    // namespace GTPR



  /**
   *  Transfer control block that handles data which should
   *  never be modified during a transfer.
   *
   *  (C)onstant (D)ata (T)ransfer (C)ontrol (B)lock
   */
  struct CDTCB
  {
    const uint8_t *buffer;
    size_t size;
    Chimera::Status_t state;

    inline void reset()
    {
      buffer = nullptr;
      size   = 0;
      state  = StateMachine::TX::TX_READY;
    }
  };

  /**
   *  Transfer control block that handles data which might
   *  be modified during a transfer.
   *
   *  (M)odifiable (D)ata (T)ransfer (C)ontrol (B)lock
   */
  struct MDTCB
  {
    uint8_t *buffer;
    size_t size;
    Chimera::Status_t state;

    inline void reset()
    {
      buffer = nullptr;
      size   = 0;
      state  = StateMachine::RX::RX_READY;
    }
  };
}    // namespace Thor::Driver::USART

#endif /* !THOR_HW_USART_TYPES_HPP */