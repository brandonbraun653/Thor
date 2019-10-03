/********************************************************************************
 *   File Name:
 *    hw_spi_types.hpp
 *
 *   Description:
 *    STM32 Types for the SPI Peripheral
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_SPI_TYPES_HPP
#define THOR_HW_SPI_TYPES_HPP

/* C++ Includes */
#include <cstdint>

/* Chimera Includes */
#include <Chimera/types/spi_types.hpp>

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/f4/spi/hw_spi_prj.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_SPI == 1 )

namespace Thor::Driver::SPI
{
  struct RegisterMap
  {
    volatile uint32_t CR1;      /**< Control Register 1,            Address Offset: 0x00 */
    volatile uint32_t CR2;      /**< Control Register 2,            Address Offset: 0x04 */
    volatile uint32_t SR;       /**< Status Register,               Address Offset: 0x08 */
    volatile uint32_t DR;       /**< Data Register,                 Address Offset: 0x0C */
    volatile uint32_t CRCPR;    /**< CRC Polynomial Register,       Address Offset: 0x10 */
    volatile uint32_t RXCRCR;   /**< RX CRC Register,               Address Offset: 0x14 */
    volatile uint32_t TXCRCR;   /**< TX CRC Register,               Address Offset: 0x18 */
    volatile uint32_t I2SCFGR;  /**< I2S Configuration Register,    Address Offset: 0x1C */
    volatile uint32_t I2SPR;    /**< I2S Prescale Register,         Address Offset: 0x20 */
  };

  static RegisterMap *const SPI1_PERIPH = reinterpret_cast<RegisterMap *const>( SPI1_BASE_ADDR );
  static RegisterMap *const SPI2_PERIPH = reinterpret_cast<RegisterMap *const>( SPI2_BASE_ADDR );
  static RegisterMap *const SPI3_PERIPH = reinterpret_cast<RegisterMap *const>( SPI3_BASE_ADDR );
  static RegisterMap *const SPI4_PERIPH = reinterpret_cast<RegisterMap *const>( SPI4_BASE_ADDR );


  /**
   *  Checks if the given address belongs to a peripheral instance
   *
   *  @return bool
   */
  bool isSPI( const std::uintptr_t address );

  /*------------------------------------------------
  Configuration
  ------------------------------------------------*/
  namespace Configuration
  {
    
  }

  /*------------------------------------------------
  Runtime
  ------------------------------------------------*/
  namespace Runtime
  {
    using Flag_t = uint32_t;
    namespace Flag
    {
//      /* Let the first 16 bits match the Status Register for consistency */
//      static constexpr Flag_t RX_PARITY_ERROR   = Configuration::Flags::FLAG_PE;
//      static constexpr Flag_t RX_FRAMING_ERROR  = Configuration::Flags::FLAG_FE;
//      static constexpr Flag_t RX_NOISE_ERROR    = Configuration::Flags::FLAG_NF;
//      static constexpr Flag_t RX_OVERRUN        = Configuration::Flags::FLAG_ORE;
//      static constexpr Flag_t RX_IDLE_DETECTED  = Configuration::Flags::FLAG_IDLE;
//      static constexpr Flag_t RX_BYTE_READY     = Configuration::Flags::FLAG_RXNE;
//      static constexpr Flag_t TX_COMPLETE       = Configuration::Flags::FLAG_TC;
//      static constexpr Flag_t TX_DR_EMPTY       = Configuration::Flags::FLAG_TXE;
//      static constexpr Flag_t RX_LINE_IN_BREAK  = Configuration::Flags::FLAG_LBD;
//      static constexpr Flag_t CTL_CLEAR_TO_SEND = Configuration::Flags::FLAG_CTS;
//
//      /* Use the remaining 16 bits for other signals */
//      static constexpr Flag_t RX_LINE_IDLE_ABORT = ( 1u << 16 );
//      static constexpr Flag_t RX_COMPLETE        = ( 1u << 17 );
    }    // namespace Flag
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

    class BIDIMODE
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
      static constexpr uint32_t mask = CR1_BIDIMODE;
    };

    class BIDIOE
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
      static constexpr uint32_t mask = CR1_BIDIOE;
    };

    class CRCEN
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
      static constexpr uint32_t mask = CR1_CRCEN;
    };

    class CRCNEXT
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
      static constexpr uint32_t mask = CR1_CRCNEXT;
    };

    class DFF
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
      static constexpr uint32_t mask = CR1_DFF;
    };

    class RXONLY
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
      static constexpr uint32_t mask = CR1_RXONLY;
    };

    class SSM
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
      static constexpr uint32_t mask = CR1_SSM;
    };

    class SSI
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
      static constexpr uint32_t mask = CR1_SSI;
    };

    class LSBFIRST
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
      static constexpr uint32_t mask = CR1_LSBFIRST;
    };

    class SPE
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
      static constexpr uint32_t mask = CR1_SPE;
    };

    class BR
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
      static constexpr uint32_t mask = CR1_BR;
    };

    class MSTR
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
      static constexpr uint32_t mask = CR1_MSTR;
    };

    class CPOL
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
      static constexpr uint32_t mask = CR1_CPOL;
    };

    class CPHA
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
      static constexpr uint32_t mask = CR1_CPHA;
    };
  }

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

    class TXEIE
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
      static constexpr uint32_t mask = CR2_TXEIE;
    };

    class RXNEIE
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
      static constexpr uint32_t mask = CR2_RXNEIE;
    };

    class ERRIE
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
      static constexpr uint32_t mask = CR2_ERRIE;
    };

    class FRF
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
      static constexpr uint32_t mask = CR2_FRF;
    };

    class SSOE
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
      static constexpr uint32_t mask = CR2_SSOE;
    };

    class TXDMAEN
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
      static constexpr uint32_t mask = CR2_TXDMAEN;
    };

    class RXDMAEN
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
      static constexpr uint32_t mask = CR2_RXDMAEN;
    };
  }

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

    class FRE
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->SR & mask;
      }

    private:
      static constexpr uint32_t mask = SR_FRE;
    };

    class BSY
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->SR & mask;
      }

    private:
      static constexpr uint32_t mask = SR_BSY;
    };

    class OVR
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->SR & mask;
      }

    private:
      static constexpr uint32_t mask = SR_OVR;
    };

    class MODF
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->SR & mask;
      }

    private:
      static constexpr uint32_t mask = SR_MODF;
    };

    class CRCERR
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
      static constexpr uint32_t mask = SR_CRCERR;
    };

    class UDR
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->SR & mask;
      }

    private:
      static constexpr uint32_t mask = SR_UDR;
    };

    class CHSIDE
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->SR & mask;
      }

    private:
      static constexpr uint32_t mask = SR_CHSIDE;
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

    class RXNE
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->SR & mask;
      }

    private:
      static constexpr uint32_t mask = SR_RXNE;
    };
  }

  /*------------------------------------------------
  Data Register
  ------------------------------------------------*/
  namespace DR
  {
    static constexpr uint32_t resetValue = DR_Rst;
    
    static inline uint32_t get( const RegisterMap *const periph )
    {
      return periph->DR & DR_Msk;
    }

    static inline void set( RegisterMap *const periph, const uint32_t val )
    {
      periph->DR = val & DR_Msk;
    }
  }

  /*------------------------------------------------
  CRC Polynomial Register
  ------------------------------------------------*/
  namespace CRCPR
  {
    static constexpr uint32_t resetValue = CRCPR_Rst;

    static inline uint32_t get( const RegisterMap *const periph )
    {
      return periph->CRCPR & CRCPR_Msk;
    }

    static inline void set( RegisterMap *const periph, const uint32_t val )
    {
      periph->CRCPR = val & CRCPR_Msk;
    }
  }

  /*------------------------------------------------
  RX CRC Register
  ------------------------------------------------*/
  namespace RXCRCR
  {
    static constexpr uint32_t resetValue = RXCRCR_Rst;

    static inline uint32_t get( const RegisterMap *const periph )
    {
      return periph->RXCRCR & RXCRCR_Msk;
    }

    static inline void set( RegisterMap *const periph, const uint32_t val )
    {
      periph->RXCRCR = val & RXCRCR_Msk;
    }
  }

  /*------------------------------------------------
  TX CRC Register
  ------------------------------------------------*/
  namespace TXCRCR
  {
    static constexpr uint32_t resetValue = TXCRCR_Rst;

    static inline uint32_t get( const RegisterMap *const periph )
    {
      return periph->TXCRCR & TXCRCR_Msk;
    }

    static inline void set( RegisterMap *const periph, const uint32_t val )
    {
      periph->TXCRCR = val & TXCRCR_Msk;
    }
  }

  /*------------------------------------------------
  I2S Configuration Register
  ------------------------------------------------*/
  namespace I2SCFGR
  {
    static constexpr uint32_t resetValue = I2SCFGR_Rst;

    static inline uint32_t get( const RegisterMap *const periph )
    {
      return periph->I2SCFGR & I2SCFGR_Msk;
    }

    static inline void set( RegisterMap *const periph, const uint32_t val )
    {
      periph->I2SCFGR = val & I2SCFGR_Msk;
    }

    class ASTREN
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->I2SCFGR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->I2SCFGR;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->I2SCFGR = tmp;
      }

    private:
      static constexpr uint32_t mask = I2SCFGR_ASTREN;
    };

    class I2SMOD
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->I2SCFGR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->I2SCFGR;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->I2SCFGR = tmp;
      }

    private:
      static constexpr uint32_t mask = I2SCFGR_I2SMOD;
    };

    class I2SE
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->I2SCFGR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->I2SCFGR;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->I2SCFGR = tmp;
      }

    private:
      static constexpr uint32_t mask = I2SCFGR_I2SE;
    };

    class I2SCFG
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->I2SCFGR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->I2SCFGR;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->I2SCFGR = tmp;
      }

    private:
      static constexpr uint32_t mask = I2SCFGR_I2SCFG;
    };

    class PCMSYNC
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->I2SCFGR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->I2SCFGR;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->I2SCFGR = tmp;
      }

    private:
      static constexpr uint32_t mask = I2SCFGR_PCMSYNC;
    };

    class I2SSTD
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->I2SCFGR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->I2SCFGR;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->I2SCFGR = tmp;
      }

    private:
      static constexpr uint32_t mask = I2SCFGR_I2SSTD;
    };

    class CKPOL
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->I2SCFGR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->I2SCFGR;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->I2SCFGR = tmp;
      }

    private:
      static constexpr uint32_t mask = I2SCFGR_CKPOL;
    };

    class DATLEN
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->I2SCFGR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->I2SCFGR;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->I2SCFGR = tmp;
      }

    private:
      static constexpr uint32_t mask = I2SCFGR_DATLEN;
    };

    class CHLEN
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->I2SCFGR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->I2SCFGR;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->I2SCFGR = tmp;
      }

    private:
      static constexpr uint32_t mask = I2SCFGR_CHLEN;
    };
  }

  /*------------------------------------------------
  I2S Prescale Register
  ------------------------------------------------*/
  namespace I2SPR
  {
    static constexpr uint32_t resetValue = I2SPR_Rst;

    static inline uint32_t get( const RegisterMap *const periph )
    {
      return periph->I2SPR & I2SPR_Msk;
    }

    static inline void set( RegisterMap *const periph, const uint32_t val )
    {
      periph->I2SPR = val & I2SPR_Msk;
    }

    class MCKOE
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->I2SPR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->I2SPR;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->I2SPR = tmp;
      }

    private:
      static constexpr uint32_t mask = I2SPR_MCKOE;
    };

    class ODD
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->I2SPR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->I2SPR;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->I2SPR = tmp;
      }

    private:
      static constexpr uint32_t mask = I2SPR_ODD;
    };

    class DIV
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->I2SPR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t val )
      {
        uint32_t tmp = periph->I2SPR;
        tmp &= ~mask;
        tmp |= val & mask;
        periph->I2SPR = tmp;
      }

    private:
      static constexpr uint32_t mask = I2SPR_DIV;
    };
  }

}    // namespace Thor::Driver::SPI

#endif /* TARGET_STM32F4 && THOR_DRIVER_SPI */
#endif /* THOR_HW_SPI_REGISTER_HPP */