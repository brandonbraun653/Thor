/********************************************************************************
 *   File Name:
 *    hw_dma_types.hpp
 *
 *   Description:
 *    STM32 Driver for the DMA Peripheral
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_DMA_TYPES_HPP
#define THOR_HW_DMA_TYPES_HPP

/* C++ Includes */
#include <cstdint>

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/f4/dma/hw_dma_prj.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_DMA == 1 )

namespace Thor::Driver::DMA
{
  struct StreamX
  {
    volatile uint32_t CR;   /**< DMA stream x configuration register     , Address offset: 0x10 + 0x18 × stream number */
    volatile uint32_t NDTR; /**< DMA stream x number of data register    , Address offset: 0x14 + 0x18 × stream number */
    volatile uint32_t PAR;  /**< DMA stream x peripheral address register, Address offset: 0x18 + 0x18 × stream number */
    volatile uint32_t M0AR; /**< DMA stream x memory 0 address register  , Address offset: 0x1C + 0x18 × stream number */
    volatile uint32_t M1AR; /**< DMA stream x memory 1 address register  , Address offset: 0x20 + 0x18 × stream number */
    volatile uint32_t FCR;  /**< DMA stream x FIFO control register      , Address offset: 0x24 + 0x18 × stream number */
  };

  struct RegisterMap
  {
    volatile uint32_t LISR;  /**< DMA low interrupt status register,      Address offset: 0x00 */
    volatile uint32_t HISR;  /**< DMA high interrupt status register,     Address offset: 0x04 */
    volatile uint32_t LIFCR; /**< DMA low interrupt flag clear register,  Address offset: 0x08 */
    volatile uint32_t HIFCR; /**< DMA high interrupt flag clear register, Address offset: 0x0C */
    StreamX STREAM0;         /**< DMA Stream control registers, Address offset: 0x18 x stream number */
    StreamX STREAM1;         /**< DMA Stream control registers, Address offset: 0x18 x stream number */
    StreamX STREAM2;         /**< DMA Stream control registers, Address offset: 0x18 x stream number */
    StreamX STREAM3;         /**< DMA Stream control registers, Address offset: 0x18 x stream number */
    StreamX STREAM4;         /**< DMA Stream control registers, Address offset: 0x18 x stream number */
    StreamX STREAM5;         /**< DMA Stream control registers, Address offset: 0x18 x stream number */
    StreamX STREAM6;         /**< DMA Stream control registers, Address offset: 0x18 x stream number */
    StreamX STREAM7;         /**< DMA Stream control registers, Address offset: 0x18 x stream number */
  };

  static RegisterMap *const DMA1_PERIPH = reinterpret_cast<RegisterMap *const>( DMA1_BASE_ADDR );
  static RegisterMap *const DMA2_PERIPH = reinterpret_cast<RegisterMap *const>( DMA2_BASE_ADDR );

  static StreamX *const DMA1_STREAM0 = reinterpret_cast<StreamX *const>( DMA1_STREAM0_BASE_ADDR );
  static StreamX *const DMA1_STREAM1 = reinterpret_cast<StreamX *const>( DMA1_STREAM1_BASE_ADDR );
  static StreamX *const DMA1_STREAM2 = reinterpret_cast<StreamX *const>( DMA1_STREAM2_BASE_ADDR );
  static StreamX *const DMA1_STREAM3 = reinterpret_cast<StreamX *const>( DMA1_STREAM3_BASE_ADDR );
  static StreamX *const DMA1_STREAM4 = reinterpret_cast<StreamX *const>( DMA1_STREAM4_BASE_ADDR );
  static StreamX *const DMA1_STREAM5 = reinterpret_cast<StreamX *const>( DMA1_STREAM5_BASE_ADDR );
  static StreamX *const DMA1_STREAM6 = reinterpret_cast<StreamX *const>( DMA1_STREAM6_BASE_ADDR );
  static StreamX *const DMA1_STREAM7 = reinterpret_cast<StreamX *const>( DMA1_STREAM7_BASE_ADDR );

  static StreamX *const DMA2_STREAM0 = reinterpret_cast<StreamX *const>( DMA2_STREAM0_BASE_ADDR );
  static StreamX *const DMA2_STREAM1 = reinterpret_cast<StreamX *const>( DMA2_STREAM1_BASE_ADDR );
  static StreamX *const DMA2_STREAM2 = reinterpret_cast<StreamX *const>( DMA2_STREAM2_BASE_ADDR );
  static StreamX *const DMA2_STREAM3 = reinterpret_cast<StreamX *const>( DMA2_STREAM3_BASE_ADDR );
  static StreamX *const DMA2_STREAM4 = reinterpret_cast<StreamX *const>( DMA2_STREAM4_BASE_ADDR );
  static StreamX *const DMA2_STREAM5 = reinterpret_cast<StreamX *const>( DMA2_STREAM5_BASE_ADDR );
  static StreamX *const DMA2_STREAM6 = reinterpret_cast<StreamX *const>( DMA2_STREAM6_BASE_ADDR );
  static StreamX *const DMA2_STREAM7 = reinterpret_cast<StreamX *const>( DMA2_STREAM7_BASE_ADDR );

  static inline StreamX *const getStream( RegisterMap *const periph, const uint32_t streamNum )
  {
    return reinterpret_cast<StreamX*const>( periph + ( offsetof( RegisterMap, STREAM0 ) * streamNum ) );
  }

  static inline const StreamX *const getStream( const RegisterMap *const periph, const uint32_t streamNum )
  {
    return reinterpret_cast<const StreamX*const>( periph + ( offsetof( RegisterMap, STREAM0 ) * streamNum ) );
  }

  /*------------------------------------------------
  Configuration Options
  ------------------------------------------------*/
  namespace Configuration
  {
    
  }

  /*------------------------------------------------
  State Machine
  ------------------------------------------------*/

  /*------------------------------------------------
  Low Interrupt Status Register
  ------------------------------------------------*/
  namespace LISR
  {
    static inline uint32_t get( const RegisterMap *const periph )
    {
      return periph->LISR & LISR_Msk;
    }

    class TCIF3
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->LISR & LISR_TCIF3;
      }
    };

    class TCIF2
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->LISR & LISR_TCIF2;
      }
    };

    class TCIF1
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->LISR & LISR_TCIF1;
      }
    };

    class TCIF0
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->LISR & LISR_TCIF0;
      }
    };

    class HTIF3
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->LISR & LISR_HTIF3;
      }
    };

    class HTIF2
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->LISR & LISR_HTIF2;
      }
    };

    class HTIF1
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->LISR & LISR_HTIF1;
      }
    };

    class HTIF0
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->LISR & LISR_HTIF0;
      }
    };

    class TEIF3
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->LISR & LISR_TEIF3;
      }
    };

    class TEIF2
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->LISR & LISR_TEIF2;
      }
    };

    class TEIF1
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->LISR & LISR_TEIF1;
      }
    };

    class TEIF0
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->LISR & LISR_TEIF0;
      }
    };

    class DMEIF3
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->LISR & LISR_DMEIF3;
      }
    };

    class DMEIF2
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->LISR & LISR_DMEIF2;
      }
    };

    class DMEIF1
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->LISR & LISR_DMEIF1;
      }
    };

    class DMEIF0
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->LISR & LISR_DMEIF0;
      }
    };

    class FEIF3
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->LISR & LISR_FEIF3;
      }
    };

    class FEIF2
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->LISR & LISR_FEIF2;
      }
    };

    class FEIF1
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->LISR & LISR_FEIF1;
      }
    };

    class FEIF0
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->LISR & LISR_FEIF0;
      }
    };
  }    // namespace LISR

  /*------------------------------------------------
  High Interrupt Status Register
  ------------------------------------------------*/
  namespace HISR
  {
    static inline uint32_t get( const RegisterMap *const periph )
    {
      return periph->HISR & HISR_Msk;
    }

    class TCIF7
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->HISR & HISR_TCIF7;
      }
    };

    class TCIF6
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->HISR & HISR_TCIF6;
      }
    };

    class TCIF5
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->HISR & HISR_TCIF5;
      }
    };

    class TCIF4
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->HISR & HISR_TCIF4;
      }
    };

    class HTIF7
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->HISR & HISR_HTIF7;
      }
    };

    class HTIF6
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->HISR & HISR_HTIF6;
      }
    };

    class HTIF5
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->HISR & HISR_HTIF5;
      }
    };

    class HTIF4
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->HISR & HISR_HTIF4;
      }
    };

    class TEIF7
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->HISR & HISR_TEIF7;
      }
    };

    class TEIF6
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->HISR & HISR_TEIF6;
      }
    };

    class TEIF5
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->HISR & HISR_TEIF5;
      }
    };

    class TEIF4
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->HISR & HISR_TEIF4;
      }
    };

    class DMEIF7
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->HISR & HISR_DMEIF7;
      }
    };

    class DMEIF6
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->HISR & HISR_DMEIF6;
      }
    };

    class DMEIF5
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->HISR & HISR_DMEIF5;
      }
    };

    class DMEIF4
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->HISR & HISR_DMEIF4;
      }
    };

    class FEIF7
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->HISR & HISR_FEIF7;
      }
    };

    class FEIF6
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->HISR & HISR_FEIF6;
      }
    };

    class FEIF5
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->HISR & HISR_FEIF5;
      }
    };

    class FEIF4
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->HISR & HISR_FEIF4;
      }
    };
  }    // namespace HISR

  /*------------------------------------------------
  Low Interrupt Flag Clear Register
  ------------------------------------------------*/
  namespace LIFCR
  {
    static inline void set( RegisterMap *const periph, const uint32_t val )
    {
      periph->LIFCR = val & LIFCR_Msk;
    }

    class CTCIF3
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->LIFCR & LIFCR_CTCIF3;
      }
    };

    class CTCIF2
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->LIFCR & LIFCR_CTCIF2;
      }
    };

    class CTCIF1
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->LIFCR & LIFCR_CTCIF1;
      }
    };

    class CTCIF0
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->LIFCR & LIFCR_CTCIF0;
      }
    };

    class CHTIF3
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->LIFCR & LIFCR_CHTIF3;
      }
    };

    class CHTIF2
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->LIFCR & LIFCR_CHTIF2;
      }
    };

    class CHTIF1
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->LIFCR & LIFCR_CHTIF1;
      }
    };

    class CHTIF0
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->LIFCR & LIFCR_CHTIF0;
      }
    };

    class CTEIF3
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->LIFCR & LIFCR_CTEIF3;
      }
    };

    class CTEIF2
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->LIFCR & LIFCR_CTEIF2;
      }
    };

    class CTEIF1
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->LIFCR & LIFCR_CTEIF1;
      }
    };

    class CTEIF0
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->LIFCR & LIFCR_CTEIF0;
      }
    };

    class CDMEIF3
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->LIFCR & LIFCR_CDMEIF3;
      }
    };

    class CDMEIF2
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->LIFCR & LIFCR_CDMEIF2;
      }
    };

    class CDMEIF1
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->LIFCR & LIFCR_CDMEIF1;
      }
    };

    class CDMEIF0
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->LIFCR & LIFCR_CDMEIF0;
      }
    };

    class CFEIF3
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->LIFCR & LIFCR_CFEIF3;
      }
    };

    class CFEIF2
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->LIFCR & LIFCR_CFEIF2;
      }
    };

    class CFEIF1
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->LIFCR & LIFCR_CFEIF1;
      }
    };

    class CFEIF0
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->LIFCR & LIFCR_CFEIF0;
      }
    };
  }    // namespace LIFCR

  /*------------------------------------------------
  High Interrupt Flag Clear Register
  ------------------------------------------------*/
  namespace HIFCR
  {
    static inline void set( RegisterMap *const periph, const uint32_t val )
    {
      periph->HIFCR = val & HIFCR_Msk;
    }

    class CTCIF7
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->HIFCR & HIFCR_CTCIF7;
      }
    };

    class CTCIF6
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->HIFCR & HIFCR_CTCIF6;
      }
    };

    class CTCIF5
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->HIFCR & HIFCR_CTCIF5;
      }
    };

    class CTCIF4
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->HIFCR & HIFCR_CTCIF4;
      }
    };

    class CHTIF7
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->HIFCR & HIFCR_CHTIF7;
      }
    };

    class CHTIF6
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->HIFCR & HIFCR_CHTIF6;
      }
    };

    class CHTIF5
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->HIFCR & HIFCR_CHTIF5;
      }
    };

    class CHTIF4
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->HIFCR & HIFCR_CHTIF4;
      }
    };

    class CTEIF7
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->HIFCR & HIFCR_CTEIF7;
      }
    };

    class CTEIF6
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->HIFCR & HIFCR_CTEIF6;
      }
    };

    class CTEIF5
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->HIFCR & HIFCR_CTEIF5;
      }
    };

    class CTEIF4
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->HIFCR & HIFCR_CTEIF4;
      }
    };

    class CDMEIF7
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->HIFCR & HIFCR_CDMEIF7;
      }
    };

    class CDMEIF6
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->HIFCR & HIFCR_CDMEIF6;
      }
    };

    class CDMEIF5
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->HIFCR & HIFCR_CDMEIF5;
      }
    };

    class CDMEIF4
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->HIFCR & HIFCR_CDMEIF4;
      }
    };

    class CFEIF7
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->HIFCR & HIFCR_CFEIF7;
      }
    };

    class CFEIF6
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->HIFCR & HIFCR_CFEIF6;
      }
    };

    class CFEIF5
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->HIFCR & HIFCR_CFEIF5;
      }
    };

    class CFEIF4
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph )
      {
        return periph->HIFCR & HIFCR_CFEIF4;
      }
    };
  }    // namespace HIFCR

  /*------------------------------------------------
  Stream X Configuration Register
  ------------------------------------------------*/
  namespace SxCR
  {
    static inline uint32_t get( const RegisterMap *const periph, const uint32_t streamNum )
    {
      auto streamPtr = getStream( periph, streamNum );
      return streamPtr->CR & SxCR_Msk;
    }

    static inline void set( RegisterMap *const periph, const uint32_t streamNum, const uint32_t val )
    {
      auto streamPtr  = getStream( periph, streamNum );
      streamPtr->CR = val & SxCR_Msk;
    }

    class CHSEL
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph, const uint32_t streamNum )
      {
        auto streamPtr = getStream( periph, streamNum );
        return streamPtr->CR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t streamNum, const uint32_t val )
      {
        auto streamPtr  = getStream( periph, streamNum );
        streamPtr->CR = val & mask;
      }

    private:
      static constexpr uint32_t mask = SxCR_CHSEL;
    };

    class MBURST
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph, const uint32_t streamNum )
      {
        auto streamPtr = getStream( periph, streamNum );
        return streamPtr->CR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t streamNum, const uint32_t val )
      {
        auto streamPtr  = getStream( periph, streamNum );
        streamPtr->CR = val & mask;
      }

    private:
      static constexpr uint32_t mask = SxCR_MBURST;
    };

    class PBURST
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph, const uint32_t streamNum )
      {
        auto streamPtr = getStream( periph, streamNum );
        return streamPtr->CR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t streamNum, const uint32_t val )
      {
        auto streamPtr  = getStream( periph, streamNum );
        streamPtr->CR = val & mask;
      }

    private:
      static constexpr uint32_t mask = SxCR_PBURST;
    };

    class CT
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph, const uint32_t streamNum )
      {
        auto streamPtr = getStream( periph, streamNum );
        return streamPtr->CR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t streamNum, const uint32_t val )
      {
        auto streamPtr  = getStream( periph, streamNum );
        streamPtr->CR = val & mask;
      }

    private:
      static constexpr uint32_t mask = SxCR_CT;
    };
      
    class DBM
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph, const uint32_t streamNum )
      {
        auto streamPtr = getStream( periph, streamNum );
        return streamPtr->CR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t streamNum, const uint32_t val )
      {
        auto streamPtr  = getStream( periph, streamNum );
        streamPtr->CR = val & mask;
      }

    private:
      static constexpr uint32_t mask = SxCR_DBM;
    };

    class PL
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph, const uint32_t streamNum )
      {
        auto streamPtr = getStream( periph, streamNum );
        return streamPtr->CR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t streamNum, const uint32_t val )
      {
        auto streamPtr  = getStream( periph, streamNum );
        streamPtr->CR = val & mask;
      }

    private:
      static constexpr uint32_t mask = SxCR_PL;
    };

    class PINCOS
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph, const uint32_t streamNum )
      {
        auto streamPtr = getStream( periph, streamNum );
        return streamPtr->CR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t streamNum, const uint32_t val )
      {
        auto streamPtr  = getStream( periph, streamNum );
        streamPtr->CR = val & mask;
      }

    private:
      static constexpr uint32_t mask = SxCR_PINCOS;
    };

    class MSIZE
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph, const uint32_t streamNum )
      {
        auto streamPtr = getStream( periph, streamNum );
        return streamPtr->CR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t streamNum, const uint32_t val )
      {
        auto streamPtr  = getStream( periph, streamNum );
        streamPtr->CR = val & mask;
      }

    private:
      static constexpr uint32_t mask = SxCR_MSIZE;
    };

    class PSIZE
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph, const uint32_t streamNum )
      {
        auto streamPtr = getStream( periph, streamNum );
        return streamPtr->CR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t streamNum, const uint32_t val )
      {
        auto streamPtr  = getStream( periph, streamNum );
        streamPtr->CR = val & mask;
      }

    private:
      static constexpr uint32_t mask = SxCR_PSIZE;
    };

    class MINC
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph, const uint32_t streamNum )
      {
        auto streamPtr = getStream( periph, streamNum );
        return streamPtr->CR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t streamNum, const uint32_t val )
      {
        auto streamPtr  = getStream( periph, streamNum );
        streamPtr->CR = val & mask;
      }

    private:
      static constexpr uint32_t mask = SxCR_MINC;
    };

    class PINC
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph, const uint32_t streamNum )
      {
        auto streamPtr = getStream( periph, streamNum );
        return streamPtr->CR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t streamNum, const uint32_t val )
      {
        auto streamPtr  = getStream( periph, streamNum );
        streamPtr->CR = val & mask;
      }

    private:
      static constexpr uint32_t mask = SxCR_PINC;
    };

    class CIRC
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph, const uint32_t streamNum )
      {
        auto streamPtr = getStream( periph, streamNum );
        return streamPtr->CR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t streamNum, const uint32_t val )
      {
        auto streamPtr  = getStream( periph, streamNum );
        streamPtr->CR = val & mask;
      }

    private:
      static constexpr uint32_t mask = SxCR_CIRC;
    };

    class DIR
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph, const uint32_t streamNum )
      {
        auto streamPtr = getStream( periph, streamNum );
        return streamPtr->CR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t streamNum, const uint32_t val )
      {
        auto streamPtr  = getStream( periph, streamNum );
        streamPtr->CR = val & mask;
      }

    private:
      static constexpr uint32_t mask = SxCR_DIR;
    };

    class PFCTRL
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph, const uint32_t streamNum )
      {
        auto streamPtr = getStream( periph, streamNum );
        return streamPtr->CR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t streamNum, const uint32_t val )
      {
        auto streamPtr  = getStream( periph, streamNum );
        streamPtr->CR = val & mask;
      }

    private:
      static constexpr uint32_t mask = SxCR_PFCTRL;
    };

    class TCIE
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph, const uint32_t streamNum )
      {
        auto streamPtr = getStream( periph, streamNum );
        return streamPtr->CR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t streamNum, const uint32_t val )
      {
        auto streamPtr  = getStream( periph, streamNum );
        streamPtr->CR = val & mask;
      }

    private:
      static constexpr uint32_t mask = SxCR_TCIE;
    };

    class HTIE
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph, const uint32_t streamNum )
      {
        auto streamPtr = getStream( periph, streamNum );
        return streamPtr->CR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t streamNum, const uint32_t val )
      {
        auto streamPtr  = getStream( periph, streamNum );
        streamPtr->CR = val & mask;
      }

    private:
      static constexpr uint32_t mask = SxCR_HTIE;
    };

    class TEIE
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph, const uint32_t streamNum )
      {
        auto streamPtr = getStream( periph, streamNum );
        return streamPtr->CR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t streamNum, const uint32_t val )
      {
        auto streamPtr  = getStream( periph, streamNum );
        streamPtr->CR = val & mask;
      }

    private:
      static constexpr uint32_t mask = SxCR_TEIE;
    };
      
    class DMEIE
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph, const uint32_t streamNum )
      {
        auto streamPtr = getStream( periph, streamNum );
        return streamPtr->CR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t streamNum, const uint32_t val )
      {
        auto streamPtr  = getStream( periph, streamNum );
        streamPtr->CR = val & mask;
      }

    private:
      static constexpr uint32_t mask = SxCR_DMEIE;
    };

    class EN
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph, const uint32_t streamNum )
      {
        auto streamPtr = getStream( periph, streamNum );
        return streamPtr->CR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t streamNum, const uint32_t val )
      {
        auto streamPtr  = getStream( periph, streamNum );
        streamPtr->CR = val & mask;
      }

    private:
      static constexpr uint32_t mask = SxCR_EN;
    };
  }

  /*------------------------------------------------
  Stream X Number of Data Register
  ------------------------------------------------*/
  namespace SxNDTR
  {
    static inline uint32_t get( const RegisterMap *const periph, const uint32_t streamNum )
    {
      auto streamPtr = getStream( periph, streamNum );
      return streamPtr->NDTR & SxNDT_Msk;
    }

    static inline void set( RegisterMap *const periph, const uint32_t streamNum, const uint32_t val )
    {
      auto streamPtr  = getStream( periph, streamNum );
      streamPtr->NDTR = val & SxNDT_Msk;
    }
  }    // namespace SxNDTR

  /*------------------------------------------------
  Stream X Peripheral Address Register
  ------------------------------------------------*/
  namespace SxPAR
  {
    static inline uint32_t get( const RegisterMap *const periph, const uint32_t streamNum )
    {
      auto streamPtr = getStream( periph, streamNum );
      return streamPtr->PAR & SxPAR_Msk;
    }

    static inline void set( RegisterMap *const periph, const uint32_t streamNum, const uint32_t val )
    {
      auto streamPtr  = getStream( periph, streamNum );
      streamPtr->PAR = val & SxPAR_Msk;
    }
  }

  /*------------------------------------------------
  Stream X Memory 0 Address Register
  ------------------------------------------------*/
  namespace SxM0AR
  {
    static inline uint32_t get( const RegisterMap *const periph, const uint32_t streamNum )
    {
      auto streamPtr = getStream( periph, streamNum );
      return streamPtr->M0AR & SxM0AR_Msk;
    }

    static inline void set( RegisterMap *const periph, const uint32_t streamNum, const uint32_t val )
    {
      auto streamPtr  = getStream( periph, streamNum );
      streamPtr->M0AR = val & SxM0AR_Msk;
    }
  }

  /*------------------------------------------------
  Stream X Memory 1 Address Register
  ------------------------------------------------*/
  namespace SxM1AR
  {
    static inline uint32_t get( const RegisterMap *const periph, const uint32_t streamNum )
    {
      auto streamPtr = getStream( periph, streamNum );
      return streamPtr->M1AR & SxM1AR_Msk;
    }

    static inline void set( RegisterMap *const periph, const uint32_t streamNum, const uint32_t val )
    {
      auto streamPtr  = getStream( periph, streamNum );
      streamPtr->M1AR = val & SxM1AR_Msk;
    }
  }

  /*------------------------------------------------
  Stream X FIFO Control Register
  ------------------------------------------------*/
  namespace SxFCR
  {
    static inline uint32_t get( const RegisterMap *const periph, const uint32_t streamNum )
    {
      auto streamPtr = getStream( periph, streamNum );
      return streamPtr->FCR & SxFCR_Msk;
    }

    static inline void set( RegisterMap *const periph, const uint32_t streamNum, const uint32_t val )
    {
      auto streamPtr  = getStream( periph, streamNum );
      streamPtr->FCR = val & SxFCR_Msk;
    }

    class FEIE
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph, const uint32_t streamNum )
      {
        auto streamPtr = getStream( periph, streamNum );
        return streamPtr->FCR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t streamNum, const uint32_t val )
      {
        auto streamPtr  = getStream( periph, streamNum );
        streamPtr->FCR = val & mask;
      }

    private:
      static constexpr uint32_t mask = SxFCR_FEIE;
    };

    class FS
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph, const uint32_t streamNum )
      {
        auto streamPtr = getStream( periph, streamNum );
        return streamPtr->FCR & mask;
      }

    private:
      static constexpr uint32_t mask = SxFCR_FS;
    };

    class DMDIS
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph, const uint32_t streamNum )
      {
        auto streamPtr = getStream( periph, streamNum );
        return streamPtr->FCR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t streamNum, const uint32_t val )
      {
        auto streamPtr  = getStream( periph, streamNum );
        streamPtr->FCR = val & mask;
      }

    private:
      static constexpr uint32_t mask = SxFCR_DMDIS;
    };

    class FTH
    {
    public:
      static inline uint32_t get( const RegisterMap *const periph, const uint32_t streamNum )
      {
        auto streamPtr = getStream( periph, streamNum );
        return streamPtr->FCR & mask;
      }

      static inline void set( RegisterMap *const periph, const uint32_t streamNum, const uint32_t val )
      {
        auto streamPtr  = getStream( periph, streamNum );
        streamPtr->FCR = val & mask;
      }

    private:
      static constexpr uint32_t mask = SxFCR_FTH;
    };

  }    // namespace SxFCR

}    // namespace Thor::Driver::DMA

#endif /* TARGET_STM32F4 && THOR_DRIVER_DMA */
#endif /* !THOR_HW_DMA_TYPES_HPP */