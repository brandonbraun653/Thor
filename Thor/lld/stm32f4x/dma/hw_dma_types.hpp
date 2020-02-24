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
#include <array>

/* Driver Includes */
#include <Thor/lld/stm32f4x/dma/hw_dma_prj.hpp>

namespace Thor::LLD::DMA
{
  struct StreamX
  {
    volatile uint32_t CR;   /**< DMA stream x configuration register     , Address offset: 0x10 + 0x18 � stream number */
    volatile uint32_t NDTR; /**< DMA stream x number of data register    , Address offset: 0x14 + 0x18 � stream number */
    volatile uint32_t PAR;  /**< DMA stream x peripheral address register, Address offset: 0x18 + 0x18 � stream number */
    volatile uint32_t M0AR; /**< DMA stream x memory 0 address register  , Address offset: 0x1C + 0x18 � stream number */
    volatile uint32_t M1AR; /**< DMA stream x memory 1 address register  , Address offset: 0x20 + 0x18 � stream number */
    volatile uint32_t FCR;  /**< DMA stream x FIFO control register      , Address offset: 0x24 + 0x18 � stream number */
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

  class Driver;

  using DriverInstanceList = std::array<Driver *, NUM_DMA_PERIPHS>;
  using PeriphRegisterList = std::array<RegisterMap *, NUM_DMA_PERIPHS>;
  using StreamRegisterList = std::array<StreamX *, NUM_DMA_STREAMS>;


  static inline StreamX *const getStreamRegisters( RegisterMap *const periph, const size_t streamNum )
  {
    /*------------------------------------------------
    This equation taken directly from register spec in datasheet.
    See 9.5.5 in RM0390
    ------------------------------------------------*/
#if defined( EMBEDDED )
    static constexpr size_t fixedOffset = 0x10;
    static constexpr size_t streamOffset = 0x18;

    auto address = reinterpret_cast<std::uintptr_t>( periph ) + fixedOffset + ( streamOffset * streamNum );
    return reinterpret_cast<StreamX *const>( address );

#elif defined( _SIM )
    auto start = reinterpret_cast<std::uintptr_t>( periph );
    auto stream0Offset = offsetof( RegisterMap, STREAM0 );
    auto streamXSize   = sizeof( StreamX );

    auto address = start + stream0Offset + ( streamNum * streamXSize );
    return reinterpret_cast<StreamX *const>( address );
#endif 
  }

  static inline const StreamX *const getStreamRegisters( const RegisterMap *const periph, const size_t streamNum )
  {
    /*------------------------------------------------
    This equation taken directly from register spec in datasheet.
    See 9.5.5 in RM0390
    ------------------------------------------------*/
    static constexpr size_t fixedOffset  = 0x10;
    static constexpr size_t streamOffset = 0x18;

    auto address = reinterpret_cast<std::uintptr_t>( periph ) + fixedOffset + ( streamOffset * streamNum );
    return reinterpret_cast<const StreamX *const>( address );
  }

  /*------------------------------------------------
  Configuration Options
  ------------------------------------------------*/
  namespace Configuration
  {
    namespace Mode
    {
      static constexpr uint32_t Normal   = 0u;
      static constexpr uint32_t Periph   = SxCR_PFCTRL;
      static constexpr uint32_t Circular = SxCR_CIRC;
    }    // namespace Mode

    namespace ChannelSelect
    {
      static constexpr uint32_t Channel0 = ( 0u << SxCR_CHSEL_Pos ) & SxCR_Msk;
      static constexpr uint32_t Channel1 = ( 1u << SxCR_CHSEL_Pos ) & SxCR_Msk;
      static constexpr uint32_t Channel2 = ( 2u << SxCR_CHSEL_Pos ) & SxCR_Msk;
      static constexpr uint32_t Channel3 = ( 3u << SxCR_CHSEL_Pos ) & SxCR_Msk;
      static constexpr uint32_t Channel4 = ( 4u << SxCR_CHSEL_Pos ) & SxCR_Msk;
      static constexpr uint32_t Channel5 = ( 5u << SxCR_CHSEL_Pos ) & SxCR_Msk;
      static constexpr uint32_t Channel6 = ( 6u << SxCR_CHSEL_Pos ) & SxCR_Msk;
      static constexpr uint32_t Channel7 = ( 7u << SxCR_CHSEL_Pos ) & SxCR_Msk;
    }    // namespace ChannelSelect

    namespace MemoryBurst
    {
      static constexpr uint32_t Single = ( 0u << SxCR_MBURST_Pos ) & SxCR_MBURST_Msk;
      static constexpr uint32_t Incr4  = ( 1u << SxCR_MBURST_Pos ) & SxCR_MBURST_Msk;
      static constexpr uint32_t Incr8  = ( 2u << SxCR_MBURST_Pos ) & SxCR_MBURST_Msk;
      static constexpr uint32_t Incr16 = ( 3u << SxCR_MBURST_Pos ) & SxCR_MBURST_Msk;
    }    // namespace MemoryBurst

    namespace PeriphBurst
    {
      static constexpr uint32_t Single = ( 0u << SxCR_PBURST_Pos ) & SxCR_PBURST_Msk;
      static constexpr uint32_t Incr4  = ( 1u << SxCR_PBURST_Pos ) & SxCR_PBURST_Msk;
      static constexpr uint32_t Incr8  = ( 2u << SxCR_PBURST_Pos ) & SxCR_PBURST_Msk;
      static constexpr uint32_t Incr16 = ( 3u << SxCR_PBURST_Pos ) & SxCR_PBURST_Msk;
    }    // namespace PeriphBurst

    namespace CurrentTarget
    {
      static constexpr uint32_t Memory0 = 0;
      static constexpr uint32_t Memory1 = SxCR_CT;
    }    // namespace CurrentTarget

    namespace DoubleBufferMode
    {
      static constexpr uint32_t NoSwitch = 0;
      static constexpr uint32_t Switch   = SxCR_DBM;
    }    // namespace DoubleBufferMode

    namespace PriorityLevel
    {
      static constexpr uint32_t Low    = ( 0u << SxCR_PL_Pos ) & SxCR_PL_Msk;
      static constexpr uint32_t Medium = ( 1u << SxCR_PL_Pos ) & SxCR_PL_Msk;
      static constexpr uint32_t High   = ( 2u << SxCR_PL_Pos ) & SxCR_PL_Msk;
      static constexpr uint32_t Ultra  = ( 3u << SxCR_PL_Pos ) & SxCR_PL_Msk;
    }    // namespace PriorityLevel

    namespace PeriphIncOffset
    {
      static constexpr uint32_t PSIZE  = 0;
      static constexpr uint32_t Fixed4 = SxCR_PINCOS;
    }    // namespace PeriphIncOffset

    namespace MemoryDataSize
    {
      static constexpr uint32_t Byte     = ( 0u << SxCR_MSIZE_Pos ) & SxCR_MSIZE_Msk;
      static constexpr uint32_t HalfWord = ( 1u << SxCR_MSIZE_Pos ) & SxCR_MSIZE_Msk;
      static constexpr uint32_t Word     = ( 2u << SxCR_MSIZE_Pos ) & SxCR_MSIZE_Msk;
    }    // namespace MemoryDataSize

    namespace PeriphDataSize
    {
      static constexpr uint32_t Byte     = ( 0u << SxCR_PSIZE_Pos ) & SxCR_PSIZE_Msk;
      static constexpr uint32_t HalfWord = ( 1u << SxCR_PSIZE_Pos ) & SxCR_PSIZE_Msk;
      static constexpr uint32_t Word     = ( 2u << SxCR_PSIZE_Pos ) & SxCR_PSIZE_Msk;
    }    // namespace PeriphDataSize

    namespace MemoryIncrementMode
    {
      static constexpr uint32_t Fixed     = 0;
      static constexpr uint32_t Increment = SxCR_MINC;
    }    // namespace MemoryIncrementMode

    namespace PeriphIncrementMode
    {
      static constexpr uint32_t Fixed     = 0;
      static constexpr uint32_t Increment = SxCR_PINC;
    }    // namespace PeriphIncrementMode

    namespace TransferDirection
    {
      static constexpr uint32_t INVALID = 0;
      static constexpr uint32_t P2M     = ( 0u << SxCR_DIR_Pos ) & SxCR_DIR_Msk;
      static constexpr uint32_t M2P     = ( 1u << SxCR_DIR_Pos ) & SxCR_DIR_Msk;
      static constexpr uint32_t M2M     = ( 2u << SxCR_DIR_Pos ) & SxCR_DIR_Msk;
    }    // namespace TransferDirection

    namespace FIFODirectMode
    {
      static constexpr uint32_t Enabled  = 0;
      static constexpr uint32_t Disabled = SxFCR_DMDIS;
    }    // namespace FIFODirectMode

    namespace FIFOThreshold
    {
      static constexpr uint32_t Threshold_1_4 = ( 0u << SxFCR_FTH_Pos ) & SxFCR_FTH_Msk;
      static constexpr uint32_t Threshold_1_2 = ( 1u << SxFCR_FTH_Pos ) & SxFCR_FTH_Msk;
      static constexpr uint32_t Threshold_3_4 = ( 2u << SxFCR_FTH_Pos ) & SxFCR_FTH_Msk;
      static constexpr uint32_t Threshold_4_4 = ( 3u << SxFCR_FTH_Pos ) & SxFCR_FTH_Msk;
    }    // namespace FIFOThreshold

  }    // namespace Configuration

  /*------------------------------------------------
  Runtime
  ------------------------------------------------*/
  namespace Runtime
  {
    using Flag_t = uint32_t;
    namespace Flag
    {
      static constexpr Flag_t TRANSFER_COMPLETE      = ( 1u << 0 );
      static constexpr Flag_t TRANSFER_HALF_COMPLETE = ( 1u << 1 );
      static constexpr Flag_t TRANSFER_ERROR         = ( 1u << 2 );
      static constexpr Flag_t DIRECT_MODE_ERROR      = ( 1u << 3 );
      static constexpr Flag_t FIFO_ERROR             = ( 1u << 4 );
      static constexpr Flag_t TRANSFER_IN_PROGRESS   = ( 1u << 5 );
      static constexpr Flag_t TRANSFER_READY         = ( 1u << 6 );
      static constexpr Flag_t TRANSFER_NOT_READY     = ( 1u << 7 );
    }    // namespace Flag
  }      // namespace Runtime

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
      // No need to read-modify-write as state is not kept
      periph->LIFCR = val & LIFCR_Msk;
    }

    static inline void setStream0( RegisterMap *const periph )
    {
      uint32_t STREAMx_BITS_PATTERN = LIFCR_CFEIF0 | LIFCR_CDMEIF0 | LIFCR_CTEIF0 | LIFCR_CHTIF0 | LIFCR_CTCIF0;
      set( periph, STREAMx_BITS_PATTERN );
    }

    static inline void setStream1( RegisterMap *const periph )
    {
      uint32_t STREAMx_BITS_PATTERN = ( LIFCR_CFEIF0 | LIFCR_CDMEIF0 | LIFCR_CTEIF0 | LIFCR_CHTIF0 | LIFCR_CTCIF0 ) << 6u;
      set( periph, STREAMx_BITS_PATTERN );
    }

    static inline void setStream2( RegisterMap *const periph )
    {
      uint32_t STREAMx_BITS_PATTERN = ( LIFCR_CFEIF0 | LIFCR_CDMEIF0 | LIFCR_CTEIF0 | LIFCR_CHTIF0 | LIFCR_CTCIF0 ) << 16u;
      set( periph, STREAMx_BITS_PATTERN );
    }

    static inline void setStream3( RegisterMap *const periph )
    {
      uint32_t STREAMx_BITS_PATTERN = ( LIFCR_CFEIF0 | LIFCR_CDMEIF0 | LIFCR_CTEIF0 | LIFCR_CHTIF0 | LIFCR_CTCIF0 ) << 22u;
      set( periph, STREAMx_BITS_PATTERN );
    }

    static inline void setStreamX( RegisterMap *const periph, const size_t stream )
    {
      switch ( stream )
      {
        case 0:
          setStream0( periph );
          break;

        case 1:
          setStream1( periph );
          break;

        case 2:
          setStream2( periph );
          break;

        case 3:
          setStream3( periph );
          break;

        default:
          break;
      }
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
      // No need to read-modify-write as state is not kept
      periph->HIFCR = val & HIFCR_Msk;
    }

    static inline void setStream4( RegisterMap *const periph )
    {
      uint32_t STREAMx_BITS_PATTERN = HIFCR_CFEIF4 | HIFCR_CDMEIF4 | HIFCR_CTEIF4 | HIFCR_CHTIF4 | HIFCR_CTCIF4;
      set( periph, STREAMx_BITS_PATTERN );
    }

    static inline void setStream5( RegisterMap *const periph )
    {
      uint32_t STREAMx_BITS_PATTERN = ( HIFCR_CFEIF4 | HIFCR_CDMEIF4 | HIFCR_CTEIF4 | HIFCR_CHTIF4 | HIFCR_CTCIF4 ) << 6u;
      set( periph, STREAMx_BITS_PATTERN );
    }

    static inline void setStream6( RegisterMap *const periph )
    {
      uint32_t STREAMx_BITS_PATTERN = ( HIFCR_CFEIF4 | HIFCR_CDMEIF4 | HIFCR_CTEIF4 | HIFCR_CHTIF4 | HIFCR_CTCIF4 ) << 16u;
      set( periph, STREAMx_BITS_PATTERN );
    }

    static inline void setStream7( RegisterMap *const periph )
    {
      uint32_t STREAMx_BITS_PATTERN = ( HIFCR_CFEIF4 | HIFCR_CDMEIF4 | HIFCR_CTEIF4 | HIFCR_CHTIF4 | HIFCR_CTCIF4 ) << 22u;
      set( periph, STREAMx_BITS_PATTERN );
    }

    static inline void setStreamX( RegisterMap *const periph, const size_t stream )
    {
      switch ( stream )
      {
        case 4:
          setStream4( periph );
          break;

        case 5:
          setStream5( periph );
          break;

        case 6:
          setStream6( periph );
          break;

        case 7:
          setStream7( periph );
          break;

        default:
          break;
      }
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
    static inline uint32_t get( const StreamX *const stream )
    {
      return stream->CR & SxCR_Msk;
    }

    static inline void set( StreamX *const stream, const uint32_t val )
    {
      auto tmp = stream->CR;
      tmp &= ~SxCR_Msk;
      tmp |= val & SxCR_Msk;
      stream->CR = tmp;
    }

    class CHSEL
    {
    public:
      static inline uint32_t get( const StreamX *const stream )
      {
        return stream->CR & mask;
      }

      static inline void set( StreamX *const stream, const uint32_t val )
      {
        auto tmp = stream->CR;
        tmp &= ~mask;
        tmp |= val & mask;
        stream->CR = tmp;
      }

    private:
      static constexpr uint32_t mask = SxCR_CHSEL;
    };

    class MBURST
    {
    public:
      static inline uint32_t get( const StreamX *const stream )
      {
        return stream->CR & mask;
      }

      static inline void set( StreamX *const stream, const uint32_t val )
      {
        auto tmp = stream->CR;
        tmp &= ~mask;
        tmp |= val & mask;
        stream->CR = tmp;
      }

    private:
      static constexpr uint32_t mask = SxCR_MBURST;
    };

    class PBURST
    {
    public:
      static inline uint32_t get( const StreamX *const stream )
      {
        return stream->CR & mask;
      }

      static inline void set( StreamX *const stream, const uint32_t val )
      {
        auto tmp = stream->CR;
        tmp &= ~mask;
        tmp |= val & mask;
        stream->CR = tmp;
      }

    private:
      static constexpr uint32_t mask = SxCR_PBURST;
    };

    class CT
    {
    public:
      static inline uint32_t get( const StreamX *const stream )
      {
        return stream->CR & mask;
      }

      static inline void set( StreamX *const stream, const uint32_t val )
      {
        auto tmp = stream->CR;
        tmp &= ~mask;
        tmp |= val & mask;
        stream->CR = tmp;
      }

    private:
      static constexpr uint32_t mask = SxCR_CT;
    };

    class DBM
    {
    public:
      static inline uint32_t get( const StreamX *const stream )
      {
        return stream->CR & mask;
      }

      static inline void set( StreamX *const stream, const uint32_t val )
      {
        auto tmp = stream->CR;
        tmp &= ~mask;
        tmp |= val & mask;
        stream->CR = tmp;
      }

    private:
      static constexpr uint32_t mask = SxCR_DBM;
    };

    class PL
    {
    public:
      static inline uint32_t get( const StreamX *const stream )
      {
        return stream->CR & mask;
      }

      static inline void set( StreamX *const stream, const uint32_t val )
      {
        auto tmp = stream->CR;
        tmp &= ~mask;
        tmp |= val & mask;
        stream->CR = tmp;
      }

    private:
      static constexpr uint32_t mask = SxCR_PL;
    };

    class PINCOS
    {
    public:
      static inline uint32_t get( const StreamX *const stream )
      {
        return stream->CR & mask;
      }

      static inline void set( StreamX *const stream, const uint32_t val )
      {
        auto tmp = stream->CR;
        tmp &= ~mask;
        tmp |= val & mask;
        stream->CR = tmp;
      }

    private:
      static constexpr uint32_t mask = SxCR_PINCOS;
    };

    class MSIZE
    {
    public:
      static inline uint32_t get( const StreamX *const stream )
      {
        return stream->CR & mask;
      }

      static inline void set( StreamX *const stream, const uint32_t val )
      {
        auto tmp = stream->CR;
        tmp &= ~mask;
        tmp |= val & mask;
        stream->CR = tmp;
      }

    private:
      static constexpr uint32_t mask = SxCR_MSIZE;
    };

    class PSIZE
    {
    public:
      static inline uint32_t get( const StreamX *const stream )
      {
        return stream->CR & mask;
      }

      static inline void set( StreamX *const stream, const uint32_t val )
      {
        auto tmp = stream->CR;
        tmp &= ~mask;
        tmp |= val & mask;
        stream->CR = tmp;
      }

    private:
      static constexpr uint32_t mask = SxCR_PSIZE;
    };

    class MINC
    {
    public:
      static inline uint32_t get( const StreamX *const stream )
      {
        return stream->CR & mask;
      }

      static inline void set( StreamX *const stream, const uint32_t val )
      {
        auto tmp = stream->CR;
        tmp &= ~mask;
        tmp |= val & mask;
        stream->CR = tmp;
      }

    private:
      static constexpr uint32_t mask = SxCR_MINC;
    };

    class PINC
    {
    public:
      static inline uint32_t get( const StreamX *const stream )
      {
        return stream->CR & mask;
      }

      static inline void set( StreamX *const stream, const uint32_t val )
      {
        auto tmp = stream->CR;
        tmp &= ~mask;
        tmp |= val & mask;
        stream->CR = tmp;
      }

    private:
      static constexpr uint32_t mask = SxCR_PINC;
    };

    class CIRC
    {
    public:
      static inline uint32_t get( const StreamX *const stream )
      {
        return stream->CR & mask;
      }

      static inline void set( StreamX *const stream, const uint32_t val )
      {
        auto tmp = stream->CR;
        tmp &= ~mask;
        tmp |= val & mask;
        stream->CR = tmp;
      }

    private:
      static constexpr uint32_t mask = SxCR_CIRC;
    };

    class DIR
    {
    public:
      static inline uint32_t get( const StreamX *const stream )
      {
        return stream->CR & mask;
      }

      static inline void set( StreamX *const stream, const uint32_t val )
      {
        auto tmp = stream->CR;
        tmp &= ~mask;
        tmp |= val & mask;
        stream->CR = tmp;
      }

    private:
      static constexpr uint32_t mask = SxCR_DIR;
    };

    class PFCTRL
    {
    public:
      static inline uint32_t get( const StreamX *const stream )
      {
        return stream->CR & mask;
      }

      static inline void set( StreamX *const stream, const uint32_t val )
      {
        auto tmp = stream->CR;
        tmp &= ~mask;
        tmp |= val & mask;
        stream->CR = tmp;
      }

    private:
      static constexpr uint32_t mask = SxCR_PFCTRL;
    };

    class TCIE
    {
    public:
      static inline uint32_t get( const StreamX *const stream )
      {
        return stream->CR & mask;
      }

      static inline void set( StreamX *const stream, const uint32_t val )
      {
        auto tmp = stream->CR;
        tmp &= ~mask;
        tmp |= val & mask;
        stream->CR = tmp;
      }

    private:
      static constexpr uint32_t mask = SxCR_TCIE;
    };

    class HTIE
    {
    public:
      static inline uint32_t get( const StreamX *const stream )
      {
        return stream->CR & mask;
      }

      static inline void set( StreamX *const stream, const uint32_t val )
      {
        auto tmp = stream->CR;
        tmp &= ~mask;
        tmp |= val & mask;
        stream->CR = tmp;
      }

    private:
      static constexpr uint32_t mask = SxCR_HTIE;
    };

    class TEIE
    {
    public:
      static inline uint32_t get( const StreamX *const stream )
      {
        return stream->CR & mask;
      }

      static inline void set( StreamX *const stream, const uint32_t val )
      {
        auto tmp = stream->CR;
        tmp &= ~mask;
        tmp |= val & mask;
        stream->CR = tmp;
      }

    private:
      static constexpr uint32_t mask = SxCR_TEIE;
    };

    class DMEIE
    {
    public:
      static inline uint32_t get( const StreamX *const stream )
      {
        return stream->CR & mask;
      }

      static inline void set( StreamX *const stream, const uint32_t val )
      {
        auto tmp = stream->CR;
        tmp &= ~mask;
        tmp |= val & mask;
        stream->CR = tmp;
      }

    private:
      static constexpr uint32_t mask = SxCR_DMEIE;
    };

    class EN
    {
    public:
      static inline uint32_t get( const StreamX *const stream )
      {
        return stream->CR & mask;
      }

      static inline void set( StreamX *const stream, const uint32_t val )
      {
        auto tmp = stream->CR;
        tmp &= ~mask;
        tmp |= val & mask;
        stream->CR = tmp;

        /*------------------------------------------------
        According to the datasheet (9.3.18), clearing this bit
        is not instant and we need to wait for it to take effect.
        This will occur when all current transfers have finished.
        ------------------------------------------------*/
        while ( get( stream ) != val ) {}
      }

    private:
      static constexpr uint32_t mask = SxCR_EN;
    };
  }    // namespace SxCR

  /*------------------------------------------------
  Stream X Number of Data Register
  ------------------------------------------------*/
  namespace SxNDTR
  {
    static constexpr uint32_t MAX_BYTES = 0xFFFF;

    static inline uint32_t get( const StreamX *const stream )
    {
      return stream->NDTR & SxNDT_Msk;
    }

    static inline void set( StreamX *const stream, const uint32_t val )
    {
      stream->NDTR = val & SxNDT_Msk;
    }
  }    // namespace SxNDTR

  /*------------------------------------------------
  Stream X Peripheral Address Register
  ------------------------------------------------*/
  namespace SxPAR
  {
    static inline uint32_t get( const StreamX *const stream )
    {
      return stream->PAR & SxPAR_Msk;
    }

    static inline void set( StreamX *const stream, const uint32_t val )
    {
      stream->PAR = val & SxPAR_Msk;
    }
  }    // namespace SxPAR

  /*------------------------------------------------
  Stream X Memory 0 Address Register
  ------------------------------------------------*/
  namespace SxM0AR
  {
    static inline uint32_t get( const StreamX *const stream )
    {
      return stream->M0AR & SxM0AR_Msk;
    }

    static inline void set( StreamX *const stream, const uint32_t val )
    {
      stream->M0AR = val & SxM0AR_Msk;
    }
  }    // namespace SxM0AR

  /*------------------------------------------------
  Stream X Memory 1 Address Register
  ------------------------------------------------*/
  namespace SxM1AR
  {
    static inline uint32_t get( const StreamX *const stream )
    {
      return stream->M1AR & SxM1AR_Msk;
    }

    static inline void set( StreamX *const stream, const uint32_t val )
    {
      stream->M1AR = val & SxM1AR_Msk;
    }
  }    // namespace SxM1AR

  /*------------------------------------------------
  Stream X FIFO Control Register
  ------------------------------------------------*/
  namespace SxFCR
  {
    static inline uint32_t get( const StreamX *const stream )
    {
      return stream->FCR & SxFCR_Msk;
    }

    static inline void set( StreamX *const stream, const uint32_t val )
    {
      auto tmp = stream->FCR;
      tmp &= ~SxFCR_Msk;
      tmp |= val & SxFCR_Msk;
      stream->FCR = tmp;
    }

    class FEIE
    {
    public:
      static inline uint32_t get( const StreamX *const stream )
      {
        return stream->FCR & mask;
      }

      static inline void set( StreamX *const stream, const uint32_t val )
      {
        auto tmp = stream->FCR;
        tmp &= ~mask;
        tmp |= val & mask;
        stream->FCR = tmp;
      }

    private:
      static constexpr uint32_t mask = SxFCR_FEIE;
    };

    class FS
    {
    public:
      static inline uint32_t get( const StreamX *const stream )
      {
        return stream->FCR & mask;
      }

    private:
      static constexpr uint32_t mask = SxFCR_FS;
    };

    class DMDIS
    {
    public:
      static inline uint32_t get( const StreamX *const stream )
      {
        return stream->FCR & mask;
      }

      static inline void set( StreamX *const stream, const uint32_t val )
      {
        auto tmp = stream->FCR;
        tmp &= ~mask;
        tmp |= val & mask;
        stream->FCR = tmp;
      }

    private:
      static constexpr uint32_t mask = SxFCR_DMDIS;
    };

    class FTH
    {
    public:
      static inline uint32_t get( const StreamX *const stream )
      {
        return stream->FCR & mask;
      }

      static inline void set( StreamX *const stream, const uint32_t val )
      {
        auto tmp = stream->FCR;
        tmp &= ~mask;
        tmp |= val & mask;
        stream->FCR = tmp;
      }

    private:
      static constexpr uint32_t mask = SxFCR_FTH;
    };

  }    // namespace SxFCR

}    // namespace Thor::LLD::DMA

#endif /* !THOR_HW_DMA_TYPES_HPP */