/********************************************************************************
 *  File Name:
 *    hw_dma_types.hpp
 *
 *  Description:
 *    STM32 Driver for the DMA Peripheral
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_DMA_TYPES_HPP
#define THOR_HW_DMA_TYPES_HPP

/* C++ Includes */
#include <cstdint>
#include <array>

/* Driver Includes */
#include <Thor/lld/stm32f4x/dma/hw_dma_prj.hpp>
#include <Thor/lld/common/registers/field_accessor.hpp>

namespace Thor::LLD::DMA
{
  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
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
      static constexpr uint32_t Channef4 = ( 4u << SxCR_CHSEL_Pos ) & SxCR_Msk;
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

    namespace Direction
    {
      static constexpr uint32_t INVALID = 0;
      static constexpr uint32_t P2M     = ( 0u << SxCR_DIR_Pos ) & SxCR_DIR_Msk;
      static constexpr uint32_t M2P     = ( 1u << SxCR_DIR_Pos ) & SxCR_DIR_Msk;
      static constexpr uint32_t M2M     = ( 2u << SxCR_DIR_Pos ) & SxCR_DIR_Msk;
    }    // namespace Direction

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

  /*-------------------------------------------------------------------------------
  Structures
  -------------------------------------------------------------------------------*/
  struct StreamMap
  {
    volatile uint32_t CR;   /**< DMA stream x configuration register     , Address offset: 0x10 + 0x18 x stream number */
    volatile uint32_t NDTR; /**< DMA stream x number of data register    , Address offset: 0x14 + 0x18 x stream number */
    volatile uint32_t PAR;  /**< DMA stream x peripheral address register, Address offset: 0x18 + 0x18 x stream number */
    volatile uint32_t M0AR; /**< DMA stream x memory 0 address register  , Address offset: 0x1C + 0x18 x stream number */
    volatile uint32_t M1AR; /**< DMA stream x memory 1 address register  , Address offset: 0x20 + 0x18 x stream number */
    volatile uint32_t FCR;  /**< DMA stream x FIFO control register      , Address offset: 0x24 + 0x18 x stream number */
  };

  struct RegisterMap
  {
    volatile uint32_t LISR;  /**< DMA low interrupt status register,      Address offset: 0x00 */
    volatile uint32_t HISR;  /**< DMA high interrupt status register,     Address offset: 0x04 */
    volatile uint32_t LIFCR; /**< DMA low interrupt flag clear register,  Address offset: 0x08 */
    volatile uint32_t HIFCR; /**< DMA high interrupt flag clear register, Address offset: 0x0C */
    StreamMap STREAM0;       /**< DMA Stream control registers, Address offset: 0x18 x stream number */
    StreamMap STREAM1;       /**< DMA Stream control registers, Address offset: 0x18 x stream number */
    StreamMap STREAM2;       /**< DMA Stream control registers, Address offset: 0x18 x stream number */
    StreamMap STREAM3;       /**< DMA Stream control registers, Address offset: 0x18 x stream number */
    StreamMap STREAM4;       /**< DMA Stream control registers, Address offset: 0x18 x stream number */
    StreamMap STREAM5;       /**< DMA Stream control registers, Address offset: 0x18 x stream number */
    StreamMap STREAM6;       /**< DMA Stream control registers, Address offset: 0x18 x stream number */
    StreamMap STREAM7;       /**< DMA Stream control registers, Address offset: 0x18 x stream number */
  };

  /*-------------------------------------------------------------------------------
  Register Classes
  -------------------------------------------------------------------------------*/
  /*-------------------------------------------------
  Low Interrupt Status (LISR)
  -------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, LISR, LISR_TCIF3, TCIF3, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, LISR, LISR_HTIF3, HTIF3, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, LISR, LISR_TEIF3, TEIF3, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, LISR, LISR_DMEIF3, DMEIF3, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, LISR, LISR_FEIF3, FEIF3, BIT_ACCESS_R );

  REG_ACCESSOR( RegisterMap, LISR, LISR_TCIF2, TCIF2, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, LISR, LISR_HTIF2, HTIF2, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, LISR, LISR_TEIF2, TEIF2, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, LISR, LISR_DMEIF2, DMEIF2, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, LISR, LISR_FEIF2, FEIF2, BIT_ACCESS_R );

  REG_ACCESSOR( RegisterMap, LISR, LISR_TCIF1, TCIF1, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, LISR, LISR_HTIF1, HTIF1, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, LISR, LISR_TEIF1, TEIF1, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, LISR, LISR_DMEIF1, DMEIF1, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, LISR, LISR_FEIF1, FEIF1, BIT_ACCESS_R );

  REG_ACCESSOR( RegisterMap, LISR, LISR_TCIF0, TCIF0, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, LISR, LISR_HTIF0, HTIF0, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, LISR, LISR_TEIF0, TEIF0, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, LISR, LISR_DMEIF0, DMEIF0, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, LISR, LISR_FEIF0, FEIF0, BIT_ACCESS_R );

  /*-------------------------------------------------
  High Interrupt Status (HISR)
  -------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, HISR, HISR_TCIF7, TCIF7, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, HISR, HISR_HTIF7, HTIF7, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, HISR, HISR_TEIF7, TEIF7, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, HISR, HISR_DMEIF7, DMEIF7, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, HISR, HISR_FEIF7, FEIF7, BIT_ACCESS_R );

  REG_ACCESSOR( RegisterMap, HISR, HISR_TCIF6, TCIF6, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, HISR, HISR_HTIF6, HTIF6, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, HISR, HISR_TEIF6, TEIF6, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, HISR, HISR_DMEIF6, DMEIF6, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, HISR, HISR_FEIF6, FEIF6, BIT_ACCESS_R );

  REG_ACCESSOR( RegisterMap, HISR, HISR_TCIF5, TCIF5, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, HISR, HISR_HTIF5, HTIF5, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, HISR, HISR_TEIF5, TEIF5, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, HISR, HISR_DMEIF5, DMEIF5, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, HISR, HISR_FEIF5, FEIF5, BIT_ACCESS_R );

  REG_ACCESSOR( RegisterMap, HISR, HISR_TCIF4, TCIF4, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, HISR, HISR_HTIF4, HTIF4, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, HISR, HISR_TEIF4, TEIF4, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, HISR, HISR_DMEIF4, DMEIF4, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, HISR, HISR_FEIF4, FEIF4, BIT_ACCESS_R );

  /*-------------------------------------------------
  Low Interrupt Flag Clear (LIFCR)
  -------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, LIFCR, LIFCR_CTCIF3, CTCIF3, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, LIFCR, LIFCR_CHTIF3, CHTIF3, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, LIFCR, LIFCR_CTEIF3, CTEIF3, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, LIFCR, LIFCR_CDMEIF3, CDMEIF3, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, LIFCR, LIFCR_CFEIF3, CFEIF3, BIT_ACCESS_W );

  REG_ACCESSOR( RegisterMap, LIFCR, LIFCR_CTCIF2, CTCIF2, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, LIFCR, LIFCR_CHTIF2, CHTIF2, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, LIFCR, LIFCR_CTEIF2, CTEIF2, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, LIFCR, LIFCR_CDMEIF2, CDMEIF2, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, LIFCR, LIFCR_CFEIF2, CFEIF2, BIT_ACCESS_W );

  REG_ACCESSOR( RegisterMap, LIFCR, LIFCR_CTCIF1, CTCIF1, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, LIFCR, LIFCR_CHTIF1, CHTIF1, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, LIFCR, LIFCR_CTEIF1, CTEIF1, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, LIFCR, LIFCR_CDMEIF1, CDMEIF1, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, LIFCR, LIFCR_CFEIF1, CFEIF1, BIT_ACCESS_W );

  REG_ACCESSOR( RegisterMap, LIFCR, LIFCR_CTCIF0, CTCIF0, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, LIFCR, LIFCR_CHTIF0, CHTIF0, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, LIFCR, LIFCR_CTEIF0, CTEIF0, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, LIFCR, LIFCR_CDMEIF0, CDMEIF0, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, LIFCR, LIFCR_CFEIF0, CFEIF0, BIT_ACCESS_W );

  /*-------------------------------------------------
  High Interrupt Flag Clear (HIFCR)
  -------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, HIFCR, HIFCR_CTCIF7, CTCIF7, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, HIFCR, HIFCR_CHTIF7, CHTIF7, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, HIFCR, HIFCR_CTEIF7, CTEIF7, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, HIFCR, HIFCR_CDMEIF7, CDMEIF7, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, HIFCR, HIFCR_CFEIF7, CFEIF7, BIT_ACCESS_W );

  REG_ACCESSOR( RegisterMap, HIFCR, HIFCR_CTCIF6, CTCIF6, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, HIFCR, HIFCR_CHTIF6, CHTIF6, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, HIFCR, HIFCR_CTEIF6, CTEIF6, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, HIFCR, HIFCR_CDMEIF6, CDMEIF6, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, HIFCR, HIFCR_CFEIF6, CFEIF6, BIT_ACCESS_W );

  REG_ACCESSOR( RegisterMap, HIFCR, HIFCR_CTCIF5, CTCIF5, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, HIFCR, HIFCR_CHTIF5, CHTIF5, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, HIFCR, HIFCR_CTEIF5, CTEIF5, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, HIFCR, HIFCR_CDMEIF5, CDMEIF5, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, HIFCR, HIFCR_CFEIF5, CFEIF5, BIT_ACCESS_W );

  REG_ACCESSOR( RegisterMap, HIFCR, HIFCR_CTCIF4, CTCIF4, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, HIFCR, HIFCR_CHTIF4, CHTIF4, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, HIFCR, HIFCR_CTEIF4, CTEIF4, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, HIFCR, HIFCR_CDMEIF4, CDMEIF4, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, HIFCR, HIFCR_CFEIF4, CFEIF4, BIT_ACCESS_W );

  /*-------------------------------------------------
  Stream x Configuration (SxCR)
  -------------------------------------------------*/
  REG_ACCESSOR( StreamMap, CR, SxCR_CHSEL, CHSEL, BIT_ACCESS_RW );
  REG_ACCESSOR( StreamMap, CR, SxCR_MBURST, MBURST, BIT_ACCESS_RW );
  REG_ACCESSOR( StreamMap, CR, SxCR_PBURST, PBURST, BIT_ACCESS_RW );
  REG_ACCESSOR( StreamMap, CR, SxCR_CT, CT, BIT_ACCESS_RW );
  REG_ACCESSOR( StreamMap, CR, SxCR_DBM, DBM, BIT_ACCESS_RW );
  REG_ACCESSOR( StreamMap, CR, SxCR_PL, PL, BIT_ACCESS_RW );
  REG_ACCESSOR( StreamMap, CR, SxCR_PINCOS, PINCOS, BIT_ACCESS_RW );
  REG_ACCESSOR( StreamMap, CR, SxCR_MSIZE, MSIZE, BIT_ACCESS_RW );
  REG_ACCESSOR( StreamMap, CR, SxCR_PSIZE, PSIZE, BIT_ACCESS_RW );
  REG_ACCESSOR( StreamMap, CR, SxCR_MINC, MINC, BIT_ACCESS_RW );
  REG_ACCESSOR( StreamMap, CR, SxCR_PINC, PINC, BIT_ACCESS_RW );
  REG_ACCESSOR( StreamMap, CR, SxCR_CIRC, CIRC, BIT_ACCESS_RW );
  REG_ACCESSOR( StreamMap, CR, SxCR_DIR, DIR, BIT_ACCESS_RW );
  REG_ACCESSOR( StreamMap, CR, SxCR_PFCTRL, PFCTRL, BIT_ACCESS_RW );
  REG_ACCESSOR( StreamMap, CR, SxCR_TCIE, TCIE, BIT_ACCESS_RW );
  REG_ACCESSOR( StreamMap, CR, SxCR_HTIE, HTIE, BIT_ACCESS_RW );
  REG_ACCESSOR( StreamMap, CR, SxCR_TEIE, TEIE, BIT_ACCESS_RW );
  REG_ACCESSOR( StreamMap, CR, SxCR_DMEIE, DMEIE, BIT_ACCESS_RW );
  REG_ACCESSOR( StreamMap, CR, SxCR_EN, EN, BIT_ACCESS_RW );

  /*-------------------------------------------------
  Stream x Number of Data (SxNDTR)
  -------------------------------------------------*/
  REG_ACCESSOR( StreamMap, NDTR, SxNDT, NDT, BIT_ACCESS_RW );

  /*-------------------------------------------------
  Stream x Peripheral Address (SxPAR)
  -------------------------------------------------*/
  REG_ACCESSOR( StreamMap, PAR, SxPAR_PA, PA, BIT_ACCESS_RW );

  /*-------------------------------------------------
  Stream x Memory 0 Address (SxM0AR)
  -------------------------------------------------*/
  REG_ACCESSOR( StreamMap, M0AR, SxM0AR_M0A, M0A, BIT_ACCESS_RW );

  /*-------------------------------------------------
  Stream x Memory 1 Address (SxM1AR)
  -------------------------------------------------*/
  REG_ACCESSOR( StreamMap, M1AR, SxM1AR_M1A, M1A, BIT_ACCESS_RW );

  /*-------------------------------------------------
  Stream x FIFO Control (SxFCR)
  -------------------------------------------------*/
  REG_ACCESSOR( StreamMap, FCR, SxFCR_FEIE, FEIE, BIT_ACCESS_RW );
  REG_ACCESSOR( StreamMap, FCR, SxFCR_FS, FS, BIT_ACCESS_R );
  REG_ACCESSOR( StreamMap, FCR, SxFCR_DMDIS, DMDIS, BIT_ACCESS_RW );
  REG_ACCESSOR( StreamMap, FCR, SxFCR_FTH, FTH, BIT_ACCESS_RW );
}    // namespace Thor::LLD::DMA

#endif /* !THOR_HW_DMA_TYPES_HPP */
