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
  Stream x Configuration (SxCR)
  -------------------------------------------------*/
  REG_ACCESSOR( StreamMap, CR, SxCR_CHSEL_Msk, CHSEL, BIT_ACCESS_RW );


}    // namespace Thor::LLD::DMA

#endif /* !THOR_HW_DMA_TYPES_HPP */