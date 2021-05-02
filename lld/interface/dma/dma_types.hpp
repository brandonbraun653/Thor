/********************************************************************************
 *  File Name:
 *    dma_types.hpp
 *
 *  Description:
 *    Common DMA types used in Thor drivers
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_DRIVER_DMA_COMMON_TYPES_HPP
#define THOR_DRIVER_DMA_COMMON_TYPES_HPP

/* C++ Includes */
#include <cstdint>

/* Chimera Includes */
#include <Chimera/event>

namespace Thor::LLD::DMA
{
  /*-------------------------------------------------------------------------------
  Forward Declarations
  -------------------------------------------------------------------------------*/
  class Driver;
  class Stream;

  /*-------------------------------------------------------------------------------
  Aliases
  -------------------------------------------------------------------------------*/
  using Driver_rPtr = Driver *;
  using Stream_rPtr = Stream *;

  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  static constexpr uint8_t ON_DMA1  = ( 1u << 0 );
  static constexpr uint8_t ON_DMA2  = ( 1u << 1 );

  static constexpr uint8_t ON_CHANNEL_POS = 2u;
  static constexpr uint8_t ON_CHANNEL_MSK = ( 0x7 << ON_CHANNEL_POS );
  static constexpr uint8_t ON_CHANNEL_0   = ( 0u << ON_CHANNEL_POS ) & ON_CHANNEL_MSK;
  static constexpr uint8_t ON_CHANNEL_1   = ( 1u << ON_CHANNEL_POS ) & ON_CHANNEL_MSK;
  static constexpr uint8_t ON_CHANNEL_2   = ( 2u << ON_CHANNEL_POS ) & ON_CHANNEL_MSK;
  static constexpr uint8_t ON_CHANNEL_3   = ( 3u << ON_CHANNEL_POS ) & ON_CHANNEL_MSK;
  static constexpr uint8_t ON_CHANNEL_4   = ( 4u << ON_CHANNEL_POS ) & ON_CHANNEL_MSK;
  static constexpr uint8_t ON_CHANNEL_5   = ( 5u << ON_CHANNEL_POS ) & ON_CHANNEL_MSK;
  static constexpr uint8_t ON_CHANNEL_6   = ( 6u << ON_CHANNEL_POS ) & ON_CHANNEL_MSK;
  static constexpr uint8_t ON_CHANNEL_7   = ( 7u << ON_CHANNEL_POS ) & ON_CHANNEL_MSK;

  static constexpr uint8_t ON_STREAM_POS = 5u;
  static constexpr uint8_t ON_STREAM_MSK = ( 0x7 << ON_STREAM_POS );
  static constexpr uint8_t ON_STREAM_0   = ( 0u << ON_STREAM_POS ) & ON_STREAM_MSK;
  static constexpr uint8_t ON_STREAM_1   = ( 1u << ON_STREAM_POS ) & ON_STREAM_MSK;
  static constexpr uint8_t ON_STREAM_2   = ( 2u << ON_STREAM_POS ) & ON_STREAM_MSK;
  static constexpr uint8_t ON_STREAM_3   = ( 3u << ON_STREAM_POS ) & ON_STREAM_MSK;
  static constexpr uint8_t ON_STREAM_4   = ( 4u << ON_STREAM_POS ) & ON_STREAM_MSK;
  static constexpr uint8_t ON_STREAM_5   = ( 5u << ON_STREAM_POS ) & ON_STREAM_MSK;
  static constexpr uint8_t ON_STREAM_6   = ( 6u << ON_STREAM_POS ) & ON_STREAM_MSK;
  static constexpr uint8_t ON_STREAM_7   = ( 7u << ON_STREAM_POS ) & ON_STREAM_MSK;


  /*-------------------------------------------------------------------------------
  Enumerations
  -------------------------------------------------------------------------------*/
  enum class Controller : uint8_t
  {
    DMA_1,
    DMA_2,

    NUM_OPTIONS
  };


  enum class Streamer : uint8_t
  {
    STREAM_0,
    STREAM_1,
    STREAM_2,
    STREAM_3,
    STREAM_4,
    STREAM_5,
    STREAM_6,
    STREAM_7,

    NUM_OPTIONS
  };


  enum class Channel : uint8_t
  {
    CHANNEL_0,
    CHANNEL_1,
    CHANNEL_2,
    CHANNEL_3,
    CHANNEL_4,
    CHANNEL_5,
    CHANNEL_6,
    CHANNEL_7,

    NUM_OPTIONS

  };


  /**
   * @brief All known DMA sources, but not all are supported by each device.
   */
  enum class Source : uint8_t
  {
    ADC1,
    ADC2,
    ADC3,
    DAC1,
    DAC2,
    DCMI,
    FMPI2C1_RX,
    I2C1_TX,
    I2C1_RX,
    I2C2_TX,
    I2C2_RX,
    I2C3_TX,
    I2C3_RX,
    SPDIFRX_DT,
    SPDIFRX_CS,
    SPI1_TX,
    SPI1_RX,
    SPI2_TX,
    SPI2_RX,
    SPI3_TX,
    SPI3_RX,
    SPI4_TX,
    SPI4_RX,
    QUADSPI,
    SAI1_A,
    SAI1_B,
    SAI2_A,
    SAI2_B,
    SDIO,
    TIM1_UP,
    TIM1_TRIG,
    TIM1_COM,
    TIM1_CH1,
    TIM1_CH2,
    TIM1_CH3,
    TIM1_CH4,
    TIM2_UP,
    TIM2_CH1,
    TIM2_CH2,
    TIM2_CH3,
    TIM2_CH4,
    TIM3_UP,
    TIM3_TRIG,
    TIM3_CH1,
    TIM3_CH2,
    TIM3_CH3,
    TIM3_CH4,
    TIM4_UP,
    TIM4_CH1,
    TIM4_CH2,
    TIM4_CH3,
    TIM5_UP,
    TIM5_TRIG,
    TIM5_CH1,
    TIM5_CH2,
    TIM5_CH3,
    TIM5_CH4,
    TIM6_UP,
    TIM7_UP,
    TIM8_UP,
    TIM8_TRIG,
    TIM8_COM,
    TIM8_CH4,
    TIM8_CH1,
    TIM8_CH2,
    TIM8_CH3,
    UART4_TX,
    UART4_RX,
    UART5_TX,
    UART5_RX,
    USART1_TX,
    USART1_RX,
    USART2_TX,
    USART2_RX,
    USART3_TX,
    USART3_RX,
    USART6_TX,
    USART6_RX,

    NUM_DMA_SOURCES,
    NONE
  };

  /*-------------------------------------------------------------------------------
  Structures
  -------------------------------------------------------------------------------*/
  struct StreamAttr
  {
    const Source src;
    const uint8_t attr;
  };


  /**
   *  Configuration structure for the Serial peripherals (UART/USART).
   *  Each member is expected to be equal to the exact value needed to
   *  configure the appropriate control register. The calculation of these
   *  values is left up to the hardware driver as this might vary from
   *  chip to chip. Expect that these values will be writen directly to
   *  a register without much translation or protection.
   */
  struct StreamConfig
  {
    /*------------------------------------------------
    Specifies the channel used for the specified stream.

    Can be a value of Thor::LLD::DMA::Configuration::ChannelSelect
    ------------------------------------------------*/
    uint32_t Channel;

    /*------------------------------------------------
    Specifies if the data will be transferred from memory
    to peripheral, from memory to memory or from peripheral to memory.

    Can be a value of Thor::LLD::DMA::Configuration::Direction
    ------------------------------------------------*/
    uint32_t Direction;

    /*------------------------------------------------
    Specifies whether the Peripheral address register should be incremented or not

    Can be a value of Thor::LLD::DMA::Configuration::PeriphIncrementMode
    ------------------------------------------------*/
    uint32_t PeriphInc;

    /*------------------------------------------------
    Specifies whether the memory address register should be incremented or not.

    Can be a value of Thor::LLD::DMA::Configuration::MemoryIncrementMode
    ------------------------------------------------*/
    uint32_t MemInc;

    /*------------------------------------------------
    Specifies the Peripheral data width.

    Can be a value of Thor::LLD::DMA::Configuration::PeriphDataSize
    ------------------------------------------------*/
    uint32_t PeriphDataAlignment;

    /*------------------------------------------------
    Specifies the Memory data width

    Can be a value of Thor::LLD::DMA::Configuration::MemoryDataSize
    ------------------------------------------------*/
    uint32_t MemDataAlignment;

    /*------------------------------------------------
    Specifies the operation mode of the stream.

    @note The circular buffer mode cannot be used if the memory-to-memory
          data transfer is configured on the selected Stream

    Can be a value of Thor::LLD::DMA::Configuration::Mode
    ------------------------------------------------*/
    uint32_t Mode;

    /*------------------------------------------------
    Specifies the software priority for the transfer
    Can be a value of Thor::LLD::DMA::Configuration::PriorityLevel
    ------------------------------------------------*/
    uint32_t Priority;

    /*------------------------------------------------
    Specifies if the FIFO mode or Direct mode will be used for the specified stream.

    @note The Direct mode (FIFO mode disabled) cannot be used if the
          memory-to-memory data transfer is configured on the selected stream

    Can be a value of Thor::LLD::DMA::Configuration::FIFODirectMode
    ------------------------------------------------*/
    uint32_t FIFOMode;

    /*------------------------------------------------
    Specifies the FIFO threshold level.

    Can be a value of Thor::LLD::DMA::Configuration::FIFOThreshold
    ------------------------------------------------*/
    uint32_t FIFOThreshold;

    /*------------------------------------------------
    Specifies the Burst transfer configuration for the memory transfers.
    It specifies the amount of data to be transferred in a single non interruptible
    transaction.

    @note The burst mode is possible only if the address Increment mode is enabled.

    Can be a value of Thor::LLD::DMA::Configuration::MemoryBurst
    ------------------------------------------------*/
    uint32_t MemBurst;

    /*------------------------------------------------
    Specifies the Burst transfer configuration for the peripheral transfers.
    It specifies the amount of data to be transferred in a single non interruptible
    transaction.

    @note The burst mode is possible only if the address Increment mode is enabled.

    Can be a value of Thor::LLD::DMA::Configuration::PeriphBurst
    ------------------------------------------------*/
    uint32_t PeriphBurst;
  };


  struct TCB
  {
    uint32_t srcAddress;       /**< Address where the data will be pulled from */
    uint32_t dstAddress;       /**< Address where the data will be transfered into */
    uint32_t transferSize;     /**< How many bytes to transfer between source and destination */
    uint32_t bytesTransfered;  /**< How many bytes were actually transfered */
    uint32_t transferState;    /**< DMA transfer state machine status */
    uint32_t selectedChannel;  /**< When the ISR fires, will contain hardware channel that was used */
    uint32_t requestGenerator; /**< When the ISR fires, will contain the peripheral that generated the event */

    bool fifoError;
    bool directModeError;
    bool transferError;

    void clear()
    {
      srcAddress       = 0u;
      dstAddress       = 0u;
      transferSize     = 0u;
      bytesTransfered  = 0u;
      transferState    = 0u;
      selectedChannel  = 0u;
      requestGenerator = 0u;
      fifoError        = false;
      directModeError  = false;
      transferError    = false;
    }
  };
}    // namespace Thor::LLD::DMA

#endif /* !THOR_DRIVER_DMA_COMMON_TYPES_HPP */