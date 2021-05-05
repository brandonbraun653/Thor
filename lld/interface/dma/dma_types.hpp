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

/* ETL Includes */
#include <etl/delegate.h>

/* Chimera Includes */
#include <Chimera/dma>

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
  static constexpr uint8_t ON_DMA1 = ( 1u << 0 );
  static constexpr uint8_t ON_DMA2 = ( 1u << 1 );

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
  /**
   * @brief Enumerates available DMA controllers
   */
  enum class Controller : uint8_t
  {
    DMA_1,
    DMA_2,

    NUM_OPTIONS,
    NONE
  };

  /**
   * @brief A stream that could be connected to a DMA controller
   */
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

    NUM_OPTIONS,
    NONE
  };

  /**
   * @brief A channel that could be connected to a DMA stream
   */
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
   * @brief Interrupts that a stream can generate
   */
  enum class Interrupt : uint8_t
  {
    HALF_TRANSFER_COMPLETE,
    FULL_TRANSFER_COMPLETE,
    ERROR_TRANSFER,
    ERROR_FIFO_UNDER_OVER_RUN,
    ERROR_DIRECT_MODE,

    NUM_OPTIONS
  };

  /**
   * @brief Selects the FIFO operational mode
   */
  enum class FifoMode : uint8_t
  {
    DIRECT_ENABLE,
    DIRECT_DISABLE,

    NUM_OPTIONS
  };

  /**
   * @brief Selects the threshold which will flush the FIFO
   */
  enum class FifoThreshold : uint8_t
  {
    QUARTER_FULL,
    HALF_FULL,
    THREE_QUARTER_FULL,
    FULL,

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
  Aliases
  -------------------------------------------------------------------------------*/
  using ISRCallback = etl::delegate<void( Interrupt )>;

  /*-------------------------------------------------------------------------------
  Structures
  -------------------------------------------------------------------------------*/
  /**
   * @brief Helper struct for mapping a DMA request signal
   */
  struct StreamAttr
  {
    Source request;     /**< DMA request signal */
    uint8_t attributes; /**< Signal attributes */
  };


  /**
   * @brief Configures a stream for a single transfer
   */
  struct StreamConfig
  {
    /*-------------------------------------------------
    Thor Specific Configuration
    -------------------------------------------------*/
    bool periphAddrIncr;
    bool memoryAddrIncr;
    Channel channel;
    FifoMode fifoMode;
    FifoThreshold fifoThreshold;

    /*-------------------------------------------------
    Common Driver Configuration
    -------------------------------------------------*/
    Chimera::DMA::Mode dmaMode;
    Chimera::DMA::Direction direction;
    Chimera::DMA::Priority priority;
    Chimera::DMA::BurstSize periphBurstSize;
    Chimera::DMA::Alignment periphAddrAlign;
    Chimera::DMA::BurstSize memoryBurstSize;
    Chimera::DMA::Alignment memoryAddrAlign;
  };

  /**
   * @brief Transfer control block
   * Keeps track of a single DMA transfer operating on a stream
   */
  struct TCB
  {
    /*-------------------------------------------------
    Control Fields
    -------------------------------------------------*/
    uint32_t srcAddress;       /**< Address where the data will be pulled from */
    uint32_t dstAddress;       /**< Address where the data will be transferred into */
    uint32_t transferSize;     /**< How many bytes to transfer between source and destination */
    uint32_t bytesTransferred; /**< How many bytes were actually transferred */
    uint32_t transferState;    /**< DMA transfer state machine status */
    uint32_t selectedChannel;  /**< When the ISR fires, will contain hardware channel that was used */
    uint32_t requestGenerator; /**< When the ISR fires, will contain the peripheral that generated the event */

    /*-------------------------------------------------
    Various Error Flags
    -------------------------------------------------*/
    bool fifoError;
    bool directModeError;
    bool transferError;
  };
}    // namespace Thor::LLD::DMA

#endif /* !THOR_DRIVER_DMA_COMMON_TYPES_HPP */