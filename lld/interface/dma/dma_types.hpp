/******************************************************************************
 *  File Name:
 *    dma_types.hpp
 *
 *  Description:
 *    Common DMA types used in Thor drivers
 *
 *  2019-2023 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_DRIVER_DMA_COMMON_TYPES_HPP
#define THOR_DRIVER_DMA_COMMON_TYPES_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/utility>
#include <Chimera/dma>
#include <Thor/cfg>
#include <cstdint>
#include <etl/delegate.h>
#include <etl/memory.h>
#include <etl/queue_spsc_atomic.h>


namespace Thor::LLD::DMA
{
  /*---------------------------------------------------------------------------
  Forward Declarations
  ---------------------------------------------------------------------------*/
  class Driver;
  class Stream;
  struct TCB;

  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  using Driver_rPtr   = Driver *;
  using Stream_rPtr   = Stream *;
  using ISREventQueue = etl::queue_spsc_atomic<TCB, 15, etl::memory_model::MEMORY_MODEL_SMALL>;

  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
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


  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/
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

    NUM_OPTIONS,
    INVALID

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
   * @brief Current state of the DMA stream
   *
   */

  enum class _state : uint32_t
  {
    transfer_idle,
    transfer_configured,
    transfer_in_progress,
    transfer_half_complete,
    transfer_complete,
    error,
  };

  enum class StreamState : uint32_t
  {
    TRANSFER_IDLE          = ( 1u << EnumValue( _state::transfer_idle ) ),
    TRANSFER_CONFIGURED    = ( 1u << EnumValue( _state::transfer_configured ) ),
    TRANSFER_IN_PROGRESS   = ( 1u << EnumValue( _state::transfer_in_progress ) ),
    TRANSFER_HALF_COMPLETE = ( 1u << EnumValue( _state::transfer_half_complete ) ),
    TRANSFER_COMPLETE      = ( 1u << EnumValue( _state::transfer_complete ) ),
    ERROR                  = ( 1u << EnumValue( _state::error ) )
  };
  ENUM_CLS_BITWISE_OPERATOR( StreamState, | );
  ENUM_CLS_BITWISE_OPERATOR( StreamState, & );

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
    I2C1_RX,
    I2C1_TX,
    I2C2_RX,
    I2C2_TX,
    I2C3_RX,
    I2C3_TX,
    I2C4_RX,
    I2C4_TX,
    QUADSPI,
    SAI1_A,
    SAI1_B,
    SAI2_A,
    SAI2_B,
    SDIO,
    SPDIFRX_CS,
    SPDIFRX_DT,
    SPI1_RX,
    SPI1_TX,
    SPI2_RX,
    SPI2_TX,
    SPI3_RX,
    SPI3_TX,
    SPI4_RX,
    SPI4_TX,
    TIM15_CH1,
    TIM15_COM,
    TIM15_TRIG,
    TIM15_UP,
    TIM16_CH1,
    TIM16_UP,
    TIM1_CH1,
    TIM1_CH2,
    TIM1_CH3,
    TIM1_CH4,
    TIM1_COM,
    TIM1_TRIG,
    TIM1_UP,
    TIM2_CH1,
    TIM2_CH2,
    TIM2_CH3,
    TIM2_CH4,
    TIM2_UP,
    TIM3_CH1,
    TIM3_CH2,
    TIM3_CH3,
    TIM3_CH4,
    TIM3_TRIG,
    TIM3_UP,
    TIM4_CH1,
    TIM4_CH2,
    TIM4_CH3,
    TIM4_UP,
    TIM5_CH1,
    TIM5_CH2,
    TIM5_CH3,
    TIM5_CH4,
    TIM5_TRIG,
    TIM5_UP,
    TIM6_UP,
    TIM7_UP,
    TIM8_CH1,
    TIM8_CH2,
    TIM8_CH3,
    TIM8_CH4,
    TIM8_COM,
    TIM8_TRIG,
    TIM8_UP,
    UART4_RX,
    UART4_TX,
    UART5_RX,
    UART5_TX,
    USART1_RX,
    USART1_TX,
    USART2_RX,
    USART2_TX,
    USART3_RX,
    USART3_TX,
    USART6_RX,
    USART6_TX,

    NUM_DMA_SOURCES,
    NONE
  };

  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  using ISRCallback = etl::delegate<void( Interrupt )>;

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  /**
   * @brief Helper struct for mapping a DMA request signal
   */
  struct StreamAttr
  {
    Source  request;    /**< DMA request signal */
    uint8_t attributes; /**< Signal attributes */
  };


  /**
   * @brief Configures a stream for a single transfer
   */
  struct StreamConfig
  {
    /*-------------------------------------------------------------------------
    Thor Specific Configuration
    -------------------------------------------------------------------------*/
    bool     dstAddrIncr;
    bool     srcAddrIncr;
    Channel  channel;

    /*-------------------------------------------------------------------------
    Common Driver Configuration
    -------------------------------------------------------------------------*/
    Chimera::DMA::Mode          dmaMode;
    Chimera::DMA::Direction     direction;
    Chimera::DMA::Priority      priority;
    Chimera::DMA::FifoMode      fifoMode;
    Chimera::DMA::FifoThreshold fifoThreshold;
    Chimera::DMA::BurstSize     dstBurstSize;
    Chimera::DMA::Alignment     dstAddrAlign;
    Chimera::DMA::BurstSize     srcBurstSize;
    Chimera::DMA::Alignment     srcAddrAlign;

    void clear()
    {
      dstAddrIncr   = false;
      srcAddrIncr   = false;
      channel       = Channel::INVALID;
      dmaMode       = Chimera::DMA::Mode::NUM_OPTIONS;
      direction     = Chimera::DMA::Direction::NUM_OPTIONS;
      priority      = Chimera::DMA::Priority::NUM_OPTIONS;
      fifoMode      = Chimera::DMA::FifoMode::NUM_OPTIONS;
      fifoThreshold = Chimera::DMA::FifoThreshold::NUM_OPTIONS;
      dstBurstSize  = Chimera::DMA::BurstSize::NUM_OPTIONS;
      dstAddrAlign  = Chimera::DMA::Alignment::NUM_OPTIONS;
      srcBurstSize  = Chimera::DMA::BurstSize::NUM_OPTIONS;
      srcAddrAlign  = Chimera::DMA::Alignment::NUM_OPTIONS;
    }
  };

  /**
   * @brief Transfer control block
   * Keeps track of a single DMA transfer operating on a stream
   */
  struct TCB
  {
    /*-------------------------------------------------------------------------
    Control Fields
    -------------------------------------------------------------------------*/
    /* Driver Controlled */
    Chimera::DMA::Errors    errorsToIgnore;      /**< Which errors to not care about */
    StreamState             state;               /**< DMA transfer state machine status */
    uint32_t                elementsTransferred; /**< How many bytes were actually transferred */
    uint32_t                selectedChannel;     /**< When the ISR fires, will contain hardware channel that was used */
    RIndex_t                resourceIndex;       /**< Resource index of the controlling stream */
    Chimera::DMA::RequestId requestId;           /**< HLD request ID of the transfer */
    Chimera::DMA::Alignment elementSize;         /**< Size of each element that was transferred */

    /* User Configured */
    uint32_t                       srcAddress;   /**< Address where the data will be pulled from */
    uint32_t                       dstAddress;   /**< Address where the data will be transferred into */
    uint32_t                       transferSize; /**< How many bytes to transfer between source and destination */
    bool                           persistent;   /**< Whether or not to leave the stream enabled after transfer complete */
    bool                           wakeUserOnComplete; /**< Wake the user thread to handle results? */
    Chimera::DMA::TransferCallback isrCallback;        /**< Callback to execute inside ISR on events */

    /*-------------------------------------------------------------------------
    Various Error Flags
    -------------------------------------------------------------------------*/
    bool fifoError;
    bool directModeError;
    bool transferError;

    void clear()
    {
      errorsToIgnore      = Chimera::DMA::Errors::NONE;
      state               = StreamState::TRANSFER_IDLE;
      elementsTransferred = 0;
      selectedChannel     = 0;
      resourceIndex       = INVALID_RESOURCE_INDEX;
      requestId           = Chimera::DMA::INVALID_REQUEST;
      elementSize         = Chimera::DMA::Alignment::NUM_OPTIONS;
      srcAddress          = 0;
      dstAddress          = 0;
      transferSize        = 0;
      persistent          = false;
      wakeUserOnComplete  = false;
      isrCallback         = {};
      fifoError           = false;
      directModeError     = false;
      transferError       = false;
    }
  };
}    // namespace Thor::LLD::DMA

// #endif /* THOR_LLD_DMA */
#endif /* !THOR_DRIVER_DMA_COMMON_TYPES_HPP */
