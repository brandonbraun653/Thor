/********************************************************************************
 *   File Name:
 *    dma_types.hpp
 *
 *   Description:
 *    Common DMA types used in Thor drivers
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once 
#ifndef THOR_DRIVER_DMA_COMMON_TYPES_HPP
#define THOR_DRIVER_DMA_COMMON_TYPES_HPP

/* C++ Includes */
#include <cstdint>

/* Chimera Includes */
#include <Chimera/types/event_types.hpp>

/* Thor Includes */
#include <Thor/definitions/dma_definitions.hpp>

namespace Thor::Driver::DMA
{
  using Config_t = uint16_t;
  namespace ConfigBitFields
  {
    static constexpr Config_t EMPTY_CONFIG = 0u;
    static constexpr Config_t DMA_ON_DMA1  = ( 1u << 0 );
    static constexpr Config_t DMA_ON_DMA2  = ( 1u << 1 );

    static constexpr Config_t DMA_CHANNEL_POS = 2u;
    static constexpr Config_t DMA_CHANNEL_MSK = ( 0x7 << DMA_CHANNEL_POS );
    static constexpr Config_t DMA_CHANNEL     = DMA_CHANNEL_MSK;
    static constexpr Config_t DMA_CHANNEL_0   = ( 0u << DMA_CHANNEL_POS ) & DMA_CHANNEL_MSK;
    static constexpr Config_t DMA_CHANNEL_1   = ( 1u << DMA_CHANNEL_POS ) & DMA_CHANNEL_MSK;
    static constexpr Config_t DMA_CHANNEL_2   = ( 2u << DMA_CHANNEL_POS ) & DMA_CHANNEL_MSK;
    static constexpr Config_t DMA_CHANNEL_3   = ( 3u << DMA_CHANNEL_POS ) & DMA_CHANNEL_MSK;
    static constexpr Config_t DMA_CHANNEL_4   = ( 4u << DMA_CHANNEL_POS ) & DMA_CHANNEL_MSK;
    static constexpr Config_t DMA_CHANNEL_5   = ( 5u << DMA_CHANNEL_POS ) & DMA_CHANNEL_MSK;
    static constexpr Config_t DMA_CHANNEL_6   = ( 6u << DMA_CHANNEL_POS ) & DMA_CHANNEL_MSK;
    static constexpr Config_t DMA_CHANNEL_7   = ( 7u << DMA_CHANNEL_POS ) & DMA_CHANNEL_MSK;

    static constexpr Config_t DMA_STREAM_POS = 5u;
    static constexpr Config_t DMA_STREAM_MSK = ( 0x7 << DMA_STREAM_POS );
    static constexpr Config_t DMA_STREAM     = DMA_STREAM_MSK;
    static constexpr Config_t DMA_STREAM_0   = ( 0u << DMA_STREAM_POS ) & DMA_STREAM_MSK;
    static constexpr Config_t DMA_STREAM_1   = ( 1u << DMA_STREAM_POS ) & DMA_STREAM_MSK;
    static constexpr Config_t DMA_STREAM_2   = ( 2u << DMA_STREAM_POS ) & DMA_STREAM_MSK;
    static constexpr Config_t DMA_STREAM_3   = ( 3u << DMA_STREAM_POS ) & DMA_STREAM_MSK;
    static constexpr Config_t DMA_STREAM_4   = ( 4u << DMA_STREAM_POS ) & DMA_STREAM_MSK;
    static constexpr Config_t DMA_STREAM_5   = ( 5u << DMA_STREAM_POS ) & DMA_STREAM_MSK;
    static constexpr Config_t DMA_STREAM_6   = ( 6u << DMA_STREAM_POS ) & DMA_STREAM_MSK;
    static constexpr Config_t DMA_STREAM_7   = ( 7u << DMA_STREAM_POS ) & DMA_STREAM_MSK;
  }

  /**
   *  Forward declarations to ease compilation
   */
  struct RegisterMap;
  struct StreamX;

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

    Can be a value of Thor::Driver::DMA::Configuration::ChannelSelect
    ------------------------------------------------*/
    uint32_t Channel;

    /*------------------------------------------------
    Specifies if the data will be transferred from memory 
    to peripheral, from memory to memory or from peripheral to memory.

    Can be a value of Thor::Driver::DMA::Configuration::TransferDirection
    ------------------------------------------------*/
    uint32_t Direction;
    
    /*------------------------------------------------
    Specifies whether the Peripheral address register should be incremented or not

    Can be a value of Thor::Driver::DMA::Configuration::PeriphIncrementMode
    ------------------------------------------------*/
    uint32_t PeriphInc;
    
    /*------------------------------------------------
    Specifies whether the memory address register should be incremented or not.

    Can be a value of Thor::Driver::DMA::Configuration::MemoryIncrementMode
    ------------------------------------------------*/
    uint32_t MemInc;
    
    /*------------------------------------------------
    Specifies the Peripheral data width.

    Can be a value of Thor::Driver::DMA::Configuration::PeriphDataSize
    ------------------------------------------------*/
    uint32_t PeriphDataAlignment;

    /*------------------------------------------------
    Specifies the Memory data width

    Can be a value of Thor::Driver::DMA::Configuration::MemoryDataSize
    ------------------------------------------------*/
    uint32_t MemDataAlignment;

    /*------------------------------------------------
    Specifies the operation mode of the stream.

    @note The circular buffer mode cannot be used if the memory-to-memory
          data transfer is configured on the selected Stream  

    Can be a value of Thor::Driver::DMA::Configuration::Mode
    ------------------------------------------------*/
    uint32_t Mode;

    /*------------------------------------------------
    Specifies the software priority for the transfer
    Can be a value of Thor::Driver::DMA::Configuration::PriorityLevel
    ------------------------------------------------*/
    uint32_t Priority;

    /*------------------------------------------------
    Specifies if the FIFO mode or Direct mode will be used for the specified stream.

    @note The Direct mode (FIFO mode disabled) cannot be used if the 
          memory-to-memory data transfer is configured on the selected stream

    Can be a value of Thor::Driver::DMA::Configuration::FIFODirectMode
    ------------------------------------------------*/
    uint32_t FIFOMode; 

    /*------------------------------------------------
    Specifies the FIFO threshold level.
    
    Can be a value of Thor::Driver::DMA::Configuration::FIFOThreshold
    ------------------------------------------------*/
    uint32_t FIFOThreshold;

    /*------------------------------------------------
    Specifies the Burst transfer configuration for the memory transfers. 
    It specifies the amount of data to be transferred in a single non interruptible
    transaction.

    @note The burst mode is possible only if the address Increment mode is enabled.

    Can be a value of Thor::Driver::DMA::Configuration::MemoryBurst
    ------------------------------------------------*/
    uint32_t MemBurst;

    /*------------------------------------------------
    Specifies the Burst transfer configuration for the peripheral transfers. 
    It specifies the amount of data to be transferred in a single non interruptible 
    transaction. 

    @note The burst mode is possible only if the address Increment mode is enabled.

    Can be a value of Thor::Driver::DMA::Configuration::PeriphBurst
    ------------------------------------------------*/
    uint32_t PeriphBurst;
  };

  /**
   *  Runtime resources for each stream. Used for looking up common properties
   *  and configuration information for a given stream request source.
   */
  struct StreamResources
  {
    uint8_t requestID;                                        /**< Source ID of the system that generated the DMA request */
    Config_t cfgBitField;                                     /**< Configuration bitfield defining meta information about the request ID */
    std::vector<Chimera::Event::Actionable> eventListeners;   /**< Action registration queue that can provide callback like functionality */

    void clear()
    {
      memset( this, 0, sizeof( StreamResources ) );
    }
  };

  struct TCB
  {
    uint32_t srcAddress;        /**< Address where the data will be pulled from */
    uint32_t dstAddress;        /**< Address where the data will be transfered into */
    uint32_t transferSize;      /**< How many bytes to transfer between source and destination */
    uint32_t bytesTransfered;   /**< How many bytes were actually transfered */
    uint32_t transferState;     /**< DMA transfer state machine status */

    uint32_t selectedChannel;   /**< When the ISR fires, will contain hardware channel that was used */
    uint32_t requestGenerator;  /**< When the ISR fires, will contain the peripheral that generated the event */

    bool fifoError;
    bool directModeError;
    bool transferError;
  };
}    // namespace Thor::Driver::DMA

#endif /* !THOR_DRIVER_DMA_COMMON_TYPES_HPP */