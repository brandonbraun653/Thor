/******************************************************************************
 *  File Name:
 *   hld_dma_driver.cpp
 *
 *  Description:
 *   Implements DMA high level drivers, which focuses on managing configurations
 *   for the wide variety of streams that may be possible.
 *
 *  2019-2022 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/constants>
#include <Aurora/logging>
#include <Chimera/common>
#include <Chimera/dma>
#include <Chimera/system>
#include <Chimera/thread>
#include <Thor/cfg>
#include <Thor/dma>
#include <Thor/lld/interface/inc/dma>
#include <Thor/lld/interface/inc/interrupt>
#include <array>
#include <cstring>
#include <etl/random.h>
#include <etl/unordered_map.h>


#if defined( THOR_DMA )
/*-------------------------------------------------------------------------------
Constants
-------------------------------------------------------------------------------*/
static constexpr size_t NUM_DRIVERS = Thor::LLD::DMA::DMA_MAX_RESOURCE_IDX;

/*-------------------------------------------------
Number of pipe configurations to remember
-------------------------------------------------*/
#ifndef THOR_HLD_DMA_MAX_PIPE_CONFIGURATIONS
#define THOR_HLD_DMA_MAX_PIPE_CONFIGURATIONS ( 5 )
#endif

/*-------------------------------------------------------------------
Stack size for the interrupt handler thread
-------------------------------------------------------------------*/
#if defined( STM32L432xx )
#define THREAD_SIZE ( 256 )
#elif defined( STM32F446xx )
#define THREAD_SIZE ( 512 )
#endif

/*-------------------------------------------------------------------------------
Aliases
-------------------------------------------------------------------------------*/
namespace LLD = ::Thor::LLD::DMA;

using PipeCfgMap = etl::unordered_map<Chimera::DMA::RequestId, Chimera::DMA::PipeConfig, THOR_HLD_DMA_MAX_PIPE_CONFIGURATIONS>;

/*-------------------------------------------------------------------------------
Structures
-------------------------------------------------------------------------------*/
struct StreamStatus
{
  ::LLD::StreamState             state;    /**< Is this stream busy? */
  ::LLD::Stream_rPtr             stream;   /**< Stream controller associated with the transaction */
  Chimera::DMA::TransferCallback callback; /**< Optional callback to be invoked on completion or error */
  Chimera::DMA::RequestId        request;  /**< Request ID of the current transfer */
};

/*-------------------------------------------------------------------------------
Static Data
-------------------------------------------------------------------------------*/
static PipeCfgMap                            s_pipe_map;          /**< Local storage of all configured pipes */
static uint32_t                              s_dma_pipe_uuid;     /**< Unique IDs for pipe registration */
static uint32_t                              s_dma_request_uuid;  /**< Unique IDs for request generation */
static Chimera::Thread::RecursiveMutex       s_dma_lock;          /**< Module lock */
static std::array<StreamStatus, NUM_DRIVERS> s_stream_status;     /**< Current state of a stream */
static uint32_t                              s_dmaX_thread_stack[ STACK_BYTES( THREAD_SIZE ) ] __attribute__((section(".app_stack")));

namespace Thor::DMA
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  bool getPipeConfig( const Chimera::DMA::RequestId id, Chimera::DMA::PipeConfig &output )
  {
    using namespace Chimera::Thread;
    LockGuard lck( s_dma_lock );

    auto iter = s_pipe_map.find( id );
    if ( iter == s_pipe_map.end() )
    {
      return false;
    }
    else
    {
      output = iter->second;
      return true;
    }
  }

  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  static LLD::RIndex_t nextFreeStream( const size_t lower_bound )
  {
    using namespace Chimera::Thread;
    LockGuard lck( s_dma_lock );

    for ( size_t idx = lower_bound; idx < s_stream_status.size(); idx++ )
    {
      if ( s_stream_status[ idx ].state == ::LLD::StreamState::TRANSFER_IDLE )
      {
        return static_cast<LLD::RIndex_t>( idx );
      }
    };

    return LLD::INVALID_RESOURCE_INDEX;
  }


  /**
   * @brief High priority thread to handle completion/error events
   *
   * This will run on every DMA peripheral and channel combination as this single thread
   * handles every event. I'm trading off execution time for RAM here cause stack space
   * is usually more scarce than processing power on most MCUs.
   *
   * Only DMA transactions that are explicitly marked as needing to notify the user of
   * an event will trigger this thread. Otherwise, events are handled at the LLD layer.
   *
   * @param arg Unused
   */
  static void DMAxStreamxISRUserThread( void *arg )
  {
    using namespace Chimera::DMA;
    using namespace Chimera::Thread;

    while ( 1 )
    {
      /*-----------------------------------------------------------------------
      Wait for something to wake this thread
      -----------------------------------------------------------------------*/
      if ( !this_thread::pendTaskMsg( ITCMsg::TSK_MSG_ISR_HANDLER ) )
      {
        continue;
      }

      /*-----------------------------------------------------------------------
      Handle all pending DMA transfer events
      -----------------------------------------------------------------------*/
      ::LLD::TCB tcb;

      auto msk = Chimera::System::disableInterrupts();
      while ( ::LLD::Resource::ISRQueue.pop( tcb ) )
      {
        /*---------------------------------------------------------------------
        Prevent invalid array access
        ---------------------------------------------------------------------*/
        if ( ( tcb.resourceIndex == LLD::INVALID_RESOURCE_INDEX ) || !( tcb.resourceIndex < s_stream_status.size() ) )
        {
          continue;
        }

        /*---------------------------------------------------------------------
        Update the stream's availability
        ---------------------------------------------------------------------*/
        s_stream_status[ tcb.resourceIndex ].state = ::LLD::StreamState::TRANSFER_IDLE;

        /*---------------------------------------------------------------------
        User has a callback?
        ---------------------------------------------------------------------*/
        if ( s_stream_status[ tcb.resourceIndex ].callback )
        {
          TransferStats stats;
          stats.size      = tcb.elementsTransferred;
          stats.requestId = tcb.requestId;
          stats.error     = tcb.transferError;

          s_stream_status[ tcb.resourceIndex ].callback( stats );
        }
      }

      Chimera::System::enableInterrupts( msk );
    }
  }

}    // namespace Thor::DMA

namespace Chimera::DMA::Backend
{
  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  static Chimera::Status_t reset()
  {
    using namespace Chimera::Thread;
    LockGuard lck( s_dma_lock );

    s_pipe_map.clear();
    s_dma_pipe_uuid    = 0;
    s_dma_request_uuid = 0;

    return Chimera::Status::OK;
  }


  static Chimera::Status_t initialize()
  {
    using namespace Chimera::Thread;
    LockGuard lck( s_dma_lock );

    /*-------------------------------------------------------------------------
    Reset DMA and the driver module
    -------------------------------------------------------------------------*/
    reset();

    for ( auto &stream : s_stream_status )
    {
      stream.stream   = nullptr;
      stream.state    = ::LLD::StreamState::TRANSFER_IDLE;
      stream.request  = Chimera::DMA::INVALID_REQUEST;
      stream.callback = {};
    }

    /*-------------------------------------------------------------------------
    Register the high priority stream event processor thread
    -------------------------------------------------------------------------*/
    Task       userThread;
    TaskConfig cfg;

    cfg.name                                  = "PP_DMAx";
    cfg.arg                                   = nullptr;
    cfg.function                              = Thor::DMA::DMAxStreamxISRUserThread;
    cfg.priority                              = Chimera::Thread::Priority::MAXIMUM;
    cfg.stackWords                            = STACK_BYTES( sizeof( s_dmaX_thread_stack ) );
    cfg.type                                  = TaskInitType::STATIC;
    cfg.specialization.staticTask.stackBuffer = s_dmaX_thread_stack;
    cfg.specialization.staticTask.stackSize   = sizeof( s_dmaX_thread_stack );

    userThread.create( cfg );
    Thor::LLD::INT::setUserTaskId( Chimera::Peripheral::Type::PERIPH_DMA, userThread.start() );

    /*-------------------------------------------------------------------------
    Initialize the low level driver
    -------------------------------------------------------------------------*/
    return ::LLD::initialize();
  }


  /**
   *  Constructs a new pipe configuration. No HW configuration is occurring
   *  because the LLD will read the required information when it's time to
   *  initiate a new DMA transfer. There are generally more DMA request signals
   *  than hardware channels, so the HW peripheral is reconfigured on the fly as
   *  needed by the software.
   *
   *  The Chimera interface presents a "permanence" of a pipe configuration, so
   *  this registry provides the memory for how things were configured in the past.
   */
  static Chimera::DMA::RequestId constructPipe( const Chimera::DMA::PipeConfig &config )
  {
    using namespace Chimera::Thread;
    LockGuard lck( s_dma_lock );

    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if ( s_pipe_map.full() )
    {
      return Chimera::DMA::INVALID_REQUEST;
    }

    /*-------------------------------------------------------------------------
    Previously registered? Overwrite the settings and return original ID.
    -------------------------------------------------------------------------*/
    for ( auto &iter : s_pipe_map )
    {
      if ( ( iter.second.channel == config.channel ) && ( iter.second.resourceIndex == config.resourceIndex ) )
      {
        iter.second = config;
        return iter.first;    // RequestId
      }
    }

    /*-------------------------------------------------------------------------
    New request pipe
    -------------------------------------------------------------------------*/
    Chimera::DMA::RequestId id = s_dma_pipe_uuid++;
    s_pipe_map.insert( { id, config } );

    return s_dma_pipe_uuid - 1u;
  }


  static Chimera::DMA::RequestId transfer( const Chimera::DMA::MemTransfer &transfer )
  {
    using namespace Chimera::Thread;
    LockGuard lck( s_dma_lock );

    /*-------------------------------------------------------------------------
    Get the next available stream. For whatever reason, memory transfers are
    only supported on DMA2.
    -------------------------------------------------------------------------*/
    auto idx = Thor::DMA::nextFreeStream( ::LLD::DMA2_FIRST_STREAM_RESOURCE_INDEX );
    if ( idx == Thor::LLD::INVALID_RESOURCE_INDEX )
    {
      return Chimera::DMA::INVALID_REQUEST;
    }

    ::LLD::Stream_rPtr stream = ::LLD::getStream( idx );
    if ( !stream )
    {
      return Chimera::DMA::INVALID_REQUEST;
    }

    /*-------------------------------------------------------------------------
    Build up the transfer configuration
    -------------------------------------------------------------------------*/
    ::LLD::StreamConfig cfg;
    cfg.dstAddrIncr   = true;
    cfg.srcAddrIncr   = true;
    cfg.channel       = ::LLD::Channel::INVALID;
    cfg.fifoMode      = Chimera::DMA::FifoMode::DIRECT_DISABLE;
    cfg.fifoThreshold = Chimera::DMA::FifoThreshold::FULL;
    cfg.dmaMode       = Chimera::DMA::Mode::DIRECT;
    cfg.direction     = Chimera::DMA::Direction::MEMORY_TO_MEMORY;
    cfg.priority      = transfer.priority;
    cfg.dstBurstSize  = Chimera::DMA::BurstSize::NUM_OPTIONS;
    cfg.dstAddrAlign  = Chimera::DMA::Alignment::NUM_OPTIONS;
    cfg.srcBurstSize  = Chimera::DMA::BurstSize::BURST_SIZE_1;
    cfg.srcAddrAlign  = transfer.alignment;

    ::LLD::TCB tcb;
    tcb.srcAddress         = transfer.src;
    tcb.dstAddress         = transfer.dst;
    tcb.transferSize       = transfer.size;
    tcb.requestId          = s_dma_request_uuid++;
    tcb.elementSize        = transfer.alignment;
    tcb.persistent         = false;
    tcb.wakeUserOnComplete = true;

    if( s_dma_request_uuid == Chimera::DMA::INVALID_REQUEST )
    {
      s_dma_request_uuid = 0;
    }

    /*-------------------------------------------------------------------------
    Set the configuration on the stream
    -------------------------------------------------------------------------*/
    if ( stream->configure( &cfg, &tcb ) == Chimera::Status::OK )
    {
      s_stream_status[ idx ].state    = ::LLD::StreamState::TRANSFER_IN_PROGRESS;
      s_stream_status[ idx ].callback = transfer.callback;
      s_stream_status[ idx ].request  = tcb.requestId;

      stream->start();
      return tcb.requestId;
    }
    else
    {
      return Chimera::DMA::INVALID_REQUEST;
    }
  }


  static Chimera::DMA::RequestId transfer( const Chimera::DMA::PipeTransfer &transfer )
  {
    using namespace Chimera::DMA;
    using namespace Chimera::Thread;
    LockGuard lck( s_dma_lock );

    /*-------------------------------------------------------------------------
    Get the pipe configuration
    -------------------------------------------------------------------------*/
    auto iter = s_pipe_map.find( transfer.pipe );
    if ( iter == s_pipe_map.end() )
    {
      return Chimera::DMA::INVALID_REQUEST;
    }

    PipeConfig pipeCfg = iter->second;

    /*-------------------------------------------------------------------------
    Grab the stream the pipe was configured for
    -------------------------------------------------------------------------*/
    ::LLD::Stream_rPtr stream = ::LLD::getStream( static_cast<Thor::LLD::RIndex_t>( pipeCfg.resourceIndex ) );
    if ( !stream )
    {
      return Chimera::DMA::INVALID_REQUEST;
    }

    /*-------------------------------------------------------------------------
    Build up the transfer configuration
    -------------------------------------------------------------------------*/
    ::LLD::StreamConfig cfg;
    ::LLD::TCB          tcb;

    if ( pipeCfg.direction == Direction::MEMORY_TO_PERIPH )
    {
      cfg.dstAddrIncr  = false;
      cfg.dstBurstSize = pipeCfg.burstSize;
      cfg.dstAddrAlign = pipeCfg.dstAlignment;
      tcb.dstAddress   = pipeCfg.periphAddr;

      cfg.srcAddrIncr  = true;
      cfg.srcBurstSize = pipeCfg.burstSize;
      cfg.srcAddrAlign = pipeCfg.srcAlignment;
      tcb.srcAddress   = transfer.addr;
    }
    else if ( pipeCfg.direction == Direction::PERIPH_TO_MEMORY )
    {
      cfg.dstAddrIncr  = true;
      cfg.dstBurstSize = pipeCfg.burstSize;
      cfg.dstAddrAlign = pipeCfg.dstAlignment;
      tcb.dstAddress   = transfer.addr;

      cfg.srcAddrIncr  = false;
      cfg.srcBurstSize = pipeCfg.burstSize;
      cfg.srcAddrAlign = pipeCfg.srcAlignment;
      tcb.srcAddress   = pipeCfg.periphAddr;
    }
    else
    {
      return Chimera::DMA::INVALID_REQUEST;
    }

    cfg.channel            = static_cast<::LLD::Channel>( pipeCfg.channel );
    cfg.fifoMode           = pipeCfg.fifoMode;
    cfg.fifoThreshold      = pipeCfg.threshold;
    cfg.dmaMode            = pipeCfg.mode;
    cfg.direction          = pipeCfg.direction;
    cfg.priority           = pipeCfg.priority;
    tcb.transferSize       = transfer.size;
    tcb.requestId          = s_dma_request_uuid++;
    tcb.errorsToIgnore     = pipeCfg.errorsToIgnore;
    tcb.elementSize        = pipeCfg.srcAlignment;
    tcb.persistent         = pipeCfg.persistent;
    tcb.wakeUserOnComplete = pipeCfg.wakeUserOnComplete;
    tcb.isrCallback        = transfer.isrCallback;

    if( s_dma_request_uuid == Chimera::DMA::INVALID_REQUEST )
    {
      s_dma_request_uuid = 0;
    }

    /*-------------------------------------------------------------------------
    Set the configuration on the stream
    -------------------------------------------------------------------------*/
    if ( stream->configure( &cfg, &tcb ) == Chimera::Status::OK )
    {
      s_stream_status[ pipeCfg.resourceIndex ].state    = ::LLD::StreamState::TRANSFER_IN_PROGRESS;
      s_stream_status[ pipeCfg.resourceIndex ].callback = transfer.userCallback;
      s_stream_status[ pipeCfg.resourceIndex ].request  = tcb.requestId;
      s_stream_status[ pipeCfg.resourceIndex ].stream   = stream;

      stream->start();
      return tcb.requestId;
    }
    else
    {
      return Chimera::DMA::INVALID_REQUEST;
    }
  }


  static void abort( const Chimera::DMA::RequestId id )
  {
    for ( auto &stream : s_stream_status )
    {
      if ( ( stream.request == id ) && ( stream.state != ::LLD::StreamState::TRANSFER_IDLE ) && ( stream.stream ) )
      {
        stream.stream->abort();
        break;
      }
    }
  }


  Chimera::Status_t registerDriver( Chimera::DMA::Backend::DriverConfig &registry )
  {
    registry.isSupported   = true;
    registry.initialize    = initialize;
    registry.reset         = reset;
    registry.constructPipe = constructPipe;
    registry.memTransfer   = transfer;
    registry.pipeTransfer  = transfer;
    registry.abortTransfer = abort;

    return Chimera::Status::OK;
  }
}    // namespace Chimera::DMA::Backend

#endif /* THOR_DMA */
