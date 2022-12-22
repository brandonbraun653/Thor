/******************************************************************************
 *  File Name:
 *    hld_serial_driver.cpp
 *
 *  Description:
 *    Serial (UART) driver interface for STM32 UART/USART peripherals
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/peripheral>
#include <Chimera/serial>
#include <Chimera/thread>
#include <Thor/lld/interface/inc/interrupt>
#include <Thor/lld/interface/inc/serial>
#include <algorithm>

namespace Chimera::Serial
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t NUM_DRIVERS = Thor::LLD::Serial::NUM_SERIAL_PERIPHS;

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  class ThorImpl
  {
  public:
    Thor::LLD::Serial::Driver_rPtr    pLLDriver;
    Chimera::Serial::Driver_rPtr      pHLDriver;
    Chimera::Serial::BipBuffer       *pTxBuffer;
    Chimera::Serial::BipBuffer       *pRxBuffer;
    Chimera::Serial::TxfrMode         mTxfrMode;

    ThorImpl();
    void postISRProcessing();
    void toggleAsyncListening( const bool state );
  };


  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static size_t                                                         s_driver_initialized;
  static uint32_t                                                       s_serX_thread_stack[ STACK_BYTES( 640 ) ];
  static DeviceManager<Driver, Chimera::Serial::Channel, NUM_DRIVERS>   s_raw_drivers;
  static DeviceManager<ThorImpl, Chimera::Serial::Channel, NUM_DRIVERS> s_impl_drivers;

  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief High priority thread for handling multi-thread aware post-processing ops
   *
   * This thread will take care of operations that cannot or should not be done
   * inside an ISR.
   *
   * @param arg   Unused
   */
  static void SERxISRUserThread( void *arg )
  {
    using namespace Chimera::Thread;

    Chimera::Serial::Channel instance_list[ EnumValue( Chimera::Serial::Channel::NUM_OPTIONS ) ];

    while ( 1 )
    {
      /*-----------------------------------------------------------------------
      Wait for the correct signal to wake this thread
      -----------------------------------------------------------------------*/
      if ( !this_thread::pendTaskMsg( ITCMsg::TSK_MSG_ISR_HANDLER ) )
      {
        continue;
      }

      /*-----------------------------------------------------------------------
      Handle every ISR. Don't know which triggered this.
      -----------------------------------------------------------------------*/
      const size_t count = s_impl_drivers.registeredInstances( instance_list, ARRAY_COUNT( instance_list ) );
      for ( size_t idx = 0; idx < count; idx++ )
      {
        auto impl = s_impl_drivers.get( instance_list[ idx ] );
        impl->postISRProcessing();
      }
    }
  }


  /**
   * @brief Initialize the HLD driver
   * @return Chimera::Status_t
   */
  static Chimera::Status_t impl_initialize()
  {
    using namespace Chimera::Thread;

    /*-------------------------------------------------------------------------
    Prevent multiple initializations (need reset first)
    -------------------------------------------------------------------------*/
    if ( s_driver_initialized == Chimera::DRIVER_INITIALIZED_KEY )
    {
      return Chimera::Status::OK;
    }

    /*-------------------------------------------------------------------------
    Register the ISR post processor thread
    -------------------------------------------------------------------------*/
    Task       userThread;
    TaskConfig cfg;

    cfg.name                                  = "PP_SERx";
    cfg.arg                                   = nullptr;
    cfg.function                              = SERxISRUserThread;
    cfg.priority                              = Priority::MAXIMUM;
    cfg.stackWords                            = STACK_BYTES( sizeof( s_serX_thread_stack ) );
    cfg.type                                  = TaskInitType::STATIC;
    cfg.specialization.staticTask.stackBuffer = s_serX_thread_stack;
    cfg.specialization.staticTask.stackSize   = sizeof( s_serX_thread_stack );

    userThread.create( cfg );
    auto threadId = userThread.start();
    Thor::LLD::INT::setUserTaskId( Chimera::Peripheral::Type::PERIPH_UART, threadId );
    Thor::LLD::INT::setUserTaskId( Chimera::Peripheral::Type::PERIPH_USART, threadId );

    /*-------------------------------------------------------------------------
    Initialize the low level driver
    -------------------------------------------------------------------------*/
    auto result = Thor::LLD::Serial::initialize();

    s_driver_initialized = Chimera::DRIVER_INITIALIZED_KEY;
    return result;
  }


  /**
   * @brief Reset the HLD driver
   * @return Chimera::Status_t
   */
  static Chimera::Status_t impl_reset()
  {
    return Chimera::Status::OK;
  }


  /**
   * @brief Gets the HLD driver
   *
   * @param channel   Which channel to look up
   * @return Driver_rPtr
   */
  static Driver_rPtr impl_getDriver( const Channel channel )
  {
    if ( Thor::LLD::Serial::isSupported( channel ) )
    {
      return s_raw_drivers.getOrCreate( channel );
    }
    else
    {
      return nullptr;
    }
  }


  /*---------------------------------------------------------------------------
  Driver Class Implementation
  ---------------------------------------------------------------------------*/
  Driver::Driver()
  {
  }


  Driver::~Driver()
  {
  }


  Chimera::Status_t Driver::open( const Chimera::Serial::Config &config )
  {
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if ( !Thor::LLD::Serial::isSupported( config.channel ) )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }

    RT_DBG_ASSERT( config.txBuffer->max_size() != 0 );
    RT_DBG_ASSERT( config.rxBuffer->max_size() != 0 );

    /*-------------------------------------------------------------------------
    Load up the driver
    -------------------------------------------------------------------------*/
    auto impl = s_impl_drivers.getOrCreate( config.channel );
    mImpl     = reinterpret_cast<void *>( impl );
    RT_DBG_ASSERT( impl );

    impl->pLLDriver = Thor::LLD::Serial::getDriver( config.channel );
    impl->pHLDriver = this;
    impl->pTxBuffer = config.txBuffer;
    impl->pRxBuffer = config.rxBuffer;
    impl->mTxfrMode = config.txfrMode;

    /*-------------------------------------------------------------------------
    Initialize AsyncIO for user notification of events
    -------------------------------------------------------------------------*/
    this->initAIO();

    /*-------------------------------------------------------------------------
    Initialize the low level driver
    -------------------------------------------------------------------------*/
    return impl->pLLDriver->open( config );
  }


  Chimera::Status_t Driver::close()
  {
    RT_DBG_ASSERT( mImpl );
    auto   thorImpl  = reinterpret_cast<ThorImpl *>( mImpl );

    thorImpl->pTxBuffer->clear();
    thorImpl->pRxBuffer->clear();
    return thorImpl->pLLDriver->close();
  }


  int Driver::write( const void *const buffer, const size_t length, const size_t timeout )
  {
    using namespace Chimera::Thread;
    using namespace Chimera::Event;

    Chimera::Thread::LockGuard _lck( *this );
    RT_DBG_ASSERT( mImpl );
    auto   thorImpl     = reinterpret_cast<ThorImpl *>( mImpl );
    size_t elapsed_time = 0;
    size_t write_size = 0;

    /*-------------------------------------------------------------------------
    Wait for a previous transaction to have completed
    -------------------------------------------------------------------------*/
    if( timeout != TIMEOUT_DONT_WAIT )
    {
      size_t start_time = Chimera::millis();
      bool   expired    = false;

      while ( !expired && ( thorImpl->pLLDriver->txStatus() != Status::TX_READY ) )
      {
        Chimera::Thread::this_thread::yield();
        elapsed_time = Chimera::millis() - start_time;
        expired      = elapsed_time < timeout;
      }

      if ( expired )
      {
        return write_size;
      }
    }

    /*-------------------------------------------------------------------------
    Determine how much data can actually be written
    -------------------------------------------------------------------------*/
    write_size = std::min<size_t>( thorImpl->pTxBuffer->available(), length );

    if ( write_size > 0 )
    {
      /*-----------------------------------------------------------------------
      Commit the data to the buffer. If HW is not immediately available for
      transfer, it will be scheduled at the end of the current transmission.
      -----------------------------------------------------------------------*/
      auto write_span = thorImpl->pTxBuffer->write_reserve( length );
      memcpy( write_span.data(), buffer, length );
      thorImpl->pTxBuffer->write_commit( write_span );

      /*-----------------------------------------------------------------------
      If possible, attempt to start the transfer now
      -----------------------------------------------------------------------*/
      if( thorImpl->pLLDriver->txStatus() == Status::TX_READY )
      {
        /*---------------------------------------------------------------------
        Reset the AIO signals to ensure callers will be get notified of this
        transaction and not a previously cached event.
        ---------------------------------------------------------------------*/
        this->resetAIO();

        /*---------------------------------------------------------------------
        Reserve a contiguous block of memory for the transaction, then use that
        for the hardware transfer. This cleanly supports DMA.
        ---------------------------------------------------------------------*/
        auto   read_span    = thorImpl->pTxBuffer->read_reserve( length );
        size_t lldWriteSize = thorImpl->pLLDriver->write( thorImpl->mTxfrMode, read_span.data(), read_span.max_size() );
        RT_DBG_ASSERT( read_span.max_size() == lldWriteSize );

        /*---------------------------------------------------------------------
        Finalize blocking type writes immediately
        ---------------------------------------------------------------------*/
        if ( thorImpl->mTxfrMode == TxfrMode::BLOCKING )
        {
          thorImpl->pTxBuffer->read_commit( read_span );
          this->signalAIO( Trigger::TRIGGER_WRITE_COMPLETE );
        }
      }

      /*-----------------------------------------------------------------------
      Use the remaining timeout window to wait for completion
      -----------------------------------------------------------------------*/
      if( ( timeout != TIMEOUT_DONT_WAIT ) && ( elapsed_time < timeout ) )
      {
        this->await( Trigger::TRIGGER_WRITE_COMPLETE, ( timeout - elapsed_time ) );
      }
    }

    return write_size;
  }


  int Driver::read( void *const buffer, const size_t length, const size_t timeout )
  {
    RT_DBG_ASSERT( mImpl );
    auto   thorImpl = reinterpret_cast<ThorImpl *>( mImpl );
    size_t readSize = std::min<size_t>( thorImpl->pRxBuffer->size(), length );

    if ( ( buffer != nullptr ) && ( readSize > 0 ) )
    {
      auto read_span = thorImpl->pRxBuffer->read_reserve( readSize );
      memcpy( buffer, read_span.data(), readSize );
      thorImpl->pRxBuffer->read_commit( read_span );
    }

    return readSize;
  }


  /*---------------------------------------------------------------------------
  Thor Driver Implementation
  ---------------------------------------------------------------------------*/
  ThorImpl::ThorImpl() : pLLDriver( nullptr ), pHLDriver( nullptr )
  {
  }

  void ThorImpl::postISRProcessing()
  {
    /*-------------------------------------------------------------------------
    Get reasoning for why the LLD decided to wake us up
    -------------------------------------------------------------------------*/
    const auto flags = pLLDriver->getFlags();

    /*-------------------------------------------------------------------------
    Handle TX Complete
    -------------------------------------------------------------------------*/
    if ( flags & Thor::LLD::Serial::Flag::TX_COMPLETE )
    {
      auto tcb = pLLDriver->getTCB_TX();

      /*-----------------------------------------------------------------------
      Clear the flags which got us here
      -----------------------------------------------------------------------*/
      pLLDriver->clearFlags( Thor::LLD::Serial::Flag::TX_COMPLETE );
      tcb->state = Thor::LLD::Serial::StateMachine::TX_READY;

      /*-----------------------------------------------------------------------
      Indicate we've consumed the read span
      -----------------------------------------------------------------------*/
      const etl::span<uint8_t> tmp( tcb->buffer, tcb->expected );
      pTxBuffer->read_commit( tmp );

      /*-----------------------------------------------------------------------
      Notify those waiting on the TX complete
      -----------------------------------------------------------------------*/
      pHLDriver->signalAIO( Chimera::Event::Trigger::TRIGGER_WRITE_COMPLETE );

      /*-----------------------------------------------------------------------
      If more data is waiting in the TX buffer, transmit it. This will only
      execute if the driver has been configured for IT/DMA based transfers.
      -----------------------------------------------------------------------*/
      if( pTxBuffer->size() )
      {
        Chimera::Thread::LockGuard _lck( *pHLDriver );
        auto read_span = pTxBuffer->read_reserve( pTxBuffer->size() );
        pLLDriver->write( mTxfrMode, read_span.data(), read_span.size() );
      }
    }

    /*-------------------------------------------------------------------------------
    Handle RX Complete or RX Idle
    -------------------------------------------------------------------------------*/
    if ( ( flags & Thor::LLD::Serial::Flag::RX_COMPLETE ) || ( flags & Thor::LLD::Serial::Flag::RX_LINE_IDLE_ABORT ) )
    {
      /*-----------------------------------------------------------------------
      Clear the flags which got us here
      -----------------------------------------------------------------------*/
      pLLDriver->clearFlags( Thor::LLD::Serial::Flag::RX_COMPLETE );
      pLLDriver->clearFlags( Thor::LLD::Serial::Flag::RX_LINE_IDLE_ABORT );


      // TODO BMB: Get the TCB and commit the write span

      /*-----------------------------------------------------------------------
      Finally, start listening once more for more data
      -----------------------------------------------------------------------*/
      this->toggleAsyncListening( true );

      /*-----------------------------------------------------------------------
      Notify those waiting on the RX complete
      -----------------------------------------------------------------------*/
      pHLDriver->signalAIO( Chimera::Event::Trigger::TRIGGER_READ_COMPLETE );
    }
  }


  void ThorImpl::toggleAsyncListening( const bool state )
  {
    /*-------------------------------------------------------------------------
    Start listening if requested, else kill ongoing
    -------------------------------------------------------------------------*/
    if ( state )
    {
      auto write_span = pRxBuffer->write_reserve( pRxBuffer->available() );
      pLLDriver->read( mTxfrMode, write_span.data(), write_span.size() );
    }
    else
    {
      pLLDriver->mHWIntf->killTransfer( Chimera::Hardware::SubPeripheral::RX );
    }
  }


  namespace Backend
  {
    Chimera::Status_t registerDriver( DriverConfig &registry )
    {
      registry.isSupported = true;
      registry.getDriver   = impl_getDriver;
      registry.initialize  = impl_initialize;
      registry.reset       = impl_reset;
      return Chimera::Status::OK;
    }
  }    // namespace Backend
}    // namespace Chimera::Serial
