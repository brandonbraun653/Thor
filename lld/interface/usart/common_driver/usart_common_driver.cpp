/********************************************************************************
 *  File Name:
 *    usart_common_driver.cpp
 *
 *  Description:
 *    Shared low level driver for the USART peripheral. This focuses on very
 *    basic functionality, like reading and writing byte streams.
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/interrupt>
#include <Chimera/serial>
#include <Chimera/thread>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/dma>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/inc/dma>
#include <Thor/lld/interface/inc/interrupt>
#include <Thor/lld/interface/inc/rcc>
#include <Thor/lld/interface/inc/usart>
#include <Thor/lld/interface/usart/common_driver/usart_common_intf.hpp>

#if defined( THOR_LLD_USART ) && ( defined( TARGET_STM32L4 ) || defined( TARGET_STM32F4 ) )

namespace Thor::LLD::USART
{
  /*-------------------------------------------------------------------------------
  Public Methods
  -------------------------------------------------------------------------------*/
  Driver::Driver() : mPeriph( nullptr )
  {
  }


  Driver::~Driver()
  {
  }


  Chimera::Status_t Driver::attach( RegisterMap *peripheral )
  {
    if ( auto idx = getResourceIndex( reinterpret_cast<std::uintptr_t>( peripheral ) ); idx != INVALID_RESOURCE_INDEX )
    {
      mPeriph        = peripheral;
      mResourceIndex = idx;

      return Chimera::Status::OK;
    }
    else
    {
      return Chimera::Status::FAIL;
    }
  }


  Chimera::Status_t Driver::init( const Thor::LLD::Serial::Config &cfg )
  {
    /*------------------------------------------------
    Initialize driver memory
    ------------------------------------------------*/
    mRuntimeFlags = static_cast<Runtime::Flag_t>( 0 );
    mTXTCB.reset();
    mRXTCB.reset();

    /*------------------------------------------------
    Enable the peripheral clock and reset the hardware
    ------------------------------------------------*/
    auto rccPeriph = Thor::LLD::RCC::getPeriphClockCtrl();
    rccPeriph->enableClock( Chimera::Peripheral::Type::PERIPH_USART, mResourceIndex );
    rccPeriph->reset( Chimera::Peripheral::Type::PERIPH_USART, mResourceIndex );

    /*------------------------------------------------
    Follow the initialization sequence as defined in
    RM0394 Rev 4, pg.1202
    ------------------------------------------------*/
    /* Disable the USART by writing the UE bit to 0 */
    UE::set( mPeriph, 0 );

    /* Program the M0 bit to define the word length. M1 Currently not supported. */
    M0::set( mPeriph, cfg.WordLength );

    /* Program the number of stop bits */
    STOP::set( mPeriph, cfg.StopBits );

    /* Program the parity */
    switch ( cfg.Parity )
    {
      case Configuration::Parity::EVEN:
        PCE::set( mPeriph, CR1_PCE );
        PS::clear( mPeriph, CR1_PS );
        break;

      case Configuration::Parity::ODD:
        PCE::set( mPeriph, CR1_PCE );
        PS::set( mPeriph, CR1_PS );
        break;

      case Configuration::Parity::NONE:
      default:
        PCE::clear( mPeriph, CR1_PCE );
        break;
    };

    /* Select the desired baud rate */
    BRR::set( mPeriph, calculateBRR( cfg.BaudRate ) );

    /* Enable the USART by writing the UE bit to 1 */
    UE::set( mPeriph, CR1_UE );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::deinit()
  {
    /*-------------------------------------------------
    Use the RCC's ability to reset whole peripherals back to default states
    -------------------------------------------------*/
    auto rcc = Thor::LLD::RCC::getPeriphClockCtrl();
    rcc->enableClock( Chimera::Peripheral::Type::PERIPH_USART, mResourceIndex );
    rcc->reset( Chimera::Peripheral::Type::PERIPH_USART, mResourceIndex );
    rcc->disableClock( Chimera::Peripheral::Type::PERIPH_USART, mResourceIndex );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::reset()
  {
    deinit();
    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::transmit( const void *const data, const size_t size )
  {
    // TODO: Add this back
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::receive( void *const data, const size_t size )
  {
    // TODO: Add this back
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::enableIT( const Chimera::Hardware::SubPeripheral subPeriph )
  {
    /*-------------------------------------------------------------------------------
    Configure and enable the global interrupt, ignoring the subperiph argument as the
    transmit and receive functions called by the user handle them appropriately.
    -------------------------------------------------------------------------------*/
    INT::setPriority( Resource::IRQSignals[ mResourceIndex ], INT::USART_IT_PREEMPT_PRIORITY, 0u );
    INT::enableIRQ( Resource::IRQSignals[ mResourceIndex ] );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::disableIT( const Chimera::Hardware::SubPeripheral subPeriph )
  {
    using namespace Chimera::Hardware;

    Chimera::Status_t result = Chimera::Status::OK;

    /*-------------------------------------------------
    Disable system level interrupts to prevent glitches
    -------------------------------------------------*/
    disableUSARTInterrupts();

    /*-------------------------------------------------
    Disable interrupt event generation inside the peripheral
    -------------------------------------------------*/
    if ( ( subPeriph == SubPeripheral::TX ) || ( subPeriph == SubPeripheral::TXRX ) )
    {
      prjDisableISRSignal( mPeriph, ISRSignal::TRANSMIT_COMPLETE );
      prjDisableISRSignal( mPeriph, ISRSignal::TRANSMIT_DATA_REG_EMPTY );
    }
    else if ( ( subPeriph == SubPeripheral::RX ) || ( subPeriph == SubPeripheral::TXRX ) )
    {
      prjDisableISRSignal( mPeriph, ISRSignal::RECEIVED_DATA_READY );
    }
    else
    {
      result = Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------
    Re-enable the system level interrupts so other events can get through
    -------------------------------------------------*/
    enableUSARTInterrupts();

    return result;
  }


  Chimera::Status_t Driver::transmitIT( const void *const data, const size_t size )
  {
    /*-------------------------------------------------
    Input protection
    -------------------------------------------------*/
    if ( !data || !size )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------
    Ensure the previous transfer has completed
    -------------------------------------------------*/
    auto status = txTransferStatus();
    if ( ( status != StateMachine::TX::TX_COMPLETE ) && ( status != StateMachine::TX::TX_READY ) )
    {
      return Chimera::Status::BUSY;
    }
    else    // No on-going transfers
    {
      auto transferData = reinterpret_cast<const uint8_t *const>( data );

      /*------------------------------------------------
      Prep the transfer control block
      ------------------------------------------------*/
      disableUSARTInterrupts();

      mTXTCB.buffer    = &transferData[ 1 ]; /* Point to the next byte */
      mTXTCB.expected  = size;
      mTXTCB.remaining = size - 1u; /* Pre-decrement to account for this first byte TX */
      mTXTCB.state     = StateMachine::TX::TX_ONGOING;

      /*------------------------------------------------
      Shove the byte into the transmit data register,
      kicking off the transfer.
      ------------------------------------------------*/
      prjWriteDataRegister( mPeriph, transferData[ 0 ] );

      /*------------------------------------------------
      Turn on the transmitter & enable TDR interrupt so
      we know when to stage the next byte.
      ------------------------------------------------*/
      prjEnableTransmitter( mPeriph );
      prjEnableISRSignal( mPeriph, ISRSignal::TRANSMIT_DATA_REG_EMPTY );
      enableIT( Chimera::Hardware::SubPeripheral::TX );

      enableUSARTInterrupts();
    }

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::receiveIT( void *const data, const size_t size )
  {
    /*-------------------------------------------------
    Input protection
    -------------------------------------------------*/
    if ( !data || !size )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------
    Ensure that the ISR has processed all data in the FIFO
    -------------------------------------------------*/
    auto status = rxTransferStatus();
    if ( ( status != StateMachine::RX::RX_COMPLETE ) && ( status != StateMachine::RX::RX_READY ) )
    {
      return Chimera::Status::BUSY;
    }
    else    // All bytes have been read out of the FIFO
    {
      disableUSARTInterrupts();
      enableIT( Chimera::Hardware::SubPeripheral::RX );

      /*------------------------------------------------
      Only turn on RXNE so as to detect when the first byte arrives
      ------------------------------------------------*/
      prjEnableISRSignal( mPeriph, ISRSignal::RECEIVED_DATA_READY );

      /*------------------------------------------------
      Prep the transfer control block to receive data
      ------------------------------------------------*/
      mRXTCB.buffer    = reinterpret_cast<uint8_t *const>( data );
      mRXTCB.expected  = size;
      mRXTCB.remaining = size;
      mRXTCB.state     = StateMachine::RX::RX_ONGOING;

      /*------------------------------------------------
      Turn on the RX hardware to begin listening for data
      ------------------------------------------------*/
      prjEnableReceiver( mPeriph );
      enableUSARTInterrupts();
    }

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::initDMA()
  {
    using namespace Chimera::DMA;
    using namespace Chimera::Peripheral;

    /*-------------------------------------------------
    Configure the TX pipe
    -------------------------------------------------*/
    PipeConfig txCfg;
    txCfg.alignment     = Alignment::BYTE;
    txCfg.direction     = Direction::MEMORY_TO_PERIPH;
    txCfg.mode          = Mode::DIRECT;
    txCfg.periphAddr    = reinterpret_cast<std::uintptr_t>( &mPeriph->DR );
    txCfg.priority      = Priority::MEDIUM;
    txCfg.resourceIndex = DMA::getResourceIndex( Resource::TXDMASignals[ mResourceIndex ] );
    txCfg.channel       = static_cast<size_t>( DMA::getChannel( Resource::TXDMASignals[ mResourceIndex ] ) );
    txCfg.threshold     = FifoThreshold::NONE;

    /*-------------------------------------------------
    Configure the RX pipe
    -------------------------------------------------*/
    PipeConfig rxCfg;
    rxCfg.alignment     = Alignment::BYTE;
    rxCfg.direction     = Direction::PERIPH_TO_MEMORY;
    rxCfg.mode          = Mode::DIRECT;
    rxCfg.periphAddr    = reinterpret_cast<std::uintptr_t>( &mPeriph->DR );
    rxCfg.priority      = Priority::MEDIUM;
    rxCfg.resourceIndex = DMA::getResourceIndex( Resource::RXDMASignals[ mResourceIndex ] );
    rxCfg.channel       = static_cast<size_t>( DMA::getChannel( Resource::TXDMASignals[ mResourceIndex ] ) );
    rxCfg.threshold     = FifoThreshold::NONE;

    /*-------------------------------------------------
    FIFO errors are thrown even though we don't use it.
    Doesn't seem to affect TX operations.
    -------------------------------------------------*/
    txCfg.errorsToIgnore = Errors::FIFO;
    rxCfg.errorsToIgnore = Errors::FIFO;

    /*-------------------------------------------------
    Construct the pipe and make a note of it's UUID
    -------------------------------------------------*/
    mTXDMARequestId = Thor::DMA::constructPipe( txCfg );
    mRXDMARequestId = Thor::DMA::constructPipe( rxCfg );

    /*-------------------------------------------------
    Configure the peripheral interrupts
    -------------------------------------------------*/
    enableIT( Chimera::Hardware::SubPeripheral::TXRX );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::deinitDMA()
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::enableDMA_IT( const Chimera::Hardware::SubPeripheral periph )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::disableDMA_IT( const Chimera::Hardware::SubPeripheral periph )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::transmitDMA( const void *const data, const size_t size )
  {
    using namespace Chimera::DMA;

    /*-------------------------------------------------
    Input protection
    -------------------------------------------------*/
    if ( !data || !size )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------
    Ensure the previous transfer has completed
    -------------------------------------------------*/
    auto status = txTransferStatus();
    if ( ( status != StateMachine::TX::TX_COMPLETE ) && ( status != StateMachine::TX::TX_READY ) )
    {
      return Chimera::Status::BUSY;
    }

    initDMA();

    disableUSARTInterrupts();
    {
      /* Instruct the USART to use DMA mode */
      DMAT::set( mPeriph, CR3_DMAT );

      /* Clear the transfer complete bit */
      TC::clear( mPeriph, SR_TC );

      /* Enable the USART completion interrupts */
      prjEnableTransmitter( mPeriph );
      prjEnableISRSignal( mPeriph, ISRSignal::TRANSMIT_COMPLETE );

      /* Configure the DMA transfer */
      PipeTransfer cfg;
      cfg.callback = TransferCallback::create<Driver, &Driver::onDMATXComplete>( *this );
      cfg.pipe     = mTXDMARequestId;
      cfg.size     = size;
      cfg.addr     = reinterpret_cast<std::uintptr_t>( data );

      /* Configure the USART transfer control block */
      mTXTCB.buffer    = nullptr;
      mTXTCB.expected  = size;
      mTXTCB.remaining = size;
      mTXTCB.state     = StateMachine::TX::TX_ONGOING;

      Thor::DMA::transfer( cfg );
    }
    enableUSARTInterrupts();

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::receiveDMA( void *const data, const size_t size )
  {
    using namespace Chimera::DMA;

    /*-------------------------------------------------
    Input protection
    -------------------------------------------------*/
    if ( !data || !size )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------
    Ensure that the ISR has processed all data in the FIFO
    -------------------------------------------------*/
    auto status = rxTransferStatus();
    if ( ( status != StateMachine::RX::RX_COMPLETE ) && ( status != StateMachine::RX::RX_READY ) )
    {
      return Chimera::Status::BUSY;
    }


    initDMA();
    disableUSARTInterrupts();
    {
      /*------------------------------------------------
      Only turn on RXNE so as to detect when the first byte arrives
      ------------------------------------------------*/
      prjEnableISRSignal( mPeriph, ISRSignal::RECEIVED_DATA_READY );

      /*------------------------------------------------
      Make sure line idle ISR is enabled. Regardless of
      if there is data left to RX, the sender might just
      suddenly stop, and that needs detection.
      ------------------------------------------------*/
      prjEnableISRSignal( mPeriph, ISRSignal::LINE_IDLE );
      prjClrISRSignal( mPeriph, ISRSignal::LINE_IDLE );

      /*------------------------------------------------
      Prep the transfer control block to receive data
      ------------------------------------------------*/
      /* Configure the DMA transfer */
      PipeTransfer cfg;
      cfg.callback = TransferCallback::create<Driver, &Driver::onDMARXComplete>( *this );
      cfg.pipe     = mRXDMARequestId;
      cfg.size     = size;
      cfg.addr     = reinterpret_cast<std::uintptr_t>( data );

      /* Configure the USART transfer control block */
      mRXTCB.buffer    = reinterpret_cast<uint8_t *const>( data );
      mRXTCB.expected  = size;
      mRXTCB.remaining = size;
      mRXTCB.state     = StateMachine::RX::RX_ONGOING;

      Thor::DMA::transfer( cfg );

      /*------------------------------------------------
      Turn on the RX hardware to begin listening for data
      ------------------------------------------------*/
      prjEnableReceiver( mPeriph );
      enableUSARTInterrupts();
    }

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::txTransferStatus()
  {
    Chimera::Status_t cacheStatus = Chimera::Status::UNKNOWN_ERROR;

    disableUSARTInterrupts();
    cacheStatus = mTXTCB.state;
    enableUSARTInterrupts();

    return cacheStatus;
  }


  Chimera::Status_t Driver::rxTransferStatus()
  {
    Chimera::Status_t cacheStatus = Chimera::Status::UNKNOWN_ERROR;

    disableUSARTInterrupts();
    cacheStatus = mRXTCB.state;
    enableUSARTInterrupts();

    return cacheStatus;
  }


  uint32_t Driver::getFlags()
  {
    return mRuntimeFlags;    // Atomic operation on this processor
  }


  void Driver::clearFlags( const uint32_t flagBits )
  {
    disableUSARTInterrupts();
    mRuntimeFlags &= ~( flagBits );
    enableUSARTInterrupts();
  }


  void Driver::killTransmit()
  {
    disableUSARTInterrupts();

    disableIT( Chimera::Hardware::SubPeripheral::TX );
    disableDMA_IT( Chimera::Hardware::SubPeripheral::TX );
    mTXTCB.reset();

    enableUSARTInterrupts();
  }


  void Driver::killReceive()
  {
    disableUSARTInterrupts();

    disableIT( Chimera::Hardware::SubPeripheral::RX );
    disableDMA_IT( Chimera::Hardware::SubPeripheral::RX );
    mRXTCB.reset();

    enableUSARTInterrupts();
  }


  Thor::LLD::Serial::CDTCB Driver::getTCB_TX()
  {
    Thor::LLD::Serial::CDTCB temp;

    disableUSARTInterrupts();
    temp = mTXTCB;
    enableUSARTInterrupts();

    return temp;
  }


  Thor::LLD::Serial::MDTCB Driver::getTCB_RX()
  {
    Thor::LLD::Serial::MDTCB temp;

    disableUSARTInterrupts();
    temp = mRXTCB;
    enableUSARTInterrupts();

    return temp;
  }


  Thor::LLD::Serial::Config Driver::getConfiguration()
  {
    Thor::LLD::Serial::Config cfg;
    memset( &cfg, 0, sizeof( cfg ) );

    cfg.BaudRate     = 0;
    cfg.Mode         = TE::get( mPeriph ) | RE::get( mPeriph );
    cfg.OverSampling = OVER8::get( mPeriph );
    cfg.Parity       = PCE::get( mPeriph ) | PS::get( mPeriph );
    cfg.StopBits     = STOP::get( mPeriph );
    cfg.WordLength   = M0::get( mPeriph );

    return cfg;
  }


  /*-------------------------------------------------------------------------------
  Protected Methods
  -------------------------------------------------------------------------------*/
  void Driver::IRQHandler()
  {
    using namespace Chimera::Interrupt;
    using namespace Chimera::Peripheral;
    using namespace Chimera::Serial;
    using namespace Chimera::Thread;
    using namespace Configuration::Flags;

    /*------------------------------------------------
    Cache the current state of the registers
    ------------------------------------------------*/
    const uint32_t ISRCache = ISR::get( mPeriph );
    const uint32_t CR1Cache = CR1::get( mPeriph );
    bool signalHLD          = false;

    /*------------------------------------------------
    Figure out which interrupt flags have been set
    ------------------------------------------------*/
    uint32_t txFlags    = ISRCache & ( FLAG_TC | FLAG_TXE );
    uint32_t rxFlags    = ISRCache & ( FLAG_RXNE | FLAG_IDLE );
    uint32_t errorFlags = ISRCache & ( FLAG_ORE | FLAG_PE | FLAG_NF | FLAG_FE );

    /*------------------------------------------------
    TX Related Handler
    ------------------------------------------------*/
    if ( txFlags && ( ( CR1Cache & CR1_TXEIE ) || ( CR1Cache & CR1_TCIE ) ) )
    {
      /*------------------------------------------------
      If no transfer is ongoing, clear out the flags and
      disable TX related interrupts. This is an invalid state.
      ------------------------------------------------*/
      if ( mTXTCB.state == StateMachine::TX::TX_READY )
      {
        prjClrISRSignal( mPeriph, ISRSignal::TRANSMIT_COMPLETE );
        prjDisableISRSignal( mPeriph, ISRSignal::TRANSMIT_COMPLETE );
        prjDisableISRSignal( mPeriph, ISRSignal::TRANSMIT_DATA_REG_EMPTY );
      }

      /*------------------------------------------------
      TDR Empty Interrupt
      ------------------------------------------------*/
      if ( ( txFlags & FLAG_TXE ) && ( mTXTCB.state == StateMachine::TX::TX_ONGOING ) )
      {
        if ( mTXTCB.remaining )    // Data left to be transmitted
        {
          /*------------------------------------------------
          Make sure the TX complete interrupt cannot fire
          ------------------------------------------------*/
          prjDisableISRSignal( mPeriph, ISRSignal::TRANSMIT_COMPLETE );

          /*------------------------------------------------
          Transfer the byte and prep for the next character
          ------------------------------------------------*/
          prjWriteDataRegister( mPeriph, *mTXTCB.buffer );
          mTXTCB.buffer++;
          mTXTCB.remaining--;
        }
        else    // All data has been transmitted
        {
          /*------------------------------------------------
          We finished pushing the last character into the TDR, so
          now we can listen for the TX complete interrupt.
          ------------------------------------------------*/
          prjDisableISRSignal( mPeriph, ISRSignal::TRANSMIT_DATA_REG_EMPTY );
          prjEnableISRSignal( mPeriph, ISRSignal::TRANSMIT_COMPLETE );

          mTXTCB.state = StateMachine::TX_COMPLETE;
        }
      }

      /*------------------------------------------------
      Transfer Complete Interrupt
      ------------------------------------------------*/
      if ( ( txFlags & FLAG_TC ) && ( mTXTCB.state == StateMachine::TX::TX_COMPLETE ) )
      {
        /*-------------------------------------------------
        Exit the TX ISR by disabling related interrupts
        -------------------------------------------------*/
        prjClrISRSignal( mPeriph, ISRSignal::TRANSMIT_COMPLETE );
        prjDisableISRSignal( mPeriph, ISRSignal::TRANSMIT_COMPLETE );
        prjDisableISRSignal( mPeriph, ISRSignal::TRANSMIT_DATA_REG_EMPTY );

        mRuntimeFlags |= Runtime::Flag::TX_COMPLETE;

        /*------------------------------------------------
        Invoke a user callback if registered
        ------------------------------------------------*/
        signalHLD                            = true;
        const SignalCallback *const callback = INT::getISRHandler( Type::PERIPH_USART, SIG_TX_COMPLETE );
        if ( callback->isrCallback )
        {
          callback->isrCallback();
        }
      }
    }

    /*------------------------------------------------
    RX Related Handler
    ------------------------------------------------*/
    if ( rxFlags )
    {
      /*------------------------------------------------
      RX register not empty, aka a new byte has arrived!
      ------------------------------------------------*/
      if ( ( rxFlags & FLAG_RXNE ) && ( CR1Cache & CR1_RXNEIE ) && ( mRXTCB.state == StateMachine::RX::RX_ONGOING ) )
      {
        /*------------------------------------------------
        Make sure line idle ISR is enabled. Regardless of
        if there is data left to RX, the sender might just
        suddenly stop, and that needs detection.
        ------------------------------------------------*/
        prjEnableISRSignal( mPeriph, ISRSignal::LINE_IDLE );
        prjClrISRSignal( mPeriph, ISRSignal::LINE_IDLE );

        /*------------------------------------------------
        Read out current byte and prep for next transfer
        ------------------------------------------------*/
        *mRXTCB.buffer = static_cast<uint8_t>( prjReadDataRegister( mPeriph ) );
        mRXTCB.buffer++;
        mRXTCB.remaining--;

        /*------------------------------------------------
        If no more bytes left to receive, stop listening
        ------------------------------------------------*/
        if ( !mRXTCB.remaining )
        {
          prjDisableISRSignal( mPeriph, ISRSignal::LINE_IDLE );
          prjDisableReceiver( mPeriph );

          mRXTCB.state = StateMachine::RX::RX_COMPLETE;
          mRuntimeFlags |= Runtime::Flag::RX_COMPLETE;
        }
      }

      /*------------------------------------------------
      Line Idle: We were in the middle of a transfer and
      then suddenly they just stopped sending data.
      ------------------------------------------------*/
      if ( ( rxFlags & FLAG_IDLE ) && ( CR1Cache & CR1_IDLEIE ) && ( mRXTCB.state == StateMachine::RX::RX_ONGOING ) )
      {
        /*-------------------------------------------------
        Clear the appropriate flags
        -------------------------------------------------*/
        prjDisableISRSignal( mPeriph, ISRSignal::LINE_IDLE );
        prjClrISRSignal( mPeriph, ISRSignal::LINE_IDLE );
        prjDisableReceiver( mPeriph );

        /*-------------------------------------------------
        Update the system state variables
        -------------------------------------------------*/
        mRXTCB.state = StateMachine::RX::RX_ABORTED;
        mRuntimeFlags |= Runtime::Flag::RX_LINE_IDLE_ABORT;

        /*-------------------------------------------------
        Disable the DMA transfer if ongoing
        -------------------------------------------------*/
        // Need interface to get the stream for an IRQ signal
        // Abort the transfer
      }

      /*------------------------------------------------
      Unblock the higher level driver
      ------------------------------------------------*/
      if ( ( mRXTCB.state == StateMachine::RX::RX_ABORTED || mRXTCB.state == StateMachine::RX_COMPLETE ) )
      {
        /*-------------------------------------------------
        Regardless of the state this entered in, the
        transfer is complete now
        -------------------------------------------------*/
        mRXTCB.state = StateMachine::RX_COMPLETE;

        /*------------------------------------------------
        Invoke a user callback if registered
        ------------------------------------------------*/
        signalHLD                            = true;
        const SignalCallback *const callback = INT::getISRHandler( Type::PERIPH_USART, SIG_RX_COMPLETE );
        if ( callback->isrCallback )
        {
          callback->isrCallback();
        }
      }
    }

    /*------------------------------------------------
    Error Related Handler
    ------------------------------------------------*/
    if ( errorFlags )
    {
      /*-------------------------------------------------
      Clear all the error flags
      -------------------------------------------------*/
      Chimera::insert_debug_breakpoint();
      mRuntimeFlags |= errorFlags;

      /*------------------------------------------------
      Invoke a user callback if registered
      ------------------------------------------------*/
      signalHLD                            = true;
      const SignalCallback *const callback = INT::getISRHandler( Type::PERIPH_USART, SIG_ERROR );
      if ( callback->isrCallback )
      {
        callback->isrCallback();
      }
    }

    /*-------------------------------------------------
    Post an event to wake up the user-space handler
    -------------------------------------------------*/
    if ( signalHLD )
    {
      sendTaskMsg( INT::getUserTaskId( Type::PERIPH_USART ), ITCMsg::TSK_MSG_ISR_HANDLER, TIMEOUT_DONT_WAIT );
    }
  }


  /*-------------------------------------------------------------------------------
  Private Methods
  -------------------------------------------------------------------------------*/
  uint32_t Driver::calculateBRR( const size_t desiredBaud )
  {
    size_t periphClock   = 0u;
    size_t calculatedBRR = 0u;
    auto periphAddress   = reinterpret_cast<std::uintptr_t>( mPeriph );

    /*------------------------------------------------
    Figure out the frequency of the clock that drives the USART
    ------------------------------------------------*/
    auto rccSys = Thor::LLD::RCC::getCoreClockCtrl();
    periphClock = rccSys->getPeriphClock( Chimera::Peripheral::Type::PERIPH_USART, periphAddress );

    /*------------------------------------------------
    Protect from div0 conditions in the math below
    ------------------------------------------------*/
    if ( !desiredBaud || !periphClock )
    {
      return 0u;
    }

    /*------------------------------------------------
    Calculate the BRR value. Mostly this was taken directly from
    the STM32 HAL Macros.
    ------------------------------------------------*/
    size_t over8Compensator = 2u;

    if ( OVER8::get( mPeriph ) )
    {
      over8Compensator = 1u;
    }

    auto divisor          = ( 25u * periphClock ) / ( 2u * over8Compensator * desiredBaud );
    auto mantissa_divisor = divisor / 100u;
    auto fraction_divisor = ( ( divisor - ( mantissa_divisor * 100u ) ) * 16u + 50u ) / 100u;
    calculatedBRR         = ( mantissa_divisor << BRR_DIV_Mantissa_Pos ) | ( fraction_divisor & BRR_DIV_Fraction_Msk );

    return static_cast<uint32_t>( calculatedBRR );
  }


  inline void Driver::disableUSARTInterrupts()
  {
    Thor::LLD::INT::disableIRQ( Resource::IRQSignals[ mResourceIndex ] );
  }


  inline void Driver::enableUSARTInterrupts()
  {
    Thor::LLD::INT::enableIRQ( Resource::IRQSignals[ mResourceIndex ] );
  }


  void Driver::onDMATXComplete( const Chimera::DMA::TransferStats &stats )
  {
    if( stats.error )
    {
      // Anything to do?
    }
    else
    {
      mTXTCB.state = StateMachine::TX::TX_COMPLETE;
      mTXTCB.remaining = mTXTCB.expected - stats.size;

      IRQHandler();
    }
  }


  void Driver::onDMARXComplete( const Chimera::DMA::TransferStats &stats )
  {
    Chimera::insert_debug_breakpoint();
  }

}    // namespace Thor::LLD::USART

#endif /* TARGET_STM32L4 && THOR_DRIVER_USART */
