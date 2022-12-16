/******************************************************************************
 *  File Name:
 *    usart_common_driver.cpp
 *
 *  Description:
 *    Shared low level driver for the USART peripheral. This focuses on very
 *    basic functionality, like reading and writing byte streams.
 *
 *  2021-2022 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <Chimera/interrupt>
#include <Chimera/serial>
#include <Chimera/thread>
#include <Thor/cfg>
#include <Thor/dma>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/inc/dma>
#include <Thor/lld/interface/inc/interrupt>
#include <Thor/lld/interface/inc/rcc>
#include <Thor/lld/interface/inc/usart>
#include <Thor/lld/interface/usart/common_driver/usart_common_intf.hpp>

#if defined( THOR_USART ) && ( defined( TARGET_STM32L4 ) || defined( TARGET_STM32F4 ) )
namespace Thor::LLD::USART
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static Driver s_usart_drivers[ NUM_USART_PERIPHS ];


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    if ( attachDriverInstances( s_usart_drivers, ARRAY_COUNT( s_usart_drivers ) ) )
    {
      return Chimera::Status::OK;
    }
    else
    {
      return Chimera::Status::FAIL;
    }
  }


  bool isChannelSupported( const Chimera::Serial::Channel channel )
  {
    if ( channel < Chimera::Serial::Channel::NUM_OPTIONS )
    {
      return ( getResourceIndex( channel ) != INVALID_RESOURCE_INDEX );
    }
    else
    {
      return false;
    }
  }


  Driver_rPtr getDriver( const Chimera::Serial::Channel channel )
  {
    if ( isChannelSupported( channel ) )
    {
      return &s_usart_drivers[ static_cast<size_t>( channel ) ];
    }

    return nullptr;
  }


  /*---------------------------------------------------------------------------
  Public Methods
  ---------------------------------------------------------------------------*/
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


  Chimera::Status_t Driver::init( const Thor::LLD::Serial::RegConfig &cfg )
  {
    /*-------------------------------------------------------------------------
    Initialize driver memory
    -------------------------------------------------------------------------*/
    mRuntimeFlags  = static_cast<Runtime::Flag_t>( 0 );
    mDMAPipesReady = false;
    mTXTCB.reset();
    mRXTCB.reset();

    /*-------------------------------------------------------------------------
    Enable the peripheral clock and reset the hardware
    -------------------------------------------------------------------------*/
    auto rccPeriph = Thor::LLD::RCC::getPeriphClockCtrl();
    rccPeriph->enableClock( Chimera::Peripheral::Type::PERIPH_USART, mResourceIndex );
    rccPeriph->reset( Chimera::Peripheral::Type::PERIPH_USART, mResourceIndex );

    /*-------------------------------------------------------------------------
    Follow the initialization sequence as defined in
    RM0394 Rev 4, pg.1202
    -------------------------------------------------------------------------*/
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

    /* Clear DMA request lines */
    DMAR::clear( mPeriph, CR3_DMAR );
    DMAT::clear( mPeriph, CR3_DMAT );

    /* Enable the USART by writing the UE bit to 1 */
    UE::set( mPeriph, CR1_UE );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::deinit()
  {
    /*-------------------------------------------------------------------------
    Use the RCC's ability to reset whole peripherals back to default states
    -------------------------------------------------------------------------*/
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


  int Driver::transmit( const Chimera::Serial::TxfrMode mode, const void *const data, const size_t size )
  {
    using namespace Chimera::Serial;

    auto result = Chimera::Status::OK;
    switch ( mode )
    {
      case TxfrMode::BLOCKING:
        result = this->txBlocking( data, size );
        break;

      case TxfrMode::INTERRUPT:
        result = this->txInterrupt( data, size );
        break;

      case TxfrMode::DMA:
        result = this->txDMA( data, size );
        break;

      default:
        return -1;
    }

    return ( result == Chimera::Status::OK ) ? size : -1;
  }

  Chimera::Status_t Driver::txBlocking( const void *const data, const size_t size )
  {
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if ( !data || !size )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------------------------------
    Pump the data as the transmit FIFO indicates it can hold more data
    -------------------------------------------------------------------------*/
    prjEnableTransmitter( mPeriph );

    auto dataPtr = reinterpret_cast<const uint8_t *const>( data );
    for ( size_t idx = 0; idx < size; idx++ )
    {
      while ( TXE::get( mPeriph ) == 0 )
      {
        continue;
      }

      prjWriteDataRegister( mPeriph, dataPtr[ idx ] );
    }

    /*-------------------------------------------------------------------------
    Wait for the last byte to finish transferring
    -------------------------------------------------------------------------*/
    while ( TC::get( mPeriph ) == 0 )
    {
      continue;
    }
    prjDisableTransmitter( mPeriph );
    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::txInterrupt( const void *const data, const size_t size )
  {
    using namespace Chimera::Hardware;

    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if ( !data || !size )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------------------------------
    Ensure the previous transfer has completed
    -------------------------------------------------------------------------*/
    auto status = txTransferStatus();
    if ( ( status != StateMachine::TX::TX_COMPLETE ) && ( status != StateMachine::TX::TX_READY ) )
    {
      return Chimera::Status::BUSY;
    }
    else    // No on-going transfers
    {
      auto transferData = reinterpret_cast<const uint8_t *const>( data );

      /*-----------------------------------------------------------------------
      Prep the transfer control block for the first TDR empty ISR event
      -----------------------------------------------------------------------*/
      disableUSARTInterrupts();

      mTXTCB.buffer    = const_cast<uint8_t *>( transferData );
      mTXTCB.offset    = 1;
      mTXTCB.remaining = size - 1u;
      mTXTCB.expected  = size;
      mTXTCB.state     = StateMachine::TX::TX_ONGOING;
      mTXTCB.mode      = PeripheralMode::INTERRUPT;

      /*-----------------------------------------------------------------------
      Shove the first byte into the transmit data register
      -----------------------------------------------------------------------*/
      prjWriteDataRegister( mPeriph, transferData[ 0 ] );

      /*-----------------------------------------------------------------------
      Turn on the TDR empty interrupt so we know when to store the next byte
      -----------------------------------------------------------------------*/
      prjEnableISRSignal( mPeriph, ISRSignal::TRANSMIT_DATA_REG_EMPTY );
      enableIT( Chimera::Hardware::SubPeripheral::TX );
      enableUSARTInterrupts();

      /*-----------------------------------------------------------------------
      Start the transfer
      -----------------------------------------------------------------------*/
      prjEnableTransmitter( mPeriph );
    }

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::txDMA( const void *const data, const size_t size )
  {
    using namespace Chimera::DMA;
    using namespace Chimera::Hardware;

#if !defined( THOR_DMA )
    return Chimera::Status::NOT_SUPPORTED;
#else
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if ( !data || !size )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------------------------------
    Ensure the previous transfer has completed
    -------------------------------------------------------------------------*/
    auto status = txTransferStatus();
    if ( ( status != StateMachine::TX::TX_COMPLETE ) && ( status != StateMachine::TX::TX_READY ) )
    {
      return Chimera::Status::BUSY;
    }

    /*-------------------------------------------------------------------------
    Configure the DMA pipe for the transaction
    -------------------------------------------------------------------------*/
    if( !mDMAPipesReady )
    {
      initDMA();
    }

    disableUSARTInterrupts();
    {
      /* Clear any previous DMA request */
      DMAT::clear( mPeriph, CR3_DMAT );

      /* Clear the transfer complete bit */
#if defined( STM32F446xx )
      TC::clear( mPeriph, SR_TC );
#elif defined( STM32L432xx )
      TC::clear( mPeriph, ISR_TC );
#endif

      /* Ensure the transmitter is turned on */
      prjEnableTransmitter( mPeriph );

      /* Configure the DMA transfer */
      PipeTransfer cfg;
      cfg.isrCallback  = TransferCallback::create<Driver, &Driver::onDMATXComplete>( *this );
      cfg.pipe         = mTXDMARequestId;
      cfg.size         = size;
      cfg.addr         = reinterpret_cast<std::uintptr_t>( data );
      Chimera::DMA::transfer( cfg );

      /* Configure the USART transfer control block */
      mTXTCB.buffer    = const_cast<uint8_t *>( reinterpret_cast<const uint8_t *const>( data ) );
      mTXTCB.expected  = size;
      mTXTCB.remaining = size;
      mTXTCB.state     = StateMachine::TX::TX_ONGOING;
      mTXTCB.mode      = PeripheralMode::DMA;

    }
    enableUSARTInterrupts();

    /*-------------------------------------------------------------------------
    Start the transfer. The USART is the transaction controller.
    -------------------------------------------------------------------------*/
    DMAT::set( mPeriph, CR3_DMAT );
    return Chimera::Status::OK;
#endif /* THOR_LLD_DMA */
  }


  int Driver::receive( const Chimera::Serial::TxfrMode mode, void *const data, const size_t size )
  {
    // TODO: Add this back
    return 0;
  }


  Chimera::Status_t Driver::enableIT( const Chimera::Hardware::SubPeripheral subPeriph )
  {
    /*-------------------------------------------------------------------------
    Configure and enable the global interrupt, ignoring the subPeriph argument
    as the RTX functions called by the user handle them appropriately.
    -------------------------------------------------------------------------*/
    INT::setPriority( Resource::IRQSignals[ mResourceIndex ], INT::USART_IT_PREEMPT_PRIORITY, 0u );
    INT::enableIRQ( Resource::IRQSignals[ mResourceIndex ] );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::disableIT( const Chimera::Hardware::SubPeripheral subPeriph )
  {
    using namespace Chimera::Hardware;

    /*-------------------------------------------------------------------------
    Disable peripheral level interrupts to prevent glitches
    -------------------------------------------------------------------------*/
    disableUSARTInterrupts();

    /*-------------------------------------------------------------------------
    Disable interrupt event generation inside the peripheral
    -------------------------------------------------------------------------*/
    Chimera::Status_t result = Chimera::Status::OK;

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

    /*-------------------------------------------------------------------------
    Re-enable the peripheral level interrupts so other events can get through
    -------------------------------------------------------------------------*/
    enableUSARTInterrupts();
    return result;
  }


  Chimera::Status_t Driver::receiveIT( void *const data, const size_t size )
  {
    using namespace Chimera::Hardware;

    /*-------------------------------------------------------------------------
    Input protection
    -------------------------------------------------------------------------*/
    if ( !data || !size )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------------------------------
    Ensure that the ISR has processed all data in the FIFO
    -------------------------------------------------------------------------*/
    auto status = rxTransferStatus();
    if ( ( status != StateMachine::RX::RX_COMPLETE ) && ( status != StateMachine::RX::RX_READY ) )
    {
      return Chimera::Status::BUSY;
    }
    else    // All bytes have been read out of the FIFO
    {
      disableUSARTInterrupts();
      enableIT( Chimera::Hardware::SubPeripheral::RX );

      /*-----------------------------------------------------------------------
      Only turn on RXNE so as to detect when the first byte arrives
      -----------------------------------------------------------------------*/
      prjEnableISRSignal( mPeriph, ISRSignal::RECEIVED_DATA_READY );

      /*-----------------------------------------------------------------------
      Prep the transfer control block to receive data
      -----------------------------------------------------------------------*/
      mRXTCB.buffer    = reinterpret_cast<uint8_t *const>( data );
      mRXTCB.expected  = size;
      mRXTCB.remaining = size;
      mRXTCB.state     = StateMachine::RX::RX_ONGOING;
      mRXTCB.mode      = PeripheralMode::INTERRUPT;

      /*-----------------------------------------------------------------------
      Turn on the RX hardware to begin listening for data
      -----------------------------------------------------------------------*/
      prjEnableReceiver( mPeriph );
      enableUSARTInterrupts();
    }

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::initDMA()
  {
    using namespace Chimera::DMA;
    using namespace Chimera::Peripheral;

#if !defined( THOR_DMA )
    return Chimera::Status::NOT_SUPPORTED;
#else
    /*-------------------------------------------------------------------------
    Configure the TX pipe
    -------------------------------------------------------------------------*/
    PipeConfig txCfg;
    txCfg.srcAlignment  = Alignment::BYTE;
    txCfg.dstAlignment  = Alignment::BYTE;
    txCfg.direction     = Direction::MEMORY_TO_PERIPH;
    txCfg.mode          = Mode::DIRECT;
    txCfg.priority      = Priority::MEDIUM;
    txCfg.resourceIndex = DMA::getResourceIndex( Resource::TXDMASignals[ mResourceIndex ] );
    txCfg.channel       = static_cast<size_t>( DMA::getChannel( Resource::TXDMASignals[ mResourceIndex ] ) );
    txCfg.threshold     = FifoThreshold::NONE;

#if defined( TARGET_STM32F4 )
    txCfg.periphAddr    = reinterpret_cast<std::uintptr_t>( &mPeriph->DR );
#elif defined( TARGET_STM32L4 )
    txCfg.periphAddr = reinterpret_cast<std::uintptr_t>( &mPeriph->TDR );
#else
#error Need to fill this out
#endif

    /*-------------------------------------------------------------------------
    Configure the RX pipe
    -------------------------------------------------------------------------*/
    PipeConfig rxCfg;
    rxCfg.srcAlignment  = Alignment::BYTE;
    rxCfg.dstAlignment  = Alignment::BYTE;
    rxCfg.direction     = Direction::PERIPH_TO_MEMORY;
    rxCfg.mode          = Mode::DIRECT;
    rxCfg.priority      = Priority::MEDIUM;
    rxCfg.resourceIndex = DMA::getResourceIndex( Resource::RXDMASignals[ mResourceIndex ] );
    rxCfg.channel       = static_cast<size_t>( DMA::getChannel( Resource::RXDMASignals[ mResourceIndex ] ) );
    rxCfg.threshold     = FifoThreshold::NONE;

#if defined( TARGET_STM32F4 )
    rxCfg.periphAddr    = reinterpret_cast<std::uintptr_t>( &mPeriph->DR );
#elif defined( TARGET_STM32L4 )
    rxCfg.periphAddr = reinterpret_cast<std::uintptr_t>( &mPeriph->RDR );
#else
#error Need to fill this out
#endif

    /*-------------------------------------------------------------------------
    FIFO errors are thrown even though we don't use it. Doesn't affect TX-ing.
    -------------------------------------------------------------------------*/
    txCfg.errorsToIgnore = Errors::FIFO;
    rxCfg.errorsToIgnore = Errors::FIFO;

    /*-------------------------------------------------------------------------
    Construct the pipe and make a note of it's UUID
    -------------------------------------------------------------------------*/
    mTXDMARequestId = Chimera::DMA::constructPipe( txCfg );
    mRXDMARequestId = Chimera::DMA::constructPipe( rxCfg );
    mDMAPipesReady  = ( mTXDMARequestId != INVALID_REQUEST ) && ( mRXDMARequestId != INVALID_REQUEST );

    /*-------------------------------------------------------------------------
    Configure the peripheral interrupts
    -------------------------------------------------------------------------*/
    enableIT( Chimera::Hardware::SubPeripheral::TXRX );

    return Chimera::Status::OK;
#endif /* !THOR_LLD_DMA */
  }


  Chimera::Status_t Driver::deinitDMA()
  {
    mDMAPipesReady = false;
    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::enableDMA_IT( const Chimera::Hardware::SubPeripheral periph )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::disableDMA_IT( const Chimera::Hardware::SubPeripheral periph )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::receiveDMA( void *const data, const size_t size )
  {
    using namespace Chimera::DMA;
    using namespace Chimera::Hardware;

#if !defined( THOR_DMA )
    return Chimera::Status::NOT_SUPPORTED;
#else
    /*-------------------------------------------------------------------------
    Input protection
    -------------------------------------------------------------------------*/
    if ( !data || !size )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------------------------------
    Ensure that the ISR has processed all data in the FIFO
    -------------------------------------------------------------------------*/
    auto status = rxTransferStatus();
    if ( ( status != StateMachine::RX::RX_COMPLETE ) && ( status != StateMachine::RX::RX_READY ) )
    {
      return Chimera::Status::BUSY;
    }


    initDMA();
    disableUSARTInterrupts();
    {
      /*-----------------------------------------------------------------------
      Instruct the USART to use DMA mode
      -----------------------------------------------------------------------*/
      DMAR::set( mPeriph, CR3_DMAR );

      /*-----------------------------------------------------------------------
      Make sure line idle ISR is enabled. Regardless of
      if there is data left to RX, the sender might just
      suddenly stop, and that needs detection.
      -----------------------------------------------------------------------*/
      prjEnableISRSignal( mPeriph, ISRSignal::LINE_IDLE );
      prjClrISRSignal( mPeriph, ISRSignal::LINE_IDLE );

      /*-----------------------------------------------------------------------
      Prep the transfer control block to receive data
      -----------------------------------------------------------------------*/
      /* Configure the DMA transfer */
      PipeTransfer cfg;
      cfg.userCallback = TransferCallback::create<Driver, &Driver::onDMARXComplete>( *this );
      cfg.pipe         = mRXDMARequestId;
      cfg.size         = size;
      cfg.addr         = reinterpret_cast<std::uintptr_t>( data );

      /* Configure the USART transfer control block */
      mRXTCB.buffer    = reinterpret_cast<uint8_t *const>( data );
      mRXTCB.expected  = size;
      mRXTCB.remaining = size;
      mRXTCB.state     = StateMachine::RX::RX_ONGOING;
      mRXTCB.mode      = PeripheralMode::DMA;


      Chimera::DMA::transfer( cfg );

      /*-----------------------------------------------------------------------
      Turn on the RX hardware to begin listening for data
      -----------------------------------------------------------------------*/
      prjEnableReceiver( mPeriph );
      enableUSARTInterrupts();
    }

    return Chimera::Status::OK;
#endif /* THOR_LLD_DMA */
  }


  Chimera::Status_t Driver::txTransferStatus()
  {
    return mTXTCB.state;
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


  void Driver::killTransfer( Chimera::Hardware::SubPeripheral periph )
  {
    using namespace Chimera::Hardware;

    disableUSARTInterrupts();

    if ( periph == SubPeripheral::RX || ( periph == SubPeripheral::TXRX ) )
    {
      disableIT( Chimera::Hardware::SubPeripheral::RX );
      disableDMA_IT( Chimera::Hardware::SubPeripheral::RX );
      mRXTCB.reset();
    }

    if ( periph == SubPeripheral::TX || ( periph == SubPeripheral::TXRX ) )
    {
      disableIT( Chimera::Hardware::SubPeripheral::TX );
      disableDMA_IT( Chimera::Hardware::SubPeripheral::TX );
      mTXTCB.reset();
    }

    enableUSARTInterrupts();
  }


  Thor::LLD::Serial::CDTCB *Driver::getTCB_TX()
  {
    return &mTXTCB;
  }


  Thor::LLD::Serial::MDTCB *Driver::getTCB_RX()
  {
    return &mRXTCB;
  }

  /*---------------------------------------------------------------------------
  Protected Methods
  ---------------------------------------------------------------------------*/
  void Driver::IRQHandler()
  {
    using namespace Chimera::Hardware;
    using namespace Chimera::Interrupt;
    using namespace Chimera::Peripheral;
    using namespace Chimera::Serial;
    using namespace Chimera::Thread;
    using namespace Configuration::Flags;

    /*-------------------------------------------------------------------------
    Cache the current state of the registers
    -------------------------------------------------------------------------*/
    const uint32_t ISRCache  = ISR::get( mPeriph );
    const uint32_t CR1Cache  = CR1::get( mPeriph );
    bool           signalHLD = false;

    /*-------------------------------------------------------------------------
    Figure out which interrupt flags have been set
    -------------------------------------------------------------------------*/
    uint32_t txFlags    = ISRCache & ( FLAG_TC | FLAG_TXE );
    uint32_t rxFlags    = ISRCache & ( FLAG_RXNE | FLAG_IDLE );
    uint32_t errorFlags = ISRCache & ( FLAG_ORE | FLAG_PE | FLAG_NF | FLAG_FE );

    /*-------------------------------------------------------------------------
    TX Related Handler
    -------------------------------------------------------------------------*/
    if ( txFlags && ( ( CR1Cache & CR1_TXEIE ) || ( CR1Cache & CR1_TCIE ) ) )
    {
      /*-----------------------------------------------------------------------
      No txfr ongoing? Invalid state. Clear TX related ISR flags.
      -----------------------------------------------------------------------*/
      if ( mTXTCB.state == StateMachine::TX::TX_READY )
      {
        txFlags = 0;
        prjClrISRSignal( mPeriph, ISRSignal::TRANSMIT_COMPLETE );
        prjDisableISRSignal( mPeriph, ISRSignal::TRANSMIT_COMPLETE );
        prjDisableISRSignal( mPeriph, ISRSignal::TRANSMIT_DATA_REG_EMPTY );
      }

      /*-----------------------------------------------------------------------
      Transmit Data Register Empty Interrupt
      -----------------------------------------------------------------------*/
      if ( ( txFlags & FLAG_TXE ) && ( mTXTCB.state == StateMachine::TX::TX_ONGOING ) )
      {
        if ( mTXTCB.remaining )    // Data left to be transmitted
        {
          /*-------------------------------------------------------------------
          Make sure the TX complete interrupt cannot fire
          -------------------------------------------------------------------*/
          prjDisableISRSignal( mPeriph, ISRSignal::TRANSMIT_COMPLETE );

          /*-------------------------------------------------------------------
          Transfer the byte and prep for the next character
          -------------------------------------------------------------------*/
          prjWriteDataRegister( mPeriph, mTXTCB.buffer[ mTXTCB.offset ] );
          mTXTCB.offset++;
          mTXTCB.remaining--;
        }
        else    // All data has been transmitted to TX FIFO
        {
          /*-------------------------------------------------------------------
          We finished pushing the last character into the TDR, so now we can
          listen for the TX complete interrupt.
          -------------------------------------------------------------------*/
          prjDisableISRSignal( mPeriph, ISRSignal::TRANSMIT_DATA_REG_EMPTY );
          prjEnableISRSignal( mPeriph, ISRSignal::TRANSMIT_COMPLETE );

          mTXTCB.state = StateMachine::TX_COMPLETE;
        }
      }

      /*-----------------------------------------------------------------------
      Transfer Complete Interrupt
      -----------------------------------------------------------------------*/
      if ( ( txFlags & FLAG_TC ) && ( mTXTCB.state == StateMachine::TX::TX_COMPLETE ) )
      {
        signalHLD = true;
        mRuntimeFlags |= Runtime::Flag::TX_COMPLETE;

        /*---------------------------------------------------------------------
        Exit the TX ISR by disabling related interrupts
        ---------------------------------------------------------------------*/
        prjClrISRSignal( mPeriph, ISRSignal::TRANSMIT_COMPLETE );
        prjDisableISRSignal( mPeriph, ISRSignal::TRANSMIT_COMPLETE );
        prjDisableISRSignal( mPeriph, ISRSignal::TRANSMIT_DATA_REG_EMPTY );

        /*---------------------------------------------------------------------
        Invoke a user callback if registered
        ---------------------------------------------------------------------*/
        const SignalCallback *const callback = INT::getISRHandler( Type::PERIPH_USART, SIG_TX_COMPLETE );
        if ( callback->isrCallback )
        {
          callback->isrCallback();
        }
      }
    }

    /*-------------------------------------------------------------------------
    RX Related Handler
    -------------------------------------------------------------------------*/
    if ( rxFlags )
    {
      /*-----------------------------------------------------------------------
      RX register not empty, aka a new byte has arrived!
      -----------------------------------------------------------------------*/
      if ( ( rxFlags & FLAG_RXNE ) && ( CR1Cache & CR1_RXNEIE ) && ( mRXTCB.state == StateMachine::RX::RX_ONGOING ) )
      {
        /*---------------------------------------------------------------------
        Make sure line idle ISR is enabled. Regardless of
        if there is data left to RX, the sender might just
        suddenly stop, and that needs detection.
        ---------------------------------------------------------------------*/
        prjEnableISRSignal( mPeriph, ISRSignal::LINE_IDLE );
        prjClrISRSignal( mPeriph, ISRSignal::LINE_IDLE );

        /*---------------------------------------------------------------------
        Read out current byte and prep for next transfer
        ---------------------------------------------------------------------*/
        *mRXTCB.buffer = static_cast<uint8_t>( prjReadDataRegister( mPeriph ) );
        mRXTCB.buffer++;
        mRXTCB.remaining--;

        /*---------------------------------------------------------------------
        If no more bytes left to receive, stop listening
        ---------------------------------------------------------------------*/
        if ( !mRXTCB.remaining )
        {
          prjDisableISRSignal( mPeriph, ISRSignal::LINE_IDLE );
          prjDisableReceiver( mPeriph );

          mRXTCB.state = StateMachine::RX::RX_COMPLETE;
          mRuntimeFlags |= Runtime::Flag::RX_COMPLETE;
        }
      }

      /*-----------------------------------------------------------------------
      Line Idle: We were in the middle of a transfer and
      then suddenly they just stopped sending data.
      -----------------------------------------------------------------------*/
      if ( ( rxFlags & FLAG_IDLE ) && ( CR1Cache & CR1_IDLEIE ) && ( mRXTCB.state == StateMachine::RX::RX_ONGOING ) )
      {
        /*---------------------------------------------------------------------
        Clear the appropriate flags
        ---------------------------------------------------------------------*/
        prjDisableISRSignal( mPeriph, ISRSignal::LINE_IDLE );
        prjClrISRSignal( mPeriph, ISRSignal::LINE_IDLE );
        prjDisableReceiver( mPeriph );

        /*---------------------------------------------------------------------
        Update the system state variables
        ---------------------------------------------------------------------*/
        mRXTCB.state = StateMachine::RX::RX_ABORTED;
        mRuntimeFlags |= Runtime::Flag::RX_LINE_IDLE_ABORT;

        /*---------------------------------------------------------------------
        Disable the DMA transfer if ongoing
        ---------------------------------------------------------------------*/
#if defined( THOR_DMA )
        if ( mRXTCB.mode == PeripheralMode::DMA )
        {
          RIndex_t dmaIdx    = Thor::LLD::DMA::getResourceIndex( Resource::RXDMASignals[ this->mResourceIndex ] );
          auto     dmaStream = Thor::LLD::DMA::getStream( dmaIdx );

          if ( dmaStream )
          {
            dmaStream->abort();
          }
        }
#endif /* THOR_LLD_DMA */
      }

      /*-----------------------------------------------------------------------
      Unblock the higher level driver
      -----------------------------------------------------------------------*/
      if ( ( mRXTCB.state == StateMachine::RX::RX_ABORTED || mRXTCB.state == StateMachine::RX_COMPLETE ) )
      {
        /*---------------------------------------------------------------------
        Regardless of the state this entered in, the
        transfer is complete now
        ---------------------------------------------------------------------*/
        mRXTCB.state = StateMachine::RX_COMPLETE;

        /*---------------------------------------------------------------------
        Invoke a user callback if registered
        ---------------------------------------------------------------------*/
        signalHLD                            = true;
        const SignalCallback *const callback = INT::getISRHandler( Type::PERIPH_USART, SIG_RX_COMPLETE );
        if ( callback->isrCallback )
        {
          callback->isrCallback();
        }
      }
    }

    /*-------------------------------------------------------------------------
    Error Related Handler
    -------------------------------------------------------------------------*/
    if ( errorFlags )
    {
      signalHLD = true;
      mRuntimeFlags |= errorFlags;

      /*-----------------------------------------------------------------------
      Invoke a user callback if registered
      -----------------------------------------------------------------------*/
      const SignalCallback *const callback = INT::getISRHandler( Type::PERIPH_USART, SIG_ERROR );
      if ( callback->isrCallback )
      {
        callback->isrCallback();
      }
    }

    /*-------------------------------------------------------------------------
    Post an event to wake up the post processor thread
    -------------------------------------------------------------------------*/
    if ( signalHLD )
    {
      sendTaskMsg( INT::getUserTaskId( Type::PERIPH_USART ), ITCMsg::TSK_MSG_ISR_HANDLER, TIMEOUT_DONT_WAIT );
    }
  }


  /*---------------------------------------------------------------------------
  Private Methods
  ---------------------------------------------------------------------------*/
  uint32_t Driver::calculateBRR( const size_t desiredBaud )
  {
    size_t periphClock   = 0u;
    size_t calculatedBRR = 0u;
    auto   periphAddress = reinterpret_cast<std::uintptr_t>( mPeriph );

    /*-------------------------------------------------------------------------
    Figure out the frequency of the clock that drives the USART
    -------------------------------------------------------------------------*/
    auto rccSys = Thor::LLD::RCC::getCoreClockCtrl();
    periphClock = rccSys->getPeriphClock( Chimera::Peripheral::Type::PERIPH_USART, periphAddress );

    /*-------------------------------------------------------------------------
    Protect from div0 conditions in the math below
    -------------------------------------------------------------------------*/
    if ( !desiredBaud || !periphClock )
    {
      return 0u;
    }

    /*-------------------------------------------------------------------------
    Calculate the BRR value. This was taken directly from the STM32 HAL macros.
    -------------------------------------------------------------------------*/
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
    if ( stats.error )
    {
      // Anything to do?
    }
    else
    {
      /*-----------------------------------------------------------------------
      Prepare the ISR handler for the proper control flow
      -----------------------------------------------------------------------*/
      mTXTCB.state     = StateMachine::TX::TX_COMPLETE;
      mTXTCB.remaining = mTXTCB.expected - stats.size;

      /*-----------------------------------------------------------------------
      Use the default ISR handler to manage the transfer complete behavior
      -----------------------------------------------------------------------*/
      DMAT::clear( mPeriph, CR3_DMAT );
      prjEnableISRSignal( mPeriph, ISRSignal::TRANSMIT_COMPLETE );
      IRQHandler();
    }
  }


  void Driver::onDMARXComplete( const Chimera::DMA::TransferStats &stats )
  {
#if defined( THOR_DMA )
    /*-------------------------------------------------------------------------
    Per DMA RX instructions, disable USART DMA at the end of the DMA interrupt
    -------------------------------------------------------------------------*/
    DMAR::clear( mPeriph, CR3_DMAR );

    Chimera::insert_debug_breakpoint();
#endif
  }

}    // namespace Thor::LLD::USART

/*-----------------------------------------------------------------------------
IRQ Handlers
-----------------------------------------------------------------------------*/
#if defined( STM32_USART1_PERIPH_AVAILABLE )
void USART1_IRQHandler( void )
{
  using namespace Thor::LLD::USART;
  s_usart_drivers[ USART1_RESOURCE_INDEX ].IRQHandler();
}
#endif /* STM32_USART1_PERIPH_AVAILABLE */


#if defined( STM32_USART2_PERIPH_AVAILABLE )
void USART2_IRQHandler( void )
{
  using namespace Thor::LLD::USART;
  s_usart_drivers[ USART2_RESOURCE_INDEX ].IRQHandler();
}
#endif /* STM32_USART2_PERIPH_AVAILABLE */


#if defined( STM32_USART3_PERIPH_AVAILABLE )
void USART3_IRQHandler( void )
{
  using namespace Thor::LLD::USART;
  s_usart_drivers[ USART3_RESOURCE_INDEX ].IRQHandler();
}
#endif /* STM32_USART3_PERIPH_AVAILABLE */

#endif /* TARGET_STM32L4 && THOR_DRIVER_USART */
