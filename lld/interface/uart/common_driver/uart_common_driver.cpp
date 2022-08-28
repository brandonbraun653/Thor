/********************************************************************************
 *  File Name:
 *    uart_common_driver.cpp
 *
 *  Description:
 *    Shared low level driver for STM32 UART peripherals
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
#include <Thor/lld/interface/inc/interrupt>
#include <Thor/lld/interface/inc/rcc>
#include <Thor/lld/interface/inc/uart>
#include <Thor/lld/interface/uart/common_driver/uart_common_intf.hpp>

#if defined( THOR_UART ) && ( defined( TARGET_STM32L4 ) || defined( TARGET_STM32F4 ) )

namespace Thor::LLD::UART
{
  /*-------------------------------------------------------------------------------
  Public Methods
  -------------------------------------------------------------------------------*/
  Driver::Driver() : periph( nullptr )
  {
  }


  Driver::~Driver()
  {
  }


  Chimera::Status_t Driver::attach( RegisterMap *peripheral )
  {
    if ( auto idx = getResourceIndex( reinterpret_cast<std::uintptr_t>( peripheral ) ); idx != INVALID_RESOURCE_INDEX )
    {
      periph        = peripheral;
      resourceIndex = idx;

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
    runtimeFlags = static_cast<Runtime::Flag_t>( 0 );
    txTCB.reset();
    rxTCB.reset();

    /*------------------------------------------------
    Enable the peripheral clock and reset the hardware
    ------------------------------------------------*/
    auto rccPeriph = Thor::LLD::RCC::getPeriphClockCtrl();
    rccPeriph->enableClock( Chimera::Peripheral::Type::PERIPH_UART, resourceIndex );
    rccPeriph->reset( Chimera::Peripheral::Type::PERIPH_UART, resourceIndex );

    /*------------------------------------------------
    Follow the initialization sequence as defined in
    RM0394 Rev 4, pg.1202
    ------------------------------------------------*/
    /* Disable the UART by writing the UE bit to 0 */
    UE::set( periph, 0 );

    /* Program the M0 bit to define the word length. M1 Currently not supported. */
    M0::set( periph, cfg.WordLength );

    /* Program the number of stop bits */
    STOP::set( periph, cfg.StopBits );

    /* Program the parity */
    switch ( cfg.Parity )
    {
      case Configuration::Parity::EVEN:
        PCE::set( periph, CR1_PCE );
        PS::clear( periph, CR1_PS );
        break;

      case Configuration::Parity::ODD:
        PCE::set( periph, CR1_PCE );
        PS::set( periph, CR1_PS );
        break;

      case Configuration::Parity::NONE:
      default:
        PCE::clear( periph, CR1_PCE );
        break;
    };

    /* Select the desired baud rate */
    BRR::set( periph, calculateBRR( cfg.BaudRate ) );

    /* Enable the UART by writing the UE bit to 1 */
    UE::set( periph, CR1_UE );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::deinit()
  {
    /*-------------------------------------------------
    Use the RCC's ability to reset whole peripherals back to default states
    -------------------------------------------------*/
    auto rcc = Thor::LLD::RCC::getPeriphClockCtrl();
    rcc->enableClock( Chimera::Peripheral::Type::PERIPH_UART, resourceIndex );
    rcc->reset( Chimera::Peripheral::Type::PERIPH_UART, resourceIndex );
    rcc->disableClock( Chimera::Peripheral::Type::PERIPH_UART, resourceIndex );

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
    /*-------------------------------------------------
    Ignore the subperiph argument as the actual transmit
    and receive functions called by the user handle that.
    -------------------------------------------------*/
    INT::setPriority( Resource::IRQSignals[ resourceIndex ], INT::UART_IT_PREEMPT_PRIORITY, 0u );
    INT::enableIRQ( Resource::IRQSignals[ resourceIndex ] );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::disableIT( const Chimera::Hardware::SubPeripheral subPeriph )
  {
    using namespace Chimera::Hardware;

    Chimera::Status_t result = Chimera::Status::OK;

    /*-------------------------------------------------
    Disable system level interrupt from peripheral, to prevent glitching
    -------------------------------------------------*/
    INT::disableIRQ( Resource::IRQSignals[ resourceIndex ] );
    INT::clearPendingIRQ( Resource::IRQSignals[ resourceIndex ] );

    /*-------------------------------------------------
    Disable interrupt event generation inside the peripheral
    -------------------------------------------------*/
    if ( ( subPeriph == SubPeripheral::TX ) || ( subPeriph == SubPeripheral::TXRX ) )
    {
      prjDisableISRSignal( periph, ISRSignal::TRANSMIT_COMPLETE );
      prjDisableISRSignal( periph, ISRSignal::TRANSMIT_DATA_REG_EMPTY );
    }
    else if ( ( subPeriph == SubPeripheral::RX ) || ( subPeriph == SubPeripheral::TXRX ) )
    {
      prjDisableISRSignal( periph, ISRSignal::RECEIVED_DATA_READY );
    }
    else
    {
      result = Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------
    Re-enable the system level interrupts so other events can get through
    -------------------------------------------------*/
    Thor::LLD::INT::enableIRQ( Resource::IRQSignals[ resourceIndex ] );

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
      enterCriticalSection();

      txTCB.buffer    = &transferData[ 1 ]; /* Point to the next byte */
      txTCB.expected  = size;
      txTCB.remaining = size - 1u; /* Pre-decrement to account for this first byte TX */
      txTCB.state     = StateMachine::TX::TX_ONGOING;

      /*------------------------------------------------
      Shove the byte into the transmit data register,
      kicking off the transfer.
      ------------------------------------------------*/
      prjWriteDataRegister( periph, transferData[ 0 ] );

      /*------------------------------------------------
      Turn on the transmitter & enable TDR interrupt so
      we know when to stage the next byte.
      ------------------------------------------------*/
      prjEnableTransmitter( periph );
      prjEnableISRSignal( periph, ISRSignal::TRANSMIT_DATA_REG_EMPTY );
      enableIT( Chimera::Hardware::SubPeripheral::TX );

      exitCriticalSection();
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
      enterCriticalSection();
      enableIT( Chimera::Hardware::SubPeripheral::RX );

      /*------------------------------------------------
      Only turn on RXNE so as to detect when the first byte arrives
      ------------------------------------------------*/
      prjEnableISRSignal( periph, ISRSignal::RECEIVED_DATA_READY );

      /*------------------------------------------------
      Prep the transfer control block to receive data
      ------------------------------------------------*/
      rxTCB.buffer    = reinterpret_cast<uint8_t *const>( data );
      rxTCB.expected  = size;
      rxTCB.remaining = size;
      rxTCB.state     = StateMachine::RX::RX_ONGOING;

      /*------------------------------------------------
      Turn on the RX hardware to begin listening for data
      ------------------------------------------------*/
      prjEnableReceiver( periph );
      exitCriticalSection();
    }

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::initDMA()
  {
    return Chimera::Status::NOT_SUPPORTED;
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
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::receiveDMA( void *const data, const size_t size )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::txTransferStatus()
  {
    Chimera::Status_t cacheStatus = Chimera::Status::UNKNOWN_ERROR;

    enterCriticalSection();
    cacheStatus = txTCB.state;
    exitCriticalSection();

    return cacheStatus;
  }


  Chimera::Status_t Driver::rxTransferStatus()
  {
    Chimera::Status_t cacheStatus = Chimera::Status::UNKNOWN_ERROR;

    enterCriticalSection();
    cacheStatus = rxTCB.state;
    exitCriticalSection();

    return cacheStatus;
  }


  uint32_t Driver::getFlags()
  {
    return runtimeFlags;    // Atomic operation on this processor
  }


  void Driver::clearFlags( const uint32_t flagBits )
  {
    enterCriticalSection();
    runtimeFlags &= ~( flagBits );
    exitCriticalSection();
  }


  void Driver::killTransmit()
  {
    enterCriticalSection();

    disableIT( Chimera::Hardware::SubPeripheral::TX );
    // disableDMA_IT( Chimera::Hardware::SubPeripheral::TX );
    txTCB.reset();

    exitCriticalSection();
  }


  void Driver::killReceive()
  {
    enterCriticalSection();

    disableIT( Chimera::Hardware::SubPeripheral::RX );
    // disableDMA_IT( Chimera::Hardware::SubPeripheral::RX );
    rxTCB.reset();

    exitCriticalSection();
  }


  Thor::LLD::Serial::CDTCB Driver::getTCB_TX()
  {
    Thor::LLD::Serial::CDTCB temp;

    enterCriticalSection();
    temp = txTCB;
    exitCriticalSection();

    return temp;
  }


  Thor::LLD::Serial::MDTCB Driver::getTCB_RX()
  {
    Thor::LLD::Serial::MDTCB temp;

    enterCriticalSection();
    temp = rxTCB;
    exitCriticalSection();

    return temp;
  }


  Thor::LLD::Serial::Config Driver::getConfiguration()
  {
    Thor::LLD::Serial::Config cfg;
    memset( &cfg, 0, sizeof( cfg ) );

    cfg.BaudRate     = 0;
    cfg.Mode         = TE::get( periph ) | RE::get( periph );
    cfg.OverSampling = OVER8::get( periph );
    cfg.Parity       = PCE::get( periph ) | PS::get( periph );
    cfg.StopBits     = STOP::get( periph );
    cfg.WordLength   = M0::get( periph );

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
    const uint32_t ISRCache = ISR::get( periph );
    const uint32_t CR1Cache = CR1::get( periph );
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
      if ( txTCB.state == StateMachine::TX::TX_READY )
      {
        prjClrISRSignal( periph, ISRSignal::TRANSMIT_COMPLETE );
        prjDisableISRSignal( periph, ISRSignal::TRANSMIT_COMPLETE );
        prjDisableISRSignal( periph, ISRSignal::TRANSMIT_DATA_REG_EMPTY );
      }

      /*------------------------------------------------
      TDR Empty Interrupt
      ------------------------------------------------*/
      if ( ( txFlags & FLAG_TXE ) && ( txTCB.state == StateMachine::TX::TX_ONGOING ) )
      {
        if ( txTCB.remaining )    // Data left to be transmitted
        {
          /*------------------------------------------------
          Make sure the TX complete interrupt cannot fire
          ------------------------------------------------*/
          prjDisableISRSignal( periph, ISRSignal::TRANSMIT_COMPLETE );

          /*------------------------------------------------
          Transfer the byte and prep for the next character
          ------------------------------------------------*/
          prjWriteDataRegister( periph, *txTCB.buffer );
          txTCB.buffer++;
          txTCB.remaining--;
        }
        else    // All data has been transmitted
        {
          /*------------------------------------------------
          We finished pushing the last character into the TDR, so
          now we can listen for the TX complete interrupt.
          ------------------------------------------------*/
          prjDisableISRSignal( periph, ISRSignal::TRANSMIT_DATA_REG_EMPTY );
          prjEnableISRSignal( periph, ISRSignal::TRANSMIT_COMPLETE );

          txTCB.state = StateMachine::TX_COMPLETE;
        }
      }

      /*------------------------------------------------
      Transfer Complete Interrupt
      ------------------------------------------------*/
      if ( ( txFlags & FLAG_TC ) && ( txTCB.state == StateMachine::TX::TX_COMPLETE ) )
      {
        /*-------------------------------------------------
        Exit the TX ISR by disabling related interrupts
        -------------------------------------------------*/
        prjClrISRSignal( periph, ISRSignal::TRANSMIT_COMPLETE );
        prjDisableISRSignal( periph, ISRSignal::TRANSMIT_COMPLETE );
        prjDisableISRSignal( periph, ISRSignal::TRANSMIT_DATA_REG_EMPTY );

        runtimeFlags |= Runtime::Flag::TX_COMPLETE;

        /*------------------------------------------------
        Invoke a user callback if registered
        ------------------------------------------------*/
        signalHLD                            = true;
        const SignalCallback *const callback = INT::getISRHandler( Type::PERIPH_UART, SIG_TX_COMPLETE );
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
      if ( ( rxFlags & FLAG_RXNE ) && ( CR1Cache & CR1_RXNEIE ) && ( rxTCB.state == StateMachine::RX::RX_ONGOING ) )
      {
        /*------------------------------------------------
        Make sure line idle ISR is enabled. Regardless of
        if there is data left to RX, the sender might just
        suddenly stop, and that needs detection.
        ------------------------------------------------*/
        prjEnableISRSignal( periph, ISRSignal::LINE_IDLE );
        prjClrISRSignal( periph, ISRSignal::LINE_IDLE );

        /*------------------------------------------------
        Read out current byte and prep for next transfer
        ------------------------------------------------*/
        *rxTCB.buffer = static_cast<uint8_t>( prjReadDataRegister( periph ) );
        rxTCB.buffer++;
        rxTCB.remaining--;

        /*------------------------------------------------
        If no more bytes left to receive, stop listening
        ------------------------------------------------*/
        if ( !rxTCB.remaining )
        {
          prjDisableISRSignal( periph, ISRSignal::LINE_IDLE );
          prjDisableReceiver( periph );

          rxTCB.state = StateMachine::RX::RX_COMPLETE;
          runtimeFlags |= Runtime::Flag::RX_COMPLETE;
        }
      }

      /*------------------------------------------------
      Line Idle: We were in the middle of a transfer and
      then suddenly they just stopped sending data.
      ------------------------------------------------*/
      if ( ( rxFlags & FLAG_IDLE ) && ( CR1Cache & CR1_IDLEIE ) && ( rxTCB.state == StateMachine::RX::RX_ONGOING ) )
      {
        /*-------------------------------------------------
        Clear the appropriate flags
        -------------------------------------------------*/
        prjDisableISRSignal( periph, ISRSignal::LINE_IDLE );
        prjClrISRSignal( periph, ISRSignal::LINE_IDLE );
        prjDisableReceiver( periph );

        /*-------------------------------------------------
        Update the system state variables
        -------------------------------------------------*/
        rxTCB.state = StateMachine::RX::RX_ABORTED;
        runtimeFlags |= Runtime::Flag::RX_LINE_IDLE_ABORT;
      }

      /*------------------------------------------------
      Unblock the higher level driver
      ------------------------------------------------*/
      if ( ( rxTCB.state == StateMachine::RX::RX_ABORTED || rxTCB.state == StateMachine::RX_COMPLETE ) )
      {
        /*-------------------------------------------------
        Regardless of the state this entered in, the
        transfer is complete now
        -------------------------------------------------*/
        rxTCB.state = StateMachine::RX_COMPLETE;

        /*------------------------------------------------
        Invoke a user callback if registered
        ------------------------------------------------*/
        signalHLD                            = true;
        const SignalCallback *const callback = INT::getISRHandler( Type::PERIPH_UART, SIG_RX_COMPLETE );
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
      runtimeFlags |= errorFlags;

      /*------------------------------------------------
      Invoke a user callback if registered
      ------------------------------------------------*/
      signalHLD                            = true;
      const SignalCallback *const callback = INT::getISRHandler( Type::PERIPH_UART, SIG_ERROR );
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
      sendTaskMsg( INT::getUserTaskId( Type::PERIPH_UART ), ITCMsg::TSK_MSG_ISR_HANDLER, TIMEOUT_DONT_WAIT );
    }
  }


  /*-------------------------------------------------------------------------------
  Private Methods
  -------------------------------------------------------------------------------*/
  uint32_t Driver::calculateBRR( const size_t desiredBaud )
  {
    size_t periphClock   = 0u;
    size_t calculatedBRR = 0u;
    auto periphAddress   = reinterpret_cast<std::uintptr_t>( periph );

    /*------------------------------------------------
    Figure out the frequency of the clock that drives the UART
    ------------------------------------------------*/
    auto rccSys = Thor::LLD::RCC::getCoreClockCtrl();
    periphClock = rccSys->getPeriphClock( Chimera::Peripheral::Type::PERIPH_UART, periphAddress );

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

    if ( OVER8::get( periph ) )
    {
      over8Compensator = 1u;
    }

    auto divisor          = ( 25u * periphClock ) / ( 2u * over8Compensator * desiredBaud );
    auto mantissa_divisor = divisor / 100u;
    auto fraction_divisor = ( ( divisor - ( mantissa_divisor * 100u ) ) * 16u + 50u ) / 100u;
    calculatedBRR         = ( mantissa_divisor << BRR_DIV_Mantissa_Pos ) | ( fraction_divisor & BRR_DIV_Fraction_Msk );

    return static_cast<uint32_t>( calculatedBRR );
  }


  inline void Driver::enterCriticalSection()
  {
    Thor::LLD::INT::disableIRQ( Resource::IRQSignals[ resourceIndex ] );
  }


  inline void Driver::exitCriticalSection()
  {
    Thor::LLD::INT::enableIRQ( Resource::IRQSignals[ resourceIndex ] );
  }
}    // namespace Thor::LLD::UART

#endif /* TARGET_STM32L4 && THOR_DRIVER_UART */
