/********************************************************************************
 *  File Name:
 *    hw_usart_driver.cpp
 *
 *  Description:
 *    Implements the LLD interface to the STM32L4 series USART hardware.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* STL Includes */
#include <memory>

/* Chimera Includes */
#include <Chimera/common>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/dma>
#include <Thor/hld/interrupt/hld_interrupt_definitions.hpp>
#include <Thor/lld/common/cortex-m4/interrupts.hpp>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/usart/usart_prv_data.hpp>
#include <Thor/lld/interface/usart/usart_detail.hpp>
#include <Thor/lld/interface/usart/usart_intf.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_driver.hpp>

#if defined( TARGET_STM32L4 ) && defined( THOR_LLD_USART )

/*-------------------------------------------------------------------------------
Aliases
-------------------------------------------------------------------------------*/
namespace LLD = Thor::LLD::USART;

namespace Thor::LLD::USART
{
  /*-------------------------------------------------------------------------------
  Variables
  -------------------------------------------------------------------------------*/
  static Driver s_usart_drivers[ NUM_USART_PERIPHS ];

  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  static constexpr Chimera::Peripheral::Type s_peripheral_type = Chimera::Peripheral::Type::PERIPH_USART;

  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    initializeRegisters();

    /*-------------------------------------------------
    Attach all the expected peripherals to the drivers
    -------------------------------------------------*/
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

  /*-------------------------------------------------------------------------------
  Low Level Driver Implementation
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
    First deinitialize the driver so we know we are
    starting from a clean slate. There are no guarantees
    on what state the system is in when this is called.
    ------------------------------------------------*/
    if ( deinit() != Chimera::Status::OK )
    {
      return Chimera::Status::FAIL;
    }

    /*------------------------------------------------
    Initialize driver memory
    ------------------------------------------------*/
    enterCriticalSection();

    runtimeFlags = static_cast<Runtime::Flag_t>( 0 );

    txTCB.reset();
    rxTCB.reset();

    exitCriticalSection();

    /*------------------------------------------------
    Ensure the clock is enabled otherwise the hardware is "dead"
    ------------------------------------------------*/
    auto rccPeriph = Thor::LLD::RCC::getPeripheralClock();
    rccPeriph->enableClock( s_peripheral_type, resourceIndex );

    /*------------------------------------------------
    Follow the initialization sequence as defined in RM0394 Rev 4, pg.1202
    ------------------------------------------------*/
    /* Disable the USART by writing the UE bit to 0 */
    UE::set( periph, 0 );

    /* Clear out all the control registers to ensure a clean slate */
    CR1::set( periph, CR1_Rst );
    CR2::set( periph, CR2_Rst );
    CR3::set( periph, CR3_Rst );

    /* Program the M0 bit to define the word length. M1 Currently not supported. */
    M0::set( periph, cfg.WordLength );
    M1::set( periph, 0 );

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

    /* Enable the USART by writing the UE bit to 1 */
    UE::set( periph, CR1_UE );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::deinit()
  {
    /*-------------------------------------------------
    Use the RCC's ability to reset whole peripherals back to default states
    -------------------------------------------------*/
    auto rcc = Thor::LLD::RCC::getPeripheralClock();
    rcc->enableClock( s_peripheral_type, resourceIndex );
    rcc->reset( s_peripheral_type, resourceIndex );
    rcc->disableClock( s_peripheral_type, resourceIndex );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::reset()
  {
    deinit();
    ISRWakeup_external = nullptr;

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::transmit( const void *const data, const size_t size )
  {
    // Blocking mode not allowed
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::receive( void *const data, const size_t size )
  {
    // Blocking mode not allowed
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::enableIT( const Chimera::Hardware::SubPeripheral subPeriph )
  {
    /*-------------------------------------------------
    Ignore the subperiph argument as the actual transmit
    and receive functions called by the user handle that.
    -------------------------------------------------*/
    Thor::LLD::IT::setPriority( Resource::IRQSignals[ resourceIndex ], Thor::Interrupt::USART_IT_PREEMPT_PRIORITY, 0u );
    Thor::LLD::IT::enableIRQ( Resource::IRQSignals[ resourceIndex ] );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::disableIT( const Chimera::Hardware::SubPeripheral subPeriph )
  {
    using namespace Chimera::Hardware;

    Chimera::Status_t result = Chimera::Status::OK;

    /*-------------------------------------------------
    Disable system level interrupt from peripheral, to prevent glitching
    -------------------------------------------------*/
    Thor::LLD::IT::disableIRQ( Resource::IRQSignals[ resourceIndex ] );
    Thor::LLD::IT::clearPendingIRQ( Resource::IRQSignals[ resourceIndex ] );

    /*-------------------------------------------------
    Disable interrupt event generation inside the peripheral
    -------------------------------------------------*/
    if ( ( subPeriph == SubPeripheral::TX ) || ( subPeriph == SubPeripheral::TXRX ) )
    {
      TCIE::set( periph, 0 );
      TXEIE::set( periph, 0 );
    }
    else if ( ( subPeriph == SubPeripheral::RX ) || ( subPeriph == SubPeripheral::TXRX ) )
    {
      RXNEIE::set( periph, 0 );
    }
    else
    {
      result = Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------
    Re-enable the system level interrupts so other events can get through
    -------------------------------------------------*/
    Thor::LLD::IT::enableIRQ( Resource::IRQSignals[ resourceIndex ] );

    return result;
  }


  Chimera::Status_t Driver::transmitIT( const void *const data, const size_t size )
  {
#if defined( DEBUG )
    /*-------------------------------------------------
    Don't bother with this check in release code as it slows down the program
    -------------------------------------------------*/
    if ( !data || !size )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }
#endif

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
      Shove the byte into the transmit data register, kicking off the
      transfer. Re-enable interrupts so we can catch the TX complete event.
      ------------------------------------------------*/
      periph->TDR = transferData[ 0 ];

      /*------------------------------------------------
      Turn on the transmitter & enable TDR interrupt so we know
      when we can stage the next byte transfer.
      ------------------------------------------------*/
      TE::set( periph, CR1_TE );
      TXEIE::set( periph, CR1_TXEIE );
      enableIT( Chimera::Hardware::SubPeripheral::TX );

      exitCriticalSection();
    }

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::receiveIT( void *const data, const size_t size )
  {
#if defined( DEBUG )
    /*-------------------------------------------------
    Don't bother with this check in release code as it slows down the program
    -------------------------------------------------*/
    if ( !data || !size )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }
#endif

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
      RXNEIE::set( periph, CR1_RXNEIE );

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
      RE::set( periph, CR1_RE );
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
    using namespace Chimera::Threading;
    return Chimera::Status::NOT_SUPPORTED;
    //    auto result = Chimera::Status::TIMEOUT;
    //
    //    /*------------------------------------------------
    //    Create the control blocks for configuring a USART DMA transfer
    //    ------------------------------------------------*/
    //    Chimera::DMA::Init init;
    //    init.direction = Chimera::DMA::TransferDirection::MEMORY_TO_PERIPH;
    //    init.mAlign    = Chimera::DMA::MemoryAlignment::ALIGN_BYTE;
    //    init.mInc      = Chimera::DMA::MemoryIncrement::ENABLED;
    //    init.mode      = Chimera::DMA::Mode::NORMAL;
    //    init.pAlign    = Chimera::DMA::PeripheralAlignment::ALIGN_BYTE;
    //    init.pInc      = Chimera::DMA::PeripheralIncrement::DISABLED;
    //    init.priority  = Chimera::DMA::Priority::MEDIUM;
    //    init.request   = Resource::TXDMASignals[ resourceIndex ];
    //
    //    Chimera::DMA::TCB tcb;
    //
    //    /*-------------------------------------------------
    //    Grab the address of the data using a type that is large enough to hold
    //    the system's sizeof(void*). Otherwise the compiler complains about losing
    //    precision when cross compiling with GCC et al in a 64-bit environment.
    //    -------------------------------------------------*/
    //    size_t tempDstAddr = reinterpret_cast<size_t>( &periph->TDR );
    //    size_t tempSrcAddr = reinterpret_cast<size_t>( data );
    //
    //    /*-------------------------------------------------
    //    The STM32 embedded system is always 32-bit, so the control block structure is
    //    expecting a 32-bit address. This might cause issues during simulation of this
    //    code on 64-bit platforms.
    //    -------------------------------------------------*/
    //    tcb.dstAddress = static_cast<uint32_t>( tempDstAddr );
    //    tcb.srcAddress = static_cast<uint32_t>( tempSrcAddr );
    //    tcb.transferSize = size;
    //
    //    /*------------------------------------------------
    //    Grab a pointer to the DMA singleton
    //    ------------------------------------------------*/
    //    auto dma = Thor::DMA::DMAClass::get();
    //    if ( !dma )
    //    {
    //      return Chimera::Status::FAIL;
    //    }
    //
    //    /*------------------------------------------------
    //    Acquire exclusive access to the hardware and start the transfer
    //    ------------------------------------------------*/
    //    if ( dma->try_lock_for( timeout ) == Chimera::Status::OK )
    //    {
    //      /*------------------------------------------------
    //      According to pg.32 of AN4031, the DMA must be initialized
    //      BEFORE the peripheral is enabled otherwise a FIFO error
    //      is likely to occur. I found this to be very true.
    //      ------------------------------------------------*/
    //      result = dma->configure( init, tcb, timeout, nullptr );
    //      dma->start();
    //
    //      /*------------------------------------------------
    //      At this point the DMA is enabled and waiting for the
    //      peripheral to send a request. Setting these two bits
    //      accomplish this.
    //      ------------------------------------------------*/
    //      enableIT( Chimera::Hardware::SubPeripheral::TX );
    //      enterCriticalSection();
    //
    //      DMAT::set( periph, CR3_DMAT );
    //      TE::set( periph, CR1_TE );
    //
    //      /*------------------------------------------------
    //      Prepare the USART state machine to correctly process the ISR request. Once the DMA
    //      transfer finishes, the TCIF will be set and trigger the USART ISR processing.
    //
    //      According to the datasheet, the TC flag must be cleared before starting the DMA transfer.
    //      ------------------------------------------------*/
    //      TCCF::set( periph, ICR_TCCF );
    //      TCIE::set( periph, CR1_TCIE );
    //      txTCB.state = StateMachine::TX::TX_COMPLETE;
    //
    //      // Allow listening for the TCIE event
    //      exitCriticalSection();
    //
    //      // Unlock after the critical section due to possibly needing ISRs with the RTOS
    //      dma->unlock();
    //    }
    //
    //    return result;
  }


  Chimera::Status_t Driver::receiveDMA( void *const data, const size_t size )
  {
    // auto dma = Thor::DMA::DMAClass::get();

    // don't forget to pull the correct signal
    // TODO if ever needed

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
    disableDMA_IT( Chimera::Hardware::SubPeripheral::TX );
    txTCB.reset();

    exitCriticalSection();
  }


  void Driver::killReceive()
  {
    enterCriticalSection();

    disableIT( Chimera::Hardware::SubPeripheral::RX );
    disableDMA_IT( Chimera::Hardware::SubPeripheral::RX );
    rxTCB.reset();

    exitCriticalSection();
  }


  void Driver::attachISRWakeup( Chimera::Threading::BinarySemaphore *const wakeup )
  {
    ISRWakeup_external = wakeup;
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


  void Driver::IRQHandler()
  {
    using namespace Configuration::Flags;

    /*------------------------------------------------
    Cache the current state of the registers
    ------------------------------------------------*/
    const uint32_t ISRCache = ISR::get( periph );
    const uint32_t CR1Cache = CR1::get( periph );

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
        TCCF::set( periph, ICR_TCCF );
        TCIE::set( periph, 0 );
        TXEIE::set( periph, 0 );
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
          TCIE::set( periph, 0 );

          /*------------------------------------------------
          Transfer the byte and prep for the next character
          ------------------------------------------------*/
          periph->TDR = *txTCB.buffer;

          txTCB.buffer++;
          txTCB.remaining--;
        }
        else    // All data has been transmitted
        {
          /*------------------------------------------------
          We finished pushing the last character into the TDR, so
          now we can listen for the TX complete interrupt.
          ------------------------------------------------*/
          TXEIE::set( periph, 0 );
          TCIE::set( periph, CR1_TCIE );
          txTCB.state = StateMachine::TX_COMPLETE;
        }
      }

      /*------------------------------------------------
      Transfer Complete Interrupt
      ------------------------------------------------*/
      if ( ( txFlags & FLAG_TC ) && ( txTCB.state == StateMachine::TX::TX_COMPLETE ) )
      {
        /*-------------------------------------------------
        Clear the appropriate flag
        -------------------------------------------------*/
        TCCF::set( periph, ICR_TCCF );

        /*------------------------------------------------
        Exit the TX ISR cleanly by disabling related interrupts
        ------------------------------------------------*/
        TCIE::clear( periph, CR1_TCIE );
        TXEIE::clear( periph, CR1_TXEIE );
        runtimeFlags |= Runtime::Flag::TX_COMPLETE;

        /*------------------------------------------------
        Unblock the higher level driver
        ------------------------------------------------*/
        if ( ISRWakeup_external )
        {
          ISRWakeup_external->releaseFromISR();
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
        Make sure the line idle interrupt is enabled. Regardless of
        whether or not we have data left to RX, the sender might just
        suddenly stop, and we need to detect that.
        ------------------------------------------------*/
        IDLEIE::set( periph, CR1_IDLEIE );
        IDLECF::set( periph, ICR_IDLECF );

        /*------------------------------------------------
        Read out the current byte and prep for the next transfer
        ------------------------------------------------*/
        *rxTCB.buffer = periph->RDR;
        rxTCB.buffer++;
        rxTCB.remaining--;

        /*------------------------------------------------
        If no more bytes left to receive, stop listening
        ------------------------------------------------*/
        if ( !rxTCB.remaining )
        {
          RE::set( periph, 0 );
          IDLEIE::set( periph, 0 );
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
        RE::set( periph, 0 );
        IDLEIE::set( periph, 0 );
        IDLECF::set( periph, ICR_IDLECF );

        /*-------------------------------------------------
        Update the system state variables
        -------------------------------------------------*/
        rxTCB.state = StateMachine::RX::RX_ABORTED;
        runtimeFlags |= Runtime::Flag::RX_LINE_IDLE_ABORT;
      }

      /*------------------------------------------------
      Unblock the higher level driver
      ------------------------------------------------*/
      if ( ( rxTCB.state == StateMachine::RX::RX_ABORTED || rxTCB.state == StateMachine::RX_COMPLETE ) && ISRWakeup_external )
      {
        // Regardless of the state this entered in, the transfer is complete now
        rxTCB.state = StateMachine::RX_COMPLETE;
        ISRWakeup_external->releaseFromISR();
      }
    }

    /*------------------------------------------------
    Error Related Handler
    ------------------------------------------------*/
    if ( errorFlags )
    {
      ORECF::set( periph, errorFlags );
      NECF::set( periph, errorFlags );
      FECF::set( periph, errorFlags );
      PECF::set( periph, errorFlags );

      runtimeFlags |= errorFlags;
    }
  }


  uint32_t Driver::calculateBRR( const size_t desiredBaud )
  {
    size_t periphClock   = 0u;
    size_t calculatedBRR = 0u;
    auto periphAddress   = reinterpret_cast<std::uintptr_t>( periph );

    /*------------------------------------------------
    Figure out the frequency of the clock that drives the USART
    ------------------------------------------------*/
    auto rccSys = Thor::LLD::RCC::getCoreClock();
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

    if ( OVER8::get( periph ) )
    {
      over8Compensator = 1u;
    }

    auto divisor          = ( 25u * periphClock ) / ( 2u * over8Compensator * desiredBaud );
    auto mantissa_divisor = divisor / 100u;
    auto fraction_divisor = ( ( divisor - ( mantissa_divisor * 100u ) ) * 16u + 50u ) / 100u;
    calculatedBRR         = ( mantissa_divisor << BRR_DIV_MANTISSA_Pos ) | ( fraction_divisor & BRR_DIV_FRACTION );

    return static_cast<uint32_t>( calculatedBRR );
  }


  inline void Driver::enterCriticalSection()
  {
    Thor::LLD::IT::disableIRQ( Resource::IRQSignals[ resourceIndex ] );
  }


  inline void Driver::exitCriticalSection()
  {
    Thor::LLD::IT::enableIRQ( Resource::IRQSignals[ resourceIndex ] );
  }
}    // namespace Thor::LLD::USART


#if defined( STM32_USART1_PERIPH_AVAILABLE )
void USART1_IRQHandler( void )
{
  ::LLD::s_usart_drivers[::LLD::USART1_RESOURCE_INDEX ].IRQHandler();
}
#endif /* STM32_USART1_PERIPH_AVAILABLE */


#if defined( STM32_USART2_PERIPH_AVAILABLE )
void USART2_IRQHandler( void )
{
  ::LLD::s_usart_drivers[::LLD::USART2_RESOURCE_INDEX ].IRQHandler();
}
#endif /* STM32_USART2_PERIPH_AVAILABLE */


#if defined( STM32_USART3_PERIPH_AVAILABLE )
void USART3_IRQHandler( void )
{
  ::LLD::s_usart_drivers[::LLD::USART3_RESOURCE_INDEX ].IRQHandler();
}
#endif /* STM32_USART3_PERIPH_AVAILABLE */


#endif /* TARGET_STM32L4 && THOR_DRIVER_USART */
