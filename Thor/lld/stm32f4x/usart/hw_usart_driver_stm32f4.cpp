/********************************************************************************
 *  File Name:
 *    hw_usart_driver_stm32f4.cpp
 *
 *  Description:
 *    STM32F4 specific driver implementation for the UART/USART driver. Both drivers
 *    are merged into one as the datasheet does not make a distinction between the
 *    two. In practice with the STM32HAL this was also found to be true.
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <array>
#include <cstring>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/dma>
#include <Chimera/thread>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/dma>
#include <Thor/hld/interrupt/hld_interrupt_definitions.hpp>
#include <Thor/lld/stm32f4x/rcc/hw_rcc_driver.hpp>
#include <Thor/lld/stm32f4x/nvic/hw_nvic_driver.hpp>
#include <Thor/lld/stm32f4x/usart/hw_usart_driver.hpp>
#include <Thor/lld/stm32f4x/usart/hw_usart_mapping.hpp>
#include <Thor/lld/stm32f4x/usart/hw_usart_prj.hpp>
#include <Thor/lld/stm32f4x/usart/hw_usart_types.hpp>


#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_USART )

namespace Thor::LLD::USART
{
  void initialize()
  {
    initializeRegisters();
    initializeMapping();
  }

  Driver::Driver( RegisterMap *const peripheral ) : periph( peripheral ), ISRWakeup_external( nullptr )
  {
    auto address   = reinterpret_cast<std::uintptr_t>( peripheral );
    peripheralType = Chimera::Peripheral::Type::PERIPH_USART;
    resourceIndex  = Thor::LLD::USART::InstanceToResourceIndex.find( address )->second;
    periphIRQn     = USART_IRQn[ resourceIndex ];
    dmaTXSignal    = TXDMASignals[ resourceIndex ];
    dmaRXSignal    = RXDMASignals[ resourceIndex ];

    usartObjects[ resourceIndex ] = this;

    /*------------------------------------------------
    Ensure the clock is enabled otherwise the hardware is "dead"
    ------------------------------------------------*/
    auto rccPeriph = Thor::LLD::RCC::getPeripheralClock();
    rccPeriph->enableClock( peripheralType, resourceIndex );
  }

  Driver::~Driver()
  {
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

    txTCB.reset();
    rxTCB.reset();

    exitCriticalSection();

    /*------------------------------------------------
    Ensure the clock is enabled otherwise the hardware is "dead"
    ------------------------------------------------*/
    auto rccPeriph = Thor::LLD::RCC::getPeripheralClock();
    rccPeriph->enableClock( peripheralType, resourceIndex );

    /*------------------------------------------------
    Follow the initialization sequence as defined in RM0390 pg.801
    ------------------------------------------------*/
    /* Clear out all the control registers to ensure a clean slate */
    CR1::set( periph, CR1::resetValue );
    CR2::set( periph, CR2::resetValue );
    CR3::set( periph, CR3::resetValue );

    /* Enable the USART by writing the UE bit to 1 */
    CR1::UE::set( periph, CR1_UE );

    /* Program the M bit to define the word length */
    CR1::M::set( periph, cfg.WordLength );

    /* Program the number of stop bits */
    CR2::STOP::set( periph, cfg.StopBits );

    /* Select the desired baud rate */
    BRR::set( periph, calculateBRR( cfg.BaudRate ) );

    return Chimera::Status::OK;
  }

  Chimera::Status_t Driver::deinit()
  {
    auto rcc = Thor::LLD::RCC::getPeripheralClock();
    rcc->enableClock( peripheralType, resourceIndex );
    rcc->reset( peripheralType, resourceIndex );
    rcc->disableClock( peripheralType, resourceIndex );

    return Chimera::Status::OK;
  }

  Chimera::Status_t Driver::reset()
  {
    /*------------------------------------------------
    Reset the hardware registers
    ------------------------------------------------*/
    deinit();

    ISRWakeup_external = nullptr;

    return Chimera::Status::OK;
  }

  Chimera::Status_t Driver::transmit( const uint8_t *const data, const size_t size, const size_t timeout )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::receive( uint8_t *const data, const size_t size, const size_t timeout )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::enableIT( const Chimera::Hardware::SubPeripheral subPeriph )
  {
    using namespace Thor::Interrupt;
    using namespace Thor::LLD::IT;

    setPriority( periphIRQn, USART_IT_PREEMPT_PRIORITY, 0u );

    /*------------------------------------------------
    The individual TX and RX functions take care of the nitty gritty details
    ------------------------------------------------*/
    enableIRQ( periphIRQn );
    return Chimera::Status::OK;
  }

  Chimera::Status_t Driver::disableIT( const Chimera::Hardware::SubPeripheral subPeriph )
  {
    using namespace Chimera::Hardware;
    using namespace Thor::LLD::IT;

    Chimera::Status_t result = Chimera::Status::OK;

    disableIRQ( periphIRQn );
    clearPendingIRQ( periphIRQn );

    if ( ( subPeriph == SubPeripheral::TX ) || ( subPeriph == SubPeripheral::TXRX ) )
    {
      CR1::TCIE::set( periph, 0 );
      CR1::TXEIE::set( periph, 0 );
    }
    else if ( ( subPeriph == SubPeripheral::RX ) || ( subPeriph == SubPeripheral::TXRX ) )
    {

    }
    else
    {
      result = Chimera::Status::INVAL_FUNC_PARAM;
    }

    enableIRQ( periphIRQn );

    return result;
  }

  Chimera::Status_t Driver::transmitIT( const uint8_t *const data, const size_t size, const size_t timeout )
  {
#if defined( DEBUG )
    if ( !data || !size )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }
#endif

    if ( !SR::TC::get( periph ) )
    {
      return Chimera::Status::BUSY;
    }
    else
    {
      enableIT( Chimera::Hardware::SubPeripheral::TX );

      enterCriticalSection();

      /*------------------------------------------------
      Turn on the transmitter & enable TDR interrupt so we know
      when we can stage the next byte transfer.
      ------------------------------------------------*/
      CR1::TE::set( periph, CR1_TE );
      CR1::TXEIE::set( periph, CR1_TXEIE );

      /*------------------------------------------------
      Prep the transfer control block
      ------------------------------------------------*/
      txTCB.buffer = &data[ 1 ]; /* Point to the next byte */
      txTCB.size   = size - 1u;  /* Pre-decrement to account for this first byte */
      txTCB.state  = StateMachine::TX::TX_ONGOING;

      /*------------------------------------------------
      Write the byte onto the wire
      ------------------------------------------------*/
      periph->DR = data[ 0 ];

      exitCriticalSection();
    }

    return Chimera::Status::OK;
  }

  Chimera::Status_t Driver::receiveIT( uint8_t *const data, const size_t size, const size_t timeout )
  {
#if defined( DEBUG )
    if ( !data || !size )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }
#endif

    if ( SR::RXNE::get( periph ) )
    {
      /* There is a byte in the register that needs to be read by the ISR */
      return Chimera::Status::BUSY;
    }
    else
    {
      enterCriticalSection();

      /*------------------------------------------------
      Only turn on RXNE so as to detect when the first byte arrives
      ------------------------------------------------*/
      CR1::RXNEIE::set( periph, CR1_RXNEIE );

      /*------------------------------------------------
      Prep the transfer control block to receive data
      ------------------------------------------------*/
      rxTCB.buffer = data;
      rxTCB.size   = size;
      rxTCB.state  = StateMachine::RX::RX_ONGOING;

      /*------------------------------------------------
      Turn on the RX hardware to begin listening
      ------------------------------------------------*/
      CR1::RE::set( periph, CR1_RE );

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

  Chimera::Status_t Driver::transmitDMA( const void *const data, const size_t size, const size_t timeout )
  {
    using namespace Chimera::Threading;

    auto result = Chimera::Status::TIMEOUT;

    /*------------------------------------------------
    Create the control blocks for configuring a USART DMA transfer
    ------------------------------------------------*/
    Chimera::DMA::Init init;
    init.direction = Chimera::DMA::TransferDirection::MEMORY_TO_PERIPH;
    init.mAlign    = Chimera::DMA::MemoryAlignment::ALIGN_BYTE;
    init.mInc      = Chimera::DMA::MemoryIncrement::ENABLED;
    init.mode      = Chimera::DMA::Mode::NORMAL;
    init.pAlign    = Chimera::DMA::PeripheralAlignment::ALIGN_BYTE;
    init.pInc      = Chimera::DMA::PeripheralIncrement::DISABLED;
    init.priority  = Chimera::DMA::Priority::MEDIUM;
    init.request   = dmaTXSignal;

    Chimera::DMA::TCB tcb;

    /*-------------------------------------------------
    Grab the address of the data using a type that is large enough to hold
    the system's sizeof(void*). Otherwise the compiler complains about losing
    precision when cross compiling with GCC et al in a 64-bit environment.
    -------------------------------------------------*/
    size_t tempDstAddr = reinterpret_cast<size_t>( &periph->DR );
    size_t tempSrcAddr = reinterpret_cast<size_t>( data );

    /*-------------------------------------------------
    The STM32 embedded system is always 32-bit, so the control block structure is
    expecting a 32-bit address. This might cause issues during simulation of this
    code on 64-bit platforms.
    -------------------------------------------------*/
    tcb.dstAddress = static_cast<uint32_t>( tempDstAddr );
    tcb.srcAddress = static_cast<uint32_t>( tempSrcAddr );
    tcb.transferSize = size;

    /*------------------------------------------------
    Grab a pointer to the DMA singleton
    ------------------------------------------------*/
    auto dma = Thor::DMA::DMAClass::get();
    if ( !dma )
    {
      return Chimera::Status::FAIL;
    }

    /*------------------------------------------------
    Acquire exclusive access to the hardware and start the transfer
    ------------------------------------------------*/
    if ( dma->try_lock_for( timeout ) == Chimera::Status::OK )
    {
      /*------------------------------------------------
      According to pg.32 of AN4031, the DMA must be initialized
      BEFORE the peripheral is enabled otherwise a FIFO error
      is likely to ensue. I found this to be very true.
      ------------------------------------------------*/
      result = dma->configure( init, tcb, timeout, nullptr );
      dma->start();

      /*------------------------------------------------
      At this point the DMA is enabled and waiting for the
      peripheral to send a request. Setting these two bits
      accomplish this.
      ------------------------------------------------*/
      enableIT( Chimera::Hardware::SubPeripheral::TX );
      enterCriticalSection();

      CR3::DMAT::set( periph, CR3_DMAT );
      CR1::TE::set( periph, CR1_TE );

      /*------------------------------------------------
      Prepare the USART state machine to correctly process the ISR request. Once the DMA
      transfer finishes, the TCIF will be set and trigger the USART ISR processing.

      According to the datasheet, the TC flag must be cleared before starting the DMA transfer.
      ------------------------------------------------*/
      SR::TC::set( periph, 0 );
      CR1::TCIE::set( periph, CR1_TCIE );
      txTCB.state = StateMachine::TX::TX_COMPLETE;

      exitCriticalSection();

      dma->unlock();
    }

    return result;
  }

  Chimera::Status_t Driver::receiveDMA( void *const data, const size_t size, const size_t timeout )
  {
    auto dma = Thor::DMA::DMAClass::get();

    // don't forget to pull the correct signal

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
    return runtimeFlags;
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

  CDTCB Driver::getTCB_TX()
  {
    CDTCB temp;
    enterCriticalSection();
    temp = txTCB;
    exitCriticalSection();

    return temp;
  }

  MDTCB Driver::getTCB_RX()
  {
    MDTCB temp;
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
    cfg.Mode         = CR1::TE::get( periph ) | CR1::RE::get( periph );
    cfg.OverSampling = CR1::OVER8::get( periph );
    cfg.Parity       = CR1::PCE::get( periph ) | CR1::PS::get( periph );
    cfg.StopBits     = CR2::STOP::get( periph );
    cfg.WordLength   = CR1::M::get( periph );

    return cfg;
  }

  void Driver::IRQHandler()
  {
    using namespace Configuration::Flags;

    /*------------------------------------------------
    Following the datasheet, flags are only cleared after
    reading the SR and then DR.
    ------------------------------------------------*/
    const uint32_t statusRegister = SR::get( periph );

    /*------------------------------------------------
    Cache the current state of the control registers
    ------------------------------------------------*/
    const uint32_t CR1 = CR1::get( periph );

    /*------------------------------------------------
    Figure out which interrupt flags have been set
    ------------------------------------------------*/
    uint32_t txFlags    = statusRegister & ( FLAG_TC | FLAG_TXE );
    uint32_t rxFlags    = statusRegister & ( FLAG_RXNE | FLAG_IDLE );
    uint32_t errorFlags = statusRegister & ( FLAG_ORE | FLAG_PE | FLAG_NF | FLAG_FE );

    /*------------------------------------------------
    TX Related Handler
    ------------------------------------------------*/
    if ( txFlags )
    {
      /*------------------------------------------------
      TDR Empty Interrupt
      ------------------------------------------------*/
      if ( ( txFlags & FLAG_TXE ) && ( CR1 & CR1_TXEIE ) && ( txTCB.state == StateMachine::TX::TX_ONGOING ) )
      {
        if ( txTCB.size )
        {
          /*------------------------------------------------
          Make sure the TX complete interrupt cannot fire
          ------------------------------------------------*/
          CR1::TCIE::set( periph, 0 );

          /*------------------------------------------------
          Transfer the byte and prep for the next character
          ------------------------------------------------*/
          periph->DR = *txTCB.buffer;

          txTCB.buffer++;
          txTCB.size--;
        }
        else
        {
          /*------------------------------------------------
          We finished pushing the last character into the TDR, so
          now listen for the TX complete interrupt.
          ------------------------------------------------*/
          CR1::TXEIE::set( periph, 0 );
          CR1::TCIE::set( periph, CR1_TCIE );
          txTCB.state = StateMachine::TX_COMPLETE;
        }
      }

      /*------------------------------------------------
      Transfer Complete Interrupt
      ------------------------------------------------*/
      if ( ( txFlags & FLAG_TC ) && ( CR1 & CR1_TCIE ) && ( txTCB.state == StateMachine::TX::TX_COMPLETE ) )
      {
        /*------------------------------------------------
        Exit the TX ISR cleanly by disabling related interrupts
        ------------------------------------------------*/
        CR1::TCIE::set( periph, 0 );
        CR1::TXEIE::set( periph, 0 );
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
      if ( ( rxFlags & FLAG_RXNE ) && ( CR1 & CR1_RXNEIE ) && ( rxTCB.state == StateMachine::RX::RX_ONGOING ) )
      {
        /*------------------------------------------------
        Make sure the line idle interrupt is enabled. Regardless of
        whether or not we have data left to RX, the sender might just
        suddenly stop, and we need to detect that.
        ------------------------------------------------*/
        CR1::IDLEIE::set( periph, CR1_IDLEIE );

        /*------------------------------------------------
        Read out the current byte and prep for the next transfer
        ------------------------------------------------*/
        *rxTCB.buffer = periph->DR;

        rxTCB.buffer++;
        rxTCB.size--;

        /*------------------------------------------------
        If no more bytes left to receive, stop listening
        ------------------------------------------------*/
        if ( !rxTCB.size )
        {
          CR1::RE::set( periph, 0 );
          CR1::IDLEIE::set( periph, 0 );
          rxTCB.state = StateMachine::RX::RX_COMPLETE;
          runtimeFlags |= Runtime::Flag::RX_COMPLETE;
        }
      }

      /*------------------------------------------------
      Line Idle: We were in the middle of a transfer and
      then suddenly they just stopped sending data.
      ------------------------------------------------*/
      if ( ( rxFlags & FLAG_IDLE ) && ( CR1 & CR1_IDLEIE ) && ( rxTCB.state == StateMachine::RX::RX_ONGOING ) )
      {
        CR1::RE::set( periph, 0 );
        CR1::IDLEIE::set( periph, 0 );
        rxTCB.state = StateMachine::RX::RX_ABORTED;
        runtimeFlags |= Runtime::Flag::RX_LINE_IDLE_ABORT;
      }

      /*------------------------------------------------
      Unblock the higher level driver
      ------------------------------------------------*/
      if ( ( rxTCB.state == StateMachine::RX::RX_ABORTED || rxTCB.state == StateMachine::RX_COMPLETE ) && ISRWakeup_external )
      {
        ISRWakeup_external->releaseFromISR();
      }
    }

    /*------------------------------------------------
    Error Related Handler
    ------------------------------------------------*/
    if ( errorFlags )
    {
      runtimeFlags |= errorFlags;
      /* All error bits are cleared by read to SR then DR, which was already performed */
    }
  }

  uint32_t Driver::calculateBRR( const size_t desiredBaud )
  {
    size_t periphClock = 0u;
    size_t calculatedBRR = 0u;
    auto periphAddress = reinterpret_cast<std::uintptr_t>( periph );

    /*------------------------------------------------
    Figure out the frequency of the clock that drives the USART
    ------------------------------------------------*/
    auto rccSys = Thor::LLD::RCC::getCoreClock();
    rccSys->getPeriphClock( Chimera::Peripheral::Type::PERIPH_USART, periphAddress, &periphClock );

    /*------------------------------------------------
    Protect from fault conditions in the math below
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

    if ( CR1::OVER8::get( periph ) )
    {
      over8Compensator = 1u;
    }

    auto divisor          = ( 25u * periphClock ) / ( 2u * over8Compensator * desiredBaud );
    auto mantissa_divisor = divisor / 100u;
    auto fraction_divisor = ( ( divisor - ( mantissa_divisor * 100u ) ) * 16u + 50u ) / 100u;
    calculatedBRR         = ( mantissa_divisor << BRR_DIV_Mantissa_Pos ) | ( fraction_divisor & BRR_DIV_Fraction );

    return static_cast<uint32_t>( calculatedBRR );
  }

  inline void Driver::enterCriticalSection()
  {
    Thor::LLD::IT::disableIRQ( periphIRQn );
  }

  inline void Driver::exitCriticalSection()
  {
    Thor::LLD::IT::enableIRQ( periphIRQn );
  }
}    // namespace Thor::LLD::USART

void USART1_IRQHandler( void )
{
  static constexpr size_t index = 0;

  if ( Thor::LLD::USART::usartObjects[ index ] )
  {
    Thor::LLD::USART::usartObjects[ index ]->IRQHandler();
  }
}

void USART2_IRQHandler( void )
{
  static constexpr size_t index = 1;

  if ( Thor::LLD::USART::usartObjects[ index ] )
  {
    Thor::LLD::USART::usartObjects[ index ]->IRQHandler();
  }
}

void USART3_IRQHandler( void )
{
  static constexpr size_t index = 2;

  if ( Thor::LLD::USART::usartObjects[ index ] )
  {
    Thor::LLD::USART::usartObjects[ index ]->IRQHandler();
  }
}

void USART6_IRQHandler( void )
{
  static constexpr size_t index = 3;

  if ( Thor::LLD::USART::usartObjects[ index ] )
  {
    Thor::LLD::USART::usartObjects[ index ]->IRQHandler();
  }
}

#endif /* TARGET_STM32F4 && THOR_DRIVER_USART */