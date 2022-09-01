/******************************************************************************
 *  File Name:
 *    hw_i2c_driver.cpp
 *
 *  Description:
 *    I2C driver for STM32L4
 *
 *  2022 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <Chimera/i2c>
#include <Thor/cfg>
#include <Thor/lld/interface/inc/i2c>
#include <Thor/lld/interface/inc/interrupt>
#include <Thor/lld/interface/inc/rcc>

#if defined( THOR_I2C ) && defined( TARGET_STM32L4 )
namespace Thor::LLD::I2C
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t   DMA_TRANS_POINT = 2; /**< When transfers switch from Interrupt to DMA driven */
  static constexpr uint32_t EV_ISR_FLAGS    =    /**< ISR Events to listen to */
      ( CR1_ERRIE | CR1_TCIE | CR1_STOPIE | CR1_NACKIE | CR1_ADDRIE );
  static constexpr uint32_t EV_ICR_FLAGS = /**< All interrupt flag clear events */
      ( ICR_ALERTCF | ICR_TIMOUTCF | ICR_PECCF | ICR_OVRCF | ICR_ARLOCF | ICR_BERRCF | ICR_STOPCF | ICR_NACKCF | ICR_ADDRCF );

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  Driver::Driver() :
      mPeriph( nullptr ), mResourceIndex( INVALID_RESOURCE_INDEX ), mISRBits( 0 ), mTXDMARequestId( 0 ), mRXDMARequestId( 0 )
  {
  }


  Driver::~Driver()
  {
  }


  Chimera::Status_t Driver::attach( RegisterMap *const peripheral )
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


  void Driver::enableClock()
  {
    auto rccPeriph = Thor::LLD::RCC::getPeriphClockCtrl();
    rccPeriph->enableClock( Chimera::Peripheral::Type::PERIPH_I2C, mResourceIndex );
  }


  void Driver::disableClock()
  {
    auto rccPeriph = Thor::LLD::RCC::getPeriphClockCtrl();
    rccPeriph->disableClock( Chimera::Peripheral::Type::PERIPH_I2C, mResourceIndex );
  }


  void Driver::enableISRSignal( const InterruptBits signal )
  {
    enterCriticalSection();
    mISRBits |= signal;
    exitCriticalSection();
  }


  void Driver::disableISRSignal( const InterruptBits signal )
  {
    enterCriticalSection();
    mISRBits &= ~signal;
    exitCriticalSection();
  }


  Chimera::Status_t Driver::configure( const Chimera::I2C::DriverConfig &cfg )
  {
    using namespace Chimera::Peripheral;

    /*-------------------------------------------------------------------------
    Initialize the driver
    -------------------------------------------------------------------------*/
    if ( !mPeriph )
    {
      return Chimera::Status::FAIL;
    }

    /* Disable software handling of ISR/DMA signals */
    mISRBits        = 0;
    mTXDMARequestId = 0;
    mRXDMARequestId = 0;
    mTransfer.clear();

    /*-------------------------------------------------------------------------
    Configure clock controllers
    -------------------------------------------------------------------------*/
    /* Reset the peripheral clock */
    auto rccPeriph = Thor::LLD::RCC::getPeriphClockCtrl();
    rccPeriph->reset( Type::PERIPH_I2C, mResourceIndex );

    /* Reset the peripheral hardware using dedicated interface */
    this->enableClock();
    PE::clear( mPeriph, CR1_PE );

    /* Assign peripheral clock frequency */
    auto rccControl = Thor::LLD::RCC::getCoreClockCtrl();
    auto freq       = rccControl->getPeriphClock( Type::PERIPH_I2C, reinterpret_cast<std::uintptr_t>( mPeriph ) );
    auto freqMHz    = freq / 1000000;

    /*-------------------------------------------------------------------------
    Rather than go through an expensive and buggy clock timing register setup,
    I used the STM32CubeMX tool to auto-calculate the required values. I've
    found that my projects don't alter the core system clock, so it's easier to
    just use these hard-coded values. The assert will guarantee compatibility
    breaks will get noticed.

    These values assume:
      I2C Clock: 80 MHz
      Digital Noise Filter Coefficient: 4
      Analog Noise Filter: Enabled
      Rise Time: 50ns
      Fall Time: 50ns
    -------------------------------------------------------------------------*/
    RT_HARD_ASSERT( freqMHz == 80 );

    ANFOFF::clear( mPeriph, CR1_ANFOFF );          // Enable analog filter
    DNF::set( mPeriph, ( 4u << CR1_DNF_Pos ) );    // Enable digital filter

    switch ( cfg.HWInit.frequency )
    {
      case Chimera::I2C::Frequency::F400KHZ:
        TIMINGR_ALL::set( mPeriph, 0x00B02585 );
        break;

      case Chimera::I2C::Frequency::F100KHZ:
        TIMINGR_ALL::set( mPeriph, 0x105081FF );
        break;

      default:
        RT_HARD_ASSERT( false );
        break;
    };

    /*-------------------------------------------------------------------------
    Configure the interrupt controller
    -------------------------------------------------------------------------*/
    for ( size_t isrVec = 0; isrVec < Resource::ISR_VEC_PER_PERIPH; isrVec++ )
    {
      INT::setPriority( Resource::IRQSignals[ mResourceIndex ][ isrVec ], INT::I2C_IT_PREEMPT_PRIORITY, 0u );
      INT::enableIRQ( Resource::IRQSignals[ mResourceIndex ][ isrVec ] );
    }

    /* Enable the peripheral */
    PE::set( mPeriph, CR1_PE );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::read( const uint16_t address, void *const data, const size_t length )
  {
    /*-------------------------------------------------------------------------
    Enable interrupts
    -------------------------------------------------------------------------*/
    ICR_ALL::set( mPeriph, EV_ICR_FLAGS );                  // Clear any pending interrupts
    TXIE::clear( mPeriph, CR1_TXIE );                       // Disable the TXIS interrupt
    CR1_ALL::setbit( mPeriph, EV_ISR_FLAGS | CR1_RXIE );    // Enable ISR events

    /*-------------------------------------------------------------------------
    Set up the read transfer
    -------------------------------------------------------------------------*/
    RDWRN::set( mPeriph, CR2_RD_WRN );
    return transfer( address, nullptr, data, length );
  }


  Chimera::Status_t Driver::write( const uint16_t address, const void *const data, const size_t length )
  {
    /*-------------------------------------------------------------------------
    Enable interrupts
    -------------------------------------------------------------------------*/
    ICR_ALL::set( mPeriph, EV_ICR_FLAGS );                  // Clear any pending interrupts
    RXIE::clear( mPeriph, CR1_RXIE );                       // Disable the RXNE interrupt
    CR1_ALL::setbit( mPeriph, EV_ISR_FLAGS | CR1_TXIE );    // Enable ISR events

    /*-------------------------------------------------------------------------
    Set up the write transfer
    -------------------------------------------------------------------------*/
    RDWRN::clear( mPeriph, CR2_RD_WRN );
    return transfer( address, data, nullptr, length );
  }


  Chimera::Status_t Driver::transfer( const uint16_t address, const void *const tx_data, void *const rx_data,
                                      const size_t length )
  {
    /*-------------------------------------------------------------------------
    For now, only an interrupt based transfer is allowed
    -------------------------------------------------------------------------*/
    return transferIT( address, tx_data, rx_data, length );
  }


  TxfrCB Driver::whatHappened()
  {
    /*-------------------------------------------------------------------------
    Another transfer could be on-going, so copy out what happened last time.
    Protected from ISRs to prevent updating mid-copy.
    -------------------------------------------------------------------------*/
    enterCriticalSection();
    auto tmp = mTransferCache;
    exitCriticalSection();

    return tmp;
  }


  void Driver::enterCriticalSection()
  {
    for ( size_t irqIdx = 0; irqIdx < Resource::ISR_VEC_PER_PERIPH; irqIdx++ )
    {
      INT::disableIRQ( Resource::IRQSignals[ mResourceIndex ][ irqIdx ] );
    }
  }


  void Driver::exitCriticalSection()
  {
    for ( size_t irqIdx = 0; irqIdx < Resource::ISR_VEC_PER_PERIPH; irqIdx++ )
    {
      INT::enableIRQ( Resource::IRQSignals[ mResourceIndex ][ irqIdx ] );
    }
  }


  Chimera::Status_t Driver::transferIT( const uint16_t address, const void *const tx_data, void *const rx_data,
                                        const size_t length )
  {
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if ( length > 255 )
    {
      /*-----------------------------------------------------------------------
      The particular driver implementation only works for byte transfers that
      are <= 255. A different architecture is needed for more per RM0394.
      -----------------------------------------------------------------------*/
      return Chimera::Status::MEMORY;
    }
    else if ( mTransfer.inProgress )
    {
      return Chimera::Status::BUSY;
    }

    /*-------------------------------------------------------------------------
    Set up the transfer control block
    -------------------------------------------------------------------------*/
    mTransfer.clear();
    mTransfer.inProgress    = true;
    mTransfer.slave_address = address;
    mTransfer.txData        = reinterpret_cast<const uint8_t *>( tx_data );
    mTransfer.rxData        = reinterpret_cast<uint8_t *>( rx_data );
    mTransfer.bytes_left    = length;
    mTransfer.state         = TxfrState::DATA;
    mTransfer.offset        = 0;
    mTransfer.txfrMode      = Chimera::Peripheral::TransferMode::INTERRUPT;

    /*-------------------------------------------------------------------------
    Configure the hardware transfer address. The address field is shifted by
    1 additional position due to forced 7-bit transfer mode. See the definition
    for this field in RM0394 for more information.
    -------------------------------------------------------------------------*/
    ADD10::clear( mPeriph, CR2_ADD10 );                                           // 7-bit addressing mode
    SADD::set( mPeriph, ( mTransfer.slave_address << ( CR2_SADD_Pos + 1 ) ) );    // Destination address

    /*-------------------------------------------------------------------------
    Setup transfers using the control flow for NBYTES <=255.
    -------------------------------------------------------------------------*/
    NBYTES::set( mPeriph, ( mTransfer.bytes_left << CR2_NBYTES_Pos ) );    // Total data bytes
    RELOAD::clear( mPeriph, CR2_RELOAD );                                  // Disable reload of NBYTES
    AUTOEND::clear( mPeriph, CR2_AUTOEND );                                // Disable auto stop at end of transfer

    /*-------------------------------------------------------------------------
    Finally, start the transfer
    -------------------------------------------------------------------------*/
    START::set( mPeriph, CR2_START );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::transferDMA( const uint16_t address, const void *const tx_data, void *const rx_data,
                                         const size_t length )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  void Driver::IRQErrorHandler()
  {
    using namespace Chimera::Thread;
    using namespace Chimera::Peripheral;

    /*-------------------------------------------------------------------------
    Local Variables
    -------------------------------------------------------------------------*/
    const uint32_t isr = ISR_ALL::get( mPeriph );

    /*-------------------------------------------------------------------------
    Update the LLD transfer control block with common information to all errors
    -------------------------------------------------------------------------*/
    mTransfer.inProgress = false;
    mTransfer.state      = TxfrState::ERROR;

    /*-------------------------------------------------------------------------
    Instruct hardware to stop the transfer
    -------------------------------------------------------------------------*/
    STOP::set( mPeriph, CR2_STOP );

    /*-------------------------------------------------------------------------
    Handle Bus Error
    -------------------------------------------------------------------------*/
    if ( isr & ISR_BERR )
    {
      BERRCF::set( mPeriph, ICR_BERRCF );
      mTransfer.errorBF |= ( 1u << TxfrError::ERR_BUS );
    }

    /*-------------------------------------------------------------------------
    Handle Arbitration Loss
    -------------------------------------------------------------------------*/
    if ( isr & ISR_ARLO )
    {
      ARLOCF::set( mPeriph, ICR_ARLOCF );
      mTransfer.errorBF |= ( 1u << TxfrError::ERR_ARBITRATION_LOSS );
    }

    /*-------------------------------------------------------------------------
    Handle Overrun/Underrun
    -------------------------------------------------------------------------*/
    if ( isr & ISR_OVR )
    {
      OVRCF::set( mPeriph, ICR_OVRCF );
      mTransfer.errorBF |= ( 1u << TxfrError::ERR_OVER_UNDER_RUN );
    }

    /*-------------------------------------------------------------------------
    Handle Packet Error Check
    -------------------------------------------------------------------------*/
    if ( isr & ISR_PECERR )
    {
      PECCF::set( mPeriph, ICR_PECCF );
      mTransfer.errorBF |= ( 1u << TxfrError::ERR_PACKET_ERROR_CHECK );
    }

    /*-------------------------------------------------------------------------
    Handle Timeout
    -------------------------------------------------------------------------*/
    if ( isr & ISR_TIMEOUT )
    {
      TIMOUTCF::set( mPeriph, ICR_TIMOUTCF );
      mTransfer.errorBF |= ( 1u << TxfrError::ERR_TIMEOUT );
    }

    /*-------------------------------------------------------------------------
    Handle SMbus Alert
    -------------------------------------------------------------------------*/
    if ( isr & ISR_ALERT )
    {
      ALERTCF::set( mPeriph, ICR_ALERTCF );
      mTransfer.errorBF |= ( 1u << TxfrError::ERR_SMBUS_ALERT );
    }

    /*-------------------------------------------------------------------------
    Wake the user task to respond to the error
    -------------------------------------------------------------------------*/
    mTransferCache = mTransfer;
    sendTaskMsg( INT::getUserTaskId( Type::PERIPH_I2C ), ITCMsg::TSK_MSG_ISR_HANDLER, TIMEOUT_DONT_WAIT );
  }


  void Driver::IRQEventHandler()
  {
    using namespace Chimera::Thread;
    using namespace Chimera::Peripheral;

    /*-------------------------------------------------------------------------
    Local Constants
    -------------------------------------------------------------------------*/
    const uint32_t isr = ISR_ALL::get( mPeriph ); /* Interrupt status register */
    const uint32_t cr2 = CR2_ALL::get( mPeriph ); /* Control register 2 */

    /*-------------------------------------------------------------------------
    Local Variables
    -------------------------------------------------------------------------*/
    bool wake_user_thread = false; /**< Does the HLD thread need waking? */

    /*-------------------------------------------------------------------------
    Edge case we shouldn't normally hit. Clear all ISR events and disable the
    interrupt enable flags for Event type signals.
    -------------------------------------------------------------------------*/
    if ( !mTransfer.inProgress || ( mTransfer.txfrMode != TransferMode::INTERRUPT ) )
    {
      ICR_ALL::set( mPeriph, EV_ICR_FLAGS );
      CR1_ALL::clear( mPeriph, EV_ISR_FLAGS );
      return;
    }

    /*-------------------------------------------------------------------------
    Handle Transfers
    -------------------------------------------------------------------------*/
    if ( ( cr2 & CR2_RD_WRN ) && ( isr & ISR_RXNE ) && mTransfer.bytes_left )    // RX
    {
      uint32_t tmp                         = RXDATA::get( mPeriph );
      mTransfer.rxData[ mTransfer.offset ] = static_cast<uint8_t>( tmp & 0xFF );
      mTransfer.offset++;
      mTransfer.bytes_left--;
    }
    else if ( ( cr2 & CR2_RD_WRN ) == 0 )    // TX
    {
      /*-----------------------------------------------------------------------
      Did a NACK event occur?
      -----------------------------------------------------------------------*/
      if ( isr & ISR_NACKF )
      {
        /*---------------------------------------------------------------------
        Update the transfer control block to reflect the event
        ---------------------------------------------------------------------*/
        mTransfer.inProgress = false;
        mTransfer.state      = TxfrState::ERROR;
        mTransfer.errorBF |= ( 1u << TxfrError::ERR_NACK );

        /*---------------------------------------------------------------------
        Ack the event in hardware
        ---------------------------------------------------------------------*/
        NACKCF::set( mPeriph, ICR_NACKCF );
        STOP::set( mPeriph, CR2_STOP );
        wake_user_thread = true;
      }

      /*-----------------------------------------------------------------------
      Did any kind of TX interrupt occur? Write the next data byte.
      -----------------------------------------------------------------------*/
      if ( ( isr & ISR_TXIS ) && mTransfer.bytes_left )
      {
        TXDATA::set( mPeriph, mTransfer.txData[ mTransfer.offset ] );
        mTransfer.offset++;
        mTransfer.bytes_left--;
      }
    }

    /*-----------------------------------------------------------------------
    If the transfer is about to end when this byte clocks in/out, the TC flag
    will immediately be set. Ensure the stop condition will be sent once the
    byte actually finishes.
    -----------------------------------------------------------------------*/
    if ( !mTransfer.bytes_left && TC::get( mPeriph ) )    // Only hit if AUTOEND == 0
    {
      STOP::set( mPeriph, CR2_STOP );
    }

    /*-----------------------------------------------------------------------
    Did a stop condition occur? This is the "real" TC event.
    -----------------------------------------------------------------------*/
    if ( isr & ISR_STOPF )
    {
      STOPCF::set( mPeriph, ICR_STOPCF );

      mTransfer.inProgress = false;
      mTransfer.state      = TxfrState::COMPLETE;
      wake_user_thread     = true;
    }

    /*-------------------------------------------------------------------------
    Wake up the HLD thread for post-processing
    -------------------------------------------------------------------------*/
    if ( wake_user_thread )
    {
      mTransferCache = mTransfer;
      sendTaskMsg( INT::getUserTaskId( Type::PERIPH_I2C ), ITCMsg::TSK_MSG_ISR_HANDLER, TIMEOUT_DONT_WAIT );
    }
  }


  void Driver::onDMAComplete( const Chimera::DMA::TransferStats &stats )
  {
    // Nothing to do here!
  }

}    // namespace Thor::LLD::I2C
#endif /* THOR_LLD_I2C */
