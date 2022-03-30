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
#include <Thor/i2c>
#include <Thor/lld/interface/inc/i2c>
#include <Thor/lld/interface/inc/interrupt>
#include <Thor/lld/interface/inc/rcc>

#if defined( THOR_LLD_I2C ) && defined( TARGET_STM32L4 )
namespace Thor::LLD::I2C
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t DMA_TRANS_POINT = 2; /**< When transfers switch from Interrupt to DMA driven */
  static constexpr uint32_t EV_ISR_FLAGS  =    /**< ISR Events to listen to */
      ( CR1_ERRIE | CR1_TCIE | CR1_STOPIE | CR1_NACKIE | CR1_ADDRIE | CR1_RXIE | CR1_TXIE );
  static constexpr uint32_t EV_ICR_FLAGS =     /**< All interrupt flag clear events */
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
    Set up the read transfer
    -------------------------------------------------------------------------*/
    RDWRN::set( mPeriph, CR2_RD_WRN );
    return transfer( address, nullptr, data, length );
  }


  Chimera::Status_t Driver::write( const uint16_t address, const void *const data, const size_t length )
  {
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


  TxfrCB *Driver::whatHappened()
  {
    /*-------------------------------------------------------------------------
    No need to protect this data. Can't guarantee the caller context will be
    safe from ISR modification after return.
    -------------------------------------------------------------------------*/
    return &mTransfer;
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
    Chimera::Thread::LockGuard _lock( *this );
    if ( mTransfer.inProgress )
    {
      return Chimera::Status::BUSY;
    }

    /* The peripheral should never be busy at this point */
    RT_HARD_ASSERT( BUSY::get( mPeriph ) == 0 );

    /*-------------------------------------------------------------------------
    Set up the transfer control block
    -------------------------------------------------------------------------*/
    mTransfer.clear();
    mTransfer.inProgress = true;
    mTransfer.address    = address;
    mTransfer.txData     = tx_data;
    mTransfer.rxData     = rx_data;
    mTransfer.length     = length;
    mTransfer.state      = TxfrState::DATA;
    mTransfer.index      = 0;
    mTransfer.mode       = Chimera::Peripheral::TransferMode::INTERRUPT;

    /*-------------------------------------------------------------------------
    Configure the hardware for the transfer. The address field is shifted by
    1 additional position due to forced 7-bit transfer mode. See the definition
    for this field in RM0394 for more information.
    -------------------------------------------------------------------------*/
    ADD10::clear( mPeriph, CR2_ADD10 );                                     // 7-bit addressing mode
    SADD::set( mPeriph, ( mTransfer.address << ( CR2_SADD_Pos + 1 ) ) );    // Destination address
    NBYTES::set( mPeriph, ( mTransfer.length << CR2_NBYTES_Pos ) );         // Total data bytes
    PECBYTE::clear( mPeriph, CR2_PECBYTE );                                 // No packet error checking
    AUTOEND::clear( mPeriph, CR2_AUTOEND );                                 // Auto end the transfer after last byte

    /*-------------------------------------------------------------------------
    Enable interrupts
    -------------------------------------------------------------------------*/
    ICR_ALL::set( mPeriph, EV_ICR_FLAGS );       // Clear any pending interrupts
    CR1_ALL::setbit( mPeriph, EV_ISR_FLAGS );    // Enable ISR events

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
    sendTaskMsg( INT::getUserTaskId( Type::PERIPH_I2C ), ITCMsg::TSK_MSG_ISR_HANDLER, TIMEOUT_DONT_WAIT );
  }


  /**
   * @brief Handle the normal event IRQ
   *
   * Handling behavior is taken from table 190 in RM0394.
   */
  void Driver::IRQEventHandler()
  {
    using namespace Chimera::Thread;
    using namespace Chimera::Peripheral;

    /*-------------------------------------------------------------------------
    Local Variables
    -------------------------------------------------------------------------*/
    const uint32_t isr = ISR_ALL::get( mPeriph );

    uint32_t data_reg          = 0;
    uint8_t *rx_data_ptr       = nullptr;
    const uint8_t *tx_data_ptr = nullptr;

    /*-------------------------------------------------------------------------
    Handle an on-going transfer in Interrupt mode
    -------------------------------------------------------------------------*/
    if ( mTransfer.inProgress && ( mTransfer.mode == Chimera::Peripheral::TransferMode::INTERRUPT ) )
    {
      /*-----------------------------------------------------------------------
      Did a NACK event occur? This is likely the most common issue.
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
      }

      /*-----------------------------------------------------------------------
      Handle the current transfer state
      -----------------------------------------------------------------------*/
      switch ( mTransfer.state )
      {
        case TxfrState::DATA:
          /*-------------------------------------------------------------------
          Pull new data off the RX shift register
          -------------------------------------------------------------------*/
          if ( isr & ISR_RXNE )
          {
            /*-----------------------------------------------------------------
            Reading the data causes the RXNE flag to be cleared
            -----------------------------------------------------------------*/
            data_reg = RXDATA::get( mPeriph );

            /*-----------------------------------------------------------------
            Copy into the user buffer if needed
            -----------------------------------------------------------------*/
            if ( mTransfer.rxData )
            {
              rx_data_ptr                    = reinterpret_cast<uint8_t *>( mTransfer.rxData );
              rx_data_ptr[ mTransfer.index ] = static_cast<uint8_t>( data_reg );
            }
          }

          /*-------------------------------------------------------------------
          Push new data to the TX shift register if required
          -------------------------------------------------------------------*/
          if ( ( isr & ISR_TXE ) && mTransfer.txData )
          {
            tx_data_ptr = reinterpret_cast<const uint8_t *>( mTransfer.txData );
            TXDATA::set( mPeriph, tx_data_ptr[ mTransfer.index ] );
          }

          /*-------------------------------------------------------------------
          Update the tracking data
          -------------------------------------------------------------------*/
          mTransfer.index++;
          if ( mTransfer.index >= mTransfer.length )
          {
            mTransfer.state = TxfrState::COMPLETE;
          }
          break;

        case TxfrState::COMPLETE:
          /*-------------------------------------------------------------------
          Send the stop condition to clear the event
          -------------------------------------------------------------------*/
          STOP::set( mPeriph, CR2_STOP );

          /*-------------------------------------------------------------------
          Update the driver transfer control block
          -------------------------------------------------------------------*/
          mTransfer.inProgress = false;
          mTransfer.errorBF    = 0;

          /*-------------------------------------------------------------------
          Let the HLD task know we're done
          -------------------------------------------------------------------*/
          sendTaskMsg( INT::getUserTaskId( Type::PERIPH_I2C ), ITCMsg::TSK_MSG_ISR_HANDLER, TIMEOUT_DONT_WAIT );
          break;

        case TxfrState::ERROR:
        default:
          /*-------------------------------------------------------------------
          Place the peripheral into a safe state
          -------------------------------------------------------------------*/
          STOP::set( mPeriph, CR2_STOP );
          ICR_ALL::set( mPeriph, EV_ICR_FLAGS );
          CR1_ALL::clear( mPeriph, EV_ISR_FLAGS );

          /*-------------------------------------------------------------------
          Ensure the LLD knows we're done
          -------------------------------------------------------------------*/
          mTransfer.inProgress = false;

          /*-------------------------------------------------------------------
          Wake the HLD task to handle the error
          -------------------------------------------------------------------*/
          sendTaskMsg( INT::getUserTaskId( Type::PERIPH_I2C ), ITCMsg::TSK_MSG_ISR_HANDLER, TIMEOUT_DONT_WAIT );
          break;
      };
    }
    else
    {
      /*-----------------------------------------------------------------------
      Edge case we shouldn't normally hit. Clear all ISR events and disable the
      interrupt enable flags for Event type signals.
      -----------------------------------------------------------------------*/
      ICR_ALL::set( mPeriph, EV_ICR_FLAGS );
      CR1_ALL::clear( mPeriph, EV_ISR_FLAGS );
    }
  }


  void Driver::onDMAComplete( const Chimera::DMA::TransferStats &stats )
  {
    // Nothing to do here!
  }

}    // namespace Thor::LLD::I2C
#endif /* THOR_LLD_I2C */
