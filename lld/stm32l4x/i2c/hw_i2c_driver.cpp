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
    /*-------------------------------------------------------------------------
    Local Constants
    -------------------------------------------------------------------------*/
    static constexpr auto pType         = Chimera::Peripheral::Type::PERIPH_I2C;
    static constexpr float period100khz = 10000.0f;
    static constexpr float maxrt100khz  = 1000.0f;
    static constexpr float period400khz = 2500.0f;
    static constexpr float maxrt400khz  = 300.0f;

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
    rccPeriph->reset( pType, mResourceIndex );

    /* Reset the peripheral hardware using dedicated interface */
    this->enableClock();
    PE::clear( mPeriph, CR1_PE );

    /* Assign peripheral clock frequency */
    auto rccControl = Thor::LLD::RCC::getCoreClockCtrl();
    auto freq       = rccControl->getPeriphClock( pType, reinterpret_cast<std::uintptr_t>( mPeriph ) );
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
    Configure hardware interrupts
    -------------------------------------------------------------------------*/
    CR1_ALL::set( mPeriph, ( CR1_NACKIE | CR1_TCIE | CR1_ERRIE | CR1_TXIE | CR1_RXIE ) );

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
    Input Protection
    -------------------------------------------------------------------------*/
    if ( !data || !length )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------------------------------
    Modify the address to indicate a read
    -------------------------------------------------------------------------*/
    uint16_t readAddress = ( ( address << 1u ) | 0x1 ) & 0xFF;
    return transfer( address, nullptr, data, length );
  }


  Chimera::Status_t Driver::write( const uint16_t address, const void *const data, const size_t length )
  {
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if ( !data || !length )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------------------------------
    Modify the address to indicate a write
    -------------------------------------------------------------------------*/
    uint16_t writeAddress = ( address << 1u ) & 0xFE;
    return transfer( writeAddress, data, nullptr, length );
  }


  Chimera::Status_t Driver::transfer( const uint16_t address, const void *const tx_data, void *const rx_data,
                                      const size_t length )
  {
    /*-------------------------------------------------------------------------
    Input Protections
    -------------------------------------------------------------------------*/
    if ( ( !tx_data && !rx_data ) || !length )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------------------------------
    Select the correct method of transaction
    -------------------------------------------------------------------------*/
    // if ( length < DMA_TRANS_POINT )
    // {
    return transferIT( address, tx_data, rx_data, length );
    // }
    // else
    // {
    //   return transferDMA( address, tx_data, rx_data, length );
    // }
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
    Set up the transfer
    -------------------------------------------------------------------------*/
    /* Update the transfer control block */
    mTransfer.clear();
    mTransfer.inProgress = true;
    mTransfer.address    = address;
    mTransfer.txData     = tx_data;
    mTransfer.rxData     = rx_data;
    mTransfer.length     = length;
    mTransfer.state      = TxfrState::DATA;
    mTransfer.index      = 0;
    mTransfer.mode       = Chimera::Peripheral::TransferMode::INTERRUPT;

    ADD10::clear( mPeriph, CR2_ADD10 );                                // 7-bit addressing mode
    SADD::set( mPeriph, ( mTransfer.address << CR2_SADD_Pos ) );       // Destination address
    NBYTES::set( mPeriph, ( mTransfer.length << CR2_NBYTES_Pos ) );    // Total data bytes

    if ( tx_data )
    {
      RDWRN::clear( mPeriph, CR2_RD_WRN );    // Write transfer
    }
    else
    {
      RDWRN::set( mPeriph, CR2_RD_WRN );    // Read transfer
    }

    /*-------------------------------------------------------------------------
    Trigger the transfer. This will jump to the event interrupt handler.
    -------------------------------------------------------------------------*/
    PECBYTE::clear( mPeriph, CR2_PECBYTE );    // No packet error checking
    AUTOEND::set( mPeriph, CR2_AUTOEND );      // Auto end the transfer after last byte
    START::set( mPeriph, CR2_START );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::transferDMA( const uint16_t address, const void *const tx_data, void *const rx_data,
                                         const size_t length )
  {
    // Probably don't need to setup a DMA callback. The peripheral should generate a
    // transfer complete event once everything is finished.
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
    Wake the user task to respond to the error
    -------------------------------------------------------------------------*/
    sendTaskMsg( INT::getUserTaskId( Type::PERIPH_I2C ), ITCMsg::TSK_MSG_ISR_HANDLER, TIMEOUT_DONT_WAIT );
  }


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
      NACK? This is the most common issue.
      -----------------------------------------------------------------------*/
      if ( isr & ISR_NACKF )
      {
        mTransfer.inProgress = false;
        mTransfer.state      = TxfrState::ERROR;
        mTransfer.errorBF |= ( 1u << TxfrError::ERR_NACK );
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
          if ( ( isr & ISR_RXNE ) && mTransfer.rxData )
          {
            data_reg                       = RXDATA::get( mPeriph );
            rx_data_ptr                    = reinterpret_cast<uint8_t *>( mTransfer.rxData );
            rx_data_ptr[ mTransfer.index ] = static_cast<uint8_t>( data_reg );
          }

          /*-------------------------------------------------------------------
          Push new data to the TX shift register
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
          if ( mTransfer.index <= mTransfer.length )
          {
            mTransfer.state = TxfrState::COMPLETE;
          }
          break;

        case TxfrState::COMPLETE:
          ICR_ALL::set( mPeriph, 0xFFFFFFFF );
          sendTaskMsg( INT::getUserTaskId( Type::PERIPH_I2C ), ITCMsg::TSK_MSG_ISR_HANDLER, TIMEOUT_DONT_WAIT );
          break;

        case TxfrState::ERROR:
        default:
          ICR_ALL::set( mPeriph, 0xFFFFFFFF );
          sendTaskMsg( INT::getUserTaskId( Type::PERIPH_I2C ), ITCMsg::TSK_MSG_ISR_HANDLER, TIMEOUT_DONT_WAIT );
          break;
      };
    }
    else
    {
      ICR_ALL::set( mPeriph, 0xFFFFFFFF );
    }
  }


  void Driver::onDMAComplete( const Chimera::DMA::TransferStats &stats )
  {
    // Nothing to do here!
  }

}    // namespace Thor::LLD::I2C
#endif /* THOR_LLD_I2C */
