/******************************************************************************
 *  File Name:
 *    hw_i2c_driver.cpp
 *
 *  Description:
 *    I2C driver for STM32F4
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
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

#if defined( THOR_LLD_I2C )
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
    SWRST::set( mPeriph, CR1_SWRST );
    SWRST::clear( mPeriph, CR1_SWRST );

    /* Assign peripheral clock frequency */
    auto rccControl = Thor::LLD::RCC::getCoreClockCtrl();
    auto freq       = rccControl->getPeriphClock( pType, reinterpret_cast<std::uintptr_t>( mPeriph ) );
    auto freqMHz    = freq / 1000000;

    RT_HARD_ASSERT( ( 2 <= freqMHz ) && ( freqMHz <= 50 ) );
    FREQ::set( mPeriph, ( freqMHz << CR2_FREQ_Pos ) );

    /* Set Full Speed or Standard mode */
    if ( cfg.HWInit.frequency == Chimera::I2C::Frequency::F400KHZ )
    {
      // Per datasheet, 400kHz speed can't be reached if peripheral clock isn't multiple of 10
      RT_HARD_ASSERT( ( freq % 10 ) == 0 );
      FS::set( mPeriph, CCR_FS );
    }
    else
    {
      RT_HARD_ASSERT( cfg.HWInit.frequency == Chimera::I2C::Frequency::F100KHZ );
      FS::clear( mPeriph, CCR_FS );
    }

    /* Set clock duty cycle */
    DUTY::clear( mPeriph, CCR_DUTY );

    /* Configure clock high/low timings */
    const float pClockPeriod   = 1.0f / static_cast<float>( freq );
    const float pClockPeriodNs = pClockPeriod * 1000000000.0f;

    float tmp     = 0.0f;
    float rtRatio = 0.0f;
    if ( cfg.HWInit.frequency == Chimera::I2C::Frequency::F100KHZ )
    {
      tmp     = ( period100khz / 2.0f ) / pClockPeriodNs;
      rtRatio = pClockPeriodNs / maxrt100khz;
    }
    else
    {
      tmp     = ( period400khz / 3.0f ) / pClockPeriodNs;
      rtRatio = pClockPeriodNs / maxrt400khz;
    }

    size_t ccr_val = static_cast<size_t>( tmp );
    CCR::set( mPeriph, ( ccr_val << CCR_CCR_Pos ) );

    /* Configure clock rise time limits */
    size_t rt_val = static_cast<size_t>( rtRatio ) + 1u;
    TRISE::set( mPeriph, ( rt_val << TRISE_TRISE_Pos ) );

    /*-------------------------------------------------------------------------
    Configure basic hardware settings
    -------------------------------------------------------------------------*/
    /* Configure interrupts */
    ITBUFEN::set( mPeriph, CR2_ITBUFEN );
    ITEVTEN::set( mPeriph, CR2_ITEVTEN );
    ITERREN::set( mPeriph, CR2_ITERREN );

    /* Configure noise filtering */
    ANOFF::clear( mPeriph, FLTR_ANOFF );
    DNF::set( mPeriph, ( 3u << FLTR_DNF_Pos ) );

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
    mTransfer.inProgress = true;
    mTransfer.address    = address;
    mTransfer.txData     = tx_data;
    mTransfer.rxData     = rx_data;
    mTransfer.length     = length;
    mTransfer.state      = TxfrState::ADDRESS;
    mTransfer.mode       = Chimera::Peripheral::TransferMode::INTERRUPT;

    /* Ensure the expected interrupts are enabled */
    enableISRSignal( bSB );

    /*-------------------------------------------------------------------------
    Trigger the transfer. This will jump to the event interrupt handler.
    -------------------------------------------------------------------------*/
    START::set( mPeriph, CR1_START );

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
    // Notify the high priority thread for any errors
  }


  void Driver::IRQEventHandler()
  {
    using namespace Chimera::Thread;
    using namespace Chimera::Peripheral;

    /*-------------------------------------------------------------------------
    Local Variables
    -------------------------------------------------------------------------*/
    const uint32_t sr1 = SR1::get( mPeriph );
    const uint32_t sr2 = SR2::get( mPeriph );
    uint32_t data_reg = 0;
    uint8_t *rx_data_ptr = nullptr;
    const uint8_t *tx_data_ptr = nullptr;

    /*-------------------------------------------------------------------------
    Handle an on-going transfer in Interrupt mode
    -------------------------------------------------------------------------*/
    if( mTransfer.inProgress && ( mTransfer.mode == Chimera::Peripheral::TransferMode::INTERRUPT ) )
    {
      switch( mTransfer.state )
      {
        case TxfrState::ADDRESS:
          if( sr1 & SR1_SB )
          {
            DR::set( mPeriph, mTransfer.address << DR_DR_Pos );
            mTransfer.index = 0;
            mTransfer.state = TxfrState::DATA;
          }
          break;

        case TxfrState::DATA:
          /*-------------------------------------------------------------------
          Pull new data off the RX shift register
          -------------------------------------------------------------------*/
          if ( ( sr1 & SR1_RXNE ) && mTransfer.rxData )
          {
            data_reg                       = DR::get( mPeriph );
            rx_data_ptr                    = reinterpret_cast<uint8_t *>( mTransfer.rxData );
            rx_data_ptr[ mTransfer.index ] = static_cast<uint8_t>( data_reg );
          }

          /*-------------------------------------------------------------------
          Push new data to the TX shift register
          -------------------------------------------------------------------*/
          if ( ( sr1 & SR1_TXE ) && mTransfer.txData )
          {
            tx_data_ptr = reinterpret_cast<const uint8_t *>( mTransfer.txData );
            DR::set( mPeriph, tx_data_ptr[ mTransfer.index ] );
          }

          /*-------------------------------------------------------------------
          Update the tracking data
          -------------------------------------------------------------------*/
          mTransfer.index++;
          if( mTransfer.index < mTransfer.length )
          {
            mTransfer.state = TxfrState::COMPLETE;
          }
          break;

        case TxfrState::COMPLETE:
          sendTaskMsg( INT::getUserTaskId( Type::PERIPH_I2C ), ITCMsg::TSK_MSG_ISR_HANDLER, TIMEOUT_DONT_WAIT );
          break;

        case TxfrState::ERROR:
        default:

          break;
      };
    }
  }


  void Driver::onDMAComplete( const Chimera::DMA::TransferStats &stats )
  {
    // Nothing to do here!
  }

}    // namespace Thor::LLD::I2C
#endif /* THOR_LLD_I2C */
