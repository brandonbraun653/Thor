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
      mPeriph( nullptr ), mTxfrComplete( false ), mResourceIndex( INVALID_RESOURCE_INDEX ), mISRBits( 0 ), mTXDMARequestId( 0 ),
      mRXDMARequestId( 0 )
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
    mTxfrComplete   = false;

    /*-------------------------------------------------------------------------
    Configure clock controllers
    -------------------------------------------------------------------------*/
    /* Reset the peripheral clock */
    auto rccPeriph = Thor::LLD::RCC::getPeriphClockCtrl();
    rccPeriph->reset( pType, mResourceIndex );

    /* Reset the peripheral hardware using dedicated interface */
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
    return transfer( address, nullptr, data, length );
  }


  Chimera::Status_t Driver::write( const uint16_t address, const void *const data, const size_t length )
  {
    return transfer( address, data, nullptr, length );
  }


  Chimera::Status_t Driver::transfer( const uint16_t address, const void *const tx_data, void *const rx_data,
                                      const size_t length )
  {
    if ( length < DMA_TRANS_POINT )
    {
      return transferIT( address, tx_data, rx_data, length );
    }
    else
    {
      return transferDMA( address, tx_data, rx_data, length );
    }
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
    return Chimera::Status::NOT_SUPPORTED;
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
    // Notify the high priority thread for transfer complete
  }


  void Driver::IRQEventHandler()
  {
    // Notify the high priority thread for transfer complete
  }


  void Driver::onDMAComplete( const Chimera::DMA::TransferStats &stats )
  {
    // Notify the high priority thread for transfer complete
  }

}    // namespace Thor::LLD::I2C
#endif /* THOR_LLD_I2C */
