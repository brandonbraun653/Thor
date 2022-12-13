/******************************************************************************
 *  File Name:
 *    spi_common_driver.cpp
 *
 *  Description:
 *    Shared low level driver for STM32 SPI peripherals
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/* STL Includes */
#include <cstring>
#include <limits>

/* Chimera Includes */
#include <Chimera/algorithm>
#include <Chimera/common>
#include <Chimera/utility>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/inc/rcc>
#include <Thor/lld/interface/inc/spi>
#include <Thor/lld/interface/inc/interrupt>
#include <Thor/lld/interface/spi/common_driver/spi_common_intf.hpp>


#if defined( THOR_SPI ) && ( defined( TARGET_STM32L4 ) || defined( TARGET_STM32F4 ) )

namespace Thor::LLD::SPI
{
  /*---------------------------------------------------------------------------
  Variables
  ---------------------------------------------------------------------------*/
  static Driver s_spi_drivers[ NUM_SPI_PERIPHS ];

  /*---------------------------------------------------------------------------
  Private Functions
  ---------------------------------------------------------------------------*/
  static float calculate_clock_performance( const size_t goal, const size_t actVal, void *const data )
  {
    /*-------------------------------------------------------------------------
    Input checks:
      data    --  System's currently configured peripheral source clock
      actVal  --  Peripheral clock divider under consideration

    If either are null, permanently indicate worst case performance.
    -------------------------------------------------------------------------*/
    if ( !data || !actVal )
    {
      return std::numeric_limits<float>::max();
    }

    size_t spi_periph_clock_freq = *( reinterpret_cast<size_t *const>( data ) );
    size_t spi_output_clock_freq = spi_periph_clock_freq / actVal;

    float fGoal     = static_cast<float>( goal );
    float fAchieved = static_cast<float>( spi_output_clock_freq );

    return fAchieved - fGoal;
  }

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    /*-------------------------------------------------------------------------
    Attach all the expected peripherals to the drivers
    -------------------------------------------------------------------------*/
    if ( attachDriverInstances( s_spi_drivers, ARRAY_COUNT( s_spi_drivers ) ) )
    {
      return Chimera::Status::OK;
    }
    else
    {
      return Chimera::Status::FAIL;
    }
  }


  Driver_rPtr getDriver( const Chimera::SPI::Channel channel )
  {
    if ( auto idx = getResourceIndex( channel ); idx != INVALID_RESOURCE_INDEX )
    {
      return &s_spi_drivers[ idx ];
    }
    else
    {
      return nullptr;
    }
  }


  /*---------------------------------------------------------------------------
  Low Level Driver Implementation
  ---------------------------------------------------------------------------*/
  Driver::Driver() : mPeriph( nullptr ), resourceIndex( std::numeric_limits<size_t>::max() ), periphConfig( nullptr )
  {
    txfr.txBuffer        = nullptr;
    txfr.txTransferCount = 0;
    txfr.txTransferSize  = 0;
    txfr.rxBuffer        = nullptr;
    txfr.rxTransferCount = 0;
    txfr.rxTransferSize  = 0;
    txfr.waitingOnTX     = false;
    txfr.waitingOnRX     = false;
    txfr.status          = Chimera::SPI::Status::TRANSFER_COMPLETE;
  }

  Driver::~Driver()
  {
  }


  Chimera::Status_t Driver::attach( RegisterMap *const peripheral )
  {
    /*-------------------------------------------------------------------------
    Get peripheral descriptor settings
    -------------------------------------------------------------------------*/
    mPeriph       = peripheral;
    resourceIndex = getResourceIndex( reinterpret_cast<std::uintptr_t>( peripheral ) );

    /*-------------------------------------------------------------------------
    Handle the ISR configuration
    -------------------------------------------------------------------------*/
    INT::disableIRQ( Resource::IRQSignals[ resourceIndex ] );
    INT::clearPendingIRQ( Resource::IRQSignals[ resourceIndex ] );
    INT::setPriority( Resource::IRQSignals[ resourceIndex ], INT::SPI_IT_PREEMPT_PRIORITY, 0u );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::reset()
  {
    auto rcc = Thor::LLD::RCC::getPeriphClockCtrl();
    return rcc->reset( Chimera::Peripheral::Type::PERIPH_SPI, resourceIndex );
  }


  void Driver::clockEnable()
  {
    auto rcc = Thor::LLD::RCC::getPeriphClockCtrl();
    rcc->enableClock( Chimera::Peripheral::Type::PERIPH_SPI, resourceIndex );
  }


  void Driver::clockDisable()
  {
    auto rcc = Thor::LLD::RCC::getPeriphClockCtrl();
    rcc->disableClock( Chimera::Peripheral::Type::PERIPH_SPI, resourceIndex );
  }


  Chimera::Status_t Driver::configure( const Chimera::SPI::HardwareInit &setup )
  {
    using namespace Chimera::Algorithm::RegisterOptimization;

    /*-------------------------------------------------------------------------
    Configure the clocks
    -------------------------------------------------------------------------*/
    clockEnable();

    /*-------------------------------------------------------------------------
    Find the best clock divisor need to achieve the desired clock frequency
    -------------------------------------------------------------------------*/
    auto systemClock = Thor::LLD::RCC::getCoreClockCtrl();
    auto periphAddr  = reinterpret_cast<std::uintptr_t>( mPeriph );
    auto clockFreq   = systemClock->getPeriphClock( Chimera::Peripheral::Type::PERIPH_SPI, periphAddr );

    if ( clockFreq == Thor::LLD::RCC::INVALID_CLOCK )
    {
      return Chimera::Status::FAIL;
    }

    RegOptimizerData cfg;
    cfg.actVals      = Configuration::ClockDivisor::valOptions.data();
    cfg.regVals      = Configuration::ClockDivisor::regOptions.data();
    cfg.numOptions   = Configuration::ClockDivisor::NUM_OPTIONS;
    cfg.desiredValue = setup.clockFreq;
    cfg.optimizer    = calculate_clock_performance;

    Reg32_t clockDivisor = findOptimalSetting( cfg, Configuration::ClockDivisor::DIV_32, &clockFreq );

    /*-------------------------------------------------------------------------
    Stop the config if there are any invalid config options
    -------------------------------------------------------------------------*/
    if ( setup.csMode == Chimera::SPI::CSMode::AUTO_BETWEEN_TRANSFER )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------------------------------
    Follow the configuration sequence from RM0390
    -------------------------------------------------------------------------*/
    /* Clear any faults */
    if ( MODF::get( mPeriph ) )
    {
      volatile Reg32_t UNUSED( dummyRead ) = SR_ALL::get( mPeriph );
    }

    /* Destroy all previous settings */
    this->reset();

    /* Bit Transfer Order */
    LSBFIRST::set( mPeriph, ConfigMap::BitOrderToRegConfig[ static_cast<size_t>( setup.bitOrder ) ] );

    /* Transfer Bit Width */
    DS::set( mPeriph, ConfigMap::DataSizeToRegConfig[ static_cast<size_t>( setup.dataSize ) ] );

    /* Peripheral Clock Divisor */
    BR::set( mPeriph, clockDivisor );

    /* Master/Slave Control Mode */
    MSTR::set( mPeriph, ConfigMap::ControlModeToRegConfig[ static_cast<size_t>( setup.controlMode ) ] );

    SSM::set( mPeriph, CR1_SSM );
    SSI::set( mPeriph, CR1_SSI );

    /* Clock Phase and Polarity */
    switch ( setup.clockMode )
    {
      case Chimera::SPI::ClockMode::MODE0:
        CPOL::set( mPeriph, 0u );
        CPHA::set( mPeriph, 0u );
        break;

      case Chimera::SPI::ClockMode::MODE1:
        CPOL::set( mPeriph, 0u );
        CPHA::set( mPeriph, CR1_CPHA );
        break;

      case Chimera::SPI::ClockMode::MODE2:
        CPOL::set( mPeriph, CR1_CPOL );
        CPHA::set( mPeriph, 0u );
        break;

      case Chimera::SPI::ClockMode::MODE3:
        CPOL::set( mPeriph, CR1_CPOL );
        CPHA::set( mPeriph, CR1_CPHA );
        break;

      default:
        break;
    }

    /* Configure CR2 */
    SSOE::set( mPeriph, CR2_SSOE );

/* Set up the transfer width */
#if defined( TARGET_STM32L4 )
    prjConfigureTransferWidth( mPeriph, periphConfig->dataSize );
#endif

    /*-------------------------------------------------------------------------
    Enable the SPI peripheral
    -------------------------------------------------------------------------*/
    SPE::set( mPeriph, CR1_SPE );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::registerConfig( Chimera::SPI::HardwareInit *config )
  {
    if ( !config )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    this->periphConfig = config;
    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::transfer( const void *txBuffer, void *rxBuffer, const size_t bufferSize )
  {
    /*-------------------------------------------------------------------------
    Make sure the transfer can begin
    -------------------------------------------------------------------------*/
    if ( BSY::get( mPeriph ) || ( txfr.status != Chimera::SPI::Status::TRANSFER_COMPLETE ) )
    {
      return Chimera::Status::BUSY;
    }

    /*-------------------------------------------------------------------------
    Disable all SPI interrupts for this peripheral
    -------------------------------------------------------------------------*/
    Thor::LLD::INT::disableIRQ( Resource::IRQSignals[ resourceIndex ] );
    CR2_ALL::clear( mPeriph, ( CR2_TXEIE | CR2_RXNEIE | CR2_ERRIE ) );

    /*-------------------------------------------------------------------------
    Queue up the transfer
    -------------------------------------------------------------------------*/
    txfr.txBuffer        = reinterpret_cast<const uint8_t *>( txBuffer );
    txfr.txTransferCount = 0u;
    txfr.txTransferSize  = bufferSize;
    txfr.rxBuffer        = reinterpret_cast<uint8_t *>( rxBuffer );
    txfr.rxTransferCount = 0u;
    txfr.rxTransferSize  = bufferSize;
    txfr.status          = Chimera::SPI::Status::TRANSFER_IN_PROGRESS;

    /*-------------------------------------------------------------------------
    Data transfers must have 8-bit or 16-bit aligned access. Force 8-bit.
    -------------------------------------------------------------------------*/
    auto dr = reinterpret_cast<volatile uint8_t *>( &mPeriph->DR );
    prjConfigureTransferWidth( mPeriph, Chimera::SPI::DataSize::SZ_8BIT );

    /*-------------------------------------------------------------------------
    Write the data in a blocking fashion
    -------------------------------------------------------------------------*/
    size_t bytesTransferred = 0;
    uint8_t tmpRxData = 0;

    while ( bytesTransferred < bufferSize )
    {
      /*-----------------------------------------------------------------------
      Wait for an empty hw transmit buffer
      -----------------------------------------------------------------------*/
#if defined( EMBEDDED )
      while ( !TXE::get( mPeriph ) && BSY::get( mPeriph ) )
      {
        continue;
      }
#endif

      /*-----------------------------------------------------------------------
      Inject the next byte into the TX FIFO
      -----------------------------------------------------------------------*/
      if ( txfr.txBuffer )
      {
        *dr = txfr.txBuffer[ txfr.txTransferCount++ ];
      }
      else
      {
        *dr = 0xFF;
      }

      /*-----------------------------------------------------------------------
      Wait for the RX FIFO to indicate it's ready. You can get stuck here in
      the debugger if single stepping and register auto-refresh is occurring.
      That will read the DR and clear RXNE.
      -----------------------------------------------------------------------*/
#if defined( EMBEDDED )
      while ( !RXNE::get( mPeriph ) )
      {
        continue;
      }
#endif

      /*-----------------------------------------------------------------------
      Read out the data
      -----------------------------------------------------------------------*/
      tmpRxData = mPeriph->DR;

      if ( txfr.rxBuffer )
      {
        txfr.rxBuffer[ txfr.rxTransferCount++ ] = tmpRxData;
      }

      /*-----------------------------------------------------------------------
      Update the loop counters
      -----------------------------------------------------------------------*/
      bytesTransferred++;
    }

    /*-------------------------------------------------------------------------
    Don't exit this blocking function before all the
    TX FIFO transfers are finished.
    -------------------------------------------------------------------------*/
#if defined( EMBEDDED )
    while ( BSY::get( mPeriph ) )
    {
      continue;
    }
#endif

    txfr.status = Chimera::SPI::Status::TRANSFER_COMPLETE;
    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::transferIT( const void *txBuffer, void *rxBuffer, const size_t bufferSize )
  {
    /*-------------------------------------------------------------------------
    Make sure the transfer can begin
    -------------------------------------------------------------------------*/
    if ( BSY::get( mPeriph ) || ( txfr.status != Chimera::SPI::Status::TRANSFER_COMPLETE ) )
    {
      return Chimera::Status::BUSY;
    }

    /*-------------------------------------------------------------------------
    Configure the interrupts
    -------------------------------------------------------------------------*/
    Thor::LLD::INT::enableIRQ( Resource::IRQSignals[ resourceIndex ] );
    enterCriticalSection();
    CR2_ALL::set( mPeriph, ( CR2_TXEIE | CR2_RXNEIE | CR2_ERRIE ) );

    /*-------------------------------------------------------------------------
    Queue up the transfer
    -------------------------------------------------------------------------*/
    txfr.txBuffer        = reinterpret_cast<const uint8_t *>( txBuffer );
    txfr.txTransferCount = 0u;
    txfr.txTransferSize  = bufferSize;
    txfr.rxBuffer        = reinterpret_cast<uint8_t *>( rxBuffer );
    txfr.rxTransferCount = 0u;
    txfr.rxTransferSize  = bufferSize;
    txfr.status          = Chimera::SPI::Status::TRANSFER_IN_PROGRESS;

    /*-------------------------------------------------------------------------
    Data transfers must have 8-bit or 16-bit aligned access. Force 8-bit.
    -------------------------------------------------------------------------*/
    auto dr = reinterpret_cast<volatile uint8_t *>( &mPeriph->DR );
    prjConfigureTransferWidth( mPeriph, Chimera::SPI::DataSize::SZ_8BIT );

    /*-------------------------------------------------------------------------
    Start the transfer by writing the first byte
    -------------------------------------------------------------------------*/
    if ( txfr.txBuffer )
    {
      *dr = txfr.txBuffer[ txfr.txTransferCount ];
    }
    else
    {
      *dr = 0xFF;
    }

    txfr.waitingOnTX = false;
    txfr.waitingOnRX = true;
    txfr.txTransferCount++;
    exitCriticalSection();

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::transferDMA( const void *txBuffer, void *rxBuffer, const size_t bufferSize )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  inline void Driver::enterCriticalSection()
  {
    Thor::LLD::INT::disableIRQ( Resource::IRQSignals[ resourceIndex ] );
  }


  inline void Driver::exitCriticalSection()
  {
    Thor::LLD::INT::enableIRQ( Resource::IRQSignals[ resourceIndex ] );
  }


  void Driver::IRQHandler()
  {
    using namespace Chimera::Thread;
    using namespace Chimera::Peripheral;

    /*-------------------------------------------------------------------------
    Save critical register information
    -------------------------------------------------------------------------*/
    const volatile Reg32_t SR  = SR_ALL::get( mPeriph );
    const volatile Reg32_t CR2 = CR2_ALL::get( mPeriph );

    /*-------------------------------------------------------------------------
    Data transfers must have 8-bit or 16-bit aligned access.
    Currently hardcoded to 8 for development...
    -------------------------------------------------------------------------*/
    auto dr_8t = reinterpret_cast<volatile uint8_t *>( &mPeriph->DR );

    /*-------------------------------------------------------------------------
    HW TX Buffer Empty (Next frame can be buffered)
    -------------------------------------------------------------------------*/
    if ( txfr.waitingOnTX && ( CR2 & CR2_TXEIE ) && ( SR & SR_TXE ) )
    {
      txfr.waitingOnTX = false;
      txfr.waitingOnRX = true;

      if ( txfr.txTransferCount < txfr.txTransferSize )
      {
        /*---------------------------------------------------------------------
        Still more data to TX...
        ---------------------------------------------------------------------*/
        if ( txfr.txBuffer )
        {
          *dr_8t = txfr.txBuffer[ txfr.txTransferCount ];
        }
        else
        {
          *dr_8t = 0xFF;
        }

        txfr.txTransferCount++;
      }
      else
      {
        /*---------------------------------------------------------------------
        The TX half of the transfer is complete. Disable the TX FIFO empty ISR.
        ---------------------------------------------------------------------*/
        TXEIE::set( mPeriph, ~CR2_TXEIE );

        /*---------------------------------------------------------------------
        The user could have also requested a TX only transfer and not cared
        about received data. In this case, the entire transfer is now complete.
        ---------------------------------------------------------------------*/
        if ( !txfr.rxBuffer )
        {
          RXNEIE::set( mPeriph, ~CR2_RXNEIE );
          txfr.status = Chimera::SPI::Status::TRANSFER_COMPLETE;
        }
      }
    }

    /*-------------------------------------------------------------------------
    HW RX Buffer Full (Data is ready to be copied out)
    -------------------------------------------------------------------------*/
    if ( txfr.waitingOnRX && ( CR2 & CR2_RXNEIE ) && ( SR & SR_RXNE ) )
    {
      txfr.waitingOnTX = true;
      txfr.waitingOnRX = false;

      if ( txfr.rxTransferCount < txfr.rxTransferSize )
      {
        txfr.rxBuffer[ txfr.rxTransferCount ] = *dr_8t;
        txfr.rxTransferCount++;
      }

      if ( txfr.rxTransferCount == txfr.rxTransferSize )
      {
        RXNEIE::set( mPeriph, ~CR2_RXNEIE );
        txfr.status = Chimera::SPI::Status::TRANSFER_COMPLETE;
      }
    }
    else if ( txfr.waitingOnRX && !txfr.rxBuffer )
    {
      /*-----------------------------------------------------------------------
      The user specified a transmit only. Ignore the RX
      half and jump back to the TX half.
      -----------------------------------------------------------------------*/
      txfr.waitingOnTX = true;
      txfr.waitingOnRX = false;
    }

    /*-------------------------------------------------------------------------
    Handle Any Errors
    -------------------------------------------------------------------------*/
    if ( ( CR2 & CR2_ERRIE ) && ( SR & ( SR_CRCERR | SR_FRE | SR_MODF | SR_OVR ) ) )
    {
      txfr.status = Chimera::SPI::Status::TRANSFER_ERROR;
    }

    /*-------------------------------------------------------------------------
    Detect the end of transfer and wake up the post processor thread
    -------------------------------------------------------------------------*/
    if ( ( txfr.status == Chimera::SPI::Status::TRANSFER_COMPLETE ) || ( txfr.status == Chimera::SPI::Status::TRANSFER_ERROR ) )
    {
      sendTaskMsg( INT::getUserTaskId( Type::PERIPH_SPI ), ITCMsg::TSK_MSG_ISR_HANDLER, TIMEOUT_DONT_WAIT );
    }
  }
}    // namespace Thor::LLD::SPI


#if defined( STM32_SPI1_PERIPH_AVAILABLE )
void SPI1_IRQHandler()
{
  using namespace Thor::LLD::SPI;
  s_spi_drivers[ SPI1_RESOURCE_INDEX ].IRQHandler();
}
#endif

#if defined( STM32_SPI2_PERIPH_AVAILABLE )
void SPI2_IRQHandler()
{
  using namespace Thor::LLD::SPI;
  s_spi_drivers[ SPI2_RESOURCE_INDEX ].IRQHandler();
}
#endif

#if defined( STM32_SPI3_PERIPH_AVAILABLE )
void SPI3_IRQHandler()
{
  using namespace Thor::LLD::SPI;
  s_spi_drivers[ SPI3_RESOURCE_INDEX ].IRQHandler();
}
#endif

#if defined( STM32_SPI4_PERIPH_AVAILABLE )
void SPI4_IRQHandler()
{
  using namespace Thor::LLD::SPI;
  s_spi_drivers[ SPI4_RESOURCE_INDEX ].IRQHandler();
}
#endif


#if defined( STM32_SPI5_PERIPH_AVAILABLE )
void SPI5_IRQHandler()
{
  using namespace Thor::LLD::SPI;
  s_spi_drivers[ SPI5_RESOURCE_INDEX ].IRQHandler();
}
#endif


#if defined( STM32_SPI6_PERIPH_AVAILABLE )
void SPI6_IRQHandler()
{
  using namespace Thor::LLD::SPI;
  s_spi_drivers[ SPI6_RESOURCE_INDEX ].IRQHandler();
}
#endif

#endif /* TARGET_STM32L4 && THOR_DRIVER_SPI */
