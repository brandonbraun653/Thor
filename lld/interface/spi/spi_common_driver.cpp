/********************************************************************************
 *  File Name:
 *    spi_common_driver.cpp
 *
 *  Description:
 *    Shared low level driver for STM32 SPI peripherals
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

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


#if defined( THOR_LLD_SPI ) && ( defined( TARGET_STM32L4 ) || defined( TARGET_STM32F4 ) )

namespace Thor::LLD::SPI
{
  /*-------------------------------------------------------------------------------
  Variables
  -------------------------------------------------------------------------------*/
  static Driver s_spi_drivers[ NUM_SPI_PERIPHS ];

  /*-------------------------------------------------------------------------------
  Private Functions
  -------------------------------------------------------------------------------*/
  static float calculate_clock_performance( const size_t goal, const size_t actVal, void *const data )
  {
    /*------------------------------------------------
    Input checks:
      data    --  System's currently configured peripheral source clock
      actVal  --  Peripheral clock divider under consideration

    If either are null, permanently indicate worst case performance.
    ------------------------------------------------*/
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

  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    /*-------------------------------------------------
    Attach all the expected peripherals to the drivers
    -------------------------------------------------*/
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


  /*-------------------------------------------------------------------------------
  Low Level Driver Implementation
  -------------------------------------------------------------------------------*/
  Driver::Driver() :
      ISRWakeup_external( nullptr ), mPeriph( nullptr ), periphConfig( nullptr ),
      resourceIndex( std::numeric_limits<size_t>::max() )
  {
    memset( &txfr, 0, sizeof( txfr ) );
    txfr.status = Chimera::SPI::Status::TRANSFER_COMPLETE;
  }

  Driver::~Driver()
  {
  }


  Chimera::Status_t Driver::attach( RegisterMap *const peripheral )
  {
    /*------------------------------------------------
    Get peripheral descriptor settings
    ------------------------------------------------*/
    mPeriph       = peripheral;
    resourceIndex = getResourceIndex( reinterpret_cast<std::uintptr_t>( peripheral ) );

    /*------------------------------------------------
    Handle the ISR configuration
    ------------------------------------------------*/
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


  size_t Driver::getErrorFlags()
  {
    return 0;
  }


  size_t Driver::getStatusFlags()
  {
    return 0;
  }


  Chimera::Status_t Driver::configure( const Chimera::SPI::DriverConfig &setup )
  {
    using namespace Chimera::Algorithm::RegisterOptimization;

    /*------------------------------------------------
    Configure the clocks
    ------------------------------------------------*/
    clockEnable();

    /*------------------------------------------------
    Find the best clock divisor need to achieve the desired clock frequency
    ------------------------------------------------*/
    auto systemClock = Thor::LLD::RCC::getCoreClockCtrl();
    auto periphAddr  = reinterpret_cast<std::uintptr_t>( mPeriph );
    auto clockFreq   = systemClock->getPeriphClock( Chimera::Peripheral::Type::PERIPH_SPI, periphAddr );

    if ( clockFreq == std::numeric_limits<size_t>::max() )
    {
      return Chimera::Status::FAIL;
    }

    RegOptimizerData cfg;
    cfg.actVals      = Configuration::ClockDivisor::valOptions.data();
    cfg.regVals      = Configuration::ClockDivisor::regOptions.data();
    cfg.numOptions   = Configuration::ClockDivisor::NUM_OPTIONS;
    cfg.desiredValue = setup.HWInit.clockFreq;
    cfg.optimizer    = calculate_clock_performance;

    Reg32_t clockDivisor = findOptimalSetting( cfg, Configuration::ClockDivisor::DIV_32, &clockFreq );

    /*------------------------------------------------
    Stop the config if there are any invalid config options
    ------------------------------------------------*/
    if ( setup.HWInit.csMode == Chimera::SPI::CSMode::AUTO_BETWEEN_TRANSFER )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*------------------------------------------------
    Follow the configuration sequence from RM0390
    ------------------------------------------------*/
    /* Clear any faults */
    if ( MODF::get( mPeriph ) )
    {
      volatile Reg32_t dummyRead = SR_ALL::get( mPeriph );
    }

    /* Destroy all previous settings */
    this->reset();

    /* Bit Transfer Order */
    LSBFIRST::set( mPeriph, ConfigMap::BitOrderToRegConfig[ static_cast<size_t>( setup.HWInit.bitOrder ) ] );

    /* Transfer Bit Width */
    DS::set( mPeriph, ConfigMap::DataSizeToRegConfig[ static_cast<size_t>( setup.HWInit.dataSize ) ] );

    /* Peripheral Clock Divisor */
    BR::set( mPeriph, clockDivisor );

    /* Master/Slave Control Mode */
    MSTR::set( mPeriph, ConfigMap::ControlModeToRegConfig[ static_cast<size_t>( setup.HWInit.controlMode ) ] );

    SSM::set( mPeriph, CR1_SSM );
    SSI::set( mPeriph, CR1_SSI );

    /* Clock Phase and Polarity */
    switch ( setup.HWInit.clockMode )
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

#if defined( TARGET_STM32L4 )
    /*-------------------------------------------------
    Adjust the FIFO RX threshold based on sizing. If
    SZ == 8Bit, allow RXNE event on 1/4 FIFO lvl, else
    allow RXNE event on 1/2 FIFO lvl. FIFO has 4 lvls.

    See RM 40.4.9
    -------------------------------------------------*/
    if ( periphConfig->HWInit.dataSize == Chimera::SPI::DataSize::SZ_8BIT )
    {
      /* FIFO must hit 1/4 level before RXNE event */
      FRXTH::set( mPeriph, CR2_FRXTH );
    }
    else
    {
      /* FIFO must hit 1/2 level before RXNE event */
      FRXTH::clear( mPeriph, CR2_FRXTH );
    }
#endif /* TARGET_STM32L4 */

    /*-------------------------------------------------
    Enable the SPI peripheral
    -------------------------------------------------*/
    SPE::set( mPeriph, CR1_SPE );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::registerConfig( Chimera::SPI::DriverConfig *config )
  {
    if ( !config )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    this->periphConfig = config;
    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::transfer( const void *const txBuffer, void *const rxBuffer, const size_t bufferSize )
  {
    uint16_t txData                  = 0u;
    uint16_t rxData                  = 0u;
    size_t bytesTransfered           = 0u;
    size_t bytesPerTransfer          = 0u;
    const uint8_t *const txBufferPtr = reinterpret_cast<const uint8_t *const>( txBuffer );
    uint8_t *const rxBufferPtr       = reinterpret_cast<uint8_t *const>( rxBuffer );

    /*------------------------------------------------
    Runtime configuration
    ------------------------------------------------*/
    if ( !periphConfig )
    {
      return Chimera::Status::NOT_INITIALIZED;
    }

    switch ( periphConfig->HWInit.dataSize )
    {
      case Chimera::SPI::DataSize::SZ_8BIT:
        bytesPerTransfer = 1u;
        break;

      case Chimera::SPI::DataSize::SZ_9BIT:
      case Chimera::SPI::DataSize::SZ_10BIT:
      case Chimera::SPI::DataSize::SZ_11BIT:
      case Chimera::SPI::DataSize::SZ_12BIT:
      case Chimera::SPI::DataSize::SZ_13BIT:
      case Chimera::SPI::DataSize::SZ_14BIT:
      case Chimera::SPI::DataSize::SZ_15BIT:
      case Chimera::SPI::DataSize::SZ_16BIT:
        bytesPerTransfer = 2u;
        break;

      default:
        return Chimera::Status::NOT_SUPPORTED;
        break;
    }

    /*------------------------------------------------
    Do any flags indicate an ongoing transfer or error?
    ------------------------------------------------*/

    /*------------------------------------------------
    Disable all interrupts
    ------------------------------------------------*/
    Thor::LLD::INT::disableIRQ( Resource::IRQSignals[ resourceIndex ] );
    CR2_ALL::clear( mPeriph, ( CR2_TXEIE | CR2_RXNEIE | CR2_ERRIE ) );

    /*------------------------------------------------
    Write the data in a blocking fashion
    ------------------------------------------------*/
    while ( bytesTransfered < bufferSize )
    {
      /*------------------------------------------------
      Wait for the hw transmit buffer to be empty, then
      assign the next set of data to be transfered
      ------------------------------------------------*/
#if defined( EMBEDDED )
      while ( !TXE::get( mPeriph ) && BSY::get( mPeriph ) )
      {
        continue;
      }
#endif

      txData = 0u;
      if ( txBufferPtr )
      {
        if ( ( bytesTransfered + 1u ) == bufferSize )
        {
          memcpy( &txData, txBufferPtr + bytesTransfered, 1u );
        }
        else
        {
          memcpy( &txData, txBufferPtr + bytesTransfered, bytesPerTransfer );
        }
      }

      /*------------------------------------------------
      The STM32L4 SPI FIFOs implement data packing, so even though the
      bit configuration above is set, the DR has to be accessed as in
      either 8 bit or 16 bit modes. Otherwise extra data can be TX'd.
      ------------------------------------------------*/
      if ( bytesPerTransfer == 1 )
      {
        auto *dr = reinterpret_cast<volatile uint8_t *>( &mPeriph->DR );
        *dr      = txData;
      }
      else
      {
        auto *dr = reinterpret_cast<volatile uint16_t *>( &mPeriph->DR );
        *dr      = txData;
      }

      /*------------------------------------------------
      Wait for the hw receive buffer to indicate data has
      arrived, then read it out.
      ------------------------------------------------*/
      if ( rxBufferPtr )
      {
#if defined( EMBEDDED )
        while ( !RXNE::get( mPeriph ) )
        {
          /*-------------------------------------------------
          You can get stuck here in the debugger if single
          stepping and register auto-refresh is occurring.
          That will read the DR and clear RXNE.
          -------------------------------------------------*/
          continue;
        }
#endif
        rxData = mPeriph->DR;

        if ( ( bytesTransfered + 1u ) == bufferSize )
        {
          memcpy( rxBufferPtr + bytesTransfered, &rxData, 1u );
        }
        else
        {
          memcpy( rxBufferPtr + bytesTransfered, &rxData, bytesPerTransfer );
        }
      }

      /*------------------------------------------------
      Update the loop counters
      ------------------------------------------------*/
      bytesTransfered += bytesPerTransfer;
    }

    /*------------------------------------------------
    Don't exit this blocking function before all the
    TX FIFO transfers are finished.
    ------------------------------------------------*/
#if defined( EMBEDDED )
    while ( BSY::get( mPeriph ) )
      continue;
#endif

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::transferIT( const void *const txBuffer, void *const rxBuffer, const size_t bufferSize )
  {
    /*------------------------------------------------
    Make sure the transfer can begin
    ------------------------------------------------*/
    if ( BSY::get( mPeriph ) || ( txfr.status != Chimera::SPI::Status::TRANSFER_COMPLETE ) )
    {
      return Chimera::Status::BUSY;
    }

    /*------------------------------------------------
    Configure the interrupts
    ------------------------------------------------*/
    Thor::LLD::INT::enableIRQ( Resource::IRQSignals[ resourceIndex ] );
    enterCriticalSection();
    CR2_ALL::set( mPeriph, ( CR2_TXEIE | CR2_RXNEIE | CR2_ERRIE ) );

    /*------------------------------------------------
    Queue up the transfer
    ------------------------------------------------*/
    txfr.txBuffer        = reinterpret_cast<uint8_t *>( const_cast<void *>( txBuffer ) );
    txfr.txTransferCount = 0u;
    txfr.txTransferSize  = bufferSize;

    txfr.rxBuffer        = reinterpret_cast<uint8_t *>( const_cast<void *>( rxBuffer ) );
    txfr.rxTransferCount = 0u;
    txfr.rxTransferSize  = bufferSize;

    txfr.status = Chimera::SPI::Status::TRANSFER_IN_PROGRESS;

    /*------------------------------------------------
    Data transfers must have 8-bit or 16-bit aligned access.
    Currently hardcoded to 8 for development...
    ------------------------------------------------*/
    // Access the data register as 8-bit aligned
    auto dr = reinterpret_cast<volatile uint8_t *>( &mPeriph->DR );

#if defined( TARGET_STM32L4 )
    // Set the RX FIFO threshold to generate an RXNE event when 8-bits are received
    FRXTH::set( mPeriph, Configuration::FIFOThreshold::RXNE_ON_8BIT );
#endif /* TARGET_STM32L4 */

    /*------------------------------------------------
    Start the transfer
    ------------------------------------------------*/
    *dr              = txfr.txBuffer[ txfr.txTransferCount ];
    txfr.waitingOnTX = false;
    txfr.waitingOnRX = true;
    txfr.txTransferCount++;
    exitCriticalSection();

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::transferDMA( const void *const txBuffer, void *const rxBuffer, const size_t bufferSize )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::killTransfer()
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  void Driver::attachISRWakeup( Chimera::Threading::BinarySemaphore *const wakeup )
  {
    ISRWakeup_external = wakeup;
  }


  HWTransfer Driver::getTransferBlock()
  {
    HWTransfer copy;
    memset( &copy, 0, sizeof( HWTransfer ) );

    enterCriticalSection();
    memcpy( &copy, &txfr, sizeof( HWTransfer ) );
    exitCriticalSection();

    return copy;
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
    /*------------------------------------------------
    Save critical register information
    ------------------------------------------------*/
    const volatile Reg32_t SR  = SR_ALL::get( mPeriph );
    const volatile Reg32_t CR2 = CR2_ALL::get( mPeriph );

    /*------------------------------------------------
    Data transfers must have 8-bit or 16-bit aligned access.
    Currently hardcoded to 8 for development...
    ------------------------------------------------*/
    auto dr = reinterpret_cast<volatile uint8_t *>( &mPeriph->DR );

    /*------------------------------------------------
    HW TX Buffer Empty (Next frame can be buffered)
    ------------------------------------------------*/
    if ( txfr.waitingOnTX && ( CR2 & CR2_TXEIE ) && ( SR & SR_TXE ) )
    {
      txfr.waitingOnTX = false;
      txfr.waitingOnRX = true;

      if ( txfr.txTransferCount < txfr.txTransferSize )
      {
        /*------------------------------------------------
        Still more data to TX...
        ------------------------------------------------*/
        *dr = txfr.txBuffer[ txfr.txTransferCount ];
        txfr.txTransferCount++;
      }
      else
      {
        /*------------------------------------------------
        The TX half of the transfer is complete. Disable the
        TX FIFO empty ISR signal.
        ------------------------------------------------*/
        TXEIE::set( mPeriph, ~CR2_TXEIE );

        /*------------------------------------------------
        The user could have also requested a TX only transfer
        and not cared about received data. In this case, the
        entire transfer is now complete.
        ------------------------------------------------*/
        if ( !txfr.rxBuffer )
        {
          RXNEIE::set( mPeriph, ~CR2_RXNEIE );
          txfr.status = Chimera::SPI::Status::TRANSFER_COMPLETE;
        }
      }
    }

    /*------------------------------------------------
    HW RX Buffer Full (Data is ready to be copied out)
    ------------------------------------------------*/
    if ( txfr.waitingOnRX && ( CR2 & CR2_RXNEIE ) && ( SR & SR_RXNE ) )
    {
      txfr.waitingOnTX = true;
      txfr.waitingOnRX = false;

      if ( txfr.rxTransferCount < txfr.rxTransferSize )
      {
        txfr.rxBuffer[ txfr.rxTransferCount ] = *dr;
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
      /*------------------------------------------------
      The user specified a transmit only. Ignore the RX
      half and jump back to the TX half.
      ------------------------------------------------*/
      txfr.waitingOnTX = true;
      txfr.waitingOnRX = false;
    }

    /*------------------------------------------------
    Handle Any Errors
    ------------------------------------------------*/
    if ( ( CR2 & CR2_ERRIE ) && ( SR & ( SR_CRCERR | SR_FRE | SR_MODF | SR_OVR ) ) )
    {
      txfr.status = Chimera::SPI::Status::TRANSFER_ERROR;
    }

    /*------------------------------------------------
    Detect the end of transfer and wake up the post processor thread
    ------------------------------------------------*/
    if ( ISRWakeup_external && ( ( txfr.status == Chimera::SPI::Status::TRANSFER_COMPLETE ) ||
                                 ( txfr.status == Chimera::SPI::Status::TRANSFER_ERROR ) ) )
    {
      ISRWakeup_external->releaseFromISR();
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
