/********************************************************************************
 *  File Name:
 *    hw_spi_driver_STM32L4.cpp
 *
 *  Description:
 *    Implements the LLD interface to the STM32L4 series SPI hardware.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* STL Includes */
#include <cstring>
#include <limits>

/* Chimera Includes */
#include <Chimera/algorithm>
#include <Chimera/common>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/hld/interrupt/hld_interrupt_definitions.hpp>
#include <Thor/lld/common/cortex-m4/interrupts.hpp>
#include <Thor/lld/stm32l4x/spi/hw_spi_driver.hpp>
#include <Thor/lld/stm32l4x/spi/hw_spi_mapping.hpp>
#include <Thor/lld/stm32l4x/spi/hw_spi_prj.hpp>
#include <Thor/lld/stm32l4x/spi/hw_spi_types.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_driver.hpp>

#if defined( TARGET_STM32L4 ) && defined( THOR_LLD_SPI )

namespace Thor::LLD::SPI
{
  static std::array<IDriver_sPtr, NUM_SPI_PERIPHS> s_spi_drivers;

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

  /*-------------------------------------------------
  LLD->HLD Interface Implementation
  -------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    initializeRegisters();
    initializeMapping();

    return Chimera::CommonStatusCodes::OK;
  }

  bool isChannelSupported( const Chimera::SPI::Channel channel )
  {
    for ( auto &num : supportedChannels )
    {
      if ( ( num == channel ) && ( num != Chimera::SPI::Channel::NOT_SUPPORTED ) )
      {
        return true;
      }
    }

    return false;
  }

  IDriver_sPtr getDriver( const Chimera::SPI::Channel channel )
  {
    size_t ch = static_cast<size_t>( channel );

    if ( !( ch < s_spi_drivers.size() ) || !isChannelSupported( channel ) )
    {
      return nullptr;
    }
    else if ( !s_spi_drivers[ ch ] )
    {
      s_spi_drivers[ ch ] = std::make_shared<Driver>();
      s_spi_drivers[ ch ]->attach( PeripheralList[ ch ] );
    }

    return s_spi_drivers[ ch ];
  }

  size_t availableChannels()
  {
    return NUM_SPI_PERIPHS;
  }

  /*-------------------------------------------------
  Private LLD Function Implementation
  -------------------------------------------------*/
  bool isSPI( const std::uintptr_t address )
  {
    bool result = false;

    for ( auto &val : periphAddressList )
    {
      if ( val == address )
      {
        result = true;
      }
    }

    return result;
  }

  /*-----------------------------------------------------
  Low Level Driver Implementation
  -----------------------------------------------------*/
  Driver::Driver() :
      ISRWakeup_external( nullptr ), periph( nullptr ), periphConfig( nullptr ),
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
    periph        = peripheral;
    resourceIndex = InstanceToResourceIndex.at( reinterpret_cast<std::uintptr_t>( periph ) ).second;
    periphIRQn    = IRQSignals[ resourceIndex ];
    dmaTXSignal   = TXDMASignals[ resourceIndex ];
    dmaRXSignal   = RXDMASignals[ resourceIndex ];

    /*------------------------------------------------
    Handle the ISR configuration
    ------------------------------------------------*/
    Thor::LLD::IT::disableIRQ( periphIRQn );
    Thor::LLD::IT::clearPendingIRQ( periphIRQn );
    Thor::LLD::IT::setPriority( periphIRQn, Thor::Interrupt::SPI_IT_PREEMPT_PRIORITY, 0u );

    /*------------------------------------------------
    Driver registration with the backend
    ------------------------------------------------*/
    Thor::LLD::SPI::spiObjects[ resourceIndex ] = this;

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::reset()
  {
    return Chimera::CommonStatusCodes::FAIL;
  }

  void Driver::clockEnable()
  {
    auto rcc = Thor::LLD::RCC::getSystemPeripheralController();
    rcc->enableClock( Chimera::Peripheral::Type::PERIPH_SPI, resourceIndex );
  }

  void Driver::clockDisable()
  {
    auto rcc = Thor::LLD::RCC::getSystemPeripheralController();
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
    auto systemClock = Thor::LLD::RCC::getSystemClockController();
    auto periphAddr  = reinterpret_cast<std::uintptr_t>( periph );
    auto clockFreq   = systemClock->getPeriphClock( Chimera::Peripheral::Type::PERIPH_SPI, periphAddr );

    if ( clockFreq == std::numeric_limits<size_t>::max() )
    {
      return Chimera::CommonStatusCodes::FAIL;
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
      return Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
    }

    /*------------------------------------------------
    Follow the configuration sequence from RM0390
    ------------------------------------------------*/
    /* Clear any faults */
    if ( MODF::get( periph ) )
    {
      volatile Reg32_t dummyRead = SR_ALL::get( periph );
    }

    /* Destroy all previous settings */
    CR1_ALL::set( periph, CR1_Rst );
    CR2_ALL::set( periph, CR2_Rst );
    CRCPOLY::set( periph, CRCPR_Rst );
    RXCRC::set( periph, RXCRCR_Rst );
    TXCRC::set( periph, TXCRCR_Rst );

    /* Bit Transfer Order */
    LSBFIRST::set( periph, BitOrderToRegConfig[ static_cast<size_t>( setup.HWInit.bitOrder ) ] );

    /* Transfer Bit Width */
    DS::set( periph, DataSizeToRegConfig[ static_cast<size_t>( setup.HWInit.dataSize ) ] );

    /* Peripheral Clock Divisor */
    BR::set( periph, clockDivisor );

    /* Master/Slave Control Mode */
    MSTR::set( periph, ControlModeToRegConfig[ static_cast<size_t>( setup.HWInit.controlMode ) ] );

    SSM::set( periph, CR1_SSM );
    SSI::set( periph, CR1_SSI );

    /* Clock Phase and Polarity */
    switch ( setup.HWInit.clockMode )
    {
      case Chimera::SPI::ClockMode::MODE0:
        CPOL::set( periph, 0u );
        CPHA::set( periph, 0u );
        break;

      case Chimera::SPI::ClockMode::MODE1:
        CPOL::set( periph, 0u );
        CPHA::set( periph, CR1_CPHA );
        break;

      case Chimera::SPI::ClockMode::MODE2:
        CPOL::set( periph, CR1_CPOL );
        CPHA::set( periph, 0u );
        break;

      case Chimera::SPI::ClockMode::MODE3:
        CPOL::set( periph, CR1_CPOL );
        CPHA::set( periph, CR1_CPHA );
        break;

      default:
        break;
    }

    /* Configure CR2 */
    SSOE::set( periph, CR2_SSOE );

    /* Finally enable the peripheral */
    SPE::set( periph, CR1_SPE );

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::registerConfig( Chimera::SPI::DriverConfig *config )
  {
    if ( !config )
    {
      return Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
    }

    this->periphConfig = config;
    return Chimera::CommonStatusCodes::OK;
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
      return Chimera::CommonStatusCodes::NOT_INITIALIZED;
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
        return Chimera::CommonStatusCodes::NOT_SUPPORTED;
        break;
    }

    /*------------------------------------------------
    Do any flags indicate an ongoing transfer or error?
    ------------------------------------------------*/

    /*------------------------------------------------
    Disable all interrupts
    ------------------------------------------------*/
    Thor::LLD::IT::disableIRQ( periphIRQn );
    CR2_ALL::clear( periph, ( CR2_TXEIE | CR2_RXNEIE | CR2_ERRIE ) );

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
      while ( !TXE::get( periph ) && BSY::get( periph ) )
        continue;
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
        auto *dr = reinterpret_cast<volatile uint8_t *>( &periph->DR );
        *dr      = txData;
      }
      else
      {
        auto *dr = reinterpret_cast<volatile uint16_t *>( &periph->DR );
        *dr      = txData;
      }

      /*------------------------------------------------
      Wait for the hw receive buffer to indicate data has
      arrived, then read it out.
      ------------------------------------------------*/
      if ( rxBufferPtr )
      {
#if defined( EMBEDDED )
        while ( !RXNE::get( periph ) )
          continue;
#endif
        rxData = periph->DR;

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
    while ( BSY::get( periph ) )
      continue;
#endif

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::transferIT( const void *const txBuffer, void *const rxBuffer, const size_t bufferSize )
  {
    /*------------------------------------------------
    Make sure the transfer can begin
    ------------------------------------------------*/
    if ( BSY::get( periph ) || ( txfr.status != Chimera::SPI::Status::TRANSFER_COMPLETE ) )
    {
      return Chimera::CommonStatusCodes::BUSY;
    }

    /*------------------------------------------------
    Configure the interrupts
    ------------------------------------------------*/
    Thor::LLD::IT::enableIRQ( periphIRQn );
    enterCriticalSection();
    CR2_ALL::set( periph, ( CR2_TXEIE | CR2_RXNEIE | CR2_ERRIE ) );

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
    auto dr = reinterpret_cast<volatile uint8_t *>( &periph->DR );
    
    // Set the RX FIFO threshold to generate an RXNE event when 8-bits are received
    FRXTH::set( periph, Configuration::FIFOThreshold::RXNE_ON_8BIT );

    /*------------------------------------------------
    Start the transfer
    ------------------------------------------------*/
    *dr              = txfr.txBuffer[ txfr.txTransferCount ];
    txfr.waitingOnTX = false;
    txfr.waitingOnRX = true;
    txfr.txTransferCount++;
    exitCriticalSection();

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::transferDMA( const void *const txBuffer, void *const rxBuffer, const size_t bufferSize )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::killTransfer()
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
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
    Thor::LLD::IT::disableIRQ( periphIRQn );
  }

  inline void Driver::exitCriticalSection()
  {
    Thor::LLD::IT::enableIRQ( periphIRQn );
  }

  void Driver::IRQHandler()
  {
    /*------------------------------------------------
    Save critical register information
    ------------------------------------------------*/
    const volatile Reg32_t SR  = SR_ALL::get( periph );
    const volatile Reg32_t CR2 = CR2_ALL::get( periph );

    /*------------------------------------------------
    Data transfers must have 8-bit or 16-bit aligned access.
    Currently hardcoded to 8 for development...
    ------------------------------------------------*/
    auto dr = reinterpret_cast<volatile uint8_t *>( &periph->DR );

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
        TXEIE::set( periph, ~CR2_TXEIE );

        /*------------------------------------------------
        The user could have also requested a TX only transfer
        and not cared about received data. In this case, the
        entire transfer is now complete.
        ------------------------------------------------*/
        if ( !txfr.rxBuffer )
        {
          RXNEIE::set( periph, ~CR2_RXNEIE );
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
        RXNEIE::set( periph, ~CR2_RXNEIE );
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
  static constexpr size_t index = 0;

  if ( Thor::LLD::SPI::spiObjects[ index ] )
  {
    Thor::LLD::SPI::spiObjects[ index ]->IRQHandler();
  }
}
#endif

#if defined( STM32_SPI2_PERIPH_AVAILABLE )
void SPI2_IRQHandler()
{
  static constexpr size_t index = 1;

  if ( Thor::LLD::SPI::spiObjects[ index ] )
  {
    Thor::LLD::SPI::spiObjects[ index ]->IRQHandler();
  }
}
#endif

#if defined( STM32_SPI3_PERIPH_AVAILABLE )
void SPI3_IRQHandler()
{
  static constexpr size_t index = 2;

  if ( Thor::LLD::SPI::spiObjects[ index ] )
  {
    Thor::LLD::SPI::spiObjects[ index ]->IRQHandler();
  }
}
#endif

#endif /* TARGET_STM32L4 && THOR_DRIVER_SPI */
