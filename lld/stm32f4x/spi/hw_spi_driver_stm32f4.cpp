/********************************************************************************
 *  File Name:
 *    hw_spi_driver_stm32f4.cpp
 *
 *  Description:
 *    STM32F4 specific driver implementation for SPI
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <array>
#include <limits>

/* Chimera Includes */
#include <Chimera/algorithm>
#include <Chimera/common>
#include <Chimera/thread>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/hld/interrupt/hld_interrupt_definitions.hpp>
#include <Thor/lld/stm32f4x/nvic/hw_nvic_driver.hpp>
#include <Thor/lld/stm32f4x/rcc/hw_rcc_driver.hpp>
#include <Thor/lld/stm32f4x/spi/hw_spi_driver.hpp>
#include <Thor/lld/stm32f4x/spi/hw_spi_mapping.hpp>
#include <Thor/lld/stm32f4x/spi/hw_spi_prj.hpp>
#include <Thor/lld/stm32f4x/spi/hw_spi_types.hpp>

#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_SPI )

namespace Thor::LLD::SPI
{
  static float calculate_clock_performance( const size_t goal, const size_t actVal, void *const data )
  {
    /*------------------------------------------------
    Input checks:
      data    --  System's currently configured peripheral source clock
      actVal  --  Peripheral clock divider under consideration

    If either are null, permanently indicate worst case performance.
    ------------------------------------------------*/
    if ( !data || !actVal)
    {
      return std::numeric_limits<float>::max();
    }

    size_t spi_periph_clock_freq = *( reinterpret_cast<size_t *const>( data ) );
    size_t spi_output_clock_freq = spi_periph_clock_freq / actVal;

    float fGoal = static_cast<float>( goal );
    float fAchieved = static_cast<float>( spi_output_clock_freq );

    return fAchieved - fGoal;
  }

  void initialize()
  {
    initializeMapping();

  }

  bool isChannelSupported( const size_t channel )
  {
    for ( auto &num : supportedChannels )
    {
      if ( static_cast<size_t>( num ) == channel )
      {
        return true;
      }
    }

    return false;
  }

  Driver::Driver() : ISRWakeup_external( nullptr )
  {
    /*------------------------------------------------
    Initialize class variables
    ------------------------------------------------*/
    periphConfig  = nullptr;
    periph        = nullptr;
    resourceIndex = std::numeric_limits<decltype( resourceIndex )>::max();

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
    resourceIndex = InstanceToResourceIndex.find( reinterpret_cast<std::uintptr_t>( periph ) )->second;
    periphIRQn    = IRQSignals[ resourceIndex ];
    dmaTXSignal   = TXDMASignals[ resourceIndex ];
    dmaRXSignal   = RXDMASignals[ resourceIndex ];

    /*------------------------------------------------
    Handle the ISR configuration
    ------------------------------------------------*/
    Thor::LLD::INT::disableIRQ( periphIRQn );
    Thor::LLD::INT::clearPendingIRQ( periphIRQn );
    Thor::LLD::INT::setPriority( periphIRQn, Thor::Interrupt::SPI_IT_PREEMPT_PRIORITY, 0u );

    /*------------------------------------------------
    Driver registration with the backend
    ------------------------------------------------*/
    Thor::LLD::SPI::spiObjects[ resourceIndex ] = this;

    return Chimera::Status::OK;
  }

  Chimera::Status_t Driver::reset()
  {
    return Chimera::Status::FAIL;
  }

  void Driver::clockEnable()
  {
    auto rcc = Thor::LLD::RCC::getPeripheralClock();
    rcc->enableClock( Chimera::Peripheral::Type::PERIPH_SPI, resourceIndex );
  }

  void Driver::clockDisable()
  {
    auto rcc = Thor::LLD::RCC::getPeripheralClock();
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
    size_t clockFreq  = std::numeric_limits<size_t>::max();
    auto systemClock  = Thor::LLD::RCC::getCoreClock();
    auto periphAddr   = reinterpret_cast<std::uintptr_t>( periph );
    auto updateStatus = systemClock->getPeriphClock( Chimera::Peripheral::Type::PERIPH_SPI, periphAddr, &clockFreq );

    if ( updateStatus != Chimera::Status::OK )
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
    if ( SR::MODF::get( periph ) )
    {
      volatile Reg32_t dummyRead = SR::get( periph );
    }

    /* Destroy all previous settings */
    CR1::set( periph, CR1::resetValue );
    CR2::set( periph, CR1::resetValue );
    CRCPR::set( periph, CRCPR::resetValue );
    RXCRCR::set( periph, RXCRCR::resetValue );
    TXCRCR::set( periph, TXCRCR::resetValue );

    /* Bit Transfer Order */
    CR1::LSBFIRST::set( periph, BitOrderToRegConfig[ static_cast<size_t>( setup.HWInit.bitOrder ) ] );

    /* Transfer Bit Width */
    CR1::DFF::set( periph, DataSizeToRegConfig[ static_cast<size_t>( setup.HWInit.dataSize ) ] );

    /* Peripheral Clock Divisor */
    CR1::BR::set( periph, clockDivisor );

    /* Master/Slave Control Mode */
    CR1::MSTR::set( periph, ControlModeToRegConfig[ static_cast<size_t>( setup.HWInit.controlMode ) ] );

    CR1::SSM::set( periph, CR1_SSM );
    CR1::SSI::set( periph, CR1_SSI );

    /* Clock Phase and Polarity */
    switch ( setup.HWInit.clockMode )
    {
      case Chimera::SPI::ClockMode::MODE0:
        CR1::CPOL::set( periph, 0u );
        CR1::CPHA::set( periph, 0u );
        break;

      case Chimera::SPI::ClockMode::MODE1:
        CR1::CPOL::set( periph, 0u );
        CR1::CPHA::set( periph, CR1_CPHA );
        break;

      case Chimera::SPI::ClockMode::MODE2:
        CR1::CPOL::set( periph, CR1_CPOL );
        CR1::CPHA::set( periph, 0u );
        break;

      case Chimera::SPI::ClockMode::MODE3:
        CR1::CPOL::set( periph, CR1_CPOL );
        CR1::CPHA::set( periph, CR1_CPHA );
        break;

      default:
        break;
    }

    /* Configure CR2 */
    CR2::SSOE::set( periph, CR2_SSOE );

    /* Finally enable the peripheral */
    CR1::SPE::set( periph, CR1_SPE );

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
    Thor::LLD::INT::disableIRQ( periphIRQn );
    CR2::clear( periph, ( CR2_TXEIE | CR2_RXNEIE | CR2_ERRIE ) );

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
      while ( !SR::TXE::get( periph ) && SR::BSY::get( periph ) )
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

      periph->DR = txData;

      /*------------------------------------------------
      Wait for the hw receive buffer to indicate data has
      arrived, then read it out.
      ------------------------------------------------*/
#if defined( EMBEDDED )
      while ( !SR::RXNE::get( periph ) )
        continue;
#endif

      rxData = periph->DR;

      if ( rxBufferPtr )
      {
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

    return Chimera::Status::OK;
  }

  Chimera::Status_t Driver::transferIT( const void *const txBuffer, void *const rxBuffer, const size_t bufferSize )
  {
    /*------------------------------------------------
    Make sure the transfer can begin
    ------------------------------------------------*/
    if ( SR::BSY::get( periph ) || ( txfr.status != Chimera::SPI::Status::TRANSFER_COMPLETE ) )
    {
      return Chimera::Status::BUSY;
    }

    /*------------------------------------------------
    Configure the interrupts
    ------------------------------------------------*/
    Thor::LLD::INT::enableIRQ( periphIRQn );
    enterCriticalSection();
    CR2::set( periph, ( CR2_TXEIE | CR2_RXNEIE | CR2_ERRIE ) );

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
    Start the transfer
    ------------------------------------------------*/
    periph->DR = txfr.txBuffer[ txfr.txTransferCount ];
    txfr.waitingOnTX = false;
    txfr.waitingOnRX = true;
    txfr.txTransferCount++;
    exitCriticalSection();

    return Chimera::Status::OK;
  }

  Chimera::Status_t Driver::transferDMA( const void *const txBuffer, void *const rxBuffer, const size_t bufferSize )
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t Driver::killTransfer()
  {
    return Chimera::Status_t();
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
    Thor::LLD::INT::disableIRQ( periphIRQn );
  }

  inline void Driver::exitCriticalSection()
  {
    Thor::LLD::INT::enableIRQ( periphIRQn );
  }

  void Driver::IRQHandler()
  {
    /*------------------------------------------------
    Save critical register information
    ------------------------------------------------*/
    const volatile Reg32_t SR = SR::get( periph );
    const volatile Reg32_t CR2 = CR2::get( periph );

    /*------------------------------------------------
    HW TX Buffer Empty (Next frame can be buffered)
    ------------------------------------------------*/
    if ( txfr.waitingOnTX && ( CR2 & CR2_TXEIE ) && ( SR & SR_TXE ) )
    {
      txfr.waitingOnTX = false;
      txfr.waitingOnRX = true;

      if ( txfr.txTransferCount < txfr.txTransferSize )
      {
        periph->DR = txfr.txBuffer[ txfr.txTransferCount ];
        txfr.txTransferCount++;
      }
      else
      {
        CR2::TXEIE::set( periph, ~CR2_TXEIE );
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
        txfr.rxBuffer[ txfr.rxTransferCount ] = periph->DR;
        txfr.rxTransferCount++;
      }

      if ( txfr.rxTransferCount == txfr.rxTransferSize )
      {
        CR2::RXNEIE::set( periph, ~CR2_RXNEIE );
        txfr.status = Chimera::SPI::Status::TRANSFER_COMPLETE;
      }
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

#if defined( STM32_SPI4_PERIPH_AVAILABLE )
void SPI4_IRQHandler()
{
  static constexpr size_t index = 3;

  if ( Thor::LLD::SPI::spiObjects[ index ] )
  {
    Thor::LLD::SPI::spiObjects[ index ]->IRQHandler();
  }
}
#endif

#if defined( STM32_SPI5_PERIPH_AVAILABLE )
void SPI5_IRQHandler()
{
  static constexpr size_t index = 4;

  if ( Thor::LLD::SPI::spiObjects[ index ] )
  {
    Thor::LLD::SPI::spiObjects[ index ]->IRQHandler();
  }
}
#endif

#if defined( STM32_SPI6_PERIPH_AVAILABLE )
void SPI6_IRQHandler()
{
  static constexpr size_t index = 5;

  if ( Thor::LLD::SPI::spiObjects[ index ] )
  {
    Thor::LLD::SPI::spiObjects[ index ]->IRQHandler();
  }
}
#endif

#endif /* TARGET_STM32F4 && THOR_DRIVER_SPI */