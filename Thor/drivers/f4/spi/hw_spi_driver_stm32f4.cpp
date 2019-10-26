/********************************************************************************
 *   File Name:
 *    hw_spi_driver_stm32f4.cpp
 *
 *   Description:
 *    STM32F4 specific driver implementation for SPI
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <array>
#include <limits>

/* Chimera Includes */
#include <Chimera/chimera.hpp>
#include <Chimera/threading.hpp>

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/dma.hpp>

#include <Thor/drivers/f4/rcc/hw_rcc_driver.hpp>
#include <Thor/drivers/f4/spi/hw_spi_driver.hpp>
#include <Thor/drivers/f4/spi/hw_spi_mapping.hpp>
#include <Thor/drivers/f4/spi/hw_spi_prj.hpp>
#include <Thor/drivers/f4/spi/hw_spi_types.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_SPI == 1 )

namespace Thor::Driver::SPI
{
  void initialize()
  {
    initializeMapping();
    initializeRegisters();
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

  Driver::Driver()
  {
    /*------------------------------------------------
    Initialize class variables
    ------------------------------------------------*/
    periph        = nullptr;
    resourceIndex = std::numeric_limits<decltype( resourceIndex )>::max();
  }

  Driver::~Driver()
  {
  }

  Chimera::Status_t Driver::attach( RegisterMap *const peripheral )
  {
    periph        = peripheral;
    resourceIndex = InstanceToResourceIndex.find( reinterpret_cast<std::uintptr_t>( periph ) )->second;
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::reset()
  {
    return Chimera::CommonStatusCodes::FAIL;
  }

  void Driver::clockEnable()
  {
  }

  void Driver::clockDisable()
  {
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
    /*------------------------------------------------
    Configure the clocks
    ------------------------------------------------*/
    auto rcc = Thor::Driver::RCC::PeripheralController::get();
    rcc->enableClock( Chimera::Peripheral::Type::PERIPH_SPI, resourceIndex );


    // TODO: Calculate the baud rate needed once the driver is working


    /*------------------------------------------------
    Follow the configuration sequence from RM0390
    ------------------------------------------------*/
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
    CR1::BR::set( periph, Configuration::ClockDivisor::DIV_32 );

    /* Master/Slave Control Mode */
    CR1::MSTR::set( periph, ControlModeToRegConfig[ static_cast<size_t>( setup.HWInit.controlMode ) ] );

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
    //TODO: Eventually setup DMA/Interrupts/CRC

    /* Finally enable the peripheral */
    CR1::SPE::set( periph, CR1_SPE );

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::transfer( const void *const txBuffer, void *const rxBuffer, const size_t bufferSize )
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t Driver::transferIT( const void *const txBuffer, void *const rxBuffer, const size_t bufferSize )
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t Driver::transferDMA( const void *const txBuffer, void *const rxBuffer, const size_t bufferSize )
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t Driver::killTransfer()
  {
    return Chimera::Status_t();
  }
}    // namespace Thor::Driver::SPI

#endif /* TARGET_STM32F4 && THOR_DRIVER_SPI */