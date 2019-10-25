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
    periph = nullptr;
    resourceIndex = std::numeric_limits<decltype( resourceIndex )>::max();
  }

  Driver::~Driver()
  {
  }

  Chimera::Status_t Driver::attach( RegisterMap *const peripheral )
  {
    periph = peripheral;
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

    /*------------------------------------------------
    Follow the configuration sequence from RM0390
    ------------------------------------------------*/

    // TODO: need to create the configuration mappings and options

    /* Configure CR1 */



    /* Configure CR2 */




    return Chimera::Status_t();
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