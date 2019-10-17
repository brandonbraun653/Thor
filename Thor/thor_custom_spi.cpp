/********************************************************************************
 *   File Name:
 *    thor_custom_spi.cpp
 *
 *   Description:
 *    SPI driver for Thor
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <array>

/* Chimera Includes */
#include <Chimera/constants/common.hpp>
#include <Chimera/threading.hpp>
#include <Chimera/interface/spi_intf.hpp>

/* Thor Includes */
#include <Thor/drivers/common/types/spi_types.hpp>
#include <Thor/drivers/spi.hpp>
#include <Thor/spi.hpp>




static std::array<Thor::SPI::SPIClass *, Thor::Driver::SPI::NUM_SPI_PERIPHS> spiobjects;
static std::array<TaskHandle_t, Thor::Driver::SPI::NUM_SPI_PERIPHS> postProcessorHandles;
static std::array<SemaphoreHandle_t, Thor::Driver::SPI::NUM_SPI_PERIPHS> postProcessorSignals;


static void SPI1ISRPostProcessorThread( void *argument );
static void SPI2ISRPostProcessorThread( void *argument );
static void SPI3ISRPostProcessorThread( void *argument );
static void SPI4ISRPostProcessorThread( void *argument );


namespace Chimera::SPI
{
  static size_t s_driver_initialized;


  Chimera::Status_t initialize()
  {
    s_driver_initialized = ~Chimera::DRIVER_INITIALIZED_KEY;

    /*------------------------------------------------
    Reset driver object memory
    ------------------------------------------------*/
    for ( size_t x = 0; x < spiobjects.size(); x++ )
    {
      if ( spiobjects[ x ] ) 
      {
        spiobjects[ x ]->deInit();
        vPortFree( spiobjects[ x ] );
      }

      spiobjects[ x ] = nullptr;
    }

    /*------------------------------------------------
    Reset thread handle memory
    ------------------------------------------------*/
    for ( size_t x = 0; x < postProcessorHandles.size(); x++ )
    {
      if ( postProcessorHandles[ x ] )
      {
        vTaskDelete( postProcessorHandles[ x ] );
        vPortFree( postProcessorHandles[ x ] );
      }

      postProcessorHandles[ x ] = nullptr;
    }

    /*------------------------------------------------
    Reset semaphore signal memory
    ------------------------------------------------*/
    for ( size_t x = 0; x < postProcessorSignals.size(); x++ )
    {
      if ( postProcessorSignals[ x ] )
      {
        vPortFree( postProcessorSignals[ x ] );
      }

      postProcessorSignals[ x ] = nullptr;
    }

    s_driver_initialized = Chimera::DRIVER_INITIALIZED_KEY;
    return Chimera::CommonStatusCodes::OK;
  }
}


namespace Thor::SPI
{
  SPIClass::SPIClass()
  {
    /*------------------------------------------------
    Lazy initialize all driver memory in case the user forgot
    ------------------------------------------------*/
    if ( Chimera::SPI::s_driver_initialized != Chimera::DRIVER_INITIALIZED_KEY )
    {
      Chimera::SPI::initialize();
    }
  }

  SPIClass::~SPIClass()
  {

  }

  /*------------------------------------------------
  HW Interface
  ------------------------------------------------*/
  Chimera::Status_t SPIClass::init( const Chimera::SPI::DriverConfig &setupStruct )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::SPI::DriverConfig SPIClass::getInit()
  {
    return Chimera::SPI::DriverConfig();
  }

  Chimera::Status_t SPIClass::deInit()
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t SPIClass::setChipSelect( const Chimera::GPIO::State value )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t SPIClass::setChipSelectControlMode( const Chimera::SPI::CSMode mode )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t SPIClass::writeBytes( const void *const txBuffer, const size_t length, const size_t timeoutMS )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t SPIClass::readBytes( void *const rxBuffer, const size_t length, const size_t timeoutMS )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t SPIClass::readWriteBytes( const void *const txBuffer, void *const rxBuffer, const size_t length,
                                              const size_t timeoutMS )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t SPIClass::setPeripheralMode( const Chimera::Hardware::PeripheralMode mode )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t SPIClass::setClockFrequency( const size_t freq, const size_t tolerance )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  size_t SPIClass::getClockFrequency()
  {
    return 0;
  }

  /*------------------------------------------------
  Async IO Interface
  ------------------------------------------------*/
  Chimera::Status_t SPIClass::await( const Chimera::Event::Trigger event, const size_t timeout )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t SPIClass::await( const Chimera::Event::Trigger event, SemaphoreHandle_t notifier, const size_t timeout )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  /*------------------------------------------------
  Listener Interface
  ------------------------------------------------*/
  Chimera::Status_t SPIClass::registerListener( Chimera::Event::Actionable &listener, const size_t timeout,
                                                size_t &registrationID )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t SPIClass::removeListener( const size_t registrationID, const size_t timeout )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }
}
