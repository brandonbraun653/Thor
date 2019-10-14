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


/* Chimera Includes */


/* Thor Includes */
#include <Thor/spi.hpp>


namespace Thor::SPI
{
  SPIClass::SPIClass()
  {

  }

  SPIClass::~SPIClass()
  {

  }

  /*------------------------------------------------
  HW Interface
  ------------------------------------------------*/
  Chimera::Status_t SPIClass::init( const Chimera::SPI::DriverConfig &setupStruct )
  {
  }

  Chimera::SPI::DriverConfig SPIClass::getInit()
  {
  }

  Chimera::Status_t SPIClass::deInit()
  {
  }

  Chimera::Status_t SPIClass::setChipSelect( const Chimera::GPIO::State value )
  {
  }

  Chimera::Status_t SPIClass::setChipSelectControlMode( const Chimera::SPI::CSMode mode )
  {
  }

  Chimera::Status_t SPIClass::writeBytes( const void *const txBuffer, const size_t length, const size_t timeoutMS )
  {
  }

  Chimera::Status_t SPIClass::readBytes( void *const rxBuffer, const size_t length, const size_t timeoutMS )
  {
  }

  Chimera::Status_t SPIClass::readWriteBytes( const void *const txBuffer, void *const rxBuffer, const size_t length,
                                              const size_t timeoutMS )
  {
  }

  Chimera::Status_t SPIClass::setPeripheralMode( const Chimera::Hardware::PeripheralMode mode )
  {
  }

  Chimera::Status_t SPIClass::setClockFrequency( const size_t freq, const size_t tolerance )
  {
  }

  size_t SPIClass::getClockFrequency()
  {
  }

  /*------------------------------------------------
  Async IO Interface
  ------------------------------------------------*/
  Chimera::Status_t SPIClass::await( const Chimera::Event::Trigger event, const size_t timeout )
  {
  }

  Chimera::Status_t SPIClass::await( const Chimera::Event::Trigger event, SemaphoreHandle_t notifier, const size_t timeout )
  {
  }

  /*------------------------------------------------
  Listener Interface
  ------------------------------------------------*/
  Chimera::Status_t SPIClass::registerListener( Chimera::Event::Actionable &listener, const size_t timeout,
                                                size_t &registrationID )
  {
  }

  Chimera::Status_t SPIClass::removeListener( const size_t registrationID, const size_t timeout )
  {
  }
}
