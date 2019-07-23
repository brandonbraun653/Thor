/********************************************************************************
 *   File Name:
 *    thor_custom_UART.cpp
 *
 *   Description:
 *    Implements the custom driver variant of the Thor UART interface.
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/


/* Thor Includes */
#include <Thor/uart.hpp>
#include <Thor/drivers/Uart.hpp>


namespace Thor::UART
{
  UARTClass::UARTClass()
  {
  }

  UARTClass::~UARTClass()
  {
  }

  Chimera::Status_t UARTClass::assignHW( const uint8_t channel, const Chimera::Serial::IOPins &pins )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t UARTClass::begin( const Chimera::Hardware::SubPeripheralMode,
                                      const Chimera::Hardware::SubPeripheralMode rxMode )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t UARTClass::end()
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t UARTClass::configure( const Chimera::Serial::Config &config )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t UARTClass::setBaud( const uint32_t baud )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t UARTClass::setMode( const Chimera::Hardware::SubPeripheral periph,
                                        const Chimera::Hardware::SubPeripheralMode mode )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t UARTClass::write( const uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t UARTClass::read( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t UARTClass::flush( const Chimera::Hardware::SubPeripheral periph )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  void UARTClass::postISRProcessing()
  {
  }

  Chimera::Status_t UARTClass::readAsync( uint8_t *const buffer, const size_t len )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t UARTClass::enableBuffering( const Chimera::Hardware::SubPeripheral periph,
                                                boost::circular_buffer<uint8_t> *const userBuffer, uint8_t *const hwBuffer,
                                                const uint32_t hwBufferSize )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t UARTClass::disableBuffering( const Chimera::Hardware::SubPeripheral periph )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  bool UARTClass::available( size_t *const bytes )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t UARTClass::await( const Chimera::Event::Trigger event, const size_t timeout )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t UARTClass::await( const Chimera::Event::Trigger event, SemaphoreHandle_t notifier, const size_t timeout )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t UARTClass::registerListener( Chimera::Event::Actionable &listener, const size_t timeout,
                                                 size_t &registrationID )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t UARTClass::removeListener( const size_t registrationID, const size_t timeout )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }
}    // namespace Thor::UART
