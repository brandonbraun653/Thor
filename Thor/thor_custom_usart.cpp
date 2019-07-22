/********************************************************************************
 *   File Name:
 *    thor_custom_usart.cpp
 *
 *   Description:
 *    Implements the custom driver variant of the Thor USART interface.
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/


/* Thor Includes */
#include <Thor/usart.hpp>
#include <Thor/drivers/Usart.hpp>


namespace Thor::USART
{
  USARTClass::USARTClass()
  {
  }

  USARTClass::~USARTClass()
  {
  }

  Chimera::Status_t USARTClass::assignHW( const uint8_t channel, const Chimera::Serial::IOPins &pins )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t USARTClass::begin( const Chimera::Hardware::SubPeripheralMode,
                                       const Chimera::Hardware::SubPeripheralMode rxMode )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t USARTClass::end()
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t USARTClass::configure( const Chimera::Serial::COMConfig &config )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t USARTClass::setBaud( const uint32_t baud )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t USARTClass::setMode( const Chimera::Hardware::SubPeripheral periph,
                                         const Chimera::Hardware::SubPeripheralMode mode )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t USARTClass::write( const uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t USARTClass::read( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t USARTClass::flush( const Chimera::Hardware::SubPeripheral periph )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  void USARTClass::postISRProcessing()
  {
  }

  Chimera::Status_t USARTClass::readAsync( uint8_t *const buffer, const size_t len )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t USARTClass::enableBuffering( const Chimera::Hardware::SubPeripheral periph,
                                                 boost::circular_buffer<uint8_t> *const userBuffer, uint8_t *const hwBuffer,
                                                 const uint32_t hwBufferSize )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t USARTClass::disableBuffering( const Chimera::Hardware::SubPeripheral periph )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  bool USARTClass::available( size_t *const bytes )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  void USARTClass::await( const Chimera::Event::Trigger event )
  {
  }

  void USARTClass::await( const Chimera::Event::Trigger event, SemaphoreHandle_t notifier )
  {
  }

  Chimera::Status_t USARTClass::registerListener( Chimera::Event::Actionable &listener, const size_t timeout,
                                                  size_t &registrationID )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t USARTClass::removeListener( const size_t registrationID, const size_t timeout )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }
}    // namespace Thor::USART
