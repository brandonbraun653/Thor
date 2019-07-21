/********************************************************************************
 * File Name:
 *   thor_serial.cpp
 *
 * Description:
 *   Implements the serial interface for Thor
 *
 * 2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Thor Includes */
#include <Thor/serial.hpp>
#include <Thor/uart.hpp>
#include <Thor/usart.hpp>
#include <Thor/types/serial_types.hpp>
#include <Thor/definitions/serial_definitions.hpp>

namespace Thor::Serial
{
  static const std::array<bool, MAX_VIRTUAL_CHANNELS> isUARTChannel = { {
#if defined( STM32F767xx ) || defined( STM32F446xx )
      false, /* Not actually a Serial channel */
      false, /* USART 1	*/
      false, /* USART 2	*/
      false, /* USART 3	*/
      true,  /* UART  4	*/
      true,  /* UART  5	*/
      false, /* USART 6	*/
      true,  /* UART  7	*/
      true   /* UART  8	*/
#endif
  } };

  SerialClass::SerialClass() :
      serialChannel( INVALID_CHANNEL ), serialObject( nullptr )
  {
    /*------------------------------------------------
    Make sure the behavior is disabled until the user calls assignHW()
    ------------------------------------------------*/
//    auto tmp     = std::make_shared<Chimera::Serial::SerialUnsupported>();
//    serialObject = std::static_pointer_cast<Chimera::Serial::Interface, Chimera::Serial::SerialUnsupported>( tmp );
  }

  Chimera::Status_t SerialClass::assignHW( const uint8_t channel, const Chimera::Serial::IOPins &pins )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    if ( ( channel != INVALID_CHANNEL ) && ( channel < MAX_VIRTUAL_CHANNELS ) )
    {
      serialChannel = channel;

      /*------------------------------------------------
      Decide which instance to create
      -------------------------------------------------*/
      if ( isUARTChannel[ channel ] )
      {
        auto uartObject   = std::make_shared<Thor::UART::UARTClass>();
        serialObject = std::static_pointer_cast<Chimera::Serial::Interface, Thor::UART::UARTClass>( uartObject );
      }
      else
      {
        auto usartObject  = std::make_shared<Thor::USART::USARTClass>();
        serialObject = std::static_pointer_cast<Chimera::Serial::Interface, Thor::USART::USARTClass>( usartObject );
      }

      /*------------------------------------------------
      Initialize the low level driver
      ------------------------------------------------*/
      result = serialObject->assignHW( channel, pins );
    }
    else
    {
      /*------------------------------------------------
      Leave the serialObject as-is since it is initialized
      to "disabled" in the class ctor
      ------------------------------------------------*/
      result = Chimera::CommonStatusCodes::FAIL;
    }

    return result;
  }

  Chimera::Status_t SerialClass::begin( const Chimera::Hardware::SubPeripheralMode txMode, const Chimera::Hardware::SubPeripheralMode rxMode )
  {
    return serialObject->begin( txMode, rxMode );
  }

  Chimera::Status_t SerialClass::end()
  {
    return serialObject->end();
  }

  Chimera::Status_t SerialClass::configure( const Chimera::Serial::COMConfig &config )
  {
    return serialObject->configure( config );
  }

  Chimera::Status_t SerialClass::setBaud( const uint32_t baud )
  {
    return serialObject->setBaud( baud );
  }

  Chimera::Status_t SerialClass::setMode( const Chimera::Hardware::SubPeripheral periph, const Chimera::Hardware::SubPeripheralMode mode )
  {
    return serialObject->setMode( periph, mode );
  }

  Chimera::Status_t SerialClass::write( const uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
  {
    return serialObject->write( buffer, length, timeout_mS );
  }

  Chimera::Status_t SerialClass::read( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
  {
    return serialObject->read( buffer, length, timeout_mS );
  }

  Chimera::Status_t SerialClass::flush( const Chimera::Hardware::SubPeripheral periph )
  {
    return serialObject->flush( periph );
  }

  void SerialClass::postISRProcessing()
  {
    serialObject->postISRProcessing();
  }

  Chimera::Status_t SerialClass::readAsync( uint8_t *const buffer, const size_t len )
  {
    return serialObject->readAsync( buffer, len );
  }

  Chimera::Status_t SerialClass::attachCallback( const Chimera::Event::Trigger event, Chimera::Callback::ISRCallback &handle )
  {
    return serialObject->attachCallback( event, handle );
  }

  Chimera::Status_t SerialClass::detachCallback( const Chimera::Event::Trigger event, Chimera::Callback::ISRCallback &handle )
  {
    return serialObject->detachCallback( event, handle );
  }

#if defined( USING_FREERTOS )
  Chimera::Status_t SerialClass::attachNotifier( const Chimera::Event::Trigger event, SemaphoreHandle_t *const semphr )
  {
    return serialObject->attachNotifier( event, semphr );
  }

  Chimera::Status_t SerialClass::detachNotifier( const Chimera::Event::Trigger event, SemaphoreHandle_t *const semphr )
  {
    return serialObject->detachNotifier( event, semphr );
  }
#endif

  Chimera::Status_t SerialClass::enableBuffering( const Chimera::Hardware::SubPeripheral periph,
                                                  boost::circular_buffer<uint8_t> *const userBuffer, uint8_t *const hwBuffer,
                                                  const uint32_t hwBufferSize )
  {
    return serialObject->enableBuffering( periph, userBuffer, hwBuffer, hwBufferSize );
  }

  Chimera::Status_t SerialClass::disableBuffering( const Chimera::Hardware::SubPeripheral periph )
  {
    return serialObject->disableBuffering( periph );
  }

  bool SerialClass::available( size_t *const bytes )
  {
    return serialObject->available( bytes );
  }

  void SerialClass::await( const Chimera::Event::Trigger event )
  {
    serialObject->await( event );
  }

  void SerialClass::await( const Chimera::Event::Trigger event, SemaphoreHandle_t notifier )
  {
    serialObject->await( event, notifier );
  }

}    // namespace Thor::Serial
