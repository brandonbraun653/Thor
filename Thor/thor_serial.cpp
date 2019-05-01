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
#include <Thor/types/serial_types.hpp>
#include <Thor/definitions/serial_definitions.hpp>

using namespace Thor;
using namespace Thor::Serial;
using namespace Thor::UART;
using namespace Thor::USART;

namespace Thor::Serial
{
  static const std::array<HardwareClassMapping, MAX_SERIAL_CHANNELS + 1> ch2Periph = { {
#if defined( STM32F767xx ) || defined( STM32F446xx )
      { false, 0 }, /* Not actually a UART instance */
      { false, 1 }, /* USART 1	*/
      { false, 2 }, /* USART 2	*/
      { false, 3 }, /* USART 3	*/
      { true, 4 },  /* UART  4	*/
      { true, 5 },  /* UART  5	*/
      { false, 6 }, /* USART 6	*/
      { true, 7 },  /* UART  7	*/
      { true, 8 }   /* UART	 8	*/
#endif
  } };

  SerialClass::SerialClass()
  {
    serialChannel = 0u;

    /*------------------------------------------------
    Make sure the behavior is disabled until the user calls assignHW()
    ------------------------------------------------*/
    auto tmp     = std::make_shared<Chimera::Serial::SerialUnsupported>();
    serialObject = std::static_pointer_cast<Chimera::Serial::Interface, Chimera::Serial::SerialUnsupported>( tmp );
  }

  Chimera::Status_t SerialClass::assignHW( const uint8_t channel, const Chimera::Serial::IOPins &pins )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    if ( channel && ( channel < MAX_SERIAL_CHANNELS ) )
    {
      serialChannel = channel;

      /*------------------------------------------------
      Decide which instance to create
      -------------------------------------------------*/
      if ( ch2Periph[ channel ].ON_UART )
      {
        auto tmp     = std::make_shared<UARTClass>();
        serialObject = std::static_pointer_cast<Chimera::Serial::Interface, UARTClass>( tmp );
      }
      else
      {
        auto tmp     = std::make_shared<USARTClass>();
        serialObject = std::static_pointer_cast<Chimera::Serial::Interface, USARTClass>( tmp );
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

  Chimera::Status_t SerialClass::configure( const uint32_t baud, const Chimera::Serial::CharWid width,
                                            const Chimera::Serial::Parity parity, const Chimera::Serial::StopBits stop,
                                            const Chimera::Serial::FlowControl flow )
  {
    return serialObject->configure( baud, width, parity, stop, flow );
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

#if defined( USING_FREERTOS )
  Chimera::Status_t SerialClass::attachNotifier( const Chimera::Event::Trigger_t event, SemaphoreHandle_t *const semphr )
  {
    return serialObject->attachNotifier( event, semphr );
  }

  Chimera::Status_t SerialClass::detachNotifier( const Chimera::Event::Trigger_t event, SemaphoreHandle_t *const semphr )
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
}    // namespace Thor::Serial
