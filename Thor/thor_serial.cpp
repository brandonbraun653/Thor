/********************************************************************************
* File Name:
*   thor_serial.cpp
*
* Description:
*   Implements the serial interface for Thor
*
* 2019 | Brandon Braun | brandonbraun653@gmail.com
********************************************************************************/

/* Project Includes */
#include <Thor/serial.hpp>
#include <Thor/utilities.hpp>

using namespace Thor;
using namespace Thor::Serial;
using namespace Thor::UART;
using namespace Thor::USART;

namespace Thor
{
  namespace Serial
  {
    static constexpr HardwareClassMapping ch2Periph[ MAX_SERIAL_CHANNELS + 1 ] = {
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
    };
    static_assert( COUNT_OF_ARRAY( ch2Periph ) == ( MAX_SERIAL_CHANNELS + 1 ), "Invalid array size" );

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
          auto tmp     = UARTClass::create( ch2Periph[ channel ].peripheral_number, bSize );
          serialObject = std::static_pointer_cast<Chimera::Serial::Interface, UARTClass>( tmp );
        }
        else
        {
          serialObject = nullptr;
          //auto tmp     = USARTClass::create( serialPeripheralMap[ channel ].peripheral_number, config );
          //serialObject = std::static_pointer_cast<Chimera::Serial::Interface, USARTClass>( tmp );
        }

        /*------------------------------------------------
        Initialize the low level driver
        ------------------------------------------------*/
        result = serialObject->assignHW( channel, pins );
      }
      else
      {
        serialObject = nullptr;
        result       = Chimera::CommonStatusCodes::FAIL;
      }
    }

    Chimera::Status_t SerialClass::begin( const Chimera::Serial::Modes txMode, const Chimera::Serial::Modes rxMode )
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

    Chimera::Status_t SerialClass::setMode( const Chimera::Serial::SubPeripheral periph, const Chimera::Serial::Modes mode )
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

    Chimera::Status_t SerialClass::flush( const Chimera::Serial::SubPeripheral periph )
    {
      return serialObject->flush( periph );
    }

    Chimera::Status_t SerialClass::readAsync( uint8_t *const buffer, const size_t len )
    {
      return serialObject->readAsync( buffer, len );
    }

#if defined( USING_FREERTOS )
    Chimera::Status_t SerialClass::attachEventNotifier( const Chimera::Serial::Event event, SemaphoreHandle_t *const semphr )
    {
      return serialObject->attachEventNotifier( event, semphr );
    }

    Chimera::Status_t SerialClass::removeEventNotifier( const Chimera::Serial::Event event, SemaphoreHandle_t *const semphr )
    {
      return serialObject->removeEventNotifier( event, semphr );
    }
#endif

    Chimera::Status_t SerialClass::enableBuffering( const Chimera::Serial::SubPeripheral periph,
                                                    boost::circular_buffer<uint8_t> *const buffer )
    {
      return serialObject->enableBuffering( periph, buffer );
    }

    Chimera::Status_t SerialClass::disableBuffering( const Chimera::Serial::SubPeripheral periph )
    {
      return serialObject->disableBuffering( periph );
    }
  }    // namespace Serial
}    // namespace Thor
