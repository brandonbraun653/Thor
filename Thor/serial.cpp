

/* Project Includes */
#include <Thor/include/serial.hpp>
#include <Thor/include/utilities.hpp>

using namespace Thor::Definitions;
using namespace Thor::Definitions::Serial;
using namespace Thor::Peripheral::UART;
using namespace Thor::Peripheral::USART;

namespace Thor
{
  namespace Peripheral
  {
    namespace Serial
    {
      static HardwareClassMapping serialPeripheralMap[ MAX_SERIAL_CHANNELS + 1 ] = {
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
      static_assert( COUNT_OF_ARRAY( serialPeripheralMap ) == ( MAX_SERIAL_CHANNELS + 1 ), "Invalid array size" );

      SerialClass::SerialClass( const uint8_t channel )
      {
      }

      SerialClass::SerialClass( const uint8_t channel, const SerialPins &config )
      {
        if ( channel && ( channel < MAX_SERIAL_CHANNELS ) )
        {
          serialChannel = channel;

          /*------------------------------------------------
          Decide which instance to create
          -------------------------------------------------*/
          if ( serialPeripheralMap[ channel ].ON_UART )
          {
            auto tmp     = UARTClass::create( serialPeripheralMap[ channel ].peripheral_number, config );
            serialObject = std::static_pointer_cast<SerialInterface, UARTClass>( tmp );
          }
          else
          {
            auto tmp     = USARTClass::create( serialPeripheralMap[ channel ].peripheral_number, config );
            serialObject = std::static_pointer_cast<SerialInterface, USARTClass>( tmp );
          }
        }
        else
        {
          serialObject = nullptr;
        }
      }

      Status SerialClass::begin( const uint32_t baud, const Modes tx_mode, const Modes rx_mode )
      {
        assert( serialObject );
        return serialObject->begin( baud, tx_mode, rx_mode );
      }

      Status SerialClass::setMode( const SubPeripheral periph, const Modes mode )
      {
        assert( serialObject );
        return serialObject->setMode( periph, mode );
      }

      Status SerialClass::setBaud( const uint32_t baud )
      {
        assert( serialObject );
        return serialObject->setBaud( baud );
      }

      Status SerialClass::write( const uint8_t *const val, const size_t length )
      {
        assert( serialObject );
        return serialObject->write( val, length );
      }

      Status SerialClass::read( uint8_t *const buffer, size_t length )
      {
        assert( serialObject );
        return serialObject->read( buffer, length );
      }

      void SerialClass::end()
      {
        assert( serialObject );
        serialObject->end();
      }

#if defined( USING_FREERTOS )
      void SerialClass::attachThreadTrigger( const Trigger trig, SemaphoreHandle_t *const semphr )
      {
        assert( serialObject );
        serialObject->attachThreadTrigger( trig, semphr );
      }

      void SerialClass::removeThreadTrigger( const Trigger trig )
      {
        assert( serialObject );
        serialObject->removeThreadTrigger( trig );
      }
#endif


#if defined( USING_CHIMERA )
      static constexpr Thor::Definitions::Modes convertMode( const Chimera::Serial::Modes mode )
      {
        switch ( mode )
        {
          case Chimera::Serial::Modes::BLOCKING:
            return Thor::Definitions::Modes::BLOCKING;
            break;

          case Chimera::Serial::Modes::INTERRUPT:
            return Thor::Definitions::Modes::INTERRUPT;
            break;

          case Chimera::Serial::Modes::DMA:
            return Thor::Definitions::Modes::DMA;
            break;

          default:
            return Thor::Definitions::Modes::MODE_UNDEFINED;
            break;
        }
      }

      static constexpr Thor::Definitions::SubPeripheral convertPeriph( const Chimera::Serial::SubPeripheral periph )
      {
        switch ( periph )
        {
          case Chimera::Serial::SubPeripheral::TX:
            return Thor::Definitions::SubPeripheral::TX;
            break;

          case Chimera::Serial::SubPeripheral::RX:
            return Thor::Definitions::SubPeripheral::RX;
            break;

          default:
            return Thor::Definitions::SubPeripheral::UNKNOWN_SUB_PERIPHERAL;
            break;
        }
      }


      ChimeraSerial::ChimeraSerial( const uint8_t channel )
      {
        serial = std::make_shared<SerialClass>( channel );
      }

      Chimera::Serial::Status ChimeraSerial::begin( const uint32_t baud, const Chimera::Serial::Modes txMode,
                                                    const Chimera::Serial::Modes rxMode )
      {
        assert( serial );
        auto chimeraError = Chimera::Serial::Status::OK;

        auto thorError = serial->begin( baud, convertMode( txMode ), convertMode( rxMode ) );

        if ( thorError != Thor::Definitions::Status::PERIPH_OK )
        {
          chimeraError = Chimera::Serial::Status::ERROR;
        }

        return chimeraError;
      }

      Chimera::Serial::Status ChimeraSerial::setMode( Chimera::Serial::SubPeripheral periph, Chimera::Serial::Modes mode )
      {
        assert( serial );
        auto chimeraError = Chimera::Serial::Status::OK;
        auto thorError    = serial->setMode( convertPeriph( periph ), convertMode( mode ) );

        if ( thorError != Thor::Definitions::Status::PERIPH_OK )
        {
          chimeraError = Chimera::Serial::Status::ERROR;
        }

        return chimeraError;
      }

      Chimera::Serial::Status ChimeraSerial::setBaud( const uint32_t baud )
      {
        assert( serial );
        auto chimeraError = Chimera::Serial::Status::OK;
        auto thorError    = serial->setBaud( baud );

        if ( thorError != Thor::Definitions::Status::PERIPH_OK )
        {
          chimeraError = Chimera::Serial::Status::ERROR;
        }

        return chimeraError;
      }

      Chimera::Serial::Status ChimeraSerial::write( const uint8_t *const buffer, const size_t length )
      {
        assert( serial );
        auto chimeraError = Chimera::Serial::Status::OK;
        auto thorError    = serial->write( buffer, length );

        if ( thorError != Thor::Definitions::Status::PERIPH_OK )
        {
          chimeraError = Chimera::Serial::Status::ERROR;
        }

        return chimeraError;
      }

      Chimera::Serial::Status read( uint8_t *const buffer, const size_t length )
      {
        return Chimera::Serial::Status::FEATURE_NOT_ENABLED;
      }

      Chimera::Serial::Status readAsync( uint8_t *const buffer, const size_t maxLen )
      {
        return Chimera::Serial::Status::FEATURE_NOT_ENABLED;
      }

      Chimera::Serial::Status enableDoubleBuffering( const Chimera::Serial::SubPeripheral periph,
                                                     volatile uint8_t *const bufferOne, volatile uint8_t *const bufferTwo,
                                                     const size_t length )
      {
        return Chimera::Serial::Status::FEATURE_NOT_ENABLED;
      }

      Chimera::Serial::Status disableDoubleBuffering()
      {
        return Chimera::Serial::Status::FEATURE_NOT_ENABLED;
      }

      Chimera::Serial::Status attachEventNotifier( const Chimera::Serial::Event event, volatile bool *const notifier )
      {
        return Chimera::Serial::Status::FEATURE_NOT_ENABLED;
      }

      Chimera::Serial::Status removeEventNotifier( const Chimera::Serial::Event event, volatile bool *const notifier )
      {
        return Chimera::Serial::Status::FEATURE_NOT_ENABLED;
      }

#if defined( USING_FREERTOS )
      Chimera::Serial::Status attachEventNotifier( const Chimera::Serial::Event event, SemaphoreHandle_t *const semphr )
      {
        return Chimera::Serial::Status::FEATURE_NOT_ENABLED;
      }

      Chimera::Serial::Status removeEventNotifier( const Chimera::Serial::Event event, SemaphoreHandle_t *const semphr )
      {
        return Chimera::Serial::Status::FEATURE_NOT_ENABLED;
      }
#endif

      void status( Chimera::Serial::HardwareStatus &status )
      {
      }

      bool available( size_t *const bytes = nullptr )
      {
        return false;
      }

      bool reserve( const uint32_t timeout_mS )
      {
        return false;
      }

      bool release( const uint32_t timeout_mS )
      {
        return false;
      }

#endif   /* !USING_CHIMERA */
    }    // namespace Serial
  }      // namespace Peripheral
}    // namespace Thor
