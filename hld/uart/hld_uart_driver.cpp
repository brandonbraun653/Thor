/********************************************************************************
 *  File Name:
 *    hld_uart_driver.cpp
 *
 *  Description:
 *    Implements the custom driver variant of the Thor UART interface.
 *
 *  2019-2022 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/constants>
#include <Chimera/assert>
#include <Chimera/common>
#include <Thor/cfg>
#include <Thor/lld/interface/inc/uart>

#if defined( THOR_UART )
namespace Chimera::UART
{
  /*-------------------------------------------------------------------------------
  Driver Implementation
  -------------------------------------------------------------------------------*/
  Driver::Driver()
  {
  }


  Driver::~Driver()
  {
  }


  Chimera::Status_t Driver::assignHW( const Chimera::Serial::Channel channel, const Chimera::Serial::IOPins &pins )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::begin( const Chimera::Hardware::PeripheralMode txMode,
                                   const Chimera::Hardware::PeripheralMode rxMode )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::end()
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::configure( const Chimera::Serial::Config &config )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::setBaud( const uint32_t baud )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::setMode( const Chimera::Hardware::SubPeripheral  periph,
                                     const Chimera::Hardware::PeripheralMode mode )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::write( const void *const buffer, const size_t length )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::read( void *const buffer, const size_t length )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::flush( const Chimera::Hardware::SubPeripheral periph )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  void Driver::postISRProcessing()
  {
  }


  Chimera::Status_t Driver::toggleAsyncListening( const bool state )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::readAsync( uint8_t *const buffer, const size_t len )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::enableBuffering( const Chimera::Hardware::SubPeripheral periph,
                                             Chimera::Serial::CircularBuffer &userBuffer, uint8_t *const hwBuffer,
                                             const size_t hwBufferSize )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::disableBuffering( const Chimera::Hardware::SubPeripheral periph )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  bool Driver::available( size_t *const bytes )
  {
    return false;
  }
}


namespace Chimera::UART::Backend
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  Chimera::Status_t registerDriver( Chimera::UART::Backend::DriverConfig &registry )
  {
    registry.isSupported   = false;
    registry.getDriver     = nullptr;
    registry.initialize    = nullptr;
    registry.reset         = nullptr;
    registry.isChannelUART = nullptr;
    return Chimera::Status::NOT_SUPPORTED;
  }
}
#endif /* THOR_UART */
