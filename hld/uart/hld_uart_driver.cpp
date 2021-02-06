/********************************************************************************
 *  File Name:
 *    hld_uart_driver.cpp
 *
 *  Description:
 *    Implements the custom driver variant of the Thor UART interface.
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Aurora Includes */
#include <Aurora/constants>

/* Chimera Includes */
#include <Chimera/assert>
#include <Chimera/common>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/uart>
#include <Thor/lld/interface/uart/uart_detail.hpp>
#include <Thor/lld/interface/uart/uart_intf.hpp>
#include <Thor/lld/interface/uart/uart_types.hpp>

#if defined( THOR_HLD_UART )

/*-------------------------------------------------------------------------------
Aliases
-------------------------------------------------------------------------------*/
namespace HLD = ::Thor::UART;
namespace LLD = ::Thor::LLD::UART;


/*-------------------------------------------------------------------------------
Constants
-------------------------------------------------------------------------------*/
static constexpr size_t NUM_DRIVERS = 1; //LLD::NUM_UART_PERIPHS;
#pragma warning("Missing LLD UART driver")

/*-------------------------------------------------------------------------------
Variables
-------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------
Private Functions
-------------------------------------------------------------------------------*/

namespace Thor::UART
{
  static size_t s_driver_initialized;

  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    s_driver_initialized = ~Chimera::DRIVER_INITIALIZED_KEY;

    /*------------------------------------------------
    Initialize the low level driver
    ------------------------------------------------*/
    Thor::LLD::UART::initialize();

    s_driver_initialized = Chimera::DRIVER_INITIALIZED_KEY;
    return Chimera::Status::OK;
  }


  Chimera::Status_t reset()
  {
    return Chimera::Status::OK;
  }


  bool isChannelUART( const Chimera::Serial::Channel channel )
  {
    return false;
  }


  Chimera::UART::Driver_rPtr getDriver( const Chimera::Serial::Channel channel )
  {
    // Currently not supported. Make sure it's not used.
    RT_HARD_ASSERT( false );
    return nullptr;
  }


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

  Chimera::Status_t Driver::setMode( const Chimera::Hardware::SubPeripheral periph,
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

  Chimera::Status_t Driver::toggleAsyncListening( const bool state )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }

  void Driver::postISRProcessing()
  {
  }

  Chimera::Status_t Driver::readAsync( uint8_t *const buffer, const size_t len )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::enableBuffering( const Chimera::Hardware::SubPeripheral periph,
                                                boost::circular_buffer<uint8_t> *const userBuffer, uint8_t *const hwBuffer,
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
    return Chimera::Status::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::await( const Chimera::Event::Trigger event, const size_t timeout )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::await( const Chimera::Event::Trigger event, Chimera::Threading::BinarySemaphore &notifier,
                                      const size_t timeout )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }

}    // namespace Thor::UART

#endif /* THOR_HLD_UART */
