/********************************************************************************
 *  File Name:
 *    hld_uart_driver.cpp
 *
 *  Description:
 *    Implements the custom driver variant of the Thor UART interface.
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Aurora Includes */
#include <Aurora/constants/common.hpp>

/* Chimera Includes */
#include <Chimera/common>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/uart>
#include <Thor/lld/interface/uart/uart.hpp>

namespace Chimera::UART::Backend
{
  Chimera::Status_t prjInitialize()
  {
    return Thor::UART::initialize();
  }
}    // namespace Chimera::UART

#if defined( THOR_HLD_UART )

namespace Thor::UART
{
  static size_t s_driver_initialized;

  Chimera::Status_t initialize()
  {
    s_driver_initialized = ~Chimera::DRIVER_INITIALIZED_KEY;

    /*------------------------------------------------
    Initialize the low level driver
    ------------------------------------------------*/
    #if defined( THOR_LLD_UART )
    Thor::LLD::UART::initialize();
    #else
    #pragma message("HLD UART driver enabled but the required LLD is not")
    #endif

    s_driver_initialized = Chimera::DRIVER_INITIALIZED_KEY;
    return Chimera::CommonStatusCodes::OK;
  }

  Driver::Driver()
  {
  }

  Driver::~Driver()
  {
  }

  Chimera::Status_t Driver::assignHW( const uint8_t channel, const Chimera::Serial::IOPins &pins )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::begin( const Chimera::Hardware::PeripheralMode txMode,
                                      const Chimera::Hardware::PeripheralMode rxMode )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::end()
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::configure( const Chimera::Serial::Config &config )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::setBaud( const uint32_t baud )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::setMode( const Chimera::Hardware::SubPeripheral periph,
                                        const Chimera::Hardware::PeripheralMode mode )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::write( const uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::read( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::flush( const Chimera::Hardware::SubPeripheral periph )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  void Driver::postISRProcessing()
  {
  }

  Chimera::Status_t Driver::readAsync( uint8_t *const buffer, const size_t len )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::enableBuffering( const Chimera::Hardware::SubPeripheral periph,
                                                boost::circular_buffer<uint8_t> *const userBuffer, uint8_t *const hwBuffer,
                                                const uint32_t hwBufferSize )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::disableBuffering( const Chimera::Hardware::SubPeripheral periph )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  bool Driver::available( size_t *const bytes )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::await( const Chimera::Event::Trigger event, const size_t timeout )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::await( const Chimera::Event::Trigger event, Chimera::Threading::BinarySemaphore &notifier,
                                      const size_t timeout )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::registerListener( Chimera::Event::Actionable &listener, const size_t timeout,
                                                 size_t &registrationID )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::removeListener( const size_t registrationID, const size_t timeout )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }
}    // namespace Thor::UART

#endif /* THOR_HLD_UART */
