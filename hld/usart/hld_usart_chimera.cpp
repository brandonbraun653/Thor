/********************************************************************************
 *  File Name:
 *    hld_usart_chimera.cpp
 *
 *  Description:
 *    Implementation of Chimera USART driver hooks
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* STL Includes */
#include <cstdint>
#include <memory>

/* Chimera Includes */
#include <Chimera/assert>
#include <Chimera/common>
#include <Chimera/usart>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/usart>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/usart/usart_intf.hpp>
#include <Thor/lld/interface/usart/usart_detail.hpp>

/*-------------------------------------------------------------------------------
Aliases
-------------------------------------------------------------------------------*/
namespace HLD = ::Thor::USART;
namespace LLD = ::Thor::LLD::USART;

/*-------------------------------------------------------------------------------
Constants
-------------------------------------------------------------------------------*/
static constexpr size_t NUM_DRIVERS = ::LLD::NUM_USART_PERIPHS;

/*-------------------------------------------------------------------------------
Variables
-------------------------------------------------------------------------------*/
static Chimera::USART::Driver s_raw_driver[ NUM_DRIVERS ];

namespace Chimera::USART::Backend
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
#if defined( THOR_HLD_USART )
  Chimera::Status_t initialize()
  {
    return Thor::USART::initialize();
  }


  Chimera::Status_t reset()
  {
    return Thor::USART::reset();
  }


  bool isChannelUSART( const Chimera::Serial::Channel channel )
  {
    return Thor::LLD::USART::isSupported( channel );
  }


  Driver_rPtr getDriver( const Chimera::Serial::Channel channel )
  {
    auto resourceIndex = ::LLD::getResourceIndex( channel );
    if ( resourceIndex != ::Thor::LLD::INVALID_RESOURCE_INDEX )
    {
      return Driver_rPtr( &s_raw_driver[ resourceIndex ] );
    }
    else
    {
      return nullptr;
    }
  }
#endif    // THOR-HLD_USART

  Chimera::Status_t registerDriver( Chimera::USART::Backend::DriverConfig &registry )
  {
#if defined( THOR_HLD_USART )
    registry.isSupported    = true;
    registry.getDriver      = getDriver;
    registry.initialize     = initialize;
    registry.reset          = reset;
    registry.isChannelUSART = isChannelUSART;
    return Chimera::Status::OK;
#else
    memset( &registry, 0, sizeof( Chimera::USART::Backend::DriverConfig ) );
    registry.isSupported = false;
    return Chimera::Status::NOT_SUPPORTED;
#endif /* THOR_HLD_USART */
  }
}    // namespace Chimera::USART::Backend


namespace Chimera::USART
{
  /*-------------------------------------------------------------------------------
  Driver Implementation
  -------------------------------------------------------------------------------*/
  Driver::Driver() : mChannel( Chimera::Serial::Channel::NOT_SUPPORTED )
  {
  }

  Driver::~Driver()
  {
  }

  /*-------------------------------------------------
  Interface: Hardware
  -------------------------------------------------*/
  Chimera::Status_t Driver::assignHW( const Chimera::Serial::Channel channel, const Chimera::Serial::IOPins &pins )
  {
#if defined( THOR_HLD_USART )
    mChannel = channel;
    return ::HLD::getDriver( mChannel )->assignHW( channel, pins );
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  Chimera::Status_t Driver::begin( const Chimera::Hardware::PeripheralMode txMode,
                                   const Chimera::Hardware::PeripheralMode rxMode )
  {
#if defined( THOR_HLD_USART )
    return ::HLD::getDriver( mChannel )->begin( txMode, rxMode );
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  Chimera::Status_t Driver::end()
  {
#if defined( THOR_HLD_USART )
    return ::HLD::getDriver( mChannel )->end();
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  Chimera::Status_t Driver::configure( const Chimera::Serial::Config &config )
  {
#if defined( THOR_HLD_USART )
    return ::HLD::getDriver( mChannel )->configure( config );
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  Chimera::Status_t Driver::setBaud( const uint32_t baud )
  {
#if defined( THOR_HLD_USART )
    return ::HLD::getDriver( mChannel )->setBaud( baud );
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  Chimera::Status_t Driver::setMode( const Chimera::Hardware::SubPeripheral periph,
                                     const Chimera::Hardware::PeripheralMode mode )
  {
#if defined( THOR_HLD_USART )
    return ::HLD::getDriver( mChannel )->setMode( periph, mode );
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  Chimera::Status_t Driver::write( const void *const buffer, const size_t length )
  {
#if defined( THOR_HLD_USART )
    return ::HLD::getDriver( mChannel )->write( buffer, length );
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  Chimera::Status_t Driver::read( void *const buffer, const size_t length )
  {
#if defined( THOR_HLD_USART )
    return ::HLD::getDriver( mChannel )->read( buffer, length );
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  Chimera::Status_t Driver::flush( const Chimera::Hardware::SubPeripheral periph )
  {
#if defined( THOR_HLD_USART )
    return ::HLD::getDriver( mChannel )->flush( periph );
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  Chimera::Status_t Driver::toggleAsyncListening( const bool state )
  {
#if defined( THOR_HLD_USART )
    return ::HLD::getDriver( mChannel )->toggleAsyncListening( state );
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  Chimera::Status_t Driver::readAsync( uint8_t *const buffer, const size_t len )
  {
#if defined( THOR_HLD_USART )
    return ::HLD::getDriver( mChannel )->readAsync( buffer, len );
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  Chimera::Status_t Driver::enableBuffering( const Chimera::Hardware::SubPeripheral periph,
                                             boost::circular_buffer<uint8_t> *const userBuffer, uint8_t *const hwBuffer,
                                             const size_t hwBufferSize )
  {
#if defined( THOR_HLD_USART )
    return ::HLD::getDriver( mChannel )->enableBuffering( periph, userBuffer, hwBuffer, hwBufferSize );
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }

  Chimera::Status_t Driver::disableBuffering( const Chimera::Hardware::SubPeripheral periph )
  {
#if defined( THOR_HLD_USART )
    return ::HLD::getDriver( mChannel )->disableBuffering( periph );
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  bool Driver::available( size_t *const bytes )
  {
#if defined( THOR_HLD_USART )
    return ::HLD::getDriver( mChannel )->available( bytes );
#else
    return false;
#endif
  }


  void Driver::postISRProcessing()
  {
#if defined( THOR_HLD_USART )
    ::HLD::getDriver( mChannel )->postISRProcessing();
#endif
  }

  /*-------------------------------------------------
  Interface: AsyncIO
  -------------------------------------------------*/
  Chimera::Status_t Driver::await( const Chimera::Event::Trigger event, const size_t timeout )
  {
#if defined( THOR_HLD_USART )
    return ::HLD::getDriver( mChannel )->await( event, timeout );
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  Chimera::Status_t Driver::await( const Chimera::Event::Trigger event, Chimera::Threading::BinarySemaphore &notifier,
                                   const size_t timeout )
  {
#if defined( THOR_HLD_USART )
    return ::HLD::getDriver( mChannel )->await( event, notifier, timeout );
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }

  /*-------------------------------------------------
  Interface: Lockable
  -------------------------------------------------*/
  void Driver::lock()
  {
#if defined( THOR_HLD_USART )
    ::HLD::getDriver( mChannel )->lock();
#endif
  }

  void Driver::lockFromISR()
  {
#if defined( THOR_HLD_USART )
    ::HLD::getDriver( mChannel )->lockFromISR();
#endif
  }

  bool Driver::try_lock_for( const size_t timeout )
  {
#if defined( THOR_HLD_USART )
    return ::HLD::getDriver( mChannel )->try_lock_for( timeout );
#else
    return false;
#endif
  }

  void Driver::unlock()
  {
#if defined( THOR_HLD_USART )
    ::HLD::getDriver( mChannel )->unlock();
#endif
  }

  void Driver::unlockFromISR()
  {
#if defined( THOR_HLD_USART )
    ::HLD::getDriver( mChannel )->unlockFromISR();
#endif
  }

}    // namespace Chimera::USART
