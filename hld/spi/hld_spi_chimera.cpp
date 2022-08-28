/********************************************************************************
 *  File Name:
 *    hld_spi_chimera.cpp
 *
 *	 Description:
 *    Implementation of Chimera SPI driver hooks
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */
#include <Chimera/assert>
#include <Chimera/common>
#include <Chimera/spi>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/spi>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/inc/spi>

/*-------------------------------------------------------------------------------
Aliases
-------------------------------------------------------------------------------*/
namespace HLD = ::Thor::SPI;
namespace LLD = ::Thor::LLD::SPI;

/*-------------------------------------------------------------------------------
Constants
-------------------------------------------------------------------------------*/
#if defined( THOR_SPI )
static constexpr size_t NUM_DRIVERS = ::LLD::NUM_SPI_PERIPHS;
#endif

/*-------------------------------------------------------------------------------
Variables
-------------------------------------------------------------------------------*/
#if defined( THOR_SPI )
static Chimera::SPI::Driver s_raw_driver[ NUM_DRIVERS ];
#endif

namespace Chimera::SPI::Backend
{
/*-------------------------------------------------------------------------------
Public Functions
-------------------------------------------------------------------------------*/
#if defined( THOR_SPI )
  Chimera::Status_t initialize()
  {
#if defined( THOR_SPI )
    return Thor::SPI::initialize();
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  Chimera::Status_t reset()
  {
#if defined( THOR_SPI )
    return Thor::SPI::reset();
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  Driver_rPtr getDriver( const Channel channel )
  {
    auto idx = ::LLD::getResourceIndex( channel );
    if ( idx < NUM_DRIVERS )
    {
#if defined( THOR_SPI )
      return &s_raw_driver[ idx ];
#else
      return Chimera::Status::NOT_SUPPORTED;
#endif
    }
    else
    {
#if defined( THOR_SPI )
      return nullptr;
#else
      return Chimera::Status::NOT_SUPPORTED;
#endif
    }
  }
#endif    // THOR_HLD_SPI

  Chimera::Status_t registerDriver( Chimera::SPI::Backend::DriverConfig &registry )
  {
#if defined( THOR_SPI )
    registry.isSupported = true;
    registry.getDriver   = getDriver;
    registry.initialize  = initialize;
    registry.reset       = reset;
#if defined( THOR_SPI )
    return Chimera::Status::OK;
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
#else
    memset( &registry, 0, sizeof( Chimera::SPI::Backend::DriverConfig ) );
    registry.isSupported = false;
#if defined( THOR_SPI )
    return Chimera::Status::NOT_SUPPORTED;
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
#endif /* THOR_HLD_SPI */
  }
}    // namespace Chimera::SPI::Backend


namespace Chimera::SPI
{
  /*-------------------------------------------------------------------------------
  Driver Implementation
  -------------------------------------------------------------------------------*/
  Driver::Driver() : mDriver( nullptr )
  {
  }

  Driver::~Driver()
  {
  }


  /*-------------------------------------------------
  Interface: Hardware
  -------------------------------------------------*/
  Chimera::Status_t Driver::init( const Chimera::SPI::DriverConfig &setupStruct )
  {
#if defined( THOR_SPI )
    mDriver = ::HLD::getDriver( setupStruct.HWInit.hwChannel );

    RT_DBG_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->init( setupStruct );
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  Chimera::SPI::DriverConfig Driver::getInit()
  {
#if defined( THOR_SPI )
    RT_DBG_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->getInit();
#else
    return {};
#endif
  }


  Chimera::Status_t Driver::deInit()
  {
#if defined( THOR_SPI )
    RT_DBG_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->deInit();
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  Chimera::Status_t Driver::setChipSelect( const Chimera::GPIO::State value )
  {
#if defined( THOR_SPI )
    RT_DBG_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->setChipSelect( value );
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  Chimera::Status_t Driver::setChipSelectControlMode( const Chimera::SPI::CSMode mode )
  {
#if defined( THOR_SPI )
    RT_DBG_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->setChipSelectControlMode( mode );
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  Chimera::Status_t Driver::writeBytes( const void *const txBuffer, const size_t length )
  {
#if defined( THOR_SPI )
    RT_DBG_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->writeBytes( txBuffer, length );
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  Chimera::Status_t Driver::readBytes( void *const rxBuffer, const size_t length )
  {
#if defined( THOR_SPI )
    RT_DBG_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->readBytes( rxBuffer, length );
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  Chimera::Status_t Driver::readWriteBytes( const void *const txBuffer, void *const rxBuffer, const size_t length )
  {
#if defined( THOR_SPI )
    RT_DBG_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->readWriteBytes( txBuffer, rxBuffer, length );
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  Chimera::Status_t Driver::setPeripheralMode( const Chimera::Hardware::PeripheralMode mode )
  {
#if defined( THOR_SPI )
    RT_DBG_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->setPeripheralMode( mode );
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  Chimera::Status_t Driver::setClockFrequency( const size_t freq, const size_t tolerance )
  {
#if defined( THOR_SPI )
    RT_DBG_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->setClockFrequency( freq, tolerance );
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  size_t Driver::getClockFrequency()
  {
#if defined( THOR_SPI )
    RT_DBG_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->getClockFrequency();
#else
    return 0;
#endif
  }


  /*-------------------------------------------------
  Interface: Listener
  -------------------------------------------------*/
  Chimera::Status_t Driver::registerListener( Chimera::Event::Actionable &listener, const size_t timeout,
                                              size_t &registrationID )
  {
#if defined( THOR_SPI )
    RT_DBG_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->registerListener( listener, timeout, registrationID );
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  Chimera::Status_t Driver::removeListener( const size_t registrationID, const size_t timeout )
  {
#if defined( THOR_SPI )
    RT_DBG_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->removeListener( registrationID, timeout );
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  /*-------------------------------------------------
  Interface: AsyncIO
  -------------------------------------------------*/
  Chimera::Status_t Driver::await( const Chimera::Event::Trigger event, const size_t timeout )
  {
#if defined( THOR_SPI )
    RT_DBG_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->await( event, timeout );
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  Chimera::Status_t Driver::await( const Chimera::Event::Trigger event, Chimera::Thread::BinarySemaphore &notifier,
                                   const size_t timeout )
  {
#if defined( THOR_SPI )
    RT_DBG_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->await( event, notifier, timeout );
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  /*-------------------------------------------------
  Interface: Lockable
  -------------------------------------------------*/
  void Driver::lock()
  {
#if defined( THOR_SPI )
    RT_DBG_ASSERT( mDriver );
    static_cast<::HLD::Driver_rPtr>( mDriver )->lock();
#endif
  }


  void Driver::lockFromISR()
  {
#if defined( THOR_SPI )
    RT_DBG_ASSERT( mDriver );
    static_cast<::HLD::Driver_rPtr>( mDriver )->lockFromISR();
#endif
  }


  bool Driver::try_lock_for( const size_t timeout )
  {
#if defined( THOR_SPI )
    RT_DBG_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->try_lock_for( timeout );
#else
    return false;
#endif
  }


  void Driver::unlock()
  {
#if defined( THOR_SPI )
    RT_DBG_ASSERT( mDriver );
    static_cast<::HLD::Driver_rPtr>( mDriver )->unlock();
#endif
  }


  void Driver::unlockFromISR()
  {
#if defined( THOR_SPI )
    RT_DBG_ASSERT( mDriver );
    static_cast<::HLD::Driver_rPtr>( mDriver )->unlockFromISR();
#endif
  }

}    // namespace Chimera::SPI
