/********************************************************************************
 *  File Name:
 *    hld_can_chimera.cpp
 *
 *  Description:
 *    CAN hooks into the Chimera driver
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/can>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/can>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/can/can_intf.hpp>
#include <Thor/lld/interface/can/can_detail.hpp>
#include <Thor/lld/interface/can/can_prv_data.hpp>

/*-------------------------------------------------------------------------------
Aliases
-------------------------------------------------------------------------------*/
namespace HLD = ::Thor::CAN;
namespace LLD = ::Thor::LLD::CAN;

/*-------------------------------------------------------------------------------
Constants
-------------------------------------------------------------------------------*/
  // Each Chimera driver is associated with a pin, not the peripheral instance
static constexpr size_t NUM_DRIVERS = ::LLD::NUM_CAN_PERIPHS;

/*-------------------------------------------------------------------------------
Variables
-------------------------------------------------------------------------------*/
static Chimera::CAN::Driver s_raw_driver[ NUM_DRIVERS ];
static Chimera::CAN::Driver_sPtr s_shared_driver[ NUM_DRIVERS ];


namespace Chimera::CAN::Backend
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    for ( size_t x = 0; x < NUM_DRIVERS; x++ )
    {
      s_shared_driver[ x ] = Chimera::CAN::Driver_sPtr( &s_raw_driver[ x ] );
    }

    return Thor::CAN::initialize();
  }


  Chimera::Status_t reset()
  {
    return Thor::CAN::reset();
  }


  Chimera::CAN::Driver_sPtr getDriver( const Channel channel )
  {
    auto idx = ::LLD::getResourceIndex( channel );
    return s_shared_driver[ idx ];
  }


  Chimera::Status_t registerDriver( Chimera::CAN::Backend::DriverConfig &registry )
  {
#if defined( THOR_HLD_CAN )
    registry.isSupported = true;
    registry.getDriver   = getDriver;
    registry.initialize  = initialize;
    registry.reset       = reset;
    return Chimera::Status::OK;
#else
    memset( &registry, 0, sizeof( Chimera::CAN::Backend::DriverConfig ) );
    registry.isSupported = false;
    return Chimera::Status::NOT_SUPPORTED;
#endif    // THOR_HLD_CAN
  }
}  // namespace Chimera::CAN


namespace Chimera::CAN
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
  Chimera::Status_t Driver::open( const DriverConfig &cfg )
  {
    mDriver = reinterpret_cast<void *>( ::LLD::getDriver( cfg.HWInit.channel ) );

    if( mDriver )
    {
      return static_cast<::HLD::Driver_rPtr>( mDriver )->open( cfg.HWInit.channel );
    }
    else
    {
      return Chimera::Status::NOT_SUPPORTED;
    }
  }


  Chimera::Status_t Driver::close()
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->close();
  }


  Chimera::CAN::CANStatus Driver::getStatus()
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->getStatus();
  }


  Chimera::Status_t Driver::send( const Chimera::CAN::BasicFrame &frame )
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->send( frame );
  }


  Chimera::Status_t Driver::receive( Chimera::CAN::BasicFrame &frame, const size_t timeout )
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->receive( frame, timeout );
  }


  Chimera::Status_t Driver::subscribe( const Chimera::CAN::Identifier_t id, Chimera::CAN::FrameCallback_t callback )
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->subscribe( id, callback );
  }


  Chimera::Status_t Driver::unsubscribe( const Chimera::CAN::Identifier_t id )
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->unsubscribe( id );
  }


  Chimera::Status_t Driver::filter( const Chimera::CAN::Filter *const list, const size_t size )
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->filter( list, size );
  }


  Chimera::Status_t Driver::flush( Chimera::CAN::BufferType buffer )
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->flush( buffer );
  }


  /*-------------------------------------------------
  Interface: Listener
  -------------------------------------------------*/
  Chimera::Status_t Driver::registerListener( Chimera::Event::Actionable &listener, const size_t timeout, size_t &registrationID )
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->registerListener( listener, timeout, registrationID );
  }


  Chimera::Status_t Driver::removeListener( const size_t registrationID, const size_t timeout )
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->removeListener( registrationID, timeout );
  }

  /*-------------------------------------------------
  Interface: AsyncIO
  -------------------------------------------------*/
  Chimera::Status_t Driver::await( const Chimera::Event::Trigger event, const size_t timeout )
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->await( event, timeout );
  }


  Chimera::Status_t Driver::await( const Chimera::Event::Trigger event, Chimera::Threading::BinarySemaphore &notifier, const size_t timeout )
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->await( event, notifier, timeout );
  }


  /*-------------------------------------------------
  Interface: Lockable
  -------------------------------------------------*/
  void Driver::lock()
  {
    static_cast<::HLD::Driver_rPtr>( mDriver )->lock();
  }


  void Driver::lockFromISR()
  {
    static_cast<::HLD::Driver_rPtr>( mDriver )->lockFromISR();
  }


  bool Driver::try_lock_for( const size_t timeout )
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->try_lock_for( timeout );
  }


  void Driver::unlock()
  {
    static_cast<::HLD::Driver_rPtr>( mDriver )->unlock();
  }


  void Driver::unlockFromISR()
  {
    static_cast<::HLD::Driver_rPtr>( mDriver )->unlockFromISR();
  }
}    // namespace Chimera::CAN
