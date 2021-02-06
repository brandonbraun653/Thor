/********************************************************************************
 *  File Name:
 *    hld_can_chimera.cpp
 *
 *  Description:
 *    CAN hooks into the Chimera driver
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Chimera Includes */
#include <Chimera/assert>
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
#if defined( THOR_HLD_CAN )
static constexpr size_t NUM_DRIVERS = ::LLD::NUM_CAN_PERIPHS;
#endif  /* THOR_HLD_CAN */

/*-------------------------------------------------------------------------------
Variables
-------------------------------------------------------------------------------*/
#if defined( THOR_HLD_CAN )
static Chimera::CAN::Driver s_raw_driver[ NUM_DRIVERS ];
#endif  /* THOR_HLD_CAN */

namespace Chimera::CAN::Backend
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
#if defined( THOR_HLD_CAN )
  Chimera::Status_t initialize()
  {
    return Thor::CAN::initialize();
  }


  Chimera::Status_t reset()
  {
    return Thor::CAN::reset();
  }


  Chimera::CAN::Driver_rPtr getDriver( const Channel channel )
  {
    auto idx = ::LLD::getResourceIndex( channel );
    if ( idx < NUM_DRIVERS )
    {
      return &s_raw_driver[ idx ];
    }
    else
    {
      return nullptr;
    }
  }
#endif  // THOR_HLD_CAN

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
}    // namespace Chimera::CAN::Backend


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
    mDriver = reinterpret_cast<void *>( ::HLD::getDriver( cfg.HWInit.channel ) );

    if ( mDriver )
    {
      return static_cast<::HLD::Driver_rPtr>( mDriver )->open( cfg );
    }
    else
    {
      return Chimera::Status::NOT_SUPPORTED;
    }
  }


  Chimera::Status_t Driver::close()
  {
    RT_HARD_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->close();
  }


  Chimera::CAN::CANStatus Driver::getStatus()
  {
    RT_HARD_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->getStatus();
  }


  Chimera::Status_t Driver::send( const Chimera::CAN::BasicFrame &frame )
  {
    RT_HARD_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->send( frame );
  }


  Chimera::Status_t Driver::receive( Chimera::CAN::BasicFrame &frame )
  {
    RT_HARD_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->receive( frame );
  }


  Chimera::Status_t Driver::filter( const Chimera::CAN::Filter *const list, const size_t size )
  {
    RT_HARD_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->filter( list, size );
  }


  Chimera::Status_t Driver::flush( Chimera::CAN::BufferType buffer )
  {
    RT_HARD_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->flush( buffer );
  }


  size_t Driver::available()
  {
    RT_HARD_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->available();
  }


  /*-------------------------------------------------
  Interface: Listener
  -------------------------------------------------*/
  Chimera::Status_t Driver::registerListener( Chimera::Event::Actionable &listener, const size_t timeout,
                                              size_t &registrationID )
  {
    RT_HARD_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->registerListener( listener, timeout, registrationID );
  }


  Chimera::Status_t Driver::removeListener( const size_t registrationID, const size_t timeout )
  {
    RT_HARD_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->removeListener( registrationID, timeout );
  }

  /*-------------------------------------------------
  Interface: AsyncIO
  -------------------------------------------------*/
  Chimera::Status_t Driver::await( const Chimera::Event::Trigger event, const size_t timeout )
  {
    RT_HARD_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->await( event, timeout );
  }


  Chimera::Status_t Driver::await( const Chimera::Event::Trigger event, Chimera::Threading::BinarySemaphore &notifier,
                                   const size_t timeout )
  {
    RT_HARD_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->await( event, notifier, timeout );
  }


  /*-------------------------------------------------
  Interface: Lockable
  -------------------------------------------------*/
  void Driver::lock()
  {
    RT_HARD_ASSERT( mDriver );
    static_cast<::HLD::Driver_rPtr>( mDriver )->lock();
  }


  void Driver::lockFromISR()
  {
    RT_HARD_ASSERT( mDriver );
    static_cast<::HLD::Driver_rPtr>( mDriver )->lockFromISR();
  }


  bool Driver::try_lock_for( const size_t timeout )
  {
    RT_HARD_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->try_lock_for( timeout );
  }


  void Driver::unlock()
  {
    RT_HARD_ASSERT( mDriver );
    static_cast<::HLD::Driver_rPtr>( mDriver )->unlock();
  }


  void Driver::unlockFromISR()
  {
    RT_HARD_ASSERT( mDriver );
    static_cast<::HLD::Driver_rPtr>( mDriver )->unlockFromISR();
  }
}    // namespace Chimera::CAN
