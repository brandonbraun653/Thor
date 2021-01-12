/********************************************************************************
 *  File Name:
 *    hld_usb_chimera.cpp
 *
 *	 Description:
 *    Implementation of Chimera USB driver hooks
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/usb>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/usb>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/usb/usb_intf.hpp>
#include <Thor/lld/interface/usb/usb_detail.hpp>
#include <Thor/lld/interface/usb/usb_prv_data.hpp>

/*-------------------------------------------------------------------------------
Aliases
-------------------------------------------------------------------------------*/
namespace HLD = ::Thor::USB;
namespace LLD = ::Thor::LLD::USB;

/*-------------------------------------------------------------------------------
Constants
-------------------------------------------------------------------------------*/
static constexpr size_t NUM_DRIVERS = ::LLD::NUM_USB_PERIPHS;

/*-------------------------------------------------------------------------------
Variables
-------------------------------------------------------------------------------*/
#if defined( THOR_HLD_USB )
static Chimera::USB::Driver s_raw_driver[ NUM_DRIVERS ];
#endif


namespace Chimera::USB::Backend
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
#if defined( THOR_HLD_USB )
  Chimera::Status_t initialize()
  {
    return Thor::USB::initialize();
  }


  Chimera::Status_t reset()
  {
    return Thor::USB::reset();
  }


  Chimera::USB::Driver_rPtr getDriver( const Channel ch )
  {
    auto idx = ::LLD::getResourceIndex( ch );
    if( idx < NUM_DRIVERS )
    {
      return &s_raw_driver[ idx ];
    }
    else
    {
      return nullptr;
    }
  }
#endif  // THOR_HLD_USB

  Chimera::Status_t registerDriver( Chimera::USB::Backend::DriverConfig &registry )
  {
#if defined( THOR_HLD_USB )
    registry.isSupported = true;
    registry.getDriver   = getDriver;
    registry.initialize  = initialize;
    registry.reset       = reset;
    return Chimera::Status::OK;
#else
    memset( &registry, 0, sizeof( Chimera::USB::Backend::DriverConfig ) );
    registry.isSupported = false;
    return Chimera::Status::NOT_SUPPORTED;
#endif    // THOR_HLD_USB
  }
}    // namespace Chimera::USB::Backend


namespace Chimera::USB
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
  Chimera::Status_t Driver::open( const PeriphConfig &cfg )
  {
    mDriver = reinterpret_cast<void *>( ::HLD::getDriver( cfg.channel ) );

    if ( mDriver )
    {
      return static_cast<::HLD::Driver_rPtr>( mDriver )->open( cfg );
    }
    else
    {
      return Chimera::Status::NOT_SUPPORTED;
    }
  }


  void Driver::close()
  {
    static_cast<::HLD::Driver_rPtr>( mDriver )->close();
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
}    // namespace Chimera::USB
