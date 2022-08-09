/******************************************************************************
 *  File Name:
 *    hld_i2c_chimera.cpp
 *
 *  Description:
 *    I2C hooks into the Chimera driver
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/assert>
#include <Chimera/common>
#include <Chimera/i2c>
#include <Thor/cfg>
#include <Thor/i2c>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/i2c/i2c_detail.hpp>
#include <Thor/lld/interface/i2c/i2c_intf.hpp>
#include <Thor/lld/interface/i2c/i2c_prv_data.hpp>

/*-----------------------------------------------------------------------------
Aliases
-----------------------------------------------------------------------------*/
namespace HLD = ::Thor::I2C;
namespace LLD = ::Thor::LLD::I2C;

/*-----------------------------------------------------------------------------
Constants
-----------------------------------------------------------------------------*/
#if defined( THOR_HLD_I2C )
static constexpr size_t NUM_DRIVERS = ::LLD::NUM_I2C_PERIPHS;
#endif  /* THOR_HLD_I2C */

/*-----------------------------------------------------------------------------
Variables
-----------------------------------------------------------------------------*/
#if defined( THOR_HLD_I2C )
static Chimera::I2C::Driver s_raw_driver[ NUM_DRIVERS ];
#endif  /* THOR_HLD_I2C */

namespace Chimera::I2C::Backend
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  #if defined( THOR_HLD_I2C )
  Chimera::Status_t initialize()
  {
    return Thor::I2C::initialize();
  }


  Chimera::Status_t reset()
  {
    return Thor::I2C::reset();
  }


  Chimera::I2C::Driver_rPtr getDriver( const Channel channel )
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
#endif  // THOR_HLD_I2C

  Chimera::Status_t registerDriver( Chimera::I2C::Backend::DriverConfig &registry )
  {
#if defined( THOR_HLD_I2C )
    registry.isSupported = true;
    registry.getDriver   = getDriver;
    registry.initialize  = initialize;
    registry.reset       = reset;
    return Chimera::Status::OK;
#else
    memset( &registry, 0, sizeof( Chimera::I2C::Backend::DriverConfig ) );
    registry.isSupported = false;
    return Chimera::Status::NOT_SUPPORTED;
#endif    // THOR_HLD_I2C
  }

}  // namespace Chimera::I2C::Backend



namespace Chimera::I2C
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  Driver::Driver() : mDriver( nullptr )
  {
  }


  Driver::~Driver()
  {
  }

  /*---------------------------------------------------------------------------
  Interface: Hardware
  ---------------------------------------------------------------------------*/
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
    RT_DBG_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->close();
  }


  Chimera::Status_t Driver::read( const uint16_t address, void *const data, const size_t length )
  {
    RT_DBG_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->read( address, data, length );
  }


  Chimera::Status_t Driver::write( const uint16_t address, const void *const data, const size_t length )
  {
    RT_DBG_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->write( address, data, length );
  }


  Chimera::Status_t Driver::transfer( const uint16_t address, const void *const tx_data, void *const rx_data, const size_t length )
  {
    RT_DBG_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->transfer( address, tx_data, rx_data, length );
  }


  Chimera::Status_t Driver::stop()
  {
    RT_DBG_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->stop();
  }


  Chimera::Status_t Driver::start()
  {
    RT_DBG_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->start();
  }


  /*---------------------------------------------------------------------------
  Interface: Listener
  ---------------------------------------------------------------------------*/
  Chimera::Status_t Driver::registerListener( Chimera::Event::Actionable &listener, const size_t timeout,
                                              size_t &registrationID )
  {
    RT_DBG_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->registerListener( listener, timeout, registrationID );
  }


  Chimera::Status_t Driver::removeListener( const size_t registrationID, const size_t timeout )
  {
    RT_DBG_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->removeListener( registrationID, timeout );
  }

  /*---------------------------------------------------------------------------
  Interface: AsyncIO
  ---------------------------------------------------------------------------*/
  Chimera::Status_t Driver::await( const Chimera::Event::Trigger event, const size_t timeout )
  {
    RT_DBG_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->await( event, timeout );
  }


  Chimera::Status_t Driver::await( const Chimera::Event::Trigger event, Chimera::Thread::BinarySemaphore &notifier,
                                   const size_t timeout )
  {
    RT_DBG_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->await( event, notifier, timeout );
  }


  /*---------------------------------------------------------------------------
  Interface: Lockable
  ---------------------------------------------------------------------------*/
  void Driver::lock()
  {
    RT_DBG_ASSERT( mDriver );
    static_cast<::HLD::Driver_rPtr>( mDriver )->lock();
  }


  void Driver::lockFromISR()
  {
    RT_DBG_ASSERT( mDriver );
    static_cast<::HLD::Driver_rPtr>( mDriver )->lockFromISR();
  }


  bool Driver::try_lock_for( const size_t timeout )
  {
    RT_DBG_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->try_lock_for( timeout );
  }


  void Driver::unlock()
  {
    RT_DBG_ASSERT( mDriver );
    static_cast<::HLD::Driver_rPtr>( mDriver )->unlock();
  }


  void Driver::unlockFromISR()
  {
    RT_DBG_ASSERT( mDriver );
    static_cast<::HLD::Driver_rPtr>( mDriver )->unlockFromISR();
  }
}    // namespace Chimera::I2C
