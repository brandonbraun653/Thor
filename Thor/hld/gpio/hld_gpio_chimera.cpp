/********************************************************************************
 *  File Name:
 *    hld_gpio_chimera.cpp
 *
 *	 Description:
 *    Implementation of Chimera GPIO driver hooks
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* STL Includes */
#include <memory>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/gpio>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/gpio>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/gpio/gpio_intf.hpp>
#include <Thor/lld/interface/gpio/gpio_detail.hpp>

/*-------------------------------------------------------------------------------
Aliases
-------------------------------------------------------------------------------*/
namespace HLD = ::Thor::GPIO;
namespace LLD = ::Thor::LLD::GPIO;

/*-------------------------------------------------------------------------------
Constants
-------------------------------------------------------------------------------*/
static constexpr size_t NUM_DRIVERS = ::LLD::NUM_GPIO_PERIPHS;

/*-------------------------------------------------------------------------------
Variables
-------------------------------------------------------------------------------*/
static Chimera::GPIO::Driver s_raw_driver[ NUM_DRIVERS ];
static Chimera::GPIO::Driver_sPtr s_shared_driver[ NUM_DRIVERS ];


namespace Chimera::GPIO::Backend
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    return Thor::GPIO::initialize();
  }


  Chimera::Status_t reset()
  {
    return Thor::GPIO::reset();
  }


  Chimera::GPIO::Driver_sPtr getDriver( const Port port, const Pin pin )
  {
    return Thor::GPIO::getDriver( port, pin );
  }


  Chimera::Status_t registerDriver( Chimera::GPIO::Backend::DriverConfig &registry )
  {
#if defined( THOR_HLD_GPIO )
    registry.isSupported = true;
    registry.getDriver   = getDriver;
    registry.initialize  = initialize;
    registry.reset       = reset;
    return Chimera::Status::OK;
#else
    memset( &registry, 0, sizeof( Chimera::GPIO::Backend::DriverConfig ) );
    registry.isSupported = false;
    return Chimera::Status::NOT_SUPPORTED;
#endif    // THOR_HLD_GPIO
  }
}    // namespace Chimera::GPIO::Backend


namespace Chimera::GPIO
{
  /*-------------------------------------------------------------------------------
  Driver Implementation
  -------------------------------------------------------------------------------*/
  Driver::Driver() : mPin( std::numeric_limits<decltype( mPin )>::max() ), mPort( Port::NUM_OPTIONS )
  {
  }


  Driver::~Driver()
  {
  }


  /*-------------------------------------------------
  Interface: Hardware
  -------------------------------------------------*/
  Chimera::Status_t Driver::init( const Chimera::GPIO::PinInit &pinInit )
  {
    return ::HLD::getDriver( mPort, mPin )->init( pinInit );
  }


  Chimera::Status_t Driver::init( const Chimera::GPIO::Port port, const uint8_t pin )
  {
    return ::HLD::getDriver( mPort, mPin )->init( port, pin );
  }


  Chimera::Status_t Driver::setMode( const Chimera::GPIO::Drive drive, const Chimera::GPIO::Pull pull )
  {
    return ::HLD::getDriver( mPort, mPin )->setMode( drive, pull );
  }


  Chimera::Status_t Driver::setState( const Chimera::GPIO::State state )
  {
    return ::HLD::getDriver( mPort, mPin )->setState( state );
  }


  Chimera::Status_t Driver::getState( Chimera::GPIO::State &state )
  {
    return ::HLD::getDriver( mPort, mPin )->getState( state );
  }


  Chimera::Status_t Driver::toggle()
  {
    return ::HLD::getDriver( mPort, mPin )->toggle();
  }


  /*-------------------------------------------------
  Interface: Lockable
  -------------------------------------------------*/
  void Driver::lock()
  {
    ::HLD::getDriver( mPort, mPin )->lock();
  }

  void Driver::lockFromISR()
  {
    ::HLD::getDriver( mPort, mPin )->lockFromISR();
  }

  bool Driver::try_lock_for( const size_t timeout )
  {
    return ::HLD::getDriver( mPort, mPin )->try_lock_for( timeout );
  }

  void Driver::unlock()
  {
    ::HLD::getDriver( mPort, mPin )->unlock();
  }

  void Driver::unlockFromISR()
  {
    ::HLD::getDriver( mPort, mPin )->unlockFromISR();
  }
}    // namespace Chimera::GPIO
