/********************************************************************************
 *  File Name:
 *    hld_gpio_chimera.cpp
 *
 *	 Description:
 *    Implementation of Chimera GPIO driver hooks
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/gpio>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/gpio>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/gpio/gpio_intf.hpp>
#include <Thor/lld/interface/gpio/gpio_detail.hpp>
#include <Thor/lld/interface/gpio/gpio_prv_data.hpp>

/*-------------------------------------------------------------------------------
Aliases
-------------------------------------------------------------------------------*/
namespace HLD = ::Thor::GPIO;
namespace LLD = ::Thor::LLD::GPIO;

/*-------------------------------------------------------------------------------
Constants
-------------------------------------------------------------------------------*/
// Each Chimera driver is associated with a pin, not the peripheral instance
static constexpr size_t NUM_DRIVERS = ::LLD::NUM_GPIO_PINS;

/*-------------------------------------------------------------------------------
Variables
-------------------------------------------------------------------------------*/
#if defined( THOR_HLD_GPIO )
static Chimera::GPIO::Driver s_raw_driver[ NUM_DRIVERS ];
#endif

namespace Chimera::GPIO::Backend
{
/*-------------------------------------------------------------------------------
Public Functions
-------------------------------------------------------------------------------*/
#if defined( THOR_HLD_GPIO )
  Chimera::Status_t initialize()
  {
    return Thor::GPIO::initialize();
  }


  Chimera::Status_t reset()
  {
    return Thor::GPIO::reset();
  }


  Chimera::GPIO::Driver_rPtr getDriver( const Port port, const Pin pin )
  {
    auto idx = ::LLD::getPinResourceIndex( port, pin );
    if ( idx < NUM_DRIVERS )
    {
      return &s_raw_driver[ idx ];
    }
    else
    {
      return nullptr;
    }
  }

#endif

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
  Driver::Driver() : mDriver( nullptr )
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
    mDriver = ::HLD::getDriver( pinInit.port, pinInit.pin );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->init( pinInit );
  }


  Chimera::Status_t Driver::init( const Chimera::GPIO::Port port, const uint8_t pin )
  {
    mDriver = ::HLD::getDriver( port, pin );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->init( port, pin );
  }


  Chimera::Status_t Driver::setMode( const Chimera::GPIO::Drive drive, const Chimera::GPIO::Pull pull )
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->setMode( drive, pull );
  }


  Chimera::Status_t Driver::setState( const Chimera::GPIO::State state )
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->setState( state );
  }


  Chimera::Status_t Driver::getState( Chimera::GPIO::State &state )
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->getState( state );
  }


  Chimera::Status_t Driver::toggle()
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->toggle();
  }


  Chimera::Status_t Driver::attachInterrupt( Chimera::Function::vGeneric &func, const Chimera::EXTI::EdgeTrigger trigger )
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->attachInterrupt( func, trigger );
  }


  void Driver::detachInterrupt()
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->detachInterrupt();
  }


  Chimera::EXTI::EventLine_t Driver::getInterruptLine()
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->getInterruptLine();
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
}    // namespace Chimera::GPIO
