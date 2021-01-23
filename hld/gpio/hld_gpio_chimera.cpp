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
#if defined( THOR_HLD_GPIO )
static constexpr size_t NUM_DRIVERS = ::LLD::NUM_GPIO_PINS;
#endif

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
#if defined( THOR_HLD_GPIO )
    mDriver = ::HLD::getDriver( pinInit.port, pinInit.pin );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->init( pinInit );
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  Chimera::Status_t Driver::init( const Chimera::GPIO::Port port, const uint8_t pin )
  {
#if defined( THOR_HLD_GPIO )
    mDriver = ::HLD::getDriver( port, pin );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->init( port, pin );
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  Chimera::Status_t Driver::setMode( const Chimera::GPIO::Drive drive, const Chimera::GPIO::Pull pull )
  {
#if defined( THOR_HLD_GPIO )
    return static_cast<::HLD::Driver_rPtr>( mDriver )->setMode( drive, pull );
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  Chimera::Status_t Driver::setState( const Chimera::GPIO::State state )
  {
#if defined( THOR_HLD_GPIO )
    return static_cast<::HLD::Driver_rPtr>( mDriver )->setState( state );
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  Chimera::Status_t Driver::getState( Chimera::GPIO::State &state )
  {
#if defined( THOR_HLD_GPIO )
    return static_cast<::HLD::Driver_rPtr>( mDriver )->getState( state );
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  Chimera::Status_t Driver::toggle()
  {
#if defined( THOR_HLD_GPIO )
    return static_cast<::HLD::Driver_rPtr>( mDriver )->toggle();
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  Chimera::Status_t Driver::attachInterrupt( Chimera::Function::vGeneric &func, const Chimera::EXTI::EdgeTrigger trigger )
  {
#if defined( THOR_HLD_GPIO )
    return static_cast<::HLD::Driver_rPtr>( mDriver )->attachInterrupt( func, trigger );
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  void Driver::detachInterrupt()
  {
#if defined( THOR_HLD_GPIO )
    static_cast<::HLD::Driver_rPtr>( mDriver )->detachInterrupt();
#endif
  }


  Chimera::EXTI::EventLine_t Driver::getInterruptLine()
  {
#if defined( THOR_HLD_GPIO )
    return static_cast<::HLD::Driver_rPtr>( mDriver )->getInterruptLine();
#else
    return 0;
#endif
  }

  /*-------------------------------------------------
  Interface: Lockable
  -------------------------------------------------*/
  void Driver::lock()
  {
#if defined( THOR_HLD_GPIO )
    static_cast<::HLD::Driver_rPtr>( mDriver )->lock();
#endif
  }


  void Driver::lockFromISR()
  {
#if defined( THOR_HLD_GPIO )
    static_cast<::HLD::Driver_rPtr>( mDriver )->lockFromISR();
#endif
  }


  bool Driver::try_lock_for( const size_t timeout )
  {
#if defined( THOR_HLD_GPIO )
    return static_cast<::HLD::Driver_rPtr>( mDriver )->try_lock_for( timeout );
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  void Driver::unlock()
  {
#if defined( THOR_HLD_GPIO )
    static_cast<::HLD::Driver_rPtr>( mDriver )->unlock();
#endif
  }


  void Driver::unlockFromISR()
  {
#if defined( THOR_HLD_GPIO )
    static_cast<::HLD::Driver_rPtr>( mDriver )->unlockFromISR();
#endif
  }
}    // namespace Chimera::GPIO
