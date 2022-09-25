/********************************************************************************
 *  File Name:
 *    hld_gpio_driver.cpp
 *
 *  Description:
 *    Implements the custom driver variant of the Thor GPIO interface.
 *
 *  2019-2022 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/constants>
#include <Chimera/common>
#include <Chimera/gpio>
#include <Chimera/peripheral>
#include <Chimera/thread>
#include <Chimera/utility>
#include <Thor/cfg>
#include <Thor/lld/interface/inc/gpio>
#include <cstring>
#include <cstddef>

#if defined( THOR_GPIO )
namespace Chimera::GPIO
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  namespace LLD = ::Thor::LLD::GPIO;

  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t NUM_DRIVERS = LLD::NUM_GPIO_PINS;

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct ThorImpl
  {
    LLD::Driver_rPtr lldriver;
    Alternate        mAlternate;
    Pin              mPin;
    Port             mPort;

    ThorImpl() :
        lldriver( nullptr ), mAlternate( Alternate::NONE ), mPin( std::numeric_limits<decltype( mPin )>::max() ),
        mPort( Port::UNKNOWN_PORT )
    {
    }
  };

  /*---------------------------------------------------------------------------
  Static Variables
  ---------------------------------------------------------------------------*/
  static DeviceManager<Driver, size_t, NUM_DRIVERS>   s_raw_drivers;
  static DeviceManager<ThorImpl, size_t, NUM_DRIVERS> s_impl_drivers;

  /*---------------------------------------------------------------------------
  Driver Class Implementation
  ---------------------------------------------------------------------------*/
  Driver::Driver()
  {
  }


  Driver::~Driver()
  {
  }


  Chimera::Status_t Driver::init( const Chimera::GPIO::PinInit &pinInit )
  {
    auto result = Chimera::Status::FAIL;

    /*-------------------------------------------------------------------------
    Ensure the driver is supported
    -------------------------------------------------------------------------*/
    auto idx = LLD::getPinResourceIndex( pinInit.port, pinInit.pin );
    auto imp = s_impl_drivers.getOrCreate( idx );

    if ( ( idx < NUM_DRIVERS ) && ( imp != nullptr ) )
    {
      /*-----------------------------------------------------------------------
      Fill out the implementation details
      -----------------------------------------------------------------------*/
      imp->lldriver   = LLD::getLLDriver( pinInit.port, pinInit.pin );
      imp->mAlternate = pinInit.alternate;
      imp->mPin       = pinInit.pin;
      imp->mPort      = pinInit.port;

      RT_DBG_ASSERT( imp->lldriver );
      result = Chimera::Status::OK;

      /*-----------------------------------------------------------------------
      Enable the peripheral clock
      -----------------------------------------------------------------------*/
      imp->lldriver->clockEnable();

      /*-----------------------------------------------------------------------
      Update the implementation pointer, needed for the following calls.
      -----------------------------------------------------------------------*/
      mImpl = reinterpret_cast<void *>( imp );

      /*-----------------------------------------------------------------------
      Configure the basic IO mode settings
      -----------------------------------------------------------------------*/
      result |= setMode( pinInit.drive, pinInit.pull );
      result |= setState( pinInit.state );
    }

    return result;
  }


  Chimera::Status_t Driver::init( const Chimera::GPIO::Port port, const uint8_t pin )
  {
    Chimera::GPIO::PinInit cfg;
    cfg.clear();

    cfg.port      = port;
    cfg.pin       = pin;
    cfg.alternate = Chimera::GPIO::Alternate::NONE;
    cfg.drive     = Chimera::GPIO::Drive::INPUT;
    cfg.state     = Chimera::GPIO::State::LOW;
    cfg.pull      = Chimera::GPIO::Pull::PULL_UP;

    return init( cfg );
  }


  Chimera::Status_t Driver::setMode( const Chimera::GPIO::Drive drive, const Chimera::GPIO::Pull pull )
  {
    /*-------------------------------------------------------------------------
    Grab object references
    -------------------------------------------------------------------------*/
    RT_DBG_ASSERT( mImpl );
    auto result = Chimera::Status::OK;
    auto impl   = reinterpret_cast<ThorImpl *>( mImpl );
    auto driver = impl->lldriver;

    /*-------------------------------------------------------------------------
    Set up the hardware to implement the desired mode
    -------------------------------------------------------------------------*/
    result |= driver->driveSet( impl->mPin, drive );
    result |= driver->pullSet( impl->mPin, pull );
    result |= driver->speedSet( impl->mPin, Thor::LLD::GPIO::Speed::HIGH );

    /*-------------------------------------------------------------------------
    Configure the alternate function options
    -------------------------------------------------------------------------*/
    if ( ( drive == Chimera::GPIO::Drive::ALTERNATE_OPEN_DRAIN ) || ( drive == Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL ) )
    {
      result |= driver->alternateFunctionSet( impl->mPin, impl->mAlternate );
    }

    return result;
  }


  Chimera::Status_t Driver::setState( const Chimera::GPIO::State state )
  {
    /*-------------------------------------------------------------------------
    Grab object references
    -------------------------------------------------------------------------*/
    RT_DBG_ASSERT( mImpl );
    auto impl   = reinterpret_cast<ThorImpl *>( mImpl );
    auto driver = impl->lldriver;

    /*-------------------------------------------------------------------------
    Invoke the hardware driver
    -------------------------------------------------------------------------*/
    return driver->write( impl->mPin, state );
  }


  Chimera::Status_t Driver::getState( Chimera::GPIO::State &state )
  {
    /*-------------------------------------------------------------------------
    Grab object references
    -------------------------------------------------------------------------*/
    RT_DBG_ASSERT( mImpl );
    auto impl   = reinterpret_cast<ThorImpl *>( mImpl );
    auto driver = impl->lldriver;

    /*-------------------------------------------------------------------------
    Invoke the hardware driver
    -------------------------------------------------------------------------*/
    state = driver->read( impl->mPin );
    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::toggle()
  {
    /*-------------------------------------------------------------------------
    Grab object references
    -------------------------------------------------------------------------*/
    RT_DBG_ASSERT( mImpl );
    auto impl   = reinterpret_cast<ThorImpl *>( mImpl );
    auto driver = impl->lldriver;

    /*-------------------------------------------------------------------------
    Invoke the hardware driver
    -------------------------------------------------------------------------*/
    auto state = driver->read( impl->mPin );
    state      = ( state == State::HIGH ) ? State::LOW : State::HIGH;
    return setState( state );
  }


  Chimera::Status_t Driver::attachInterrupt( Chimera::Function::vGeneric &func, const Chimera::EXTI::EdgeTrigger trigger )
  {
    /*-------------------------------------------------------------------------
    Grab object references
    -------------------------------------------------------------------------*/
    RT_DBG_ASSERT( mImpl );
    auto impl   = reinterpret_cast<ThorImpl *>( mImpl );
    auto driver = impl->lldriver;

    /*-------------------------------------------------------------------------
    Invoke the hardware driver
    -------------------------------------------------------------------------*/
    return driver->attachInterrupt( impl->mPin, func, trigger );
  }


  void Driver::detachInterrupt()
  {
    /*-------------------------------------------------------------------------
    Grab object references
    -------------------------------------------------------------------------*/
    RT_DBG_ASSERT( mImpl );
    auto impl   = reinterpret_cast<ThorImpl *>( mImpl );
    auto driver = impl->lldriver;

    /*-------------------------------------------------------------------------
    Invoke the hardware driver
    -------------------------------------------------------------------------*/
    return driver->detachInterrupt( impl->mPin );
  }


  Chimera::EXTI::EventLine_t Driver::getInterruptLine()
  {
    /*-------------------------------------------------------------------------
    Grab object references
    -------------------------------------------------------------------------*/
    RT_DBG_ASSERT( mImpl );
    auto impl = reinterpret_cast<ThorImpl *>( mImpl );

    /*-------------------------------------------------------------------------
    Invoke the hardware driver
    -------------------------------------------------------------------------*/
    return LLD::findEventLine( impl->mPort, impl->mPin );
  }
}    // namespace Chimera::GPIO


namespace Chimera::GPIO::Backend
{

  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  static Chimera::Status_t initialize()
  {
    return Thor::LLD::GPIO::initialize();
  }


  static Chimera::Status_t reset()
  {
    return Chimera::Status::OK;
  }


  static Driver_rPtr getDriver( const Chimera::GPIO::Port port, const Chimera::GPIO::Pin pin )
  {
    auto idx = LLD::getPinResourceIndex( port, pin );
    if ( idx == ::Thor::LLD::INVALID_RESOURCE_INDEX )
    {
      return nullptr;
    }

    return s_raw_drivers.getOrCreate( idx );
  }

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  Chimera::Status_t registerDriver( Chimera::GPIO::Backend::DriverConfig &registry )
  {
    registry.isSupported = true;
    registry.getDriver   = ::Chimera::GPIO::Backend::getDriver;
    registry.initialize  = ::Chimera::GPIO::Backend::initialize;
    registry.reset       = ::Chimera::GPIO::Backend::reset;
    return Chimera::Status::OK;
  }
}    // namespace Chimera::GPIO::Backend
#endif /* THOR_HLD_GPIO */
