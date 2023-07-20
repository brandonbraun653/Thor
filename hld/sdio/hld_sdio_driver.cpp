/******************************************************************************
 *  File Name:
 *    hld_sdio_driver.cpp
 *
 *  Description:
 *    SDIO driver for Thor
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/peripheral>
#include <Chimera/sdio>
#include <Thor/cfg>
#include <Thor/lld/interface/inc/sdio>

#if defined( THOR_SDIO )
namespace Chimera::SDIO
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  namespace LLD = ::Thor::LLD::SDIO;

  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t NUM_DRIVERS = LLD::NUM_SDIO_PERIPHS;

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct ThorImpl
  {
    LLD::Driver_rPtr           lldriver;
    Chimera::SDIO::Driver_rPtr hldriver;
  };

  /*---------------------------------------------------------------------------
  Variables
  ---------------------------------------------------------------------------*/
  static DeviceManager<Driver, Channel, NUM_DRIVERS>   s_raw_drivers;
  static DeviceManager<ThorImpl, Channel, NUM_DRIVERS> s_impl_drivers;

  /*---------------------------------------------------------------------------
  Driver Implementation
  ---------------------------------------------------------------------------*/
  Driver::Driver()
  {
  }


  Driver::~Driver()
  {
  }


  Chimera::Status_t Driver::open( const Chimera::SDIO::HWConfig &init )
  {
    /*-------------------------------------------------------------------------
    Ensure the AsyncIO driver is ready
    -------------------------------------------------------------------------*/
    this->initAIO();

    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    auto idx = LLD::getResourceIndex( init.channel );
    if ( idx == ::Thor::LLD::INVALID_RESOURCE_INDEX )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------------------------------
    Bind the driver to the correct LLD instance
    -------------------------------------------------------------------------*/
    auto impl = s_impl_drivers.getOrCreate( init.channel );
    mImpl     = reinterpret_cast<void *>( impl );
    RT_DBG_ASSERT( impl );

    impl->lldriver = LLD::getDriver( init.channel );
    impl->hldriver = this;
    RT_DBG_ASSERT( impl->lldriver );

    /*-------------------------------------------------------------------------
    Initialize the LLD driver
    -------------------------------------------------------------------------*/
    return impl->lldriver->init();
  }


  Chimera::Status_t Driver::connect()
  {
    return Chimera::Status::FAIL;
  }


  void Driver::close()
  {
  }


  Chimera::Status_t Driver::write( const uint32_t address, const void *const buffer, const size_t length )
  {
    return Chimera::Status::FAIL;
  }


  Chimera::Status_t Driver::read( const uint32_t address, void *const buffer, const size_t length )
  {
    return Chimera::Status::FAIL;
  }


  Chimera::Status_t Driver::getCardStatus( CardStatus &status )
  {
    return Chimera::Status::FAIL;
  }


  Chimera::Status_t Driver::getCardIdentity( CardIdentity &identity )
  {
    return Chimera::Status::FAIL;
  }


  Chimera::Status_t Driver::getCardSpecificData( CardSpecificData &data )
  {
    return Chimera::Status::FAIL;
  }
}    // namespace Chimera::SDIO


namespace Chimera::SDIO::Backend
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static size_t s_driver_initialized;

  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  static Chimera::Status_t initialize()
  {
    /*-------------------------------------------------------------------------
    Prevent multiple initializations
    -------------------------------------------------------------------------*/
    if ( s_driver_initialized == Chimera::DRIVER_INITIALIZED_KEY )
    {
      return Chimera::Status::OK;
    }

    /*-------------------------------------------------------------------------
    Lock the init sequence and exit
    -------------------------------------------------------------------------*/
    s_driver_initialized = Chimera::DRIVER_INITIALIZED_KEY;
    return LLD::initialize();
  }


  static Chimera::Status_t reset()
  {
    s_driver_initialized = ~Chimera::DRIVER_INITIALIZED_KEY;
    return Chimera::Status::OK;
  }


  static Driver_rPtr getDriver( const Chimera::SDIO::Channel channel )
  {
    if ( !LLD::isSupported( channel ) )
    {
      return nullptr;
    }

    return s_raw_drivers.getOrCreate( channel );
  }


  Chimera::Status_t registerDriver( Chimera::SDIO::Backend::DriverConfig &registry )
  {
    registry.isSupported = true;
    registry.getDriver   = ::Chimera::SDIO::Backend::getDriver;
    registry.initialize  = ::Chimera::SDIO::Backend::initialize;
    registry.reset       = ::Chimera::SDIO::Backend::reset;
    return Chimera::Status::OK;
  }
}    // namespace Chimera::SDIO::Backend
#endif /* THOR_SDIO */
