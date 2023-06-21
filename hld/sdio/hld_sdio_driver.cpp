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
  };

  /*---------------------------------------------------------------------------
  Variables
  ---------------------------------------------------------------------------*/
  static DeviceManager<Driver, Channel, NUM_DRIVERS>   s_raw_drivers;
  static DeviceManager<ThorImpl, Channel, NUM_DRIVERS> s_impl_drivers;
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
    return Chimera::Status::FAIL;
  }


  static Chimera::Status_t reset()
  {
    s_driver_initialized = Chimera::DRIVER_INITIALIZED_KEY;
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
}
#endif /* THOR_SDIO */
