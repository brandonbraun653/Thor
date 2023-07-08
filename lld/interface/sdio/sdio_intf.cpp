/******************************************************************************
 *  File Name:
 *    sdio_intf.cpp
 *
 *  Description:
 *    LLD interface functions that are processor independent
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Thor/cfg>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/inc/sdio>

#if defined( THOR_SDIO )
namespace Thor::LLD::SDIO
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static Driver s_sdio_drivers[ NUM_SDIO_PERIPHS ];

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    /*-------------------------------------------------------------------------
    Attach all the expected peripherals to the drivers
    -------------------------------------------------------------------------*/
    if ( !attachDriverInstances( s_sdio_drivers, ARRAY_COUNT( s_sdio_drivers ) ) )
    {
      return Chimera::Status::FAIL;
    }

    return Chimera::Status::OK;
  }


  bool isSupported( const Chimera::SDIO::Channel channel )
  {
    switch ( channel )
    {
#if defined( STM32_SDIO1_PERIPH_AVAILABLE )
      case Chimera::SDIO::Channel::SDIO1:
        return true;
#endif

#if defined( STM32_SDIO2_PERIPH_AVAILABLE )
      case Chimera::SDIO::Channel::SDIO2:
        return true;
#endif

      default:
        return false;
    }
  }


  Driver_rPtr getDriver( const Chimera::SDIO::Channel channel )
  {
    if ( auto idx = getResourceIndex( channel ); idx != INVALID_RESOURCE_INDEX )
    {
      return &s_sdio_drivers[ idx ];
    }
    else
    {
      return nullptr;
    }
  }


  RIndex_t getResourceIndex( const Chimera::SDIO::Channel channel )
  {
    switch ( channel )
    {
#if defined( STM32_SDIO1_PERIPH_AVAILABLE )
      case Chimera::SDIO::Channel::SDIO1:
        return SDIO1_RESOURCE_INDEX;
#endif
#if defined( STM32_SDIO2_PERIPH_AVAILABLE )
      case Chimera::SDIO::Channel::SDIO2:
        return SDIO2_RESOURCE_INDEX;
#endif

      default:
        return INVALID_RESOURCE_INDEX;
    }
  }


  RIndex_t getResourceIndex( const std::uintptr_t address )
  {
#if defined( STM32_SDIO1_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( SDIO1_PERIPH ) )
    {
      return SDIO1_RESOURCE_INDEX;
    }
#endif
#if defined( STM32_SDIO2_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( SDIO2_PERIPH ) )
    {
      return SDIO2_RESOURCE_INDEX;
    }
#endif

    return INVALID_RESOURCE_INDEX;
  }


  Chimera::SDIO::Channel getChannel( const std::uintptr_t address )
  {
#if defined( STM32_SDIO1_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( SDIO1_PERIPH ) )
    {
      return Chimera::SDIO::Channel::SDIO1;
    }
#endif
#if defined( STM32_SDIO2_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( SDIO2_PERIPH ) )
    {
      return Chimera::SDIO::Channel::SDIO2;
    }
#endif

    return Chimera::SDIO::Channel::UNKNOWN;
  }


  bool attachDriverInstances( Driver *const driverList, const size_t numDrivers )
  {
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if ( !driverList || ( numDrivers != NUM_SDIO_PERIPHS ) )
    {
      return false;
    }

    /*-------------------------------------------------------------------------
    Attach the peripherals to the drivers
    -------------------------------------------------------------------------*/
    Chimera::Status_t result = Chimera::Status::OK;

#if defined( STM32_SDIO1_PERIPH_AVAILABLE )
    result |= driverList[ SDIO1_RESOURCE_INDEX ].attach( SDIO1_PERIPH );
#endif
#if defined( STM32_SDIO2_PERIPH_AVAILABLE )
    result |= driverList[ SDIO2_RESOURCE_INDEX ].attach( SDIO2_PERIPH );
#endif

    return result == Chimera::Status::OK;
  }
}    // namespace Thor::LLD::SDIO
#endif /* THOR_SDIO */
