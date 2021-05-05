/********************************************************************************
 *  File Name:
 *    spi_intf.cpp
 *
 *  Description:
 *    LLD interface functions that are processor independent
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Chimera Includes */
#include <Chimera/common>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/inc/spi>

#if defined( THOR_LLD_SPI )
namespace Thor::LLD::SPI
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  bool isSupported( const Chimera::SPI::Channel channel )
  {
    switch( channel )
    {
#if defined( STM32_SPI1_PERIPH_AVAILABLE )
      case Chimera::SPI::Channel::SPI1:
        return true;
        break;
#endif
#if defined( STM32_SPI2_PERIPH_AVAILABLE )
      case Chimera::SPI::Channel::SPI2:
        return true;
        break;
#endif
#if defined( STM32_SPI3_PERIPH_AVAILABLE )
      case Chimera::SPI::Channel::SPI3:
        return true;
        break;
#endif

      default:
        return false;
        break;
    };
  }


  RIndex_t getResourceIndex( const Chimera::SPI::Channel channel )
  {
    switch( channel )
    {
#if defined( STM32_SPI1_PERIPH_AVAILABLE )
      case Chimera::SPI::Channel::SPI1:
        return SPI1_RESOURCE_INDEX;
        break;
#endif
#if defined( STM32_SPI2_PERIPH_AVAILABLE )
      case Chimera::SPI::Channel::SPI2:
        return SPI2_RESOURCE_INDEX;
        break;
#endif
#if defined( STM32_SPI3_PERIPH_AVAILABLE )
      case Chimera::SPI::Channel::SPI3:
        return SPI3_RESOURCE_INDEX;
        break;
#endif

      default:
        return INVALID_RESOURCE_INDEX;
        break;
    };
  }


  RIndex_t getResourceIndex( const std::uintptr_t address )
  {
#if defined( STM32_SPI1_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( SPI1_PERIPH ) )
    {
      return SPI1_RESOURCE_INDEX;
    }
#endif
#if defined( STM32_SPI2_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( SPI2_PERIPH ) )
    {
      return SPI2_RESOURCE_INDEX;
    }
#endif
#if defined( STM32_SPI3_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( SPI3_PERIPH ) )
    {
      return SPI3_RESOURCE_INDEX;
    }
#endif

    return INVALID_RESOURCE_INDEX;
  }


  Chimera::SPI::Channel getChannel( const std::uintptr_t address )
  {
#if defined( STM32_SPI1_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( SPI1_PERIPH ) )
    {
      return Chimera::SPI::Channel::SPI1;
    }
#endif
#if defined( STM32_SPI2_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( SPI2_PERIPH ) )
    {
      return Chimera::SPI::Channel::SPI2;
    }
#endif
#if defined( STM32_SPI3_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( SPI3_PERIPH ) )
    {
      return Chimera::SPI::Channel::SPI3;
    }
#endif

    return Chimera::SPI::Channel::NOT_SUPPORTED;
  }


  bool attachDriverInstances( Driver *const driverList, const size_t numDrivers )
  {
    /*-------------------------------------------------
    Reject bad inputs
    -------------------------------------------------*/
    if ( !driverList || !numDrivers || ( numDrivers != NUM_SPI_PERIPHS ) )
    {
      return false;
    }

    /*-------------------------------------------------
    Attach the drivers. The architecture of the LLD
    ensures the ordering and number is correct.
    -------------------------------------------------*/
    Chimera::Status_t result = Chimera::Status::OK;

#if defined( STM32_SPI1_PERIPH_AVAILABLE )
    result |= driverList[ SPI1_RESOURCE_INDEX ].attach( SPI1_PERIPH );
#endif
#if defined( STM32_SPI2_PERIPH_AVAILABLE )
    result |= driverList[ SPI2_RESOURCE_INDEX ].attach( SPI2_PERIPH );
#endif
#if defined( STM32_SPI3_PERIPH_AVAILABLE )
    result |= driverList[ SPI3_RESOURCE_INDEX ].attach( SPI3_PERIPH );
#endif

    return result == Chimera::Status::OK;
  }
}  // namespace Thor::LLD::SPI

#endif /* THOR_LLD_SPI */
