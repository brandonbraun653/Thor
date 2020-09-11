/********************************************************************************
 *  File Name:
 *    spi_intf.cpp
 *
 *  Description:
 *    LLD interface functions that are processor independent
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Chimera Includes */
#include <Chimera/common>

/* Thor Includes */
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/spi/spi_detail.hpp>
#include <Thor/lld/interface/spi/spi_prv_data.hpp>
#include <Thor/lld/interface/spi/spi_types.hpp>
#include <Thor/lld/interface/spi/spi_intf.hpp>

namespace Thor::LLD::SPI
{

  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  bool isSupported( const Chimera::SPI::Channel channel )
  {
    return false;
  }


  RIndex_t getResourceIndex( const Chimera::SPI::Channel channel )
  {
    return INVALID_RESOURCE_INDEX;
  }


  RIndex_t getResourceIndex( const std::uintptr_t address )
  {
    return INVALID_RESOURCE_INDEX;
  }


  Chimera::SPI::Channel getChannel( const std::uintptr_t address )
  {
    return Chimera::SPI::Channel::NOT_SUPPORTED;
  }


  bool attachDriverInstances( Driver *const driverList, const size_t numDrivers )
  {
    return false;
  }
}  // namespace Thor::LLD::SPI
