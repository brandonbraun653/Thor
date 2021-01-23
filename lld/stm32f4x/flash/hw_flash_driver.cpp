/********************************************************************************
 *  File Name:
 *    hw_flash_driver.cpp
 *
 *  Description:
 *    Implements the Flash interface layer
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Thor Includes */
#include <Thor/lld/interface/inc/flash>

namespace Thor::LLD::FLASH
{
  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  static constexpr size_t MIN_LATENCY = 0;
  static constexpr size_t MAX_LATENCY = 15;

  /*-------------------------------------------------------------------------------
  Public Function
  -------------------------------------------------------------------------------*/
  RIndex_t getResourceIndex( const std::uintptr_t address )
  {
    if ( address == FLASH_BASE_ADDR )
    {
      return FLASH_RESOURCE_INDEX;
    }
    else
    {
      return INVALID_RESOURCE_INDEX;
    }
  }

  Chimera::Status_t setLatency( const size_t waitStates )
  {
    /*-------------------------------------------------
    Input Protection
    -------------------------------------------------*/
    if ( waitStates > MAX_LATENCY )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*------------------------------------------------
    Validate latency configuration since this is such
    a critical operating parameter.
    ------------------------------------------------*/
    uint32_t regVal = waitStates << ACR_LATENCY_Pos;
    LATENCY::set( FLASH_PERIPH, regVal );
    while ( LATENCY::get( FLASH_PERIPH ) != regVal )
    {
      continue;
    }

    return Chimera::Status::OK;
  }
}    // namespace Thor::LLD::FLASH
