/********************************************************************************
 *  File Name:
 *    usb_intf.cpp
 *
 *  Description:
 *    LLD interface functions that are processor independent
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Chimera Includes */
#include <Chimera/common>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/usb/usb_detail.hpp>
#include <Thor/lld/interface/usb/usb_prv_data.hpp>
#include <Thor/lld/interface/usb/usb_types.hpp>
#include <Thor/lld/interface/usb/usb_intf.hpp>

#if defined( THOR_USB )
namespace Thor::LLD::USB
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  bool isSupported( const Chimera::USB::Channel channel )
  {
    switch ( channel )
    {
#if defined( STM32_USB1_PERIPH_AVAILABLE )
      case Chimera::USB::Channel::USB1:
        return true;
        break;
#endif

      default:
        return false;
        break;
    };
  }


  RIndex_t getResourceIndex( const Chimera::USB::Channel channel )
  {
    switch ( channel )
    {
#if defined( STM32_USB1_PERIPH_AVAILABLE )
      case Chimera::USB::Channel::USB0:
        return USB1_RESOURCE_INDEX;
        break;
#endif

      default:
        return INVALID_RESOURCE_INDEX;
        break;
    };
  }


  RIndex_t getResourceIndex( const std::uintptr_t address )
  {
#if defined( STM32_USB1_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( USB1_PERIPH ) )
    {
      return USB1_RESOURCE_INDEX;
    }
#endif

    return INVALID_RESOURCE_INDEX;
  }


  Chimera::USB::Channel getChannel( const std::uintptr_t address )
  {
#if defined( STM32_USB1_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( USB1_PERIPH ) )
    {
      return Chimera::USB::Channel::USB1;
    }
#endif

    return Chimera::USB::Channel::UNKNOWN;
  }


  bool attachDriverInstances( Driver *const driverList, const size_t numDrivers )
  {
    /*-------------------------------------------------
    Reject bad inputs
    -------------------------------------------------*/
    if ( !driverList || !numDrivers || ( numDrivers != NUM_USB_PERIPHS ) )
    {
      return false;
    }

    /*-------------------------------------------------
    Attach the drivers. The architecture of the LLD
    ensures the ordering and number is correct.
    -------------------------------------------------*/
    Chimera::Status_t result = Chimera::Status::OK;

#if defined( STM32_USB1_PERIPH_AVAILABLE )
    result |= driverList[ USB1_RESOURCE_INDEX ].attach( USB1_PERIPH );
#endif

    return result == Chimera::Status::OK;
  }
}    // namespace Thor::LLD::USB

#endif  /* THOR_LLD_USB */
