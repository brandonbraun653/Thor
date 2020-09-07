/********************************************************************************
 *  File Name:
 *    peripheral_interface.cpp
 *
 *  Description:
 *    Implementation of common peripheral functions
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Thor Includes */
#include <Thor/lld/interface/common/peripheral_interface.hpp>
#include <Thor/lld/interface/gpio/gpio_intf.hpp>

namespace Thor::LLD
{
  RIndex_t getResourceIndex( const Chimera::Peripheral::Type type, const std::uintptr_t address )
  {
    switch ( type )
    {
      case Chimera::Peripheral::Type::PERIPH_GPIO:
        return GPIO::getResourceIndex( address );
        break;

      default:
        return INVALID_RESOURCE_INDEX;
        break;
    };
  }


}    // namespace Thor::LLD
