/********************************************************************************
 *  File Name:
 *    gpio_intf.cpp
 *
 *  Description:
 *    LLD interface functions that are processor independent
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Thor Includes */
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/gpio/gpio_prv_data.hpp>
#include <Thor/lld/interface/gpio/gpio_types.hpp>

namespace Thor::LLD::GPIO
{
  /* clang-format off */
  static const uint8_t s_port_index[ DRIVER_MAX_PORTS ] = {
    GPIOA_RESOURCE_INDEX,
    GPIOB_RESOURCE_INDEX,
    GPIOC_RESOURCE_INDEX,
    GPIOD_RESOURCE_INDEX,
    GPIOE_RESOURCE_INDEX,
    GPIOF_RESOURCE_INDEX,
    GPIOG_RESOURCE_INDEX,
    GPIOH_RESOURCE_INDEX,
    GPIOI_RESOURCE_INDEX,
    GPIOJ_RESOURCE_INDEX,
    GPIOK_RESOURCE_INDEX,
    GPIOL_RESOURCE_INDEX
  };

  static const uint8_t s_pin_count[ DRIVER_MAX_PORTS ] = {
    GPIOA_NUM_PINS,
    GPIOB_NUM_PINS,
    GPIOC_NUM_PINS,
    GPIOD_NUM_PINS,
    GPIOE_NUM_PINS,
    GPIOF_NUM_PINS,
    GPIOG_NUM_PINS,
    GPIOH_NUM_PINS,
    GPIOI_NUM_PINS,
    GPIOJ_NUM_PINS,
    GPIOK_NUM_PINS,
    GPIOL_NUM_PINS
  };
  /* clang-format on */


  RIndex_t getResourceIndex( const Chimera::GPIO::Port port, const Chimera::GPIO::Pin pin )
  {
    auto _port = static_cast<uint8_t>( port );
    auto _pin  = static_cast<uint8_t>( pin );

    /*-------------------------------------------------
    Boundary check against the project's description
    -------------------------------------------------*/
    if( !( _port < PRJ_MAX_PORTS ) || !( _pin < PRJ_MAX_PINS ))
    {
      return INVALID_RESOURCE_INDEX;
    }


  }

}  // namespace Thor::LLD::GPIO
