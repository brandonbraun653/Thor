/********************************************************************************
 *  File Name:
 *    gpio_prv_data.hpp
 *
 *  Description:
 *    Declaration of data that must be defined by the LLD implementation or is
 *    shared among all possible drivers.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_GPIO_DATA_HPP
#define THOR_LLD_GPIO_DATA_HPP

/* STL Includes */
#include <cstddef>

/* Chimera Includes */
#include <Chimera/gpio>

/* Thor Includes */
#include <Thor/lld/common/macros.hpp>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/gpio/gpio_detail.hpp>

namespace Thor::LLD::GPIO
{
  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  static constexpr size_t DRIVER_MAX_PORTS         = 8;  /**< Max physical GPIO ports supported by the driver */
  static constexpr size_t DRIVER_MAX_PINS_PER_PORT = 16; /**< Max pins per port supported by the driver */
  static constexpr size_t DRIVER_MAX_PINS          = DRIVER_MAX_PORTS * DRIVER_MAX_PINS_PER_PORT;

  /*-------------------------------------------------
  These allow the project to describe available GPIOs
  in a highly configurable way.
  -------------------------------------------------*/
  extern const RIndex_t GPIOA_RESOURCE_INDEX; /**< Chip defined resource index for PortA */
  extern const RIndex_t GPIOB_RESOURCE_INDEX; /**< Chip defined resource index for PortB */
  extern const RIndex_t GPIOC_RESOURCE_INDEX; /**< Chip defined resource index for PortC */
  extern const RIndex_t GPIOD_RESOURCE_INDEX; /**< Chip defined resource index for PortD */
  extern const RIndex_t GPIOE_RESOURCE_INDEX; /**< Chip defined resource index for PortE */
  extern const RIndex_t GPIOF_RESOURCE_INDEX; /**< Chip defined resource index for PortF */
  extern const RIndex_t GPIOG_RESOURCE_INDEX; /**< Chip defined resource index for PortG */
  extern const RIndex_t GPIOH_RESOURCE_INDEX; /**< Chip defined resource index for PortH */

  extern const uint8_t GPIOA_NUM_PINS; /**< Chip defined number of pins available on PortA */
  extern const uint8_t GPIOB_NUM_PINS; /**< Chip defined number of pins available on PortB */
  extern const uint8_t GPIOC_NUM_PINS; /**< Chip defined number of pins available on PortC */
  extern const uint8_t GPIOD_NUM_PINS; /**< Chip defined number of pins available on PortD */
  extern const uint8_t GPIOE_NUM_PINS; /**< Chip defined number of pins available on PortE */
  extern const uint8_t GPIOF_NUM_PINS; /**< Chip defined number of pins available on PortF */
  extern const uint8_t GPIOG_NUM_PINS; /**< Chip defined number of pins available on PortG */
  extern const uint8_t GPIOH_NUM_PINS; /**< Chip defined number of pins available on PortH */

  extern const uint8_t PRJ_MAX_PORTS; /**< Chip defined absolute max GPIO ports */
  extern const uint8_t PRJ_MAX_PINS;  /**< Chip defined absolute max GPIO pins */

}    // namespace Thor::LLD::GPIO

#endif /* !THOR_LLD_GPIO_DATA_HPP */
