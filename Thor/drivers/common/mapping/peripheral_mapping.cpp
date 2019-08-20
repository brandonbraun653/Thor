/********************************************************************************
 *   File Name:
 *      peripheral_mapping.cpp
 *
 *   Description:
 *      Mappings for the peripherals (such a great description)
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/drivers/common/mapping/peripheral_mapping.hpp>

namespace Thor::Driver::Mapping
{
  const Chimera::Container::LightFlatMap<Chimera::Peripheral::Type, uint8_t> PeriphTypeToIterator{
    { Chimera::Peripheral::Type::PERIPH_GPIO, 0u },
    { Chimera::Peripheral::Type::PERIPH_UART, 1u },
    { Chimera::Peripheral::Type::PERIPH_USART, 2u },
  };
}