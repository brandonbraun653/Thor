/********************************************************************************
 *  File Name:
 *    hw_sys_data.cpp
 *
 *  Description:
 *    Implements interface layer data requirements
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Chimera Includes */
#include <Chimera/common>

/* Thor Includes */
#include <Thor/lld/interface/system/sys_prv_data.hpp>

namespace Thor::LLD::SYS
{
  /*-------------------------------------------------------------------------------
  Public Data
  -------------------------------------------------------------------------------*/
  const Chimera::Peripheral::Type AvailablePeriphs[ NUM_PERIPHERAL_TYPES ] = {
    /* clang-format off */
    Chimera::Peripheral::Type::PERIPH_ADC,
    Chimera::Peripheral::Type::PERIPH_CAN,
    Chimera::Peripheral::Type::PERIPH_CRC,
    Chimera::Peripheral::Type::PERIPH_CRS,
    Chimera::Peripheral::Type::PERIPH_DAC,
    Chimera::Peripheral::Type::PERIPH_DMA,
    Chimera::Peripheral::Type::PERIPH_FLASH,
    Chimera::Peripheral::Type::PERIPH_GPIO,
    Chimera::Peripheral::Type::PERIPH_I2C,
    Chimera::Peripheral::Type::PERIPH_IWDG,
    Chimera::Peripheral::Type::PERIPH_NVIC,
    Chimera::Peripheral::Type::PERIPH_PWR,
    Chimera::Peripheral::Type::PERIPH_RCC,
    Chimera::Peripheral::Type::PERIPH_RTC,
    Chimera::Peripheral::Type::PERIPH_SPI,
    Chimera::Peripheral::Type::PERIPH_SYSCFG,
    Chimera::Peripheral::Type::PERIPH_TIM,
    Chimera::Peripheral::Type::PERIPH_UART,
    Chimera::Peripheral::Type::PERIPH_USART,
    Chimera::Peripheral::Type::PERIPH_USB,
    Chimera::Peripheral::Type::PERIPH_WWDG
    /* clang-format on */
  };
}    // namespace Thor::LLD::SYS
