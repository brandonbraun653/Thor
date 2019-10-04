/********************************************************************************
 *   File Name:
 *    hw_rcc_mapping_spi.cpp
 *
 *   Description:
 *    RCC configuration maps for the SPI peripherals
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/drivers/f4/rcc/hw_rcc_mapping.hpp>
#include <Thor/drivers/f4/spi/hw_spi_mapping.hpp>

namespace Thor::Driver::RCC::LookupTables
{
/*------------------------------------------------
SPI Peripheral RCC Configuration Resources
------------------------------------------------*/
#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_SPI == 1 )
  /**
   *  SPI clock enable register access lookup table
   *
   *  @note Indexing must match the lookup table in hw_spi_mapping.hpp
   */
  const RegisterConfig SPI_ClockConfig[ spiTableSize ] = {
    /* SPI1 */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, APB2ENR ) ), APB2ENR_SPI1EN },
    /* SPI2 */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, APB1ENR ) ), APB1ENR_SPI2EN },
    /* SPI3 */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, APB1ENR ) ), APB1ENR_SPI3EN },
    /* SPI4 */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, APB2ENR ) ), APB2ENR_SPI4EN }
  };

  /**
   *  SPI low power clock enable register access lookup table
   *
   *  @note Indexing must match the lookup table in hw_spi_mapping.hpp
   */
  const RegisterConfig SPI_ClockConfigLP[ spiTableSize ] = {
    /* SPI1 */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, APB2LPENR ) ),
      APB2LPENR_SPI1LPEN },
    /* SPI2 */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, APB1LPENR ) ),
      APB1LPENR_SPI2LPEN },
    /* SPI3 */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, APB1LPENR ) ),
      APB1LPENR_SPI3LPEN },
    /* SPI4 */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, APB2LPENR ) ),
      APB2LPENR_SPI4LPEN }
  };

  /**
   *  SPI reset register access lookup table
   *
   *  @note Indexing must match the lookup table in hw_spi_mapping.hpp
   */
  const RegisterConfig SPI_ResetConfig[ spiTableSize ] = {
    /* SPI1 */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, APB2RSTR ) ),
      APB2RSTR_SPI1RST },
    /* SPI2 */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, APB1RSTR ) ),
      APB1RSTR_SPI2RST },
    /* SPI3 */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, APB1RSTR ) ),
      APB1RSTR_SPI3RST },
    /* SPI4 */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, APB2RSTR ) ), APB2RSTR_SPI4RST }
  };

  /**
   *  SPI clocking bus source identifier
   *
   *  @note Indexing must match the lookup table in hw_spi_mapping.hpp
   */
  const Configuration::ClockType_t SPI_SourceClock[ spiTableSize ] = {
    /* SPI1 */
    Configuration::ClockType::PCLK2,
    /* SPI2 */
    Configuration::ClockType::PCLK1,
    /* SPI3 */
    Configuration::ClockType::PCLK1,
    /* SPI4 */
    Configuration::ClockType::PCLK2
  };

  const PCC SPILookup = {
    SPI_ClockConfig, SPI_ClockConfigLP, SPI_ResetConfig, SPI_SourceClock, &Thor::Driver::SPI::InstanceToResourceIndex,
    spiTableSize
  };

#endif /* TARGET_STM32F4 && THOR_DRIVER_SPI */
}    // namespace Thor::Driver::RCC::LookupTables
