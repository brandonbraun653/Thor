/********************************************************************************
 *  File Name:
 *    spi_sim_variant.cpp
 *
 *  Description:
 *    Simulator variant of the SPI driver
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Chimera Includes */
#include <Chimera/utility>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/spi/spi_types.hpp>
#include <Thor/lld/interface/spi/spi_intf.hpp>
#include <Thor/lld/interface/spi/spi_prv_data.hpp>
#include <Thor/lld/interface/spi/sim/spi_sim_variant.hpp>

#if defined( TARGET_LLD_TEST ) && defined( THOR_LLD_SPI )

namespace Thor::LLD::SPI
{
  void initializeRegisters()
  {
  }
}  // namespace Thor::LLD::SPI

namespace Thor::LLD::RCC::LookupTables
{
  /*------------------------------------------------
  Lookup tables for register access on a peripheral by peripheral basis.
  Indexing must match the lookup table hw_spi_mapping.hpp
  ------------------------------------------------*/
  // RegisterConfig SPI_ClockConfig[ Thor::LLD::SPI::NUM_SPI_PERIPHS ];
  // RegisterConfig SPI_ResetConfig[ Thor::LLD::SPI::NUM_SPI_PERIPHS ];
  // Chimera::Clock::Bus SPI_SourceClock[ Thor::LLD::SPI::NUM_SPI_PERIPHS ];

  // PCC SPILookup = { SPI_ClockConfig,
  //                   nullptr,
  //                   SPI_ResetConfig,
  //                   SPI_SourceClock,
  //                   Thor::LLD::SPI::NUM_SPI_PERIPHS,
  //                   Thor::LLD::SPI::getResourceIndex };

  void SPIInit()
  {
    using namespace Thor::LLD::SPI;

// /*------------------------------------------------
// SPI clock enable register access lookup table
// ------------------------------------------------*/
// #if defined( STM32_SPI1_PERIPH_AVAILABLE )
//     SPI_ClockConfig[ SPI1_RESOURCE_INDEX ].mask = APB2ENR_SPI1EN;
//     SPI_ClockConfig[ SPI1_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB2ENR;
// #endif

// #if defined( STM32_SPI2_PERIPH_AVAILABLE )
//     SPI_ClockConfig[ SPI2_RESOURCE_INDEX ].mask = APB1ENR1_SPI2EN;
//     SPI_ClockConfig[ SPI2_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB1ENR1;
// #endif

// #if defined( STM32_SPI3_PERIPH_AVAILABLE )
//     SPI_ClockConfig[ SPI3_RESOURCE_INDEX ].mask = APB1ENR1_SPI3EN;
//     SPI_ClockConfig[ SPI3_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB1ENR1;
// #endif

// /*------------------------------------------------
// SPI reset register access lookup table
// ------------------------------------------------*/
// #if defined( STM32_SPI1_PERIPH_AVAILABLE )
//     SPI_ResetConfig[ SPI1_RESOURCE_INDEX ].mask = APB2RSTR_SPI1RST;
//     SPI_ResetConfig[ SPI1_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB2RSTR;
// #endif

// #if defined( STM32_SPI2_PERIPH_AVAILABLE )
// #pragma message( "NEED SPI 2 DEFINITIONS" )
// #endif

// #if defined( STM32_SPI3_PERIPH_AVAILABLE )
//     SPI_ResetConfig[ SPI3_RESOURCE_INDEX ].mask = APB1RSTR1_SPI3RST;
//     SPI_ResetConfig[ SPI3_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB1RSTR1;
// #endif

// /*------------------------------------------------
// SPI clocking bus source identifier
// ------------------------------------------------*/
// #if defined( STM32_SPI1_PERIPH_AVAILABLE )
//     SPI_SourceClock[ SPI1_RESOURCE_INDEX ] = Chimera::Clock::Bus::APB2;
// #endif

// #if defined( STM32_SPI2_PERIPH_AVAILABLE )
//     SPI_SourceClock[ SPI2_RESOURCE_INDEX ] = Chimera::Clock::Bus::APB1;
// #endif

// #if defined( STM32_SPI3_PERIPH_AVAILABLE )
//     SPI_SourceClock[ SPI3_RESOURCE_INDEX ] = Chimera::Clock::Bus::APB1;
// #endif
  };

}    // namespace Thor::LLD::RCC::LookupTables
#endif