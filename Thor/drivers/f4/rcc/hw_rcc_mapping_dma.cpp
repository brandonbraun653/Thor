/********************************************************************************
 *   File Name:
 *    hw_rcc_mapping_DMA.cpp
 *
 *   Description:
 *    RCC configuration maps for the DMA peripherals
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/drivers/f4/rcc/hw_rcc_mapping.hpp>
#include <Thor/drivers/f4/dma/hw_dma_mapping.hpp>

namespace Thor::Driver::RCC::LookupTables
{
/*------------------------------------------------
DMA Peripheral RCC Configuration Resources
------------------------------------------------*/
#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_DMA == 1 )
  /**
   *  DMA clock enable register access lookup table
   *
   *  @note Indexing must match the lookup table in hw_dma_mapping.hpp
   */
  const RegisterConfig DMA_ClockConfig[ dmaTableSize ] = {
    /* DMA1 */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, AHB1ENR ) ),
      AHB1ENR_DMA1EN },
    /* DMA2 */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, AHB1ENR ) ),
      AHB1ENR_DMA2EN }
  };

  /**
   *  DMA low power clock enable register access lookup table
   *
   *  @note Indexing must match the lookup table in hw_dma_mapping.hpp
   */
  const RegisterConfig DMA_ClockConfigLP[ dmaTableSize ] = {
    /* DMA1 */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, AHB1LPENR ) ),
      AHB1LPENR_DMA1LPEN },
    /* DMA2 */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, AHB1LPENR ) ),
      AHB1LPENR_DMA2LPEN }
  };

  /**
   *  DMA reset register access lookup table
   *
   *  @note Indexing must match the lookup table in hw_dma_mapping.hpp
   */
  const RegisterConfig DMA_ResetConfig[ dmaTableSize ] = {
    /* DMA1 */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, AHB1RSTR ) ),
      AHB1RSTR_DMA1RST },
    /* DMA2 */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, AHB1RSTR ) ),
      AHB1RSTR_DMA2RST }
  };

  /**
   *  DMA clocking bus source identifier
   *
   *  @note Indexing must match the lookup table in hw_dma_mapping.hpp
   */
  const Configuration::ClockType_t DMA_SourceClock[ dmaTableSize ] = {
    /* DMA1 */
    Configuration::ClockType::HCLK,
    /* DMA2 */
    Configuration::ClockType::HCLK
  };

  const PCC DMALookup = {
    DMA_ClockConfig, DMA_ClockConfigLP, DMA_ResetConfig, DMA_SourceClock, &Thor::Driver::DMA::InstanceToResourceIndex,
    dmaTableSize
  };

#endif /* TARGET_STM32F4 && THOR_DRIVER_DMA */
}    // namespace Thor::Driver::RCC::LookupTables
