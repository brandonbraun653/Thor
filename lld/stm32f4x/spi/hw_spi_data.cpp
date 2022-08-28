/********************************************************************************
 *  File Name:
 *    hw_spi_data.cpp
 *
 *  Description:
 *    Provides structures for conversion and mapping between data types for fast
 *    runtime performance of driver code.
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* STL Includes */
#include <limits>

/* Chimera Includes */
#include <Chimera/spi>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/inc/spi>
#include <Thor/lld/interface/inc/dma>

#if defined( TARGET_STM32F4 ) && defined( THOR_SPI )

namespace Thor::LLD::SPI
{
  /*-------------------------------------------------------------------------------
  Peripheral Memory Maps
  -------------------------------------------------------------------------------*/
#if defined( STM32_SPI1_PERIPH_AVAILABLE )
  RegisterMap *SPI1_PERIPH = reinterpret_cast<RegisterMap *>( SPI1_BASE_ADDR );
#endif
#if defined( STM32_SPI2_PERIPH_AVAILABLE )
  RegisterMap *SPI2_PERIPH = reinterpret_cast<RegisterMap *>( SPI2_BASE_ADDR );
#endif
#if defined( STM32_SPI3_PERIPH_AVAILABLE )
  RegisterMap *SPI3_PERIPH = reinterpret_cast<RegisterMap *>( SPI3_BASE_ADDR );
#endif
#if defined( STM32_SPI4_PERIPH_AVAILABLE )
  RegisterMap *SPI4_PERIPH = reinterpret_cast<RegisterMap *>( SPI4_BASE_ADDR );
#endif

  /*-------------------------------------------------------------------------------
  Configuration Maps
  -------------------------------------------------------------------------------*/
  namespace ConfigMap
  { /* clang-format off */
    LLD_CONST Reg32_t BitOrderToRegConfig[ static_cast<size_t>( Chimera::SPI::BitOrder::NUM_OPTIONS ) ] = {
      Configuration::DataFormat::MSB,
      Configuration::DataFormat::LSB
    };

    LLD_CONST Reg32_t ClockModeToRegConfig[ static_cast<size_t>( Chimera::SPI::ClockMode::NUM_OPTIONS ) ] = {
      Configuration::ClockFormat::MODE0,
      Configuration::ClockFormat::MODE1,
      Configuration::ClockFormat::MODE2,
      Configuration::ClockFormat::MODE3
    };

    LLD_CONST Reg32_t ControlModeToRegConfig[ static_cast<size_t>( Chimera::SPI::ControlMode::NUM_OPTIONS ) ] = {
      Configuration::Mode::MASTER,
      Configuration::Mode::SLAVE
    };

    LLD_CONST Reg32_t DataSizeToRegConfig[ static_cast<size_t>( Chimera::SPI::DataSize::NUM_OPTIONS ) ] = {
      Configuration::Width::WIDTH_8_BIT,
      Configuration::Width::WIDTH_16_BIT,
      Configuration::Width::WIDTH_16_BIT,
      Configuration::Width::WIDTH_16_BIT,
      Configuration::Width::WIDTH_16_BIT,
      Configuration::Width::WIDTH_16_BIT,
      Configuration::Width::WIDTH_16_BIT,
      Configuration::Width::WIDTH_16_BIT,
      Configuration::Width::WIDTH_16_BIT
    };
  } /* clang-format on */


  /*-------------------------------------------------------------------------------
  Peripheral Resources
  -------------------------------------------------------------------------------*/
  namespace Resource
  { /* clang-format off */
    LLD_CONST DMA::Source RXDMASignals[ NUM_SPI_PERIPHS ] = {
#if defined( STM32_SPI1_PERIPH_AVAILABLE )
      Thor::LLD::DMA::Source::SPI1_RX,
#endif
#if defined( STM32_SPI2_PERIPH_AVAILABLE )
      Thor::LLD::DMA::Source::SPI2_RX,
#endif
#if defined( STM32_SPI3_PERIPH_AVAILABLE )
      Thor::LLD::DMA::Source::SPI3_RX,
#endif
#if defined( STM32_SPI4_PERIPH_AVAILABLE )
      Thor::LLD::DMA::Source::SPI4_RX
#endif
    };

    LLD_CONST DMA::Source TXDMASignals[ NUM_SPI_PERIPHS ] = {
#if defined( STM32_SPI1_PERIPH_AVAILABLE )
      Thor::LLD::DMA::Source::SPI1_TX,
#endif
#if defined( STM32_SPI2_PERIPH_AVAILABLE )
      Thor::LLD::DMA::Source::SPI2_TX,
#endif
#if defined( STM32_SPI3_PERIPH_AVAILABLE )
      Thor::LLD::DMA::Source::SPI3_TX,
#endif
#if defined( STM32_SPI4_PERIPH_AVAILABLE )
      Thor::LLD::DMA::Source::SPI4_TX
#endif
    };

    LLD_CONST IRQn_Type IRQSignals[ NUM_SPI_PERIPHS ] = {
#if defined( STM32_SPI1_PERIPH_AVAILABLE )
      SPI1_IRQn,
#endif
#if defined( STM32_SPI2_PERIPH_AVAILABLE )
      SPI2_IRQn,
#endif
#if defined( STM32_SPI3_PERIPH_AVAILABLE )
      SPI3_IRQn,
#endif
#if defined( STM32_SPI4_PERIPH_AVAILABLE )
      SPI4_IRQn
#endif
    };
  } /* clang-format on */
}  // namespace Thor::LLD::SPI

#endif /* TARGET_STM32L4 && THOR_LLD_SPI */
