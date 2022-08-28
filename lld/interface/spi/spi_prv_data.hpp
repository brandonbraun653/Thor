/********************************************************************************
 *  File Name:
 *    spi_prv_data.hpp
 *
 *  Description:
 *    Declaration of data that must be defined by the LLD implementation or is
 *    shared among all possible drivers.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_SPI_DATA
#define THOR_LLD_SPI_DATA

/* STL Includes */
#include <cstddef>

/* Chimera Includes */
#include <Chimera/spi>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/common/macros.hpp>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/dma/dma_types.hpp>
#include <Thor/lld/interface/spi/spi_detail.hpp>
#include <Thor/lld/interface/interrupt/interrupt_detail.hpp>

#if defined( THOR_SPI )
namespace Thor::LLD::SPI
{
  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  static constexpr size_t DRIVER_MAX_PERIPHS = static_cast<size_t>( Chimera::SPI::Channel::NUM_OPTIONS );

  /*-------------------------------------------------------------------------------
  Project Defined Constants
  -------------------------------------------------------------------------------*/


  /*-------------------------------------------------------------------------------
  Peripheral Instances:
    Memory mapped structures that allow direct access to peripheral registers
  -------------------------------------------------------------------------------*/
#if defined( STM32_SPI1_PERIPH_AVAILABLE )
  extern RegisterMap *SPI1_PERIPH;
#endif
#if defined( STM32_SPI2_PERIPH_AVAILABLE )
  extern RegisterMap *SPI2_PERIPH;
#endif
#if defined( STM32_SPI3_PERIPH_AVAILABLE )
  extern RegisterMap *SPI3_PERIPH;
#endif


  /*-------------------------------------------------------------------------------
  Configuration Maps:
    These convert high level configuration options into low level register config
    options. The idea is to allow the user to specify some general options, then
    convert that over to what the peripheral understands during config/init steps.
  -------------------------------------------------------------------------------*/
  namespace ConfigMap
  {
    extern LLD_CONST Reg32_t BitOrderToRegConfig[ static_cast<size_t>( Chimera::SPI::BitOrder::NUM_OPTIONS ) ];
    extern LLD_CONST Reg32_t ClockModeToRegConfig[ static_cast<size_t>( Chimera::SPI::ClockMode::NUM_OPTIONS ) ];
    extern LLD_CONST Reg32_t ControlModeToRegConfig[ static_cast<size_t>( Chimera::SPI::ControlMode::NUM_OPTIONS ) ];
    extern LLD_CONST Reg32_t DataSizeToRegConfig[ static_cast<size_t>( Chimera::SPI::DataSize::NUM_OPTIONS ) ];
  }    // namespace ConfigMap


  /*-------------------------------------------------------------------------------
  Peripheral Resources
  -------------------------------------------------------------------------------*/
  namespace Resource
  {
    extern LLD_CONST Thor::LLD::DMA::Source RXDMASignals[ NUM_SPI_PERIPHS ];
    extern LLD_CONST Thor::LLD::DMA::Source TXDMASignals[ NUM_SPI_PERIPHS ];
    extern LLD_CONST IRQn_Type IRQSignals[ NUM_SPI_PERIPHS ];
  }    // namespace ResourceMap
}    // namespace Thor::LLD::SPI

#endif /* THOR_LLD_SPI */
#endif /* !THOR_LLD_SPI_DATA */
