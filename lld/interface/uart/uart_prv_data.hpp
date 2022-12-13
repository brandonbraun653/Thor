/******************************************************************************
 *  File Name:
 *    uart_prv_data.hpp
 *
 *  Description:
 *    Declaration of data that must be defined by the LLD implementation or is
 *    shared among all possible drivers.
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_UART_DATA_HPP
#define THOR_LLD_UART_DATA_HPP

/* STL Includes */
#include <cstddef>

/* Chimera Includes */
#include <Chimera/serial>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/common/macros.hpp>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/dma/dma_types.hpp>
#include <Thor/lld/interface/interrupt/interrupt_detail.hpp>
#include <Thor/lld/interface/uart/uart_detail.hpp>

#if defined( THOR_UART )
namespace Thor::LLD::UART
{
  /*---------------------------------------------------------------------------
  Peripheral Instances:
    Memory mapped structs that allow direct access to the registers of a peripheral
  ---------------------------------------------------------------------------*/
#if defined( STM32_UART4_PERIPH_AVAILABLE )
  extern RegisterMap *UART4_PERIPH;
#endif
#if defined( STM32_UART5_PERIPH_AVAILABLE )
  extern RegisterMap *UART5_PERIPH;
#endif


  /*---------------------------------------------------------------------------
  Configuration Maps:
    These convert high level configuration options into low level register config
    options. The idea is to allow the user to specify some general options, then
    convert that over to what the peripheral understands during config/init steps.
  ---------------------------------------------------------------------------*/
  namespace ConfigMap
  {
    extern LLD_CONST Reg32_t CharWidth[ static_cast<size_t>( Chimera::Serial::CharWid::NUM_OPTIONS ) ];
    extern LLD_CONST Reg32_t Parity[ static_cast<size_t>( Chimera::Serial::Parity::NUM_OPTIONS ) ];
    extern LLD_CONST Reg32_t StopBits[ static_cast<size_t>( Chimera::Serial::StopBits::NUM_OPTIONS ) ];
  }    // namespace ConfigMap


  /*---------------------------------------------------------------------------
  Peripheral Resources:
    These objects define critical resources used in the low level driver. The goal
    is to minimize memory consumption, so these arrays only hold enough information
    for the currently configured number of peripherals. They are intended to be
    accessed directly via the _ResourceIndex_ attribute of the ConfigMap namespace.
  ---------------------------------------------------------------------------*/
  namespace Resource
  {
    extern LLD_CONST DMA::Source RXDMASignals[ NUM_UART_PERIPHS ];
    extern LLD_CONST DMA::Source TXDMASignals[ NUM_UART_PERIPHS ];
    extern LLD_CONST IRQn_Type IRQSignals[ NUM_UART_PERIPHS ];
  }
}    // namespace Thor::LLD::UART

#endif /* THOR_LLD_HAS_UART */
#endif /* !THOR_LLD_UART_DATA_HPP */
