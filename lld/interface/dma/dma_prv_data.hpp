/********************************************************************************
 *  File Name:
 *    dma_prv_data.hpp
 *
 *  Description:
 *    Declaration of data that must be defined by the LLD implementation or is
 *    shared among all possible drivers.
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_DMA_PRV_DATA_HPP
#define THOR_LLD_DMA_PRV_DATA_HPP

/* Aurora Includes */
#include <Aurora/utility>

/* Thor Includes */
#include <Thor/lld/common/macros.hpp>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/dma/dma_detail.hpp>
#include <Thor/lld/interface/interrupt/interrupt_detail.hpp>

namespace Thor::LLD::DMA
{
  /*-------------------------------------------------------------------------------
  Peripheral Instances:
    Memory mapped structures that allow direct access to peripheral registers
  -------------------------------------------------------------------------------*/
  /*------------------------------------------------
  DMA1 Register and Stream Memory Mappings
  ------------------------------------------------*/
#if defined( STM32_DMA1_PERIPH_AVAILABLE )
  extern RegisterMap *DMA1_PERIPH;

#if defined( STM32_DMA1_STREAM0_AVAILABLE )
  extern StreamMap *DMA1_STREAM0;
#endif

#if defined( STM32_DMA1_STREAM1_AVAILABLE )
  extern StreamMap *DMA1_STREAM1;
#endif

#if defined( STM32_DMA1_STREAM2_AVAILABLE )
  extern StreamMap *DMA1_STREAM2;
#endif

#if defined( STM32_DMA1_STREAM3_AVAILABLE )
  extern StreamMap *DMA1_STREAM3;
#endif

#if defined( STM32_DMA1_STREAM4_AVAILABLE )
  extern StreamMap *DMA1_STREAM4;
#endif

#if defined( STM32_DMA1_STREAM5_AVAILABLE )
  extern StreamMap *DMA1_STREAM5;
#endif

#if defined( STM32_DMA1_STREAM6_AVAILABLE )
  extern StreamMap *DMA1_STREAM6;
#endif

#if defined( STM32_DMA1_STREAM7_AVAILABLE )
  extern StreamMap *DMA1_STREAM7;
#endif

#endif /* STM32_DMA1_PERIPH_AVAILABLE */

  /*------------------------------------------------
  DMA2 Register and Stream Memory Mappings
  ------------------------------------------------*/
#if defined( STM32_DMA2_PERIPH_AVAILABLE )
  extern RegisterMap *DMA2_PERIPH;

#if defined( STM32_DMA2_STREAM0_AVAILABLE )
  extern StreamMap *DMA2_STREAM0;
#endif

#if defined( STM32_DMA2_STREAM1_AVAILABLE )
  extern StreamMap *DMA2_STREAM1;
#endif

#if defined( STM32_DMA2_STREAM2_AVAILABLE )
  extern StreamMap *DMA2_STREAM2;
#endif

#if defined( STM32_DMA2_STREAM3_AVAILABLE )
  extern StreamMap *DMA2_STREAM3;
#endif

#if defined( STM32_DMA2_STREAM4_AVAILABLE )
  extern StreamMap *DMA2_STREAM4;
#endif

#if defined( STM32_DMA2_STREAM5_AVAILABLE )
  extern StreamMap *DMA2_STREAM5;
#endif

#if defined( STM32_DMA2_STREAM6_AVAILABLE )
  extern StreamMap *DMA2_STREAM6;
#endif

#if defined( STM32_DMA2_STREAM7_AVAILABLE )
  extern StreamMap *DMA2_STREAM7;
#endif

#endif /* STM32_DMA2_PERIPH_AVAILABLE */


  /*-------------------------------------------------------------------------------
  Configuration Maps:
    These convert high level configuration options into low level register config
    options. The idea is to allow the user to specify some general options, then
    convert that over to what the peripheral understands during config/init steps.
  -------------------------------------------------------------------------------*/
  namespace Config
  {
    extern LLD_CONST uint32_t TransferMap[ EnumValue( Chimera::DMA::Direction::NUM_OPTIONS ) ];
    extern LLD_CONST uint32_t ModeMap[ EnumValue( Chimera::DMA::Mode::NUM_OPTIONS ) ];
    extern LLD_CONST uint32_t PriorityMap[ EnumValue( Chimera::DMA::Priority::NUM_OPTIONS ) ];
    extern LLD_CONST StreamAttr RequestMap[ NUM_DMA_SOURCES ];
  }    // namespace Config


  /*-------------------------------------------------------------------------------
  Peripheral Resources
  -------------------------------------------------------------------------------*/
  namespace Resource
  {
    extern LLD_CONST IRQn_Type IRQSignals[ NUM_DMA_STREAMS_TOTAL ];
  }    // namespace Resource

}    // namespace Thor::LLD::DMA

#endif /* !THOR_LLD_DMA_PRV_DATA_HPP */
