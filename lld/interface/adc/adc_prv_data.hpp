/******************************************************************************
 *  File Name:
 *    adc_prv_data.hpp
 *
 *  Description:
 *    Declaration of data that must be defined by the LLD implementation or is
 *    shared among all possible drivers.
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_ADC_DATA
#define THOR_LLD_ADC_DATA

/* STL Includes */
#include <cstddef>

/* Aurora Includes */
#include <Aurora/utility>

/* Chimera Includes */
#include <Chimera/adc>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/common/macros.hpp>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/adc/adc_detail.hpp>
#include <Thor/lld/interface/adc/adc_types.hpp>
#include <Thor/lld/interface/dma/dma_types.hpp>
#include <Thor/lld/interface/interrupt/interrupt_detail.hpp>

#if defined( THOR_ADC )

namespace Thor::LLD::ADC
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t DRIVER_MAX_PERIPHS         = static_cast<size_t>( Chimera::ADC::Channel::NUM_OPTIONS );
  static constexpr size_t CHANNEL_QUEUE_SAMPLE_DEPTH = 8;
  static constexpr size_t CHANNEL_QUEUE_SIZE         = EnumValue( Chimera::ADC::Channel::NUM_OPTIONS );

  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  /**
   * @brief Array of queues for each supported ADC hardware channel
   * Queue is indexed like myQueue[ channel ]->push/pop.
   */
  using PeriphQueue = std::array<ChannelQueue<CHANNEL_QUEUE_SAMPLE_DEPTH>*, CHANNEL_QUEUE_SIZE>;

  /*---------------------------------------------------------------------------
  Peripheral Instances:
    Memory mapped structures that allow direct access to peripheral registers
  ---------------------------------------------------------------------------*/
  extern CommonRegisterMap *ADC_COMMON;

#if defined( STM32_ADC1_PERIPH_AVAILABLE )
  extern RegisterMap *ADC1_PERIPH;
#endif
#if defined( STM32_ADC2_PERIPH_AVAILABLE )
  extern RegisterMap *ADC2_PERIPH;
#endif
#if defined( STM32_ADC3_PERIPH_AVAILABLE )
  extern RegisterMap *ADC3_PERIPH;
#endif

  /*---------------------------------------------------------------------------
  Shared Data
  ---------------------------------------------------------------------------*/
#if defined( STM32_ADC1_PERIPH_AVAILABLE )
  extern PeriphQueue ADC1_Queue;
#endif
#if defined( STM32_ADC2_PERIPH_AVAILABLE )
  extern PeriphQueue ADC2_Queue;
#endif
#if defined( STM32_ADC3_PERIPH_AVAILABLE )
  extern PeriphQueue ADC3_Queue;
#endif

  /*---------------------------------------------------------------------------
  Configuration Maps:
    These convert high level configuration options into low level register config
    options. The idea is to allow the user to specify some general options, then
    convert that over to what the peripheral understands during config/init steps.
  ---------------------------------------------------------------------------*/
  namespace ConfigMap
  {
    extern LLD_CONST Chimera::ADC::Channel SensorToChannel[ static_cast<size_t>( Chimera::ADC::Sensor::NUM_OPTIONS ) ];
  }    // namespace ConfigMap


  /*---------------------------------------------------------------------------
  Peripheral Resources
  ---------------------------------------------------------------------------*/
  namespace Resource
  {
    extern LLD_CONST Thor::LLD::DMA::Source DMASignals[ NUM_ADC_PERIPHS ];
    extern LLD_CONST IRQn_Type IRQSignals[ NUM_ADC_IRQ_HANDLERS ];
  }    // namespace Resource
}    // namespace Thor::LLD::ADC

#endif /* THOR_LLD_ADC */
#endif /* !THOR_LLD_ADC_DATA */
