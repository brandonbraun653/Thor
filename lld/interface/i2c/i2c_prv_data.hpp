/********************************************************************************
 *  File Name:
 *    i2c_prv_data.hpp
 *
 *  Description:
 *    Declaration of data that must be defined by the LLD implementation or is
 *    shared among all possible drivers.
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_I2C_DATA_HPP
#define THOR_LLD_I2C_DATA_HPP

/* STL Includes */
#include <cstddef>

/* Chimera Includes */
#include <Chimera/i2c>

/* Thor Includes */
#include <Thor/lld/common/macros.hpp>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/i2c/i2c_detail.hpp>
#include <Thor/lld/interface/i2c/i2c_types.hpp>
#include <Thor/lld/interface/inc/dma>
#include <Thor/lld/interface/interrupt/interrupt_detail.hpp>

#if defined( THOR_I2C )
namespace Thor::LLD::I2C
{
  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/

  /*-------------------------------------------------------------------------------
  Project Defined Constants
  -------------------------------------------------------------------------------*/

  /*-------------------------------------------------------------------------------
  Peripheral Instances:
    Memory mapped structures that allow direct access to peripheral registers
  -------------------------------------------------------------------------------*/
  #if defined( STM32_I2C1_PERIPH_AVAILABLE )
    extern RegisterMap *I2C1_PERIPH;
  #endif
  #if defined( STM32_I2C2_PERIPH_AVAILABLE )
    extern RegisterMap *I2C2_PERIPH;
  #endif
  #if defined( STM32_I2C3_PERIPH_AVAILABLE )
    extern RegisterMap *I2C3_PERIPH;
  #endif

  /*-------------------------------------------------------------------------------
  Configuration Maps:
    These convert high level configuration options into low level register config
    options. The idea is to allow the user to specify some general options, then
    convert that over to what the peripheral understands during config/init steps.
  -------------------------------------------------------------------------------*/
  namespace ConfigMap
  {}


  /*-------------------------------------------------------------------------------
  Peripheral Resources
  -------------------------------------------------------------------------------*/
  namespace Resource
  {
    static constexpr size_t DMA_SIG_PER_PERIPH = static_cast<size_t>( DMADirection::NUM_OPTIONS );
    static constexpr size_t ISR_VEC_PER_PERIPH = static_cast<size_t>( IRQHandlerIndex::NUM_OPTIONS );

    extern LLD_CONST Thor::LLD::DMA::Source DMASignals[ NUM_I2C_PERIPHS ][ DMA_SIG_PER_PERIPH ];
    extern LLD_CONST IRQn_Type IRQSignals[ NUM_I2C_PERIPHS ][ ISR_VEC_PER_PERIPH ];
  }    // namespace ResourceMap
}    // namespace Thor::LLD::I2C

#endif /* THOR_LLD_I2C */
#endif /* !THOR_LLD_I2C_DATA_HPP */
