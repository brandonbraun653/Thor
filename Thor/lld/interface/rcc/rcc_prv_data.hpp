/********************************************************************************
 *  File Name:
 *    rcc_prv_data.hpp
 *
 *  Description:
 *    Declaration of data that must be defined by the LLD implementation or is
 *    shared among all possible drivers.
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_RCC_DATA_HPP
#define THOR_LLD_RCC_DATA_HPP

/* STL Includes */
#include <cstddef>

/* Thor Includes */
#include <Thor/lld/common/macros.hpp>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/rcc/rcc_detail.hpp>

namespace Thor::LLD::RCC
{
  /*-------------------------------------------------------------------------------
  Peripheral Instances:
    Memory mapped structures that allow direct access to peripheral registers
  -------------------------------------------------------------------------------*/
#if defined( STM32_RCC1_PERIPH_AVAILABLE )
  extern RegisterMap *RCC1_PERIPH;
  extern const PCC * PeripheralControlRegistry[ static_cast<size_t>( Chimera::Peripheral::Type::NUM_OPTIONS ) ];
#endif


  /*-------------------------------------------------------------------------------
  Configuration Maps:
    These convert high level configuration options into low level register config
    options. The idea is to allow the user to specify some general options, then
    convert that over to what the peripheral understands during config/init steps.
  -------------------------------------------------------------------------------*/
  namespace ConfigMap
  {
  }    // namespace ConfigMap


  /*-------------------------------------------------------------------------------
  Peripheral Resources
  -------------------------------------------------------------------------------*/
  namespace Resource
  {
    // extern LLD_CONST uint8_t AHBPrescaleTable[ NUM_AHB_PRESCALERS ];
    // extern LLD_CONST uint8_t APBPrescaleTable[ NUM_APB_PRESCALERS ];
    // extern LLD_CONST uint32_t MSIRangeTable[ NUM_MSI_RANGES ];
  }    // namespace ResourceMap
}  // namespace

#endif  /* !THOR_LLD_RCC_DATA_HPP */
