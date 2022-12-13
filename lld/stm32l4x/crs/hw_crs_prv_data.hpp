/******************************************************************************
 *  File Name:
 *    hw_crs_prv_data.hpp
 *
 *  Description:
 *    Declaration of data that must be defined by the LLD implementation or is
 *    shared among all possible drivers.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_CRS_DATA_HPP
#define THOR_LLD_CRS_DATA_HPP

/* STL Includes */
#include <cstddef>

/* Thor Includes */
#include <Thor/lld/common/macros.hpp>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/stm32l4x/crs/hw_crs_prj.hpp>
#include <Thor/lld/stm32l4x/crs/hw_crs_types.hpp>

namespace Thor::LLD::CRS
{
  /*---------------------------------------------------------------------------
  Peripheral Instances:
    Memory mapped structures that allow direct access to peripheral registers
  ---------------------------------------------------------------------------*/
#if defined( STM32_CRS1_PERIPH_AVAILABLE )
  extern RegisterMap *CRS1_PERIPH;
#endif

  /*---------------------------------------------------------------------------
  Configuration Maps:
    These convert high level configuration options into low level register config
    options. The idea is to allow the user to specify some general options, then
    convert that over to what the peripheral understands during config/init steps.
  ---------------------------------------------------------------------------*/
  namespace ConfigMap
  {
    extern LLD_CONST Reg32_t SyncSourceMap[ static_cast<size_t>( SyncSource::NUM_OPTIONS ) ];
    extern LLD_CONST Reg32_t SyncDivMap[ static_cast<size_t>( SyncDiv::NUM_OPTIONS ) ];
  }    // namespace ConfigMap

}    // namespace Thor::LLD::CRS

#endif /* !THOR_LLD_CRS_DATA_HPP */
