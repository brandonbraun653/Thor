/******************************************************************************
 *  File Name:
 *    power_prv_data.hpp
 *
 *  Description:
 *    Declaration of data that must be defined by the LLD implementation or is
 *    shared among all possible drivers.
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_FLASH_DATA_HPP
#define THOR_LLD_FLASH_DATA_HPP

/* STL Includes */
#include <cstddef>

/* Thor Includes */
#include <Thor/lld/common/macros.hpp>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/flash/flash_detail.hpp>

namespace Thor::LLD::FLASH
{
  /*---------------------------------------------------------------------------
  Peripheral Instances:
    Memory mapped structures that allow direct access to peripheral registers
  ---------------------------------------------------------------------------*/
#if defined( STM32_FLASH_PERIPH_AVAILABLE )
  extern RegisterMap *FLASH_PERIPH;
#endif


  /*---------------------------------------------------------------------------
  Configuration Maps:
    These convert high level configuration options into low level register config
    options. The idea is to allow the user to specify some general options, then
    convert that over to what the peripheral understands during config/init steps.
  ---------------------------------------------------------------------------*/
  namespace ConfigMap
  {
  }    // namespace ConfigMap


  /*---------------------------------------------------------------------------
  Peripheral Resources
  ---------------------------------------------------------------------------*/
  namespace Resource
  {
  }    // namespace ResourceMap
}  // namespace

#endif  /* !THOR_LLD_FLASH_DATA_HPP */
