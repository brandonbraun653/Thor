/********************************************************************************
 *  File Name:
 *    usb_prv_data.hpp
 *
 *  Description:
 *    Declaration of data that must be defined by the LLD implementation or is
 *    shared among all possible drivers.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_USB_DATA
#define THOR_LLD_USB_DATA

/* STL Includes */
#include <cstddef>

/* Chimera Includes */
#include <Chimera/usb>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/hld/dma/hld_dma_intf.hpp>
#include <Thor/lld/common/macros.hpp>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/usb/usb_detail.hpp>
#include <Thor/lld/interface/interrupt/interrupt_detail.hpp>

#if defined( THOR_LLD_USB )
namespace Thor::LLD::USB
{
  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  static constexpr size_t DRIVER_MAX_PERIPHS = static_cast<size_t>( Chimera::USB::Channel::NUM_OPTIONS );

  /*-------------------------------------------------------------------------------
  Project Defined Constants
  -------------------------------------------------------------------------------*/


  /*-------------------------------------------------------------------------------
  Peripheral Instances:
    Memory mapped structures that allow direct access to peripheral registers
  -------------------------------------------------------------------------------*/
#if defined( STM32_USB1_PERIPH_AVAILABLE )
  extern RegisterMap *USB1_PERIPH;
#endif

  /*-------------------------------------------------------------------------------
  Configuration Maps:
    These convert high level configuration options into low level register config
    options. The idea is to allow the user to specify some general options, then
    convert that over to what the peripheral understands during config/init steps.
  -------------------------------------------------------------------------------*/
  namespace ConfigMap
  {
    // extern LLD_CONST <some_map>
  }    // namespace ConfigMap


  /*-------------------------------------------------------------------------------
  Peripheral Resources
  -------------------------------------------------------------------------------*/
  namespace Resource
  {
    extern LLD_CONST IRQn_Type IRQSignals[ NUM_USB_PERIPHS ];
  }    // namespace Resource
}    // namespace Thor::LLD::USB

#endif /* THOR_LLD_USB */
#endif /* !THOR_LLD_USB_DATA */
