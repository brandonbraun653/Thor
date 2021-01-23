/********************************************************************************
 *  File Name:
 *    sys_prv_data.hpp
 *
 *  Description:
 *    Insert Description
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_SYS_DATA_HPP
#define THOR_LLD_SYS_DATA_HPP

/* Chimera Includes */
#include <Chimera/common>

/* Thor Includes */
#include <Thor/lld/interface/system/sys_detail.hpp>

namespace Thor::LLD::SYS
{
  /*-------------------------------------------------------------------------------
  Peripheral Instances:
    Memory mapped structures that allow direct access to peripheral registers
  -------------------------------------------------------------------------------*/
  extern RegisterMap *SYSCFG1_PERIPH;

  /**
   *  A list of all supported peripherals on the target device, regardless
   *  of whether or not a driver exists for it yet. This is meant to describe
   *  possible system functionality.
   */
  extern const Chimera::Peripheral::Type AvailablePeriphs[ NUM_PERIPHERAL_TYPES ];
}  // namespace Thor::LLD::SYS

#endif  /* !THOR_LLD_SYS_DATA_HPP */
