/********************************************************************************
 *  File Name:
 *    watchdog_prv_data.hpp
 *
 *  Description:
 *    Declaration of data that must be defined by the LLD implementation or is
 *    shared among all possible drivers.
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_WATCHDOG_DATA
#define THOR_LLD_WATCHDOG_DATA

/* STL Includes */
#include <cstddef>

/* Chimera Includes */
#include <Chimera/clock>
#include <Chimera/watchdog>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/hld/dma/hld_dma_intf.hpp>
#include <Thor/lld/common/macros.hpp>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/watchdog/watchdog_detail.hpp>
#include <Thor/lld/interface/interrupt/interrupt_detail.hpp>

namespace Thor::LLD::Watchdog
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
#if defined( STM32_IWDG1_PERIPH_AVAILABLE ) && defined( THOR_LLD_IWDG )
  extern IRegisterMap *IWDG1_PERIPH;
#endif

#if defined( STM32_WWDG1_PERIPH_AVAILABLE ) && defined( THOR_LLD_WWDG )
  extern WRegisterMap *WWDG1_PERIPH;
#endif

}    // namespace Thor::LLD::Watchdog

#endif /* !THOR_LLD_WATCHDOG_DATA */
