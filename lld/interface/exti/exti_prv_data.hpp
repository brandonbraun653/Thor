/********************************************************************************
 *  File Name:
 *    exti_prv_data.hpp
 *
 *  Description:
 *    Declaration of data that must be defined by the LLD implementation or is
 *    shared among all possible drivers.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_EXTI_DATA_HPP
#define THOR_LLD_EXTI_DATA_HPP

/* STL Includes */
#include <cstddef>

/* Chimera Includes */
#include <Chimera/exti>

/* Thor Includes */
#include <Thor/lld/common/macros.hpp>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/exti/exti_detail.hpp>
#include <Thor/lld/interface/exti/exti_types.hpp>

namespace Thor::LLD::EXTI
{
  /*-------------------------------------------------------------------------------
  Peripheral Instances:
    Memory mapped structures that allow direct access to peripheral registers
  -------------------------------------------------------------------------------*/
  #if defined( STM32_EXTI1_PERIPH_AVAILABLE )
    extern RegisterMap *EXTI1_PERIPH;
  #endif

  /*-------------------------------------------------------------------------------
  Configuration Maps
  -------------------------------------------------------------------------------*/
  namespace Config
  {
    extern LLD_CONST LineConfig lineConfig[ NUM_EXTI_LINES ];
  }
}    // namespace Thor::LLD::EXTI

#endif /* !THOR_LLD_EXTI_DATA_HPP */
