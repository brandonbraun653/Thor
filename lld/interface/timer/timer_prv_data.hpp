/******************************************************************************
 *  File Name:
 *    timer_prv_data.hpp
 *
 *  Description:
 *    Declaration of data that must be defined by the LLD implementation or is
 *    shared among all possible drivers.
 *
 *  2021-2022 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_TIMER_DATA_HPP
#define THOR_LLD_TIMER_DATA_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstddef>
#include <Chimera/timer>
#include <Thor/lld/common/macros.hpp>
#include <Thor/lld/common/types.hpp>

namespace Thor::LLD::TIMER
{
  /*---------------------------------------------------------------------------
  Peripheral Instances
  ---------------------------------------------------------------------------*/
#if defined( STM32_TIMER1_PERIPH_AVAILABLE )
  extern RegisterMap *TIMER1_PERIPH;
#endif
#if defined( STM32_TIMER2_PERIPH_AVAILABLE )
  extern RegisterMap *TIMER2_PERIPH;
#endif
#if defined( STM32_TIMER3_PERIPH_AVAILABLE )
  extern RegisterMap *TIMER3_PERIPH;
#endif
#if defined( STM32_TIMER6_PERIPH_AVAILABLE )
  extern RegisterMap *TIMER6_PERIPH;
#endif
#if defined( STM32_TIMER7_PERIPH_AVAILABLE )
  extern RegisterMap *TIMER7_PERIPH;
#endif
#if defined( STM32_TIMER15_PERIPH_AVAILABLE )
  extern RegisterMap *TIMER15_PERIPH;
#endif
#if defined( STM32_TIMER16_PERIPH_AVAILABLE )
  extern RegisterMap *TIMER16_PERIPH;
#endif

  /*-----------------------------------------------------------------------------
  Map peripheral instances at a channel level
  -----------------------------------------------------------------------------*/
  extern RegisterMap *const PeriphRegisterBlock[ EnumValue( Chimera::Timer::Instance::NUM_OPTIONS ) ];

}    // namespace Thor::LLD::TIMER

#endif /* !THOR_LLD_TIMER_DATA_HPP */
