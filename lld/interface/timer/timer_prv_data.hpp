/******************************************************************************
 *  File Name:
 *    timer_prv_data.hpp
 *
 *  Description:
 *    Declaration of data that must be defined by the LLD implementation or is
 *    shared among all possible drivers.
 *
 *  2021-2023 | Brandon Braun | brandonbraun653@gmail.com
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
#if defined( STM32_TIMER4_PERIPH_AVAILABLE )
  extern RegisterMap *TIMER4_PERIPH;
#endif
#if defined( STM32_TIMER5_PERIPH_AVAILABLE )
  extern RegisterMap *TIMER5_PERIPH;
#endif
#if defined( STM32_TIMER6_PERIPH_AVAILABLE )
  extern RegisterMap *TIMER6_PERIPH;
#endif
#if defined( STM32_TIMER7_PERIPH_AVAILABLE )
  extern RegisterMap *TIMER7_PERIPH;
#endif
#if defined( STM32_TIMER8_PERIPH_AVAILABLE )
  extern RegisterMap *TIMER8_PERIPH;
#endif
#if defined( STM32_TIMER9_PERIPH_AVAILABLE )
  extern RegisterMap *TIMER9_PERIPH;
#endif
#if defined( STM32_TIMER10_PERIPH_AVAILABLE )
  extern RegisterMap *TIMER10_PERIPH;
#endif
#if defined( STM32_TIMER11_PERIPH_AVAILABLE )
  extern RegisterMap *TIMER11_PERIPH;
#endif
#if defined( STM32_TIMER12_PERIPH_AVAILABLE )
  extern RegisterMap *TIMER12_PERIPH;
#endif
#if defined( STM32_TIMER13_PERIPH_AVAILABLE )
  extern RegisterMap *TIMER13_PERIPH;
#endif
#if defined( STM32_TIMER14_PERIPH_AVAILABLE )
  extern RegisterMap *TIMER14_PERIPH;
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
