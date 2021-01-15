/********************************************************************************
 *  File Name:
 *    hld_interrupt_driver.hpp
 *
 *  Description:
 *    Interrupt driver interface for the high level layer
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_HLD_INTERRUPT_HPP
#define THOR_HLD_INTERRUPT_HPP

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/interrupt>

namespace Thor::Interrupt
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize();
  Chimera::Status_t reset();
  Chimera::Status_t registerISRHandler( const Chimera::Peripheral::Type type, const Chimera::Interrupt::Signal_t signal,
                                        const Chimera::Interrupt::SignalCallback &callback );

}  // namespace Thor::Interrupt

#endif  /* !THOR_HLD_INTERRUPT_HPP */
