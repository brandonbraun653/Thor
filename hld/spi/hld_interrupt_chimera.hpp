/********************************************************************************
 *  File Name:
 *    hld_interrupt_chimera.hpp
 *
 *  Description:
 *    Interrupt driver interface for the Chimera registration layer
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef CHIMERA_INTERRUPT_BACKEND_HPP
#define CHIMERA_INTERRUPT_BACKEND_HPP

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/interrupt>

namespace Chimera::Interrupt::Backend
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize();
  Chimera::Status_t reset();
  Chimera::Status_t registerISRHandler( const Peripheral::Type type, const Signal_t signal, const SignalCallback &callback );

}    // namespace Chimera::Interrupt::Backend

#endif /* !CHIMERA_INTERRUPT_BACKEND_HPP */
