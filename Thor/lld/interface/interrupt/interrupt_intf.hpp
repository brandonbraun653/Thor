/********************************************************************************
 *  File Name:
 *    interrupt_model.hpp
 *
 *  Description:
 *    LLD Interrupt Interface
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_DRIVER_MODEL_INTERRUPT_HPP
#define THOR_DRIVER_MODEL_INTERRUPT_HPP

/* C++ Includes */
#include <cstdint>
#include <cstdlib>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/system>

/* Driver Includes */
#include <Thor/lld/interface/interrupt/interrupt_types.hpp>

namespace Thor::LLD::IT
{
  /**
   *  Disables all interrupts and returns a mask indicating which
   *  ISR vector signals should be unmasked in enableInterrupts()
   *
   *	@return Chimera::System::InterruptMask
   */
  Chimera::System::InterruptMask disableInterrupts();

  /**
   *  Restores previously disabled interrupts
   *
   *  @param[in]  interruptMask     Mask returned from disableInterrupts()
   *	@return void
   */
  void enableInterrupts( Chimera::System::InterruptMask &interruptMask );

}    // namespace Thor::Driver

#endif /* !THOR_DRIVER_MODEL_INTERRUPT_HPP */
