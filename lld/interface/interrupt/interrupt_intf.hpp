/******************************************************************************
 *  File Name:
 *    interrupt_model.hpp
 *
 *  Description:
 *    LLD Interrupt Interface
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_DRIVER_MODEL_INTERRUPT_HPP
#define THOR_DRIVER_MODEL_INTERRUPT_HPP

/* C++ Includes */
#include <cstdint>
#include <cstdlib>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/interrupt>
#include <Chimera/system>
#include <Chimera/thread>

/* Driver Includes */
#include <Thor/lld/interface/interrupt/interrupt_types.hpp>

namespace Thor::LLD::INT
{
  /*---------------------------------------------------------------------------
  Public Functions: INTF Implemented
  ---------------------------------------------------------------------------*/
  /**
   *  Initialize the interrupt layer memory
   *  @return Chimera::Status_t
   */
  Chimera::Status_t initialize();

  /**
   *  Reset the interrupt layer
   *  @return Chimera::Status_t
   */
  Chimera::Status_t reset();

  /**
   *  Registers a set of handler functions to be invoked on a particular
   *  peripheral ISR signal.
   *
   *  @param[in]  type      Which peripheral to register this against
   *  @param[in]  signal    The ISR event being listened to
   *  @param[in]  callback  The callback structure being registered
   *  @return Chimera::Status_t
   */
  Chimera::Status_t registerISRHandler( const Chimera::Peripheral::Type type, const Chimera::Interrupt::Signal_t signal,
                                        const Chimera::Interrupt::SignalCallback &callback );

  /**
   *  Retrieves the ISR handler callbacks for a peripheral and signal
   *
   *  @param[in]  type      Which peripheral to look up
   *  @param[in]  signal    The signal to look up
   *  @return const Chimera::Interrupt::SignalCallback *const
   */
  const Chimera::Interrupt::SignalCallback *const getISRHandler( const Chimera::Peripheral::Type type,
                                                                 const Chimera::Interrupt::Signal_t signal );

  /**
   *  Caches the ID assigned to the high priority user-space thread
   *  designated to handle ISR events.
   *
   *  @param[in]  type      Which peripheral to register this against
   *  @param[in]  id        The ID being cached
   *  @return void
   */
  void setUserTaskId( const Chimera::Peripheral::Type type, const Chimera::Thread::TaskId id );

  /**
   *  Gets the user-space thread ID registered with the interface
   *
   *  @param[in]  type      Which peripheral to look up
   *  @return Chimera::Thread::TaskId
   */
  Chimera::Thread::TaskId getUserTaskId( const Chimera::Peripheral::Type type );

  /**
   *  Disables all interrupts and returns a mask indicating which
   *  ISR vector signals should be unmasked in enableInterrupts()
   *
   *  @return Chimera::System::InterruptMask
   */
  Chimera::System::InterruptMask disableInterrupts();

  /**
   *  Restores previously disabled interrupts
   *
   *  @param[in]  interruptMask     Mask returned from disableInterrupts()
   *  @return void
   */
  void enableInterrupts( const Chimera::System::InterruptMask &interruptMask );

}    // namespace Thor::LLD::INT

#endif /* !THOR_DRIVER_MODEL_INTERRUPT_HPP */
