/********************************************************************************
 *  File Name:
 *    exti_intf.hpp
 *
 *  Description:
 *    STM32 LLD EXTI Interface Spec
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_LLD_EXTI_DRIVER_INTERFACE_HPP
#define THOR_LLD_EXTI_DRIVER_INTERFACE_HPP

/* STL Includes */
#include <limits>

/* Chimera Includes */
#include <Chimera/exti>

/* Thor Includes */
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/exti/exti_types.hpp>

namespace Thor::LLD::EXTI
{
  /*-------------------------------------------------------------------------------
  Public Functions (Implemented by the project)
  -------------------------------------------------------------------------------*/
  /**
   *  Initializes the external interrupt hardware driver and its resources
   *  @return Chimera::Status_t
   */
  Chimera::Status_t open();

  /**
   *  Tears down the external interrupt hardware driver and its resources
   *  @return Chimera::Status_t
   */
  Chimera::Status_t close();

  /**
   *  Configures the interrupt line
   *
   *  @param[in]  listener      The listener being configured
   *  @param[in]  edge          Which edge to trigger off of
   *  @param[in]  callback      What callback to invoke upon interrupt firing
   *  @return Chimera::Status_t
   */
  Chimera::Status_t attach( const Chimera::EXTI::EventLine_t listener, const Chimera::EXTI::EdgeTrigger edge, Chimera::Function::vGeneric callback );

  /**
   *  Disables the given interrupt and resets it to defaults
   *
   *  @param[in]  listener      Which interrupt listener to detach from
   *  @return Chimera::Status_t
   */
  Chimera::Status_t detach( const Chimera::EXTI::EventLine_t listener );

  /**
   *  Manually trigger an interrupt even on the given line
   *
   *  @param[in]  listener      Which interrupt listener is being triggered
   *  @return Chimera::Status_t
   */
  Chimera::Status_t trigger( const Chimera::EXTI::EventLine_t listener );

  /**
   *  Places the interrupt line in a disabled state so it cannot fire.
   *
   *  @param[in]  listener      Which interrupt listener is being disabled
   *  @return Chimera::Status_t
   */
  Chimera::Status_t disable( const Chimera::EXTI::EventLine_t listener );

  /**
   *  Enables the interrupt so that it can listen for events
   *
   *  @param[in]  listener      Which interrupt listener is being enabled
   *  @return Chimera::Status_t
   */
  Chimera::Status_t enable( const Chimera::EXTI::EventLine_t listener );

  /**
   *  How many external interrupt lines are available on the system
   *  @return Chimera::EXTI::EventLine_t
   */
  Chimera::EXTI::EventLine_t numInterruptLines();

}    // namespace Thor::LLD::EXTI

#endif /* !THOR_LLD_EXTI_DRIVER_INTERFACE_HPP */
