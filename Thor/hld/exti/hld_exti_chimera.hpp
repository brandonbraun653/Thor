/********************************************************************************
 *  File Name:
 *    hld_exti_chimera.hpp
 *
 *	 Description:
 *    Chimera hooks for implementing EXTI
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_EXTI_CHIMERA_HOOKS_HPP
#define THOR_EXTI_CHIMERA_HOOKS_HPP

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/exti>

namespace Chimera::EXTI::Backend
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t open();
  Chimera::Status_t close();
  Chimera::Status_t attach( const EventLine_t listener, const EdgeTrigger edge, Chimera::Function::vGeneric callback );
  Chimera::Status_t detach( const EventLine_t listener );
  Chimera::Status_t trigger( const EventLine_t listener );
  Chimera::Status_t disable( const EventLine_t listener );
  Chimera::Status_t enable( const EventLine_t listener );
  EventLine_t numInterruptLines();
}    // namespace Chimera::EXTI::Backend

#endif /* !THOR_EXTI_CHIMERA_HOOKS_HPP */
