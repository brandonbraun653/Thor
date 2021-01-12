/********************************************************************************
 *  File Name:
 *    hld_exti_driver.hpp
 *
 *  Description:
 *    Thor EXTI high level driver
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HLD_EXTI_HPP
#define THOR_HLD_EXTI_HPP

/* C++ Includes */
#include <cstdint>
#include <cstdlib>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/exti>
#include <Chimera/function>

/* Thor Includes */
#include <Thor/hld/exti/hld_exti_types.hpp>

namespace Thor::EXTI
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t open();
  Chimera::Status_t close();
  Chimera::Status_t attach( const Chimera::EXTI::EventLine_t listener, const Chimera::EXTI::EdgeTrigger edge, Chimera::Function::vGeneric callback );
  Chimera::Status_t detach( const Chimera::EXTI::EventLine_t listener );
  Chimera::Status_t trigger( const Chimera::EXTI::EventLine_t listener );
  Chimera::Status_t disable( const Chimera::EXTI::EventLine_t listener );
  Chimera::Status_t enable( const Chimera::EXTI::EventLine_t listener );
  Chimera::EXTI::EventLine_t numInterruptLines();

}    // namespace Thor::EXTI

#endif /* THOR_HLD_EXTI_HPP */
