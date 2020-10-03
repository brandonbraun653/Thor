/********************************************************************************
 *  File Name:
 *    hld_can_chimera.hpp
 *
 *	 Description:
 *    Chimera hooks for implementing CAN
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_CAN_CHIMERA_HOOKS_HPP
#define THOR_CAN_CHIMERA_HOOKS_HPP

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/can>

namespace Chimera::CAN::Backend
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize();
  Chimera::Status_t reset();
  Driver_sPtr getDriver( const Channel channel );
}    // namespace Chimera::CAN::Backend

#endif /* !THOR_CAN_CHIMERA_HOOKS_HPP */
