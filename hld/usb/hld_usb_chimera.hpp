/********************************************************************************
 *  File Name:
 *    hld_usb_chimera.hpp
 *
 *	 Description:
 *    Chimera hooks for implementing USB
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_USB_CHIMERA_HOOKS_HPP
#define THOR_USB_CHIMERA_HOOKS_HPP

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/usb>

namespace Chimera::USB::Backend
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize();
  Chimera::Status_t reset();
  Driver_rPtr getDriver( const Channel channel );
}    // namespace Chimera::USB::Backend

#endif /* !THOR_USB_CHIMERA_HOOKS_HPP */
