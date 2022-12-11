/********************************************************************************
 *  File Name:
 *    serial_intf.hpp
 *
 *  Description:
 *    STM32 Driver Model for Serial Communication
 *
 *  2019-2022 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_DRIVER_MODEL_SERIAL_HPP
#define THOR_DRIVER_MODEL_SERIAL_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <Chimera/serial>
#include <Thor/lld/interface/serial/serial_types.hpp>
#include <cstdint>
#include <cstdlib>


namespace Thor::LLD::Serial
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void registerInterface( const Chimera::Serial::Channel channel, const HwInterface &intf );
}    // namespace Thor::LLD::Serial


#endif /* !THOR_DRIVER_MODEL_SERIAL_HPP */