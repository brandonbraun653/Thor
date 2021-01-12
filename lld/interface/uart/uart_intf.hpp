/********************************************************************************
 *  File Name:
 *    uart_intf.hpp
 *
 *  Description:
 *    LLD interface to the UART module
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef LLD_UART_INTERFACE_HPP
#define LLD_UART_INTERFACE_HPP

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/usart>
#include <Chimera/thread>

/* Thor Includes */
#include <Thor/lld/interface/uart/uart_types.hpp>

namespace Thor::LLD::UART
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  /**
   *  Initializes the low level driver
   *
   *  @return Chimera::Status_t
   */
  static Chimera::Status_t initialize()
  {
    // Currently no UARTs are available on supported STM32L4xxx chips.
    return Chimera::Status::OK;
  }

  /**
   *  Checks if the given hardware channel is supported on this device.
   *
   *  @param[in]  channel       The channel number to be checked
   *  @return bool
   */
  static bool isChannelSupported( const Chimera::Serial::Channel channel )
  {
    // Currently no UARTs are available on supported STM32L4xxx chips.
    return false;
  }
}    // namespace Thor::LLD::UART

#endif /* !LLD_UART_INTERFACE_HPP */
