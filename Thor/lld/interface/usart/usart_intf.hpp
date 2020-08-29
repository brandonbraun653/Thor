/********************************************************************************
 *  File Name:
 *    usart_intf.hpp
 *
 *  Description:
 *    STM32 Driver UART Interface
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef LLD_USART_INTERFACE_HPP
#define LLD_USART_INTERFACE_HPP

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/usart>
#include <Chimera/thread>

/* Thor Includes */
#include <Thor/lld/interface/usart/usart_types.hpp>

namespace Thor::LLD::USART
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  /**
   *  Initializes the low level driver
   */
  extern Chimera::Status_t initialize();

  /**
   *  Checks if the given hardware channel is supported on this device.
   *
   *  @param[in]  channel       The channel number to be checked
   *  @return bool
   */
  bool isChannelSupported( const Chimera::Serial::Channel channel );

}    // namespace Thor::LLD::USART

#endif /* !LLD_USART_INTERFACE_HPP */
