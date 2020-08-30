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
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/serial/serial_intf.hpp>
#include <Thor/lld/interface/serial/serial_types.hpp>
#include <Thor/lld/interface/usart/usart_types.hpp>

namespace Thor::LLD::USART
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  /**
   *  Initializes the low level driver
   *
   *  @return Chimera::Status_t
   */
  Chimera::Status_t initialize();

  /**
   *  Checks if the given hardware channel is supported on this device.
   *
   *  @param[in]  channel       The channel number to be checked
   *  @return bool
   */
  bool isChannelSupported( const Chimera::Serial::Channel channel );

  /**
   *  Gets a raw pointer to the driver for a particular channel
   *
   *  @param[in] channel        The channel to grab
   *  @return IDriver_rPtr      Instance of the driver for the requested channel
   */
  IDriver_rPtr getDriver( const Chimera::Serial::Channel channel );

  /**
   *  Get's the resource index associated with a particular channel. If not
   *  supported, will return INVALID_RESOURCE_INDEX
   *
   *  @param[in]  channel       The channel number to be checked
   *  @return RIndexType
   */
  RIndexType getResourceIndex( const Chimera::Serial::Channel channel );

  /**
   *  Looks up a resource index based on a raw peripheral instance
   *
   *  @param[in]  address       The peripheral address
   *  @return RIndexType
   */
  RIndexType getResourceIndex( const std::uintptr_t address );

  /*-------------------------------------------------------------------------------
  Classes
  -------------------------------------------------------------------------------*/
  class IDriver : virtual public Thor::LLD::Serial::Basic, virtual public Thor::LLD::Serial::Extended
  {
  public:
    virtual ~IDriver() = default;
  };
}    // namespace Thor::LLD::USART

#endif /* !LLD_USART_INTERFACE_HPP */
