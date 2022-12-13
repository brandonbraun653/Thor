/******************************************************************************
 *  File Name:
 *    sys_intf.hpp
 *
 *  Description:
 *    STM32 LLD SYS Interface Spec
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_SYS_DRIVER_INTERFACE_HPP
#define THOR_LLD_SYS_DRIVER_INTERFACE_HPP

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/gpio>

/* Thor Includes */
#include <Thor/lld/common/types.hpp>


namespace Thor::LLD::SYS
{
  /*---------------------------------------------------------------------------
  Public Functions (Implemented by the driver layer)
  ---------------------------------------------------------------------------*/
  /**
   *  Architecturally dependent check on whether or not the CPU is servicing
   *  an ISR at the time of this function call.
   *
   *  @return bool
   */
  bool inISR();

  /**
   *  Instruct the processor to issue a reset command
   *  @return void
   */
  void softwareReset();

  /**
   * @brief Checks if the debugger is attached to the system
   *
   * @return true
   * @return false
   */
  bool isDebuggerAttached();

  /**
   *  Configures the SYSCFG registers to select the proper pin to interrupt
   *  the external interrupt lines 0-15.
   *
   *  @param[in]  port      GPIO port being configured
   *  @param[in]  pin       GPIO pin being configured
   *  @return void
   */
  void configureExtiSource( const Chimera::GPIO::Port port, const uint8_t pin );


  /*---------------------------------------------------------------------------
  Public Functions (Implemented at the interface layer)
  ---------------------------------------------------------------------------*/
  /**
   *  Checks if the hardware is supported on this device.
   *  @return bool
   */
  bool isSupported();

  /**
   *  Looks up an index value that can be used to access distributed resources
   *  associated with a peripheral instance. If the address is invalid, this will
   *  return INVALID_RESOURCE_INDEX.
   *
   *  @param[in]  address       Memory address the peripheral is mapped to
   *  @return RIndex_t
   */
  RIndex_t getResourceIndex( const std::uintptr_t address );

  /**
   *  Gets the resource index of a system level peripheral type
   *
   *  @param[in]  periph        The type of peripheral to look up
   *  @return RIndex_t
   */
  RIndex_t getResourceIndex( const Chimera::Peripheral::Type periph );

  /**
   *  Enables the peripheral clock
   *  @return void
   */
  void clockEnable();

  /**
   *  Disables the peripheral clock
   *  @return void
   */
  void clockDisable();

}    // namespace Thor::LLD::SYS

#endif /* !THOR_LLD_SYS_DRIVER_INTERFACE_HPP */
