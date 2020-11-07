/********************************************************************************
 *  File Name:
 *    sys_config.hpp
 *
 *  Description:
 *    API for interacting with the SYSCFG registers
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_SYS_CFG_HPP
#define THOR_LLD_SYS_CFG_HPP

/* Chimera Includes */
#include <Chimera/gpio>

namespace Thor::LLD::SYS
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  /**
   *  Configures the SYSCFG registers to select the proper pin to interrupt
   *  the external interrupt lines 0-15.
   *
   *  @param[in]  port      GPIO port being configured
   *  @param[in]  pin       GPIO pin being configured
   *  @return void
   */
  void configureExtiSource( const Chimera::GPIO::Port port, const uint8_t pin );

}  // namespace Thor::LLD::SYS

#endif  /* !THOR_LLD_SYS_CFG_HPP */
