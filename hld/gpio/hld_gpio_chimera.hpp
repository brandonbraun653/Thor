/********************************************************************************
 *  File Name:
 *    hld_gpio_chimera.hpp
 *
 *	 Description:
 *    Chimera hooks for implementing GPIO
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_GPIO_CHIMERA_HOOKS_HPP
#define THOR_GPIO_CHIMERA_HOOKS_HPP

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/gpio>

namespace Chimera::GPIO::Backend
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize();
  Chimera::Status_t reset();
  Driver_rPtr getDriver( const Port port, const Pin pin );
}    // namespace Chimera::GPIO::Backend

#endif /* !THOR_GPIO_CHIMERA_HOOKS_HPP */
