/********************************************************************************
 *  File Name:
 *    hld_gpio_user.hpp
 *
 *	 Description:
 *    User interface to the Thor GPIO objects
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_GPIO_DRIVER_USER_INTERFACE_HPP
#define THOR_GPIO_DRIVER_USER_INTERFACE_HPP


namespace Chimera::GPIO
{

  // Proably need to create a registration structure with function pointers
  void register_driver();

}


namespace Thor::GPIO
{

  void create_things();

  void create_more_things();

}

#endif  /* !THOR_GPIO_DRIVER_USER_INTERFACE_HPP */
