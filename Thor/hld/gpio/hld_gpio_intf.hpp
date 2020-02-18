/********************************************************************************
 *  File Name:
 *    hld_gpio_intf.hpp
 *
 *	 Description:
 *    High level driver GPIO interface spec
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_HLD_GPIO_INTERFACE_HPP
#define THOR_HLD_GPIO_INTERFACE_HPP


namespace Thor::GPIO
{

  class HWInterface
  {

  };

  class IGPIO // Needs to inherit from the Chimera interface
  {
  public:
    virtual ~IGPIO() = default;



  };
}

#endif  /* !THOR_HLD_GPIO_INTERFACE_HPP */
