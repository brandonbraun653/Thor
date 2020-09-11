/********************************************************************************
 *  File Name:
 *    gpio_sim_types.hpp
 *
 *  Description:
 *    Types for the simulator
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_GPIO_SIM_TYPES_HPP
#define THOR_LLD_GPIO_SIM_TYPES_HPP

/* STL Includes */
#include <cstdint>

namespace Thor::LLD::GPIO
{
  struct RegisterMap
  {
    uint32_t dummyRegister;
  };
}  // namespace Thor::LLD::GPIO

#endif  /* !THOR_LLD_GPIO_SIM_TYPES_HPP */
