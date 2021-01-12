/********************************************************************************
 *  File Name:
 *    usart_sim_types.hpp
 *
 *  Description:
 *    Simulator types
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_USART_SIM_TYPES_HPP
#define THOR_LLD_USART_SIM_TYPES_HPP

/* STL Includes */
#include <cstdint>

namespace Thor::LLD::USART
{
  struct RegisterMap
  {
    uint32_t dummyRegister;
  };
}  // namespace Thor::LLD::USART

#endif  /* !THOR_LLD_USART_SIM_TYPES_HPP */
