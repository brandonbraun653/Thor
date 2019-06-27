/********************************************************************************
 *   File Name:
 *    hw_rcc_driver.hpp
 *
 *   Description:
 *    STM32F4 RCC driver interface
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_DRIVER_RCC_HPP
#define THOR_HW_DRIVER_RCC_HPP

/* C++ Includes */
#include <cstdlib>

/* Chimera Includes */
#include <Chimera/types/common_types.hpp>
#include <Chimera/types/peripheral_types.hpp>

namespace Thor::Driver::RCC
{
  
  Chimera::Status_t enablePeripheralClock( const Chimera::Peripheral::Type periph, const size_t instance );
}

#endif /* !THOR_HW_DRIVER_RCC_HPP */