/********************************************************************************
 *   File Name:
 *    hw_power_mapping.hpp
 *
 *   Description:
 *    Provides structures for conversion and mapping between data types for fast
 *    runtime performance of driver code.
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once 
#ifndef THOR_HW_POWER_MAPPING_HPP
#define THOR_HW_POWER_MAPPING_HPP

/* Driver Includes */
#include <Thor/lld/stm32f4x/power/hw_power_types.hpp>

#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_PWR )

namespace Thor::Driver::PWR
{
  extern RegisterMap *const PWR_PERIPH;
}

#endif /* TARGET_STM32F4 && THOR_DRIVER_PWR */
#endif /* !THOR_HW_POWER_MAPPING_HPP */