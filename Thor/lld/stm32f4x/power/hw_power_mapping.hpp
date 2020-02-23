/********************************************************************************
 *  File Name:
 *    hw_power_mapping.hpp
 *
 *  Description:
 *    Provides structures for conversion and mapping between data types for fast
 *    runtime performance of driver code.
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once 
#ifndef THOR_HW_POWER_MAPPING_HPP
#define THOR_HW_POWER_MAPPING_HPP

/* Driver Includes */
#include <Thor/lld/stm32f4x/power/hw_power_types.hpp>

namespace Thor::LLD::PWR
{
  extern RegisterMap *const PWR_PERIPH;
}

#endif /* !THOR_HW_POWER_MAPPING_HPP */