/********************************************************************************
 *  File Name:
 *    hw_power_driver.hpp
 *
 *  Description:
 *    Declares the LLD interface to the STM32L4 series POWER hardware.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_POWER_DRIVER_STM32L4_HPP
#define THOR_HW_POWER_DRIVER_STM32L4_HPP

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/power>
#include <Chimera/thread>

/* Thor Includes */
#include <Thor/lld/stm32l4x/power/hw_power_types.hpp>
#include <Thor/lld/stm32l4x/power/hw_power_mapping.hpp>

namespace Thor::LLD::PWR
{
}    // namespace Thor::LLD::POWER

#endif /* !THOR_HW_POWER_DRIVER_STM32L4_HPP */
