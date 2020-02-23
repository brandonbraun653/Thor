/********************************************************************************
 *  File Name:
 *    power.hpp
 *
 *  Description:
 *    Common header for Thor Power that configures the driver based on which
 *    chip family is being compiled against.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_PWR_CONFIG_HPP
#define THOR_PWR_CONFIG_HPP

#if defined( TARGET_STM32F4 )
#include <Thor/lld/stm32f4x/power/hw_power_driver.hpp>
#endif

#endif /* !THOR_PWR_CONFIG_HPP */
