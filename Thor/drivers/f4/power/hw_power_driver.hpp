/********************************************************************************
 *   File Name:
 *    hw_power_driver.hpp
 *
 *   Description:
 *    STM32F4 PWR driver interface
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_DRIVER_PWR_HPP
#define THOR_HW_DRIVER_PWR_HPP

/* Driver Includes */
#include <Thor/headers.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_PWR == 1 )

namespace Thor::Driver::PWR
{

}

#endif /* TARGET_STM32F4 && THOR_DRIVER_PWR */
#endif /* !THOR_HW_DRIVER_PWR_HPP */