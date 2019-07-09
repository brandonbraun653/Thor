/********************************************************************************
 *   File Name:
 *    hw_nvic_register_stm32f446xx.hpp
 *
 *   Description:
 *    Implements NVIC register definitions for the STM32F446xx chips
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_DRIVER_NVIC_REGISTER_STM32F446XX_HPP
#define THOR_HW_DRIVER_NVIC_REGISTER_STM32F446XX_HPP

/* Driver Includes */
#include <Thor/headers.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_NVIC == 1 )

namespace Thor::Driver::Interrupt
{
  // Currently just a placeholder because all the register definitions are taken
  // care of inside of the CMSIS drivers.
}

#endif /* TARGET_STM32F4 && THOR_DRIVER_NVIC */
#endif /* !THOR_HW_DRIVER_FLASH_REGISTER_STM32F446XX_HPP */