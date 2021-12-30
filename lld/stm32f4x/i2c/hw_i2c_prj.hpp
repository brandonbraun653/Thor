/******************************************************************************
 *  File Name:
 *    hw_i2c_prj.hpp
 *
 *  Description:
 *    Pulls in target specific definitions and resources used in the driver
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_HW_I2C_PROJECT_HPP
#define THOR_HW_I2C_PROJECT_HPP

/*-----------------------------------------------------------------------------
All STM32F4 Devices
-----------------------------------------------------------------------------*/
#include <Thor/lld/stm32f4x/i2c/variant/hw_i2c_register_stm32f4xxxx.hpp>

/*-----------------------------------------------------------------------------
Chip specific STM32F4 devices
-----------------------------------------------------------------------------*/
#if defined( STM32F446xx )
#include <Thor/lld/stm32f4x/i2c/variant/hw_i2c_register_stm32f446re.hpp>
#endif

#endif  /* !THOR_HW_I2C_PROJECT_HPP */
