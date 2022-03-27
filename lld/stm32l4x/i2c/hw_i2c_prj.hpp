/******************************************************************************
 *  File Name:
 *    hw_i2c_prj.hpp
 *
 *  Description:
 *    Pulls in target specific definitions and resources used in the driver
 *
 *  2022 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_HW_I2C_PROJECT_HPP
#define THOR_HW_I2C_PROJECT_HPP

/*-----------------------------------------------------------------------------
All STM32F4 Devices
-----------------------------------------------------------------------------*/
#include <Thor/lld/stm32l4x/i2c/variant/hw_i2c_register_stm32l4xxxx.hpp>

/*-----------------------------------------------------------------------------
Chip specific STM32F4 devices
-----------------------------------------------------------------------------*/
#if defined( STM32L432xx )
#include <Thor/lld/stm32l4x/i2c/variant/hw_i2c_register_stm32l432kc.hpp>
#endif

#endif  /* !THOR_HW_I2C_PROJECT_HPP */
