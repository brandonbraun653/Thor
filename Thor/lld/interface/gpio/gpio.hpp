/********************************************************************************
 *  File Name:
 *    gpio.hpp
 *
 *  Description:
 *    Common header for Thor GPIO that configures the driver based on which
 *    chip family is being compiled against.
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_GPIO_CONFIG_HPP
#define THOR_GPIO_CONFIG_HPP

#include <Thor/hld/common/preprocessor.hpp>

#if defined( TARGET_STM32F4 )
#include <Thor/lld/stm32f4x/gpio/hw_gpio_driver.hpp>
#endif

#if defined( TARGET_STM32F7 )
#include <Thor/drivers/f7/gpio/hw_gpio_driver.hpp>
#endif

#endif /* !THOR_GPIO_CONFIG_HPP */