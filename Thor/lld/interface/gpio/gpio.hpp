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

#if defined( TARGET_LLD_MOCK )
#include <Thor/lld/interface/gpio/mock/gpio_mock.hpp>
#elif defined( TARGET_STM32F4 )
#include <Thor/lld/stm32f4x/gpio/hw_gpio_driver.hpp>
#else
#pragma message( "Unknown target device for GPIO low level driver" )
#endif

#endif /* !THOR_GPIO_CONFIG_HPP */