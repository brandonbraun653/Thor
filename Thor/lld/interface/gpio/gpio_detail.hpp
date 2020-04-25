/********************************************************************************
 *  File Name:
 *    gpio_detail.hpp
 *
 *  Description:
 *    Includes the LLD specific headers for chip implementation details
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_GPIO_INTF_DETAIL_HPP
#define THOR_LLD_GPIO_INTF_DETAIL_HPP


#if defined( TARGET_STM32F4 )
#include <Thor/lld/stm32f4x/gpio/hw_gpio_driver.hpp>
#include <Thor/lld/stm32f4x/gpio/hw_gpio_mapping.hpp>
#include <Thor/lld/stm32f4x/gpio/hw_gpio_prj.hpp>
#include <Thor/lld/stm32f4x/gpio/hw_gpio_types.hpp>
#elif defined( TARGET_STM32L4 )
#include <Thor/lld/stm32l4x/gpio/hw_gpio_driver.hpp>
#include <Thor/lld/stm32l4x/gpio/hw_gpio_mapping.hpp>
#include <Thor/lld/stm32l4x/gpio/hw_gpio_prj.hpp>
#include <Thor/lld/stm32l4x/gpio/hw_gpio_types.hpp>
#else
#pragma message( "gpio_detail.hpp: Unknown target for LLD" )
#endif 

#endif  /* !THOR_LLD_GPIO_INTF_DETAIL_HPP */