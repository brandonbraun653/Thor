/********************************************************************************
 *  File Name:
 *    watchdog.hpp
 *
 *  Description:
 *    Watchdog includes
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_WATCHDOG_CONFIG_HPP
#define THOR_WATCHDOG_CONFIG_HPP

#if defined( TARGET_STM32F4 )
#include <Thor/lld/stm32f4x/wwdg/hw_wwdg_driver.hpp>
#include <Thor/lld/stm32f4x/wwdg/hw_wwdg_prj.hpp>
#include <Thor/lld/stm32f4x/wwdg/hw_wwdg_mapping.hpp>

#include <Thor/lld/stm32f4x/iwdg/hw_iwdg_driver.hpp>
#include <Thor/lld/stm32f4x/iwdg/hw_iwdg_prj.hpp>
#include <Thor/lld/stm32f4x/iwdg/hw_iwdg_mapping.hpp>
#endif

#endif  /* !THOR_WATCHDOG_CONFIG_HPP */