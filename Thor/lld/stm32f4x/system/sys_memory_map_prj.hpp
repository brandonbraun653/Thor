/********************************************************************************
 *   File Name:
 *    sys_memory_map_prj.hpp
 *
 *   Description:
 *    Project specific memory mapping definitions.
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_DRIVER_SYS_MEMORY_MAP_PRJ_HPP
#define THOR_DRIVER_SYS_MEMORY_MAP_PRJ_HPP

#if defined( STM32F446xx )
#include <Thor/lld/stm32f4x/system/variant/sys_memory_map_stm32f446xx.hpp>
#endif

#endif /* !THOR_DRIVER_SYS_MEMORY_MAP_PRJ_HPP */