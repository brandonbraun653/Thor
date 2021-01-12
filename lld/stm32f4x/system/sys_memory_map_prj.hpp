/********************************************************************************
 *  File Name:
 *    sys_memory_map_prj.hpp
 *
 *  Description:
 *    Chip specific memory map includes
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_SYS_MEMORY_MAP_PROJECT_HPP
#define THOR_LLD_SYS_MEMORY_MAP_PROJECT_HPP

/*-------------------------------------------------
Common memory mapping values
-------------------------------------------------*/
#include <Thor/lld/stm32f4x/system/variant/sys_memory_map_stm32f4xxxx.hpp>

/*-------------------------------------------------
Chip specific additional memory mappings
-------------------------------------------------*/
#if defined( STM32F446xx )
#include <Thor/lld/stm32f4x/system/variant/sys_memory_map_stm32f446xx.hpp>
#endif

#endif  /* !THOR_LLD_SYS_MEMORY_MAP_PROJECT_HPP */
