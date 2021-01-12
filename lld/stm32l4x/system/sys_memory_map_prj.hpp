/********************************************************************************
 *  File Name:
 *    sys_memory_map_prj.hpp
 *
 *  Description:
 *    Chip specific memory map includes
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_SYS_MEMORY_MAP_PROJECT_HPP
#define THOR_LLD_SYS_MEMORY_MAP_PROJECT_HPP

/*-------------------------------------------------
Common memory mapping values
-------------------------------------------------*/
#include <Thor/lld/stm32l4x/system/variant/sys_memory_map_stm32l4xxxx.hpp>

/*-------------------------------------------------
Chip specific additional memory mappings
-------------------------------------------------*/
#if defined( STM32L432xx )
#include <Thor/lld/stm32l4x/system/variant/sys_memory_map_stm32l432kc.hpp>
#endif 

#endif  /* !THOR_LLD_SYS_MEMORY_MAP_PROJECT_HPP */
