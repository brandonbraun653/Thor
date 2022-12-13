/******************************************************************************
 *  File Name:
 *    flash.hpp
 *
 *  Description:
 *    Common header for Thor Flash that configures the driver based on which
 *    chip family is being compiled against.
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_FLASH_DETAIL_HPP
#define THOR_FLASH_DETAIL_HPP

#if defined( TARGET_LLD_MOCK )
#include <Thor/lld/interface/flash/mock/flash_mock.hpp>
#elif defined( TARGET_STM32F4 )
#include <Thor/lld/stm32f4x/flash/hw_flash_prj.hpp>
#include <Thor/lld/stm32f4x/flash/hw_flash_types.hpp>
#elif defined( TARGET_STM32L4 )
#include <Thor/lld/stm32l4x/flash/hw_flash_prj.hpp>
#include <Thor/lld/stm32l4x/flash/hw_flash_types.hpp>
#else
#pragma message( "flash_detail.hpp: Unknown target for LLD" )
#endif

#endif /* !THOR_FLASH_DETAIL_HPP */
