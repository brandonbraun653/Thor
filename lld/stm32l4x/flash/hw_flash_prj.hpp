/******************************************************************************
 *  File Name:
 *    hw_flash_prj.hpp
 *
 *  Description:
 *    Pulls in target specific definitions and resources used in the actual driver
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_HW_FLASH_PROJECT_HPP
#define THOR_HW_FLASH_PROJECT_HPP

#if defined( STM32L432xx )
#include <Thor/lld/stm32l4x/flash/variant/hw_flash_register_stm32l432kc.hpp>
#endif

#endif /* !THOR_HW_FLASH_PROJECT_HPP */
