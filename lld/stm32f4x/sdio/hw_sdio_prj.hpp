/******************************************************************************
 *  File Name:
 *    hw_sdio_prj.hpp
 *
 *  Description:
 *    Pulls in target specific definitions and resources used in the actual driver
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_HW_SDIO_PROJECT_HPP
#define THOR_HW_SDIO_PROJECT_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Thor/lld/stm32f4x/sdio/variant/hw_sdio_register_stm32f4xxxx.hpp>

#if defined( STM32F446xx )
#include <Thor/lld/stm32f4x/sdio/variant/hw_sdio_register_stm32f446xx.hpp>
#endif

#endif  /* !THOR_HW_SDIO_PROJECT_HPP */
