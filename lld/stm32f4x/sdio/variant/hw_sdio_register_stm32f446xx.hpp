/******************************************************************************
 *  File Name:
 *    hw_sdio_register_stm32f446xx.hpp
 *
 *  Description:
 *    SDIO register definitions for the STM32F446xx series chips.
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_HW_SDIO_REGISTER_HPP
#define THOR_HW_SDIO_REGISTER_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstddef>

/*-----------------------------------------------------------------------------
Macros
-----------------------------------------------------------------------------*/
#define STM32_SDIO1_PERIPH_AVAILABLE

namespace Thor::LLD::SDIO
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t NUM_SDIO_PERIPHS = 1;

}  // namespace Th

#endif  /* !THOR_HW_SDIO_REGISTER_HPP */
