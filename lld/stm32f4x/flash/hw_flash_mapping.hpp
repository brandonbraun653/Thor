/********************************************************************************
 *  File Name:
 *    hw_flash_mapping.hpp
 *
 *  Description:
 *    Provides structures for conversion and mapping between data types for fast
 *    runtime performance of driver code.
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_FLASH_MAPPING_HPP
#define THOR_HW_FLASH_MAPPING_HPP

/* Driver Includes */
#include <Thor/lld/stm32f4x/flash/hw_flash_types.hpp>

namespace Thor::LLD::FLASH
{
  /*-------------------------------------------------
  Define Memory Mapped Structs for Peripheral
  -------------------------------------------------*/
#if defined( STM32_FLASH_PERIPH_AVAILABLE )
  extern RegisterMap *FLASH_PERIPH;
#endif
}

#endif /* !THOR_HW_FLASH_MAPPING_HPP */