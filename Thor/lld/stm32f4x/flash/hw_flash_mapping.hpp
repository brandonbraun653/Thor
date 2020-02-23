/********************************************************************************
 *  File Name:
 *    hw_flash_mapping.hpp
 *
 *  Description:
 *    Provides structures for conversion and mapping between data types for fast
 *    runtime performance of driver code.
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once 
#ifndef THOR_HW_FLASH_MAPPING_HPP
#define THOR_HW_FLASH_MAPPING_HPP

/* Driver Includes */
#include <Thor/lld/stm32f4x/flash/hw_flash_types.hpp>

namespace Thor::LLD::FLASH
{
  extern RegisterMap *const FLASH_PERIPH;
}

#endif /* !THOR_HW_FLASH_MAPPING_HPP */