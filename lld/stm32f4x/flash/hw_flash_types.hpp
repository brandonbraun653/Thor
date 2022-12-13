/******************************************************************************
 *  File Name:
 *    hw_flash_types.hpp
 *
 *  Description:
 *    Implements Flash peripheral types
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_HW_DRIVER_FLASH_TYPES_HPP
#define THOR_HW_DRIVER_FLASH_TYPES_HPP

/* C++ Includes */
#include <cstdint>

/* Driver Includes */
#include <Thor/lld/common/registers/field_accessor.hpp>
#include <Thor/lld/stm32f4x/flash/hw_flash_prj.hpp>

namespace Thor::LLD::FLASH
{
  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct RegisterMap
  {
    volatile uint32_t ACR;     /**< FLASH access control register,   Address offset: 0x00 */
    volatile uint32_t KEYR;    /**< FLASH key register,              Address offset: 0x04 */
    volatile uint32_t OPTKEYR; /**< FLASH option key register,       Address offset: 0x08 */
    volatile uint32_t SR;      /**< FLASH status register,           Address offset: 0x0C */
    volatile uint32_t CR;      /**< FLASH control register,          Address offset: 0x10 */
    volatile uint32_t OPTCR;   /**< FLASH option control register ,  Address offset: 0x14 */
    volatile uint32_t OPTCR1;  /**< FLASH option control register 1, Address offset: 0x18 */
  };

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, ACR, ACR_LATENCY_Msk, LATENCY, BIT_ACCESS_RW );

}    // namespace Thor::LLD::FLASH

#endif /* !THOR_HW_DRIVER_FLASH_TYPES_HPP */