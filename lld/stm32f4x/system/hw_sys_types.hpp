/******************************************************************************
 *  File Name:
 *    sys_config_types.hpp
 *
 *  Description:
 *    STM32F4 types for the SYSCFG module
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_HW_SYS_CFG_TYPES_HPP
#define THOR_HW_SYS_CFG_TYPES_HPP

/* STL Includes */
#include <cstdint>

/* Driver Includes */
#include <Thor/lld/stm32f4x/system/hw_sys_prj.hpp>


namespace Thor::LLD::SYS
{
  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct RegisterMap
  {
    volatile uint32_t MEMRMP;      /**< SYSCFG memory remap register,                      Address offset: 0x00      */
    volatile uint32_t PMC;         /**< SYSCFG peripheral mode configuration register,     Address offset: 0x04      */
    volatile uint32_t EXTICR[ 4 ]; /**< SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14 */
    uint32_t RESERVED0;
    uint32_t RESERVED1;
    volatile uint32_t CMPCR;       /**< SYSCFG compensation cell control register,         Address offset: 0x20      */
    uint32_t RESERVED2;
    uint32_t RESERVED3;
    volatile uint32_t CFGR;        /**< SYSCFG configuration register,                     Address offset: 0x2C      */
  };
}    // namespace Thor::LLD::SYS

#endif /* !THOR_HW_SYS_CFG_TYPES_HPP */
