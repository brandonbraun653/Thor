/********************************************************************************
 *  File Name:
 *    sys_config_types.hpp
 *
 *  Description:
 *    STM32L4 types for the SYSCFG module
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_HW_SYS_CFG_TYPES_HPP
#define THOR_HW_SYS_CFG_TYPES_HPP

/* STL Includes */
#include <cstdint>

/* Driver Includes */
#include <Thor/lld/stm32l4x/system/hw_sys_prj.hpp>


namespace Thor::LLD::SYS
{
  /*-------------------------------------------------------------------------------
  Structures
  -------------------------------------------------------------------------------*/
  struct RegisterMap
  {
    volatile uint32_t MEMRMP;      /**< SYSCFG memory remap register,                      Address offset: 0x00      */
    volatile uint32_t CFGR1;       /**< SYSCFG configuration register 1,                   Address offset: 0x04      */
    volatile uint32_t EXTICR[4];   /**< SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14 */
    volatile uint32_t SCSR;        /**< SYSCFG SRAM2 control and status register,          Address offset: 0x18      */
    volatile uint32_t CFGR2;       /**< SYSCFG configuration register 2,                   Address offset: 0x1C      */
    volatile uint32_t SWPR;        /**< SYSCFG SRAM2 write protection register,            Address offset: 0x20      */
    volatile uint32_t SKR;         /**< SYSCFG SRAM2 key register,                         Address offset: 0x24      */
  };
}  // namespace Thor::LLD::SYS

#endif  /* !THOR_HW_SYS_CFG_TYPES_HPP */
