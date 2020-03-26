/********************************************************************************
 *  File Name:
 *    hw_flash_types.hpp
 *
 *  Description:
 *    STM32L4 Types for the FLASH Peripheral
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_FLASH_TYPES_HPP
#define THOR_HW_FLASH_TYPES_HPP

/* C++ Includes */
#include <cstdint>
#include <array>

/* Driver Includes */
#include <Thor/lld/common/registers/field_accessor.hpp>
#include <Thor/lld/stm32l4x/flash/hw_flash_prj.hpp>

namespace Thor::LLD::FLASH
{
  struct RegisterMap
  {
    volatile uint32_t ACR;       /*!< FLASH access control register,            Address offset: 0x00 */
    volatile uint32_t PDKEYR;    /*!< FLASH power down key register,            Address offset: 0x04 */
    volatile uint32_t KEYR;      /*!< FLASH key register,                       Address offset: 0x08 */
    volatile uint32_t OPTKEYR;   /*!< FLASH option key register,                Address offset: 0x0C */
    volatile uint32_t SR;        /*!< FLASH status register,                    Address offset: 0x10 */
    volatile uint32_t CR;        /*!< FLASH control register,                   Address offset: 0x14 */
    volatile uint32_t ECCR;      /*!< FLASH ECC register,                       Address offset: 0x18 */
    volatile uint32_t RESERVED1; /*!< Reserved1,                                Address offset: 0x1C */
    volatile uint32_t OPTR;      /*!< FLASH option register,                    Address offset: 0x20 */
    volatile uint32_t PCROP1SR;  /*!< FLASH bank1 PCROP start address register, Address offset: 0x24 */
    volatile uint32_t PCROP1ER;  /*!< FLASH bank1 PCROP end address register,   Address offset: 0x28 */
    volatile uint32_t WRP1AR;    /*!< FLASH bank1 WRP area A address register,  Address offset: 0x2C */
    volatile uint32_t WRP1BR;    /*!< FLASH bank1 WRP area B address register,  Address offset: 0x30 */
  };

   using PeriphRegisterList = std::array<RegisterMap *, NUM_FLASH_PERIPHS>;

   /*------------------------------------------------
   Auxiliary Control Register (ACR)
   ------------------------------------------------*/
   REG_ACCESSOR( RegisterMap, ACR, ACR_LATENCY_Msk, LATENCY, BIT_ACCESS_RW );

   namespace Config::ACR
   {
    
   }

}    // namespace Thor::LLD::FLASH

#endif /* !THOR_HW_FLASH_TYPES_HPP*/
