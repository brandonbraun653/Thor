/********************************************************************************
 *  File Name:
 *    hw_iwdg_types.hpp
 *
 *  Description:
 *    LLD types for the IWDG Peripheral
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_IWDG_TYPES_HPP
#define THOR_HW_IWDG_TYPES_HPP

/* C++ Includes */
#include <cstdint>

/* Driver Includes */
#include <Thor/lld/stm32l4x/iwdg/hw_iwdg_prj.hpp>

namespace Thor::LLD::IWDG
{
  /*-------------------------------------------------------------------------------
  Enumerations
  -------------------------------------------------------------------------------*/


  /*-------------------------------------------------------------------------------
  Structures
  -------------------------------------------------------------------------------*/
  struct RegisterMap
  {
    volatile uint32_t KR;   /**< IWDG Key register,       Address offset: 0x00 */
    volatile uint32_t PR;   /**< IWDG Prescaler register, Address offset: 0x04 */
    volatile uint32_t RLR;  /**< IWDG Reload register,    Address offset: 0x08 */
    volatile uint32_t SR;   /**< IWDG Status register,    Address offset: 0x0C */
    volatile uint32_t WINR; /**< IWDG Window register,    Address offset: 0x10 */
  };

}    // namespace Thor::LLD::IWDG

#endif /* !THOR_HW_IWDG_TYPES_HPP*/
