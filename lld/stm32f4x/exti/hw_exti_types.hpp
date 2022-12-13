/******************************************************************************
 *  File Name:
 *    hw_exti_types.hpp
 *
 *  Description:
 *    STM32F4 Types for the EXTI Peripheral
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_HW_EXTI_TYPES_HPP
#define THOR_HW_EXTI_TYPES_HPP

/* C++ Includes */
#include <cstdint>

/* Driver Includes */
#include <Thor/lld/stm32f4x/exti/hw_exti_prj.hpp>

namespace Thor::LLD::EXTI
{
  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct RegisterMap
  {
    volatile uint32_t IMR1;   /**< EXTI Interrupt mask register 1,             Address offset: 0x00 */
    volatile uint32_t EMR1;   /**< EXTI Event mask register 1,                 Address offset: 0x04 */
    volatile uint32_t RTSR1;  /**< EXTI Rising trigger selection register 1,   Address offset: 0x08 */
    volatile uint32_t FTSR1;  /**< EXTI Falling trigger selection register 1,  Address offset: 0x0C */
    volatile uint32_t SWIER1; /**< EXTI Software interrupt event register 1,   Address offset: 0x10 */
    volatile uint32_t PR1;    /**< EXTI Pending register 1,                    Address offset: 0x14 */
  };

}    // namespace Thor::LLD::EXTI

#endif /* !THOR_HW_EXTI_TYPES_HPP*/
