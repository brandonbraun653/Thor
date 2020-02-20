/********************************************************************************
 *   File Name:
 *    hw_nvic_register_stm32f446xx.hpp
 *
 *   Description:
 *    Implements NVIC register definitions for the STM32F446xx chips
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_DRIVER_NVIC_REGISTER_STM32F446XX_HPP
#define THOR_HW_DRIVER_NVIC_REGISTER_STM32F446XX_HPP

#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_NVIC ) && defined( STM32F446xx )

namespace Thor::Driver::Interrupt
{
  static constexpr uint32_t NVIC_PRIORITYGROUP_0 = 0x00000007u; /*!< 0 bits for pre-emption priority, 4 bits for subpriority */
  static constexpr uint32_t NVIC_PRIORITYGROUP_1 = 0x00000006u; /*!< 1 bits for pre-emption priority, 3 bits for subpriority */
  static constexpr uint32_t NVIC_PRIORITYGROUP_2 = 0x00000005u; /*!< 2 bits for pre-emption priority, 2 bits for subpriority */
  static constexpr uint32_t NVIC_PRIORITYGROUP_3 = 0x00000004u; /*!< 3 bits for pre-emption priority, 1 bits for subpriority */
  static constexpr uint32_t NVIC_PRIORITYGROUP_4 = 0x00000003u; /*!< 4 bits for pre-emption priority, 0 bits for subpriority */
}

#endif /* TARGET_STM32F4 && THOR_DRIVER_NVIC */
#endif /* !THOR_HW_DRIVER_FLASH_REGISTER_STM32F446XX_HPP */