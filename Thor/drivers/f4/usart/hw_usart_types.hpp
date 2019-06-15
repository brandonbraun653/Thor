/********************************************************************************
 *   File Name:
 *    hw_usart_types.hpp
 *
 *   Description:
 *    STM32 Types for the USART Peripheral
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_USART_TYPES_HPP
#define THOR_HW_USART_TYPES_HPP

/* C++ Includes */
#include <cstdint>

/* Driver Includes */
#include <Thor/drivers/f4/usart/hw_usart_prj.hpp>

namespace Thor::Driver::USART
{
  struct RegisterMap
  {
    volatile uint32_t SR;   /**< USART Status register,                   Address offset: 0x00 */
    volatile uint32_t DR;   /**< USART Data register,                     Address offset: 0x04 */
    volatile uint32_t BRR;  /**< USART Baud rate register,                Address offset: 0x08 */
    volatile uint32_t CR1;  /**< USART Control register 1,                Address offset: 0x0C */
    volatile uint32_t CR2;  /**< USART Control register 2,                Address offset: 0x10 */
    volatile uint32_t CR3;  /**< USART Control register 3,                Address offset: 0x14 */
    volatile uint32_t GTPR; /**< USART Guard time and prescaler register, Address offset: 0x18 */
  };

  static RegisterMap *const USART1_PERIPH = reinterpret_cast<RegisterMap *const>( USART1_BASE_ADDR );
  static RegisterMap *const USART2_PERIPH = reinterpret_cast<RegisterMap *const>( USART2_BASE_ADDR );
  static RegisterMap *const USART3_PERIPH = reinterpret_cast<RegisterMap *const>( USART3_BASE_ADDR );
  static RegisterMap *const USART6_PERIPH = reinterpret_cast<RegisterMap *const>( USART6_BASE_ADDR );

}    // namespace Thor::Driver::USART

#endif /* !THOR_HW_USART_TYPES_HPP */