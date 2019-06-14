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

  struct Config
  {
    uint32_t BaudRate; /*!< This member configures the Usart communication baud rate.
                            The baud rate is computed using the following formula:
                            - IntegerDivider = ((PCLKx) / (8 * (husart->Init.BaudRate)))
                            - FractionalDivider = ((IntegerDivider - ((uint32_t) IntegerDivider)) * 8) + 0.5 */

    uint32_t WordLength; /*!< Specifies the number of data bits transmitted or received in a frame.
                              This parameter can be a value of @ref USART_Word_Length */

    uint32_t StopBits; /*!< Specifies the number of stop bits transmitted.
                            This parameter can be a value of @ref USART_Stop_Bits */

    uint32_t Parity; /*!< Specifies the parity mode.
                           This parameter can be a value of @ref USART_Parity
                           @note When parity is enabled, the computed parity is inserted
                                 at the MSB position of the transmitted data (9th bit when
                                 the word length is set to 9 data bits; 8th bit when the
                                 word length is set to 8 data bits). */

    uint32_t Mode; /*!< Specifies whether the Receive or Transmit mode is enabled or disabled.
                        This parameter can be a value of @ref USART_Mode */

    uint32_t CLKPolarity; /*!< Specifies the steady state of the serial clock.
                               This parameter can be a value of @ref USART_Clock_Polarity */

    uint32_t CLKPhase; /*!< Specifies the clock transition on which the bit capture is made.
                            This parameter can be a value of @ref USART_Clock_Phase */

    uint32_t CLKLastBit; /*!< Specifies whether the clock pulse corresponding to the last transmitted
                              data bit (MSB) has to be output on the SCLK pin in synchronous mode.
                              This parameter can be a value of @ref USART_Last_Bit */
  };

  static RegisterMap *const USART1 = reinterpret_cast<RegisterMap *const>( USART1_BASE );
  static RegisterMap *const USART2 = reinterpret_cast<RegisterMap *const>( USART2_BASE );
  static RegisterMap *const USART3 = reinterpret_cast<RegisterMap *const>( USART3_BASE );
  static RegisterMap *const UART4  = reinterpret_cast<RegisterMap *const>( UART4_BASE );
  static RegisterMap *const UART5  = reinterpret_cast<RegisterMap *const>( UART5_BASE );
  static RegisterMap *const USART6 = reinterpret_cast<RegisterMap *const>( USART6_BASE );

}    // namespace Thor::Driver::USART

#endif /* !THOR_HW_USART_TYPES_HPP */