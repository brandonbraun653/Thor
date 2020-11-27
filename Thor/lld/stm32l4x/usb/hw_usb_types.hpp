/********************************************************************************
 *  File Name:
 *    hw_usb_types.hpp
 *
 *  Description:
 *    STM32L4 Types for the USB Peripheral
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_HW_USB_TYPES_HPP
#define THOR_HW_USB_TYPES_HPP

/* C++ Includes */
#include <cstdint>

namespace Thor::LLD::USB
{
  /*-------------------------------------------------------------------------------
  Structures
  -------------------------------------------------------------------------------*/
  struct RegisterMap
  {
    volatile uint16_t EP0R; /**< USB Endpoint 0 register,                Address offset: 0x00 */
    volatile uint16_t _RESERVED0;
    volatile uint16_t EP1R; /**< USB Endpoint 1 register,                Address offset: 0x04 */
    volatile uint16_t _RESERVED1;
    volatile uint16_t EP2R; /**< USB Endpoint 2 register,                Address offset: 0x08 */
    volatile uint16_t _RESERVED2;
    volatile uint16_t EP3R; /**< USB Endpoint 3 register,                Address offset: 0x0C */
    volatile uint16_t _RESERVED3;
    volatile uint16_t EP4R; /**< USB Endpoint 4 register,                Address offset: 0x10 */
    volatile uint16_t _RESERVED4;
    volatile uint16_t EP5R; /**< USB Endpoint 5 register,                Address offset: 0x14 */
    volatile uint16_t _RESERVED5;
    volatile uint16_t EP6R; /**< USB Endpoint 6 register,                Address offset: 0x18 */
    volatile uint16_t _RESERVED6;
    volatile uint16_t EP7R; /**< USB Endpoint 7 register,                Address offset: 0x1C */
    volatile uint16_t _RESERVED7[ 17 ];
    volatile uint16_t CNTR; /**< Control register,                       Address offset: 0x40 */
    volatile uint16_t _RESERVED8;
    volatile uint16_t ISTR; /**< Interrupt status register,              Address offset: 0x44 */
    volatile uint16_t _RESERVED9;
    volatile uint16_t FNR; /**< Frame number register,                  Address offset: 0x48 */
    volatile uint16_t _RESERVEDA;
    volatile uint16_t DADDR; /**< Device address register,                Address offset: 0x4C */
    volatile uint16_t _RESERVEDB;
    volatile uint16_t BTABLE; /**< Buffer Table address register,          Address offset: 0x50 */
    volatile uint16_t _RESERVEDC;
    volatile uint16_t LPMCSR; /**< LPM Control and Status register,        Address offset: 0x54 */
    volatile uint16_t _RESERVEDD;
    volatile uint16_t BCDR; /**< Battery Charging detector register,     Address offset: 0x58 */
    volatile uint16_t _RESERVEDE;
  };
}    // namespace Thor::LLD::USB

#endif /* !THOR_HW_USB_TYPES_HPP */
