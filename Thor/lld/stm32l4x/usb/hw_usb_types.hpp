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

/* Driver Includes */
#include <Thor/lld/common/registers/field_accessor.hpp>
#include <Thor/lld/stm32l4x/usb/hw_usb_prj.hpp>

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


  /*-------------------------------------------------------------------------------
  Classes
  -------------------------------------------------------------------------------*/
  /*-------------------------------------------------
  Control Register (CNTR)
  -------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CNTR, CNTR_ALL_Msk, CNTR_ALL, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CNTR, CNTR_CTRM, CTRM, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CNTR, CNTR_PMAOVRM, PMAOVRM, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CNTR, CNTR_ERRM, ERRM, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CNTR, CNTR_WKUPM, WKUPM, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CNTR, CNTR_SUSPM, SUSPM, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CNTR, CNTR_RESETM, RESETM, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CNTR, CNTR_SOFM, SOFM, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CNTR, CNTR_ESOFM, ESOFM, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CNTR, CNTR_L1REQM, L1REQM, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CNTR, CNTR_L1RESUME, L1RESUME, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CNTR, CNTR_RESUME, RESUME, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CNTR, CNTR_FSUSP, FSUSP, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CNTR, CNTR_LPMODE, LPMODE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CNTR, CNTR_PDWN, PDWN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CNTR, CNTR_FRES, FRES, BIT_ACCESS_RW );


  /*-------------------------------------------------
  Battery Charging Detector Register (BCDR)
  -------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, BCDR, BCDR_DPPU, DPPU, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, BCDR, BCDR_PS2DET, PS2DET, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, BCDR, BCDR_SDET, SDET, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, BCDR, BCDR_PDET, PDET, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, BCDR, BCDR_DCDET, DCDET, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, BCDR, BCDR_SDEN, SDEN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, BCDR, BCDR_PDEN, PDEN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, BCDR, BCDR_DCDEN, DCDEN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, BCDR, BCDR_BCDEN, BCDEN, BIT_ACCESS_RW );


}    // namespace Thor::LLD::USB

#endif /* !THOR_HW_USB_TYPES_HPP */
