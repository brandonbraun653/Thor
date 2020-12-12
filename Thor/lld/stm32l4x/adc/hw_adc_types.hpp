/********************************************************************************
 *  File Name:
 *    hw_adc_types.hpp
 *
 *  Description:
 *    LLD types for the ADC Peripheral
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_ADC_TYPES_HPP
#define THOR_HW_ADC_TYPES_HPP

/* C++ Includes */
#include <cstdint>

/* Driver Includes */
#include <Thor/lld/stm32l4x/adc/hw_adc_prj.hpp>

namespace Thor::LLD::ADC
{
  /*-------------------------------------------------------------------------------
  Enumerations
  -------------------------------------------------------------------------------*/


  /*-------------------------------------------------------------------------------
  Structures
  -------------------------------------------------------------------------------*/
  struct RegisterMap
  {
    volatile uint32_t ISR;            /**< ADC interrupt and status register,             Address offset: 0x00 */
    volatile uint32_t IER;            /**< ADC interrupt enable register,                 Address offset: 0x04 */
    volatile uint32_t CR;             /**< ADC control register,                          Address offset: 0x08 */
    volatile uint32_t CFGR;           /**< ADC configuration register 1,                  Address offset: 0x0C */
    volatile uint32_t CFGR2;          /**< ADC configuration register 2,                  Address offset: 0x10 */
    volatile uint32_t SMPR1;          /**< ADC sampling time register 1,                  Address offset: 0x14 */
    volatile uint32_t SMPR2;          /**< ADC sampling time register 2,                  Address offset: 0x18 */
    volatile uint32_t RESERVED1;      /**< Reserved,                                                      0x1C */
    volatile uint32_t TR1;            /**< ADC analog watchdog 1 threshold register,      Address offset: 0x20 */
    volatile uint32_t TR2;            /**< ADC analog watchdog 2 threshold register,      Address offset: 0x24 */
    volatile uint32_t TR3;            /**< ADC analog watchdog 3 threshold register,      Address offset: 0x28 */
    volatile uint32_t RESERVED2;      /**< Reserved,                                                      0x2C */
    volatile uint32_t SQR1;           /**< ADC group regular sequencer register 1,        Address offset: 0x30 */
    volatile uint32_t SQR2;           /**< ADC group regular sequencer register 2,        Address offset: 0x34 */
    volatile uint32_t SQR3;           /**< ADC group regular sequencer register 3,        Address offset: 0x38 */
    volatile uint32_t SQR4;           /**< ADC group regular sequencer register 4,        Address offset: 0x3C */
    volatile uint32_t DR;             /**< ADC group regular data register,               Address offset: 0x40 */
    volatile uint32_t RESERVED3;      /**< Reserved,                                                      0x44 */
    volatile uint32_t RESERVED4;      /**< Reserved,                                                      0x48 */
    volatile uint32_t JSQR;           /**< ADC group injected sequencer register,         Address offset: 0x4C */
    volatile uint32_t RESERVED5[ 4 ]; /**< Reserved,                                               0x50 - 0x5C */
    volatile uint32_t OFR1;           /**< ADC offset register 1,                         Address offset: 0x60 */
    volatile uint32_t OFR2;           /**< ADC offset register 2,                         Address offset: 0x64 */
    volatile uint32_t OFR3;           /**< ADC offset register 3,                         Address offset: 0x68 */
    volatile uint32_t OFR4;           /**< ADC offset register 4,                         Address offset: 0x6C */
    volatile uint32_t RESERVED6[ 4 ]; /**< Reserved,                                               0x70 - 0x7C */
    volatile uint32_t JDR1;           /**< ADC group injected rank 1 data register,       Address offset: 0x80 */
    volatile uint32_t JDR2;           /**< ADC group injected rank 2 data register,       Address offset: 0x84 */
    volatile uint32_t JDR3;           /**< ADC group injected rank 3 data register,       Address offset: 0x88 */
    volatile uint32_t JDR4;           /**< ADC group injected rank 4 data register,       Address offset: 0x8C */
    volatile uint32_t RESERVED7[ 4 ]; /**< Reserved,                                             0x090 - 0x09C */
    volatile uint32_t AWD2CR;         /**< ADC analog watchdog 1 configuration register,  Address offset: 0xA0 */
    volatile uint32_t AWD3CR;         /**< ADC analog watchdog 3 Configuration Register,  Address offset: 0xA4 */
    volatile uint32_t RESERVED8;      /**< Reserved,                                                     0x0A8 */
    volatile uint32_t RESERVED9;      /**< Reserved,                                                     0x0AC */
    volatile uint32_t DIFSEL;         /**< ADC differential mode selection register,      Address offset: 0xB0 */
    volatile uint32_t CALFACT;        /**< ADC calibration factors,                       Address offset: 0xB4 */
  };


  struct CommonRegisterMap
  {
    volatile uint32_t RESERVED1; /**< Reserved,                          Address offset: ADC1 base address + 0x300 */
    volatile uint32_t RESERVED2; /**< Reserved,                          Address offset: ADC1 base address + 0x304 */
    volatile uint32_t CCR;       /**< ADC common configuration register, Address offset: ADC1 base address + 0x308 */
    volatile uint32_t RESERVED3; /**< Reserved,                          Address offset: ADC1 base address + 0x30C */
  };

}    // namespace Thor::LLD::ADC

#endif /* !THOR_HW_ADC_TYPES_HPP*/
