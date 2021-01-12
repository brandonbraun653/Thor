/********************************************************************************
 *   File Name:
 *    register.hpp
 *
 *   Description:
 *    Cortex-M7 specific register definitions
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef CORTEX_M7_REGISTERS_HPP
#define CORTEX_M7_REGISTERS_HPP

#include <Thor/preprocessor.hpp>

/*------------------------------------------------
Definitions taken from section 4.3 System Control Block in the Cortex-M7 Devices Generic User Guide
------------------------------------------------*/
#if defined( CORTEX_M7 )

#define SCB_REG_ACTLR ( ( volatile uint32_t * )0xE000E008 ) /**< Auxiliary Control Register */
#define SCB_REG_CPUID ( ( volatile uint32_t * )0xE000ED00 ) /**< CPUID Base Register */
#define SCB_REG_ICSR ( ( volatile uint32_t * )0xE000ED04 )  /**< Interrupt Control and State Register */
#define SCB_REG_CFSR ( ( volatile uint32_t * )0xE000ED28 )  /**< Configurable Fault Status Register */
#define SCB_REG_MMSR ( ( volatile uint32_t * )0xE000ED28 )  /**< MemManage Fault Status Register (sub register of CFSR) */
#define SCB_REG_BFSR ( ( volatile uint32_t * )0xE000ED29 )  /**< Bus Fault Status Register (sub register of CFSR) */
#define SCB_REG_UFSR ( ( volatile uint32_t * )0xE000ED2A )  /**< Usage Fault Status Register (sub register of CFSR) */
#define SCB_REG_HFSR ( ( volatile uint32_t * )0xE000ED2C )  /**< Hard Fault Status Register */
#define SCB_REG_MMAR ( ( volatile uint32_t * )0xE000ED34 )  /**< MemManage Fault Address Register */
#define SCB_REG_BFAR ( ( volatile uint32_t * )0xE000ED38 )  /**< Bus Fault Address Register */
#define SCB_REG_AFSR ( ( volatile uint32_t * )0xE000ED3C )  /**< Auxiliary Fault Status Register */

#endif /* !CORTEX_M7 */
#endif /* !CORTEX_M7_REGISTERS_HPP */