#pragma once
#ifndef THOR_SYSTEM_HPP
#define THOR_SYSTEM_HPP
#include <stdint.h>
#include <Thor/include/preprocessor.hpp>

//Definitions taken from section 4.3 System Control Block in the Cortex-M4 Devices Generic User Guide
#if defined(CORTEX_M4)
#define SCB_REG_ACTLR	((volatile uint32_t*)0xE000E008)	/**< Auxiliary Control Register */
#define SCB_REG_CPUID	((volatile uint32_t*)0xE000ED00)	/**< CPUID Base Register */
#define SCB_REG_ICSR	((volatile uint32_t*)0xE000ED04)	/**< Interrupt Control and State Register */
#define SCB_REG_CFSR	((volatile uint32_t*)0xE000ED28)	/**< Configurable Fault Status Register */
#define SCB_REG_MMSR	((volatile uint32_t*)0xE000ED34)	/**< MemManage Fault Status Register (sub register of CFSR) */
#define SCB_REG_BFSR	((volatile uint32_t*)0xE000ED29)	/**< Bus Fault Status Register (sub register of CFSR) */
#define SCB_REG_UFSR	((volatile uint32_t*)0xE000ED2A)	/**< Usage Fault Status Register (sub register of CFSR) */
#define SCB_REG_HFSR	((volatile uint32_t*)0xE000ED2C)	/**< Hard Fault Status Register */
#define SCB_REG_AFSR	((volatile uint32_t*)0xE000ED3C)	/**< Auxiliary Fault Status Register */
#define SCB_REG_DFSR	((volatile uint32_t*)0xE000ED30)	/**< Debug Fault Status Register */
#define SCB_REG_MMAR	((volatile uint32_t*)0xE000ED34)	/**< MemManage Fault Address Register */
#define SCB_REG_BFAR	((volatile uint32_t*)0xE000ED38)	/**< Bus Fault Address Register */
#endif /* !CORTEX_M4 */

/* Auxiliary Control Register Definitions */
#if defined(CORTEX_M4)
#define ACTLR_DISOOFP_Pos			9U
#define ACTLR_DISOOFP_Msk			(1UL << ACTLR_DISOOFP_Pos)

#define ACTLR_DISFPCA_Pos			8U
#define ACTLR_DISFPCA_Msk			(1UL << ACTLR_DISFPCA_Pos)

#define ACTLR_DISFOLD_Pos			2U
#define ACTLR_DISFOLD_Msk			(1UL << ACTLR_DISFOLD_Pos)

#define ACTLR_DISDEFWBUF_Pos		1U
#define ACTLR_DISDEFWBUF_Msk		(1UL << ACTLR_DISDEFWBUF_Pos)

#define ACTLR_DISMCYCINT_Pos		0U
#define ACTLR_DISMCYCINT_Msk		(1UL << ACTLR_DISMCYCINT_Pos)
#endif






#endif