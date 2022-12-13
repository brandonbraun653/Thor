/******************************************************************************
 *  File Name:
 *    register.hpp
 *
 *  Description:
 *    Cortex-M4 specific register definitions
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef CORTEX_M4_REGISTERS_HPP
#define CORTEX_M4_REGISTERS_HPP

/* Thor Includes */
#include <Thor/cfg>

#if defined( CORTEX_M4 )

/*-------------------------------------------------------------------------------
Definitions from section 4.1 System Control Registers in the Cortex-M4 TRM
-------------------------------------------------------------------------------*/
#define SCB_REG_ACTLR ( ( volatile uint32_t * )0xE000E008 ) /**< Auxiliary Control Register */
#define SCB_REG_AFSR  ( ( volatile uint32_t * )0xE000ED3C ) /**< Auxiliary Fault Status Register */
#define SCB_REG_AIRCR ( ( volatile uint32_t * )0xE000ED0C ) /**< Application Interrupt and Reset Control Register */
#define SCB_REG_BFAR  ( ( volatile uint32_t * )0xE000ED38 ) /**< Bus Fault Address Register */
#define SCB_REG_BFSR  ( ( volatile uint32_t * )0xE000ED29 ) /**< Bus Fault Status Register (sub register of CFSR) */
#define SCB_REG_CFSR  ( ( volatile uint32_t * )0xE000ED28 ) /**< Configurable Fault Status Register */
#define SCB_REG_CPUID ( ( volatile uint32_t * )0xE000ED00 ) /**< CPUID Base Register */
#define SCB_REG_DEMCR ( ( volatile uint32_t * )0xE000EDFC ) /**< Debug Exception and Monitor Control Register */
#define SCB_REG_DFSR  ( ( volatile uint32_t * )0xE000ED30 ) /**< Debug Fault Status Register */
#define SCB_REG_DHCSR ( ( volatile uint32_t * )0xE000EDF0 ) /**< Debug Halting Control and Status Register */
#define SCB_REG_HFSR  ( ( volatile uint32_t * )0xE000ED2C ) /**< Hard Fault Status Register */
#define SCB_REG_ICSR  ( ( volatile uint32_t * )0xE000ED04 ) /**< Interrupt Control and State Register */
#define SCB_REG_MMAR  ( ( volatile uint32_t * )0xE000ED34 ) /**< MemManage Fault Address Register */
#define SCB_REG_MMSR  ( ( volatile uint32_t * )0xE000ED28 ) /**< MemManage Fault Status Register (sub register of CFSR) */
#define SCB_REG_STCSR ( ( volatile uint32_t * )0xE000E010 ) /**< SysTick Control and Status Register */
#define SCB_REG_UFSR  ( ( volatile uint32_t * )0xE000ED2A ) /**< Usage Fault Status Register (sub register of CFSR) */

/*-----------------------------------------------------------------------------
Data Watchpoint and Trace
-----------------------------------------------------------------------------*/
#define DWT_CONTROL  ( ( volatile uint32_t * )0xE0001000 )  /**< */
#define DWT_CYCCNT   ( ( volatile uint32_t * )0xE0001004 )  /**< */

/*-------------------------------------------------------------------------------
NVIC SYSTICK Register
-------------------------------------------------------------------------------*/
#define NVIC_REG_SYSTICK_CTRL ( ( volatile uint32_t * )( 0xE000E010 ) )
#define NVIC_REG_SYSTICK_LOAD ( ( volatile uint32_t * )( 0xE000E014 ) )
#define NVIC_REG_SYSTICK_VAL  ( ( volatile uint32_t * )( 0xE000E018 ) )


/*-------------------------------------------------------------------------------
Auxiliary Control Register Definitions
-------------------------------------------------------------------------------*/
#define ACTLR_DISOOFP_Pos    ( 9U )
#define ACTLR_DISOOFP_Msk    ( 1UL << ACTLR_DISOOFP_Pos )
#define ACTLR_DISFPCA_Pos    ( 8U )
#define ACTLR_DISFPCA_Msk    ( 1UL << ACTLR_DISFPCA_Pos )
#define ACTLR_DISFOLD_Pos    ( 2U )
#define ACTLR_DISFOLD_Msk    ( 1UL << ACTLR_DISFOLD_Pos )
#define ACTLR_DISDEFWBUF_Pos ( 1U )
#define ACTLR_DISDEFWBUF_Msk ( 1UL << ACTLR_DISDEFWBUF_Pos )
#define ACTLR_DISMCYCINT_Pos ( 0U )
#define ACTLR_DISMCYCINT_Msk ( 1UL << ACTLR_DISMCYCINT_Pos )


/*-------------------------------------------------------------------------------
Application Interrupt and Reset Control Register (AIRCR)
-------------------------------------------------------------------------------*/
static constexpr uint32_t AIRCR_VECTKEY     = ( 0x05FA << 16 );
static constexpr uint32_t AIRCR_VECTKEY_Msk = ( 0xFFFF0000 );
static constexpr uint32_t AIRCR_SYSRESETREQ = ( 1u << 2 );


/*-------------------------------------------------------------------------------
Bus Fault Status Register (BFSR)
-------------------------------------------------------------------------------*/
static constexpr uint32_t BFSR_BFARVALID   = ( 1u << 7 );
static constexpr uint32_t BFSR_LSPERR      = ( 1u << 5 );
static constexpr uint32_t BFSR_STKERR      = ( 1u << 4 );
static constexpr uint32_t BFSR_UNSTKERR    = ( 1u << 3 );
static constexpr uint32_t BFSR_IMPRECISERR = ( 1u << 2 );
static constexpr uint32_t BFSR_PRECISERR   = ( 1u << 1 );
static constexpr uint32_t BFSR_IBUSERR     = ( 1u << 0 );


/*-----------------------------------------------------------------------------
Debug Halting and Control Status Register (DHCSR)
-----------------------------------------------------------------------------*/
static constexpr uint32_t DHCSR_C_DEBUGEN = ( 1u << 0 );


/*-------------------------------------------------------------------------------
Interrupt Control and State Register (ICSR)
-------------------------------------------------------------------------------*/
static constexpr uint32_t ICSR_VECTACTIVE_Pos = 0u;
static constexpr uint32_t ICSR_VECTACTIVE_Msk = 0x1FF;
static constexpr uint32_t ICSR_VECTACTIVE     = ( ICSR_VECTACTIVE_Msk << ICSR_VECTACTIVE_Pos );


/*-------------------------------------------------------------------------------
Usage Fault Status Register (UFSR)
-------------------------------------------------------------------------------*/
static constexpr uint32_t UFSR_DIVBYZERO  = ( 1u << 9 );
static constexpr uint32_t UFSR_UNALIGNED  = ( 1u << 8 );
static constexpr uint32_t UFSR_NOCP       = ( 1u << 3 );
static constexpr uint32_t UFSR_INVPC      = ( 1u << 2 );
static constexpr uint32_t UFSR_INVSTATE   = ( 1u << 1 );
static constexpr uint32_t UFSR_UNDEFINSTR = ( 1u << 0 );

#endif /* !CORTEX_M4 */
#endif /* !CORTEX_M4_REGISTERS_HPP */
