/********************************************************************************
 *  File Name:
 *    hw_startup_stm32l4xxxx.hpp
 *
 *  Description:
 *    High level startup information for STM32L4 variant chips
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/*-------------------------------------------------
Export the startup vector table so that the linker doesn't throw it away.
Used elsewhere in the Thor code.
-------------------------------------------------*/
extern void *StartupVectorTable;
#define THOR_SYSTEM_ISR_VECTOR_SYMBOL( StartupVectorTable )