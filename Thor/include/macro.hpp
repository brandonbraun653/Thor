#pragma once
#ifndef THOR_MACRO_HPP
#define THOR_MACRO_HPP
#include <Thor/include/system.hpp>

/*! @def DISABLE_WRITE_BUFFERING
 *	@brief Instructs the processor to disable the write buffer, thus linearizing code execution 
 */
#define DISABLE_WRITE_BUFFERING (*SCB_REG_ACTLR |= ACTLR_DISDEFWBUF_Msk)

/*! @def ENABLE_WRITE_BUFFERING
 *	@brief Instructs the processor to enable the write buffer, allowing for out of order execution 
 */
#define ENABLE_WRITE_BUFFERING	(*SCB_REG_ACTLR &= ~ACTLR_DISDEFWBUF_Msk)


/*! @def INSERT_BREAKPOINT
 *	@brief Manually instructs the CPU to break and go into debug mode for debugger.	
 */
#ifndef INSERT_BREAKPOINT
#define INSERT_BREAKPOINT __asm("BKPT #0\n")
#endif

#endif /* !THOR_MACRO_HPP */