/** @file */
#pragma once
#ifndef THOR_CONFIG_HPP
#define THOR_CONFIG_HPP
#include <Thor/include/preprocessor.hpp>

/*! @def USE_SERIAL_DEBUG_OUTPUT 
 *	@brief Reroutes printf to use a serial channel as output 
 *
 *	@note	In order for this to work properly, the Visual GDB Semihosting code must not be in use. If it is, all 
 *			printf() statements will be redirected to whichever serial port VGDB is using.
 */
#define USE_SERIAL_DEBUG_OUTPUT			1

/*! @def USE_SERIAL_DEBUG_EXT_PINS 
 *	@brief Instructs the setup code to either use default pin settings for a Serial channel (0) or user defined settings (1)
 *	
 *	If using external defintion, it must be of type Thor::Definitions::Serial::SerialPins, be named "serialDebugPinConfig" 
 *	and declared with C linkage, otherwise the setup code will not be able to find it and throw a compiler error. It is 
 *	vital that this struct has been initialized with values before calling ThorInit(), or if using Chimera, ChimeraInit().
 */
#define USE_SERIAL_DEBUG_EXT_PINS		1

/*! @def SERIAL_DEBUG_CHANNEL
 *	@brief Defines which serial channel to use for printf() redirection 
 */
#define SERIAL_DEBUG_CHANNEL			1			

/*! @def SERIAL_DEBUG_BAUDRATE
 *	@brief Defines the baud rate to use for printf() redirection 
 */
#define SERIAL_DEBUG_BAUDRATE			Thor::Definitions::Serial::BaudRate::SERIAL_BAUD_115200

/*! @def WRITE_BUFFERING_DISABLED 
 *	@brief Disables write buffer during default memory map access. (Default 0)
 *
 *	This causes all BusFaults to be precise BusFaults, but decreases performance because any store to memory must
 *	complete before the processor can execute the next instruction.
 *
 *	@note	If the IMPRECISERR bit is set in the BFSR register on a Hard Fault, enabling this macro should cause the error to become precise, thus
 *			loading the value of the offending instruction BFAR register. Currently only supported on Cortex-M3/M4.
 */
#define WRITE_BUFFERING_DISABLED		1

#endif /* !THOR_CONFIG_HPP */