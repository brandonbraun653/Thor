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
#define USE_SERIAL_DEBUG_OUTPUT 1

/*! @def USE_SERIAL_DEBUG_EXT_PINS
 *	@brief Instructs the setup code to either use default pin settings for a Serial channel (0) or user defined settings (1)
 *
 *	If using external defintion, it must be of type Thor::Definitions::Serial::SerialPins, be named "serialDebugPinConfig"
 *	and declared with C linkage, otherwise the setup code will not be able to find it and throw a compiler error. It is
 *	vital that this struct has been initialized with values before calling ThorInit(), or if using Chimera, ChimeraInit().
 */
#define USE_SERIAL_DEBUG_EXT_PINS 0

/*! @def SERIAL_DEBUG_CHANNEL
 *	@brief Defines which serial channel to use for printf() redirection
 */
#define SERIAL_DEBUG_CHANNEL 1

/*! @def SERIAL_DEBUG_BAUDRATE
 *	@brief Defines the baud rate to use for printf() redirection
 */
#define SERIAL_DEBUG_BAUDRATE Thor::Definitions::Serial::BaudRate::SERIAL_BAUD_115200

/*! @def WRITE_BUFFERING_DISABLED
 *	@brief Disables write buffer during default memory map access. (Default 0)
 *
 *	This causes all BusFaults to be precise BusFaults, but decreases performance because any store to memory must
 *	complete before the processor can execute the next instruction.
 *
 *	@note	If the IMPRECISERR bit is set in the BFSR register on a Hard Fault, enabling this macro should cause the error to
 *become precise, thus loading the value of the offending instruction BFAR register. Currently only supported on Cortex-M3/M4.
 */
#define WRITE_BUFFERING_DISABLED 0


/**
 *   Keep track of which modules are currently supported for each chipset
 */
#if defined( TARGET_STM32F4 )

/*----------------------------------------------
Library Support
----------------------------------------------*/
#define THOR_LIB_GPIO 1
#define THOR_LIB_CAN 0
#define THOR_LIB_EXTI 0
#define THOR_LIB_DMA 0
#define THOR_LIB_NVIC 0
#define THOR_LIB_DAC 0
#define THOR_LIB_ADC 1
#define THOR_LIB_RNG 0
#define THOR_LIB_HASH 0
#define THOR_LIB_CRYPTO 0
#define THOR_LIB_SPI 1
#define THOR_LIB_QSPI 0
#define THOR_LIB_I2C 0
#define THOR_LIB_I2S 0
#define THOR_LIB_RTC 0
#define THOR_LIB_UART 1
#define THOR_LIB_USART 1
#define THOR_LIB_SERIAL 1
#define THOR_LIB_TIMER 0
#define THOR_LIB_PWM 0
#define THOR_LIB_USB 0
#define THOR_LIB_SDMMC 0
#define THOR_LIB_ETHR 0
#define THOR_LIB_FLASH 0

/*----------------------------------------------
Hardware Support
----------------------------------------------*/
#define THOR_HW_SFPU 1
#define THOR_HW_DFPU 0
#define THOR_HW_CRYPTO 0
#define THOR_HW_HASH 0
#define THOR_HW_HDMI 0
#define THOR_HW_DCMI 0
#define THOR_HW_TFT_LCD 0
#define THOR_HW_SPI 1


#elif defined( TARGET_STM32F7 )

#endif

#endif /* !THOR_CONFIG_HPP */