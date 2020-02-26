/********************************************************************************
 *  File Name:
 *    thor_stm32f4x_config.hpp
 *
 *  Description:
 *    Configure Thor for an STM32F4x device
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_CONFIG_STM32F4X_HPP
#define THOR_CONFIG_STM32F4X_HPP

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
 *	If using external defintion, it must be of type Thor::Serial::SerialPins, be named "serialDebugPinConfig"
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
#define SERIAL_DEBUG_BAUDRATE Thor::Serial::BaudRate::SERIAL_BAUD_115200

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

#if defined( THOR_ENABLE_ALL_DRIVERS )
/*----------------------------------------------
High Level Driver Support
----------------------------------------------*/
#define THOR_HLD_ADC
#define THOR_HLD_CAN
#define THOR_HLD_CRYPTO
#define THOR_HLD_DAC
#define THOR_HLD_DMA
#define THOR_HLD_ETHR
#define THOR_HLD_EXTI
#define THOR_HLD_FLASH
#define THOR_HLD_GPIO
#define THOR_HLD_HASH
#define THOR_HLD_I2C
#define THOR_HLD_I2S
#define THOR_HLD_NVIC
#define THOR_HLD_PWM
#define THOR_HLD_QSPI
#define THOR_HLD_RNG
#define THOR_HLD_RTC
#define THOR_HLD_SDMMC
#define THOR_HLD_SERIAL
#define THOR_HLD_SPI
//#define THOR_HLD_TIMER
#define THOR_HLD_UART
#define THOR_HLD_USART
#define THOR_HLD_USB

/*----------------------------------------------
Low Level Driver Support
----------------------------------------------*/
#define THOR_LLD_ADC
#define THOR_LLD_CAN
#define THOR_LLD_CRYPTO
#define THOR_LLD_DAC
#define THOR_LLD_DMA
#define THOR_LLD_ETHR
#define THOR_LLD_EXTI
#define THOR_LLD_FLASH
#define THOR_LLD_GPIO
#define THOR_LLD_HASH
#define THOR_LLD_I2C
#define THOR_LLD_I2S
#define THOR_LLD_NVIC
#define THOR_LLD_PWM
#define THOR_LLD_QSPI
#define THOR_LLD_RNG
#define THOR_LLD_RTC
#define THOR_LLD_SDMMC
#define THOR_LLD_SERIAL
#define THOR_LLD_SPI
#define THOR_LLD_TIMER
#define THOR_LLD_UART
#define THOR_LLD_USART
#define THOR_LLD_USB

#else /* !THOR_ENABLE_ALL_DRIVERS */


#endif 

#endif  /* !THOR_CONFIG_STM32F4X_HPP */