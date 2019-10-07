/********************************************************************************
 *   File Name:
 *       chimeraPort.hpp
 *
 *   Description:
 *       Provide the class inheritance typedefs needed for Chimera to stub into
 *       the Thor hardware interface.
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef CHIMERA_PORT_THOR_HPP_
#define CHIMERA_PORT_THOR_HPP_

/* Thor Includes */
#include <Thor/headers.hpp>
#include <Thor/thor.hpp>
#include <Thor/crc.hpp>
#include <Thor/dma.hpp>
#include <Thor/flash.hpp>
#include <Thor/gpio.hpp>
#include <Thor/power.hpp>
#include <Thor/serial.hpp>
#include <Thor/spi.hpp>
#include <Thor/sram.hpp>
#include <Thor/system.hpp>
#include <Thor/watchdog.hpp>


#define BACKEND_NAMESPACE Thor


#if defined( THOR_DRIVER_DMA ) && ( THOR_DRIVER_DMA == 1 )
#define CHIMERA_INHERITED_DMA                 Thor::DMA::DMAClass
#endif

#if defined( THOR_DRIVER_GPIO ) && ( THOR_DRIVER_GPIO == 1 )
#define CHIMERA_INHERITED_GPIO                Thor::GPIO::GPIOClass
#endif 

#if defined( TARGET_STM32F4 ) && ( ( THOR_DRIVER_UART == 1 ) || ( THOR_DRIVER_USART == 1 ) )
#define CHIMERA_INHERITED_SERIAL              Thor::Serial::SerialClass
#endif

#if defined( THOR_DRIVER_IWDG ) && ( THOR_DRIVER_IWDG == 1 )
#define CHIMERA_INHERITED_WATCHDOG            Thor::Watchdog::Independent
#endif 

//#define CHIMERA_INHERITED_SYSTEM_IDENTIFIER   Thor::System::Identifier

#endif
