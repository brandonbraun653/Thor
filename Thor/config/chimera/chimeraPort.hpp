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

//#define CHIMERA_INHERITED_HW_CRC              Thor::HWCRC::HW
#define CHIMERA_INHERITED_DMA                 Thor::DMA::DMAClass
#define CHIMERA_INHERITED_GPIO                Thor::GPIO::GPIOClass
//#define CHIMERA_INHERITED_POWER_INFO          Thor::Power::SystemPower
#define CHIMERA_INHERITED_SERIAL              Thor::Serial::SerialClass
//#define CHIMERA_INHERITED_SPI                 Thor::SPI::SPIClass
//#define CHIMERA_INHERITED_SYSTEM_FLASH        Thor::Memory::InternalFlash
//#define CHIMERA_INHERITED_SYSTEM_SRAM         Thor::Memory::InternalSRAM
#define CHIMERA_INHERITED_WATCHDOG            Thor::Watchdog::Independent
//#define CHIMERA_INHERITED_SYSTEM_IDENTIFIER   Thor::System::Identifier

#endif
