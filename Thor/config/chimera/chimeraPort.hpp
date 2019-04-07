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
#include <Thor/thor.hpp>
#include <Thor/spi.hpp>
#include <Thor/gpio.hpp>
#include <Thor/serial.hpp>
#include <Thor/system.hpp>
#include <Thor/watchdog.hpp>


#define BACKEND_NAMESPACE Thor

#define CHIMERA_INHERITED_GPIO Thor::GPIO::GPIOClass
#define CHIMERA_INHERITED_SERIAL Thor::Serial::SerialClass

#define CHIMERA_INHERITED_SYSTEM_IDENTIFIER Thor::System::Identifier

#endif
