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

/* Chimera Includes */
#include <Chimera/config.hpp>

/* Thor Includes */
#include <Thor/include/thor.hpp>
#include <Thor/include/spi.hpp>
#include <Thor/include/gpio.hpp>
#include <Thor/include/serial.hpp>
#include <Thor/include/watchdog.hpp>


#define CHIMERA_INHERITED_GLOBAL_NAMESPACE Thor

/*------------------------------------------------
Define the classes for Chimera to inherit from. These modules are turned
on and off by the #defines inside of <Chimera/config.hpp>
------------------------------------------------*/
#if (CHIMERA_HWM_GPIO)
typedef Thor::Peripheral::GPIO::ChimeraGPIO		        CHIMERA_INHERITED_GPIO;
#endif

#if (CHIMERA_HWM_SPI)
typedef Thor::Peripheral::SPI::ChimeraSPI		        CHIMERA_INHERITED_SPI;
#endif 

#if (CHIMERA_HWM_WATCHDOG)
typedef Thor::Peripheral::Watchdog::ChimeraWatchdog     CHIMERA_INHERITED_WATCHDOG;
#endif

#if (CHIMERA_MOD_SERIAL)
typedef	Thor::Peripheral::Serial::SerialClass	        CHIMERA_INHERITED_SERIAL;
#endif



#endif