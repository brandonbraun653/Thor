/********************************************************************************
 *  File Name:
 *    thor_system.cpp
 *
 *  Description:
 *    Implements system interface functions for Thor
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */

/* Chimera Includes */
#include <Chimera/interface/system_intf.hpp>

/* Thor Includes */
#include <Thor/preprocessor.hpp>
#include <Thor/definitions/interrupt_definitions.hpp>
#include <Thor/headers.hpp>
#include <Thor/system.hpp>
#include <Thor/dma.hpp>
#include <Thor/gpio.hpp>
#include <Thor/spi.hpp>
#include <Thor/uart.hpp>
#include <Thor/usart.hpp>
#include <Thor/watchdog.hpp>

/* Driver Includes */
#include <Thor/drivers/nvic.hpp>
#include <Thor/drivers/rcc.hpp>

namespace Thor::System
{


}    // namespace Thor::System

namespace Chimera::System
{
  Chimera::Status_t prjSystemStartup()
  {
    /*------------------------------------------------
    Initialize the system clocks
    ------------------------------------------------*/
    Thor::Driver::RCC::initialize();
    Thor::Driver::RCC::SystemClock::get()->configureProjectClocks();

    /*------------------------------------------------
    Hardware Specific Initialization
    ------------------------------------------------*/
#if defined( THOR_DRIVER_DMA ) && ( THOR_DRIVER_DMA == 1 )
    Thor::DMA::initialize();
    Thor::DMA::DMAClass::get()->init();
#endif

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_GPIO == 1 )
    Thor::GPIO::initialize();
#endif 

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_IWDG == 1 )
    Thor::Watchdog::initializeIWDG();
#endif 

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_SPI == 1 )
    Thor::SPI::initialize();
#endif 

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_UART == 1 )
    Thor::UART::initialize();
#endif

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_USART == 1 )
    Thor::USART::initialize();
#endif

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_WWDG == 1 )
    Thor::Watchdog::initializeWWDG();
#endif 

    /*------------------------------------------------
    Initialize interrupt settings
    ------------------------------------------------*/
    Thor::Driver::Interrupt::setPriorityGrouping( Thor::Interrupt::SYSTEM_NVIC_PRIORITY_GROUPING );


    return Chimera::CommonStatusCodes::OK;
  }
}
