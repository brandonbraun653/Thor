/********************************************************************************
 *  File Name:
 *    thor_system.cpp
 *
 *  Description:
 *    Implements system interface functions for Thor
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/system>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/interrupt>
#include <Thor/dma>
#include <Thor/gpio>
#include <Thor/spi>
#include <Thor/uart>
#include <Thor/usart>
#include <Thor/watchdog>

/* Driver Includes */
#include <Thor/lld/interface/nvic/nvic.hpp>
#include <Thor/lld/interface/rcc/rcc.hpp>
#include <Thor/lld/interface/startup/startup.hpp>

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
    Thor::LLD::RCC::initialize();
    Thor::LLD::RCC::SystemClock::get()->configureProjectClocks();

    /*------------------------------------------------
    Hardware Specific Initialization
    ------------------------------------------------*/
    Thor::DMA::initialize();
    Thor::DMA::DMAClass::get()->init();
    Thor::GPIO::initialize();
    Thor::Watchdog::initializeIWDG();
    Thor::SPI::initialize();
    Thor::UART::initialize();
    Thor::USART::initialize();
    Thor::Watchdog::initializeWWDG();

    /*------------------------------------------------
    Initialize interrupt settings
    ------------------------------------------------*/
    Thor::LLD::IT::setPriorityGrouping( Thor::Interrupt::SYSTEM_NVIC_PRIORITY_GROUPING );


    return Chimera::CommonStatusCodes::OK;
  }

  InterruptMask prjDisableInterrupts()
  {
    return InterruptMask();
  }

  void prjEnableInterrupts( InterruptMask &interruptMask )
  {

  }

  int prjMaxConcurrentThreads()
  {
    return 1;
  }
}
