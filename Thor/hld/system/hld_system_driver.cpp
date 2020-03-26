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
#include <Thor/lld/interface/rcc/rcc_intf.hpp>
#include <Thor/lld/interface/startup/startup.hpp>

namespace Chimera::System
{
  Chimera::Status_t prjSystemStartup()
  {
    /*------------------------------------------------
    Initialize the system clocks
    ------------------------------------------------*/
    #if defined( THOR_LLD_RCC )
    Thor::LLD::RCC::initialize();
    Thor::LLD::RCC::getSystemClockController()->configureProjectClocks();
    #endif

    /*------------------------------------------------
    Hardware Specific Initialization
    ------------------------------------------------*/
    #if defined( THOR_HLD_DMA )
    Thor::DMA::initialize();
    Thor::DMA::DMAClass::get()->init();
    #endif

    #if defined( THOR_HLD_GPIO )
    Thor::GPIO::initialize();
    #endif

    #if defined( THOR_HLD_IWDG )
    Thor::Watchdog::initializeIWDG();
    #endif 

    #if defined( THOR_HLD_SPI )
    Thor::SPI::initialize();
    #endif 

    #if defined( THOR_HLD_UART )
    Thor::UART::initialize();
    #endif 

    #if defined( THOR_HLD_USART )
    Thor::USART::initialize();
    #endif 

    #if defined( THOR_HLD_WWDG )
    Thor::Watchdog::initializeWWDG();
    #endif

    /*------------------------------------------------
    Initialize interrupt settings
    ------------------------------------------------*/
    #if defined( THOR_LLD_IT )
    Thor::LLD::IT::setPriorityGrouping( Thor::Interrupt::SYSTEM_NVIC_PRIORITY_GROUPING );
    #endif

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
