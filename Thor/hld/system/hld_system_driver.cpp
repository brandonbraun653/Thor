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
#if defined( THOR_DRIVER_DMA ) && defined( THOR_LLD_DMA )
    Thor::DMA::initialize();
    Thor::DMA::DMAClass::get()->init();
#endif

#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_GPIO )
    Thor::GPIO::initialize();
#endif

#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_IWDG )
    Thor::Watchdog::initializeIWDG();
#endif 

#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_SPI )
    Thor::SPI::initialize();
#endif

#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_UART )
    Thor::UART::initialize();
#endif

#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_USART )
    Thor::USART::initialize();
#endif

#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_WWDG )
    Thor::Watchdog::initializeWWDG();
#endif 

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
