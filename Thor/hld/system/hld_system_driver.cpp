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
#include <Thor/pwm>
#include <Thor/spi>
#include <Thor/uart>
#include <Thor/usart>
#include <Thor/watchdog>

/* Driver Includes */
#include <Thor/lld/interface/des/des_intf.hpp>
#include <Thor/lld/interface/interrupt/interrupt_detail.hpp>
#include <Thor/lld/interface/interrupt/interrupt_intf.hpp>
#include <Thor/lld/interface/nvic/nvic_intf.hpp>
#include <Thor/lld/interface/rcc/rcc_intf.hpp>
#include <Thor/lld/interface/startup/startup_intf.hpp>

namespace Thor::System
{
  static Chimera::System::Information s_system_info;

  Chimera::Status_t initialize()
  {
    s_system_info = {};
    return Chimera::Status::OK;
  }

  Chimera::Status_t reset()
  {
    return Chimera::Status::OK;
  }

  Chimera::Status_t systemStartup()
  {
    /*------------------------------------------------
    Initialize the system clocks
    ------------------------------------------------*/
#if defined( THOR_LLD_RCC )
    Thor::LLD::RCC::initialize();
    Thor::LLD::RCC::getCoreClock()->configureProjectClocks();
#endif

    /*------------------------------------------------
    HLD Specific Initialization
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

#if defined( THOR_HLD_PWM )
    Thor::PWM::initializeModule();
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
    LLD Specific Initialization
    ------------------------------------------------*/
#if defined( THOR_LLD_DES )
    Thor::LLD::DES::initialize();
#endif

    /*------------------------------------------------
    Initialize interrupt settings
    ------------------------------------------------*/
#if defined( THOR_LLD_IT )
    Thor::LLD::IT::setPriorityGrouping( Thor::Interrupt::SYSTEM_NVIC_PRIORITY_GROUPING );
#endif

    return Chimera::Status::OK;
  }

  Chimera::System::InterruptMask disableInterrupts()
  {
    return Thor::LLD::IT::disableInterrupts();
  }

  void enableInterrupts( Chimera::System::InterruptMask &interruptMask )
  {
    Thor::LLD::IT::enableInterrupts( interruptMask );
  }

  int maxConcurrentThreads()
  {
    return 1;
  }

  Chimera::System::ResetEvent getResetReason()
  {
    return Thor::LLD::RCC::getResetReason();
  }

  void getSystemInformation( Chimera::System::Information *&info )
  {
    /*------------------------------------------------
    Update the static data fields
    ------------------------------------------------*/
    Thor::LLD::DES::getUniqueId( s_system_info.uniqueId );
    s_system_info.chipPackage = Thor::LLD::DES::getICPackaging();
    s_system_info.flashSize   = Thor::LLD::DES::getFlashSize();

    /*------------------------------------------------
    Assign the data to the caller
    ------------------------------------------------*/
    info = &s_system_info;
  }
}    // namespace Thor::System
