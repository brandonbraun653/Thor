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
#include <Chimera/gpio>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/interrupt>
#include <Thor/dma>
#include <Thor/spi>
#include <Thor/uart>
#include <Thor/usart>
#include <Thor/watchdog>

/* Driver Includes */
#include <Thor/lld/common/interrupts/nvic_detail.hpp>
#include <Thor/lld/interface/inc/des>
#include <Thor/lld/interface/inc/interrupt>
#include <Thor/lld/interface/inc/rcc>
#include <Thor/lld/interface/inc/startup>
#include <Thor/lld/interface/inc/sys>


namespace Thor::System
{
  /*-------------------------------------------------------------------------------
  Static Data
  -------------------------------------------------------------------------------*/
  static Chimera::System::Information s_system_info;

  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
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
    Thor::LLD::RCC::initialize();
    Thor::LLD::RCC::getCoreClockCtrl()->configureProjectClocks();

    /*------------------------------------------------
    LLD Specific Initialization
    ------------------------------------------------*/
    Thor::LLD::INT::setPriorityGrouping( Thor::LLD::INT::SYSTEM_NVIC_PRIORITY_GROUPING );

#if defined( THOR_DES )
    Thor::LLD::DES::initialize();
#endif

    return Chimera::Status::OK;
  }


  Chimera::System::InterruptMask disableInterrupts()
  {
    return Thor::LLD::INT::disableInterrupts();
  }


  void enableInterrupts( Chimera::System::InterruptMask &interruptMask )
  {
    Thor::LLD::INT::enableInterrupts( interruptMask );
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


  bool inISR()
  {
    return Thor::LLD::SYS::inISR();
  }


  void softwareReset()
  {
    return Thor::LLD::SYS::softwareReset();
  }


  bool isDebuggerAttached()
  {
    return Thor::LLD::SYS::isDebuggerAttached();
  }

}    // namespace Thor::System
