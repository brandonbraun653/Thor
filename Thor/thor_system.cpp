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
#include <Thor/spi.hpp>

/* Driver Includes */
#include <Thor/drivers/nvic.hpp>
#include <Thor/drivers/rcc.hpp>

namespace Thor::System
{
  Identifier::Identifier()
  {
  }

  Identifier::~Identifier()
  {
  }

  uint32_t Identifier::deviceID()
  {
    return 0;
  }

  uint32_t Identifier::uniqueID()
  {
    /* Will likely need to switch this up as the unique ID is 96 bits */
    // uint8_t * + length
    return 0u;
  }

}    // namespace Thor::System

namespace Chimera::System
{
  Chimera::Status_t prjSystemStartup()
  {
    /*------------------------------------------------
    Initialize the system clocks
    ------------------------------------------------*/
    Thor::Driver::RCC::initialize();
    auto sys = Thor::Driver::RCC::SystemClock::get();
    sys->configureProjectClocks();

    /*------------------------------------------------
    Hardware Specific Initialization
    ------------------------------------------------*/
#if defined( THOR_DRIVER_DMA ) && ( THOR_DRIVER_DMA == 1 )
    Thor::DMA::initialize();
#endif

    /*------------------------------------------------
    Initialize interrupt settings
    ------------------------------------------------*/
    Thor::Driver::Interrupt::setPriorityGrouping( Thor::Interrupt::SYSTEM_NVIC_PRIORITY_GROUPING );

    /*------------------------------------------------
    Initialize the DMA Driver
    ------------------------------------------------*/
    auto dma = Thor::DMA::DMAClass::get();
    dma->init();

    return Chimera::CommonStatusCodes::OK;
  }
}
