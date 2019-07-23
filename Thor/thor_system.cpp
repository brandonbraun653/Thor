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

/* Driver Includes */
#include <Thor/drivers/NVIC.hpp>
#include <Thor/drivers/RCC.hpp>

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
    Thor::Driver::RCC::init();
    auto sys = Thor::Driver::RCC::SystemClock::get();
    sys->configureProjectClocks();

    Thor::Driver::Interrupt::setPriorityGrouping( Thor::Interrupt::SYSTEM_NVIC_PRIORITY_GROUPING );

    return Chimera::CommonStatusCodes::OK;
  }
}
