/********************************************************************************
 *  File Name:
 *    hld_system_chimera.cpp
 *
 *  Description:
 *    Thor SYSTEM registration hooks for Chimera
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Chimera Includes */
#include <Chimera/system>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/system>
#include <Thor/hld/system/hld_system_chimera.hpp>

namespace Chimera::System::Backend
{
  static bool s_system_initialized        = false;
  static ResetEvent s_system_reset_reason = ResetEvent::UNKNOWN;

  Chimera::Status_t initialize()
  {
    if ( !s_system_initialized )
    {
      s_system_initialized = true;
      getResetReason();

      return Thor::System::initialize();
    }

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t reset()
  {
    auto lld_reset_result = Thor::System::reset();

    if ( lld_reset_result == Chimera::CommonStatusCodes::OK )
    {
      s_system_initialized = false;
    }

    return lld_reset_result;
  }

  Chimera::Status_t systemStartup()
  {
    return Thor::System::systemStartup();
  }

  Chimera::System::InterruptMask disableInterrupts()
  {
    return Thor::System::disableInterrupts();
  }

  void enableInterrupts( Chimera::System::InterruptMask &interruptMask )
  {
    return Thor::System::enableInterrupts( interruptMask );
  }

  int maxConcurrentThreads()
  {
    return Thor::System::maxConcurrentThreads();
  }

  Chimera::System::ResetEvent getResetReason()
  {
    auto reason = Thor::System::getResetReason();

    /*------------------------------------------------
    If the flags were cleared, use the cached reset
    ------------------------------------------------*/
    if ( reason == ResetEvent::CLEARED )
    {
      return s_system_reset_reason;
    }

    /*------------------------------------------------
    Otherwise cache off the reset code
    ------------------------------------------------*/
    s_system_reset_reason = reason;
    return reason;
  }

  void getSystemInformation( Chimera::System::Information *&info )
  {
    Thor::System::getSystemInformation( info );
  }

  Chimera::Status_t registerDriver( DriverConfig &registry )
  {
#if defined( THOR_HLD_SYSTEM )
    registry.isSupported          = true;
    registry.disableInterrupts    = disableInterrupts;
    registry.enableInterrupts     = enableInterrupts;
    registry.getResetReason       = getResetReason;
    registry.getSystemInformation = getSystemInformation;
    registry.initialize           = initialize;
    registry.maxConcurrentThreads = maxConcurrentThreads;
    registry.reset                = reset;
    registry.systemStartup        = systemStartup;
    return Chimera::CommonStatusCodes::OK;
#else
    registry             = {};
    registry.isSupported = false;
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
#endif
  }
}    // namespace Chimera::System::Backend
     // namespace Chimera::System::Backend