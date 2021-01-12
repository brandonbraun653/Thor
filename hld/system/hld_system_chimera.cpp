/********************************************************************************
 *  File Name:
 *    hld_system_chimera.cpp
 *
 *  Description:
 *    Thor system registration hooks for Chimera
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
  /*-------------------------------------------------------------------------------
  Static Data
  -------------------------------------------------------------------------------*/
  static bool s_system_initialized        = false;
  static ResetEvent s_system_reset_reason = ResetEvent::UNKNOWN;

  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    if ( !s_system_initialized )
    {
      s_system_initialized = true;
      getResetReason();

      return Thor::System::initialize();
    }

    return Chimera::Status::OK;
  }


  Chimera::Status_t reset()
  {
    auto lld_reset_result = Thor::System::reset();

    if ( lld_reset_result == Chimera::Status::OK )
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


  void softwareReset()
  {
    Thor::System::softwareReset();
  }


  bool inISR()
  {
    return Thor::System::inISR();
  }


  namespace Version
  {
    std::string_view asString()
    {
      return Thor::HLD::VersionString;
    }


    size_t major()
    {
      return Thor::HLD::VersionMajor;
    }


    size_t minor()
    {
      return Thor::HLD::VersionMinor;
    }


    size_t patch()
    {
      return Thor::HLD::VersionPatch;
    }
  }    // namespace Version


  namespace Description
  {
    std::string_view about()
    {
      return "Thor is a hardware abstraction layer focused on STM32 chips.";
    }


    std::string_view backendDriverName()
    {
      return "Thor";
    }


    std::string_view documentationLink()
    {
      return "N/A";
    }
  }    // namespace Description


  Chimera::Status_t registerDriver( DriverConfig &registry )
  {
#if defined( THOR_HLD_SYSTEM )
    registry.isSupported            = true;
    registry.disableInterrupts      = disableInterrupts;
    registry.enableInterrupts       = enableInterrupts;
    registry.getResetReason         = getResetReason;
    registry.getSystemInformation   = getSystemInformation;
    registry.initialize             = initialize;
    registry.maxConcurrentThreads   = maxConcurrentThreads;
    registry.reset                  = reset;
    registry.systemStartup          = systemStartup;
    registry.inISR                  = inISR;
    registry.softwareReset          = softwareReset;
    registry.desc_About             = Description::about;
    registry.desc_BackendDriverName = Description::backendDriverName;
    registry.desc_DocumentationLink = Description::documentationLink;
    registry.version_AsString       = Version::asString;
    registry.version_Major          = Version::major;
    registry.version_Minor          = Version::minor;
    registry.version_Patch          = Version::patch;
    return Chimera::Status::OK;
#else
    registry             = {};
    registry.isSupported = false;
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }
}    // namespace Chimera::System::Backend
