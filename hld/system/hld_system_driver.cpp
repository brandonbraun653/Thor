/******************************************************************************
 *  File Name:
 *    thor_system.cpp
 *
 *  Description:
 *    Implements system interface functions for Thor
 *
 *  2019-2022 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <Chimera/gpio>
#include <Chimera/system>
#include <Thor/cfg>
#include <Thor/dma>
#include <Thor/lld/common/interrupts/nvic_detail.hpp>
#include <Thor/lld/interface/inc/des>
#include <Thor/lld/interface/inc/interrupt>
#include <Thor/lld/interface/inc/rcc>
#include <Thor/lld/interface/inc/startup>
#include <Thor/lld/interface/inc/sys>
#include <Thor/watchdog>


namespace Chimera::System::Backend
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static Chimera::System::Information s_system_info;

  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  static Chimera::Status_t initialize()
  {
    s_system_info = {};
    return Chimera::Status::OK;
  }


  static Chimera::Status_t reset()
  {
    return Chimera::Status::OK;
  }


  static Chimera::Status_t systemStartup()
  {
    /*-------------------------------------------------------------------------
    Initialize the system clocks
    -------------------------------------------------------------------------*/
    Thor::LLD::RCC::initialize();
    Thor::LLD::RCC::configureProjectClocks();

    /*-------------------------------------------------------------------------
    LLD Specific Initialization
    -------------------------------------------------------------------------*/
    Thor::LLD::INT::setPriorityGrouping( Thor::LLD::INT::SYSTEM_NVIC_PRIORITY_GROUPING );

#if defined( THOR_DES )
    Thor::LLD::DES::initialize();
#endif

    return Chimera::Status::OK;
  }


  static int maxConcurrentThreads()
  {
    return 1;
  }


  static void getSystemInformation( Chimera::System::Information *&info )
  {
    /*-------------------------------------------------------------------------
    Update the static data fields
    -------------------------------------------------------------------------*/
    Thor::LLD::DES::getUniqueId( s_system_info.uniqueId );
    s_system_info.chipPackage = Thor::LLD::DES::getICPackaging();
    s_system_info.flashSize   = Thor::LLD::DES::getFlashSize();

    /*-------------------------------------------------------------------------
    Assign the data to the caller
    -------------------------------------------------------------------------*/
    info = &s_system_info;
  }


  namespace Version
  {
    static std::string_view asString()
    {
      return Thor::HLD::VersionString;
    }


    static size_t major()
    {
      return Thor::HLD::VersionMajor;
    }


    static size_t minor()
    {
      return Thor::HLD::VersionMinor;
    }


    static size_t patch()
    {
      return Thor::HLD::VersionPatch;
    }
  }    // namespace Version


  namespace Description
  {
    static std::string_view about()
    {
      return "Thor is a hardware abstraction layer focused on STM32 chips.";
    }


    static std::string_view backendDriverName()
    {
      return "Thor";
    }


    static std::string_view documentationLink()
    {
      return "N/A";
    }
  }    // namespace Description


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  Chimera::Status_t registerDriver( DriverConfig &registry )
  {
    registry.isSupported            = true;
    registry.disableInterrupts      = Thor::LLD::INT::disableInterrupts;
    registry.enableInterrupts       = Thor::LLD::INT::enableInterrupts;
    registry.getResetReason         = Thor::LLD::RCC::getResetReason;
    registry.getSystemInformation   = Chimera::System::Backend::getSystemInformation;
    registry.initialize             = Chimera::System::Backend::initialize;
    registry.maxConcurrentThreads   = Chimera::System::Backend::maxConcurrentThreads;
    registry.reset                  = Chimera::System::Backend::reset;
    registry.systemStartup          = Chimera::System::Backend::systemStartup;
    registry.inISR                  = Thor::LLD::SYS::inISR;
    registry.softwareReset          = Thor::LLD::SYS::softwareReset;
    registry.isDebuggerAttached     = Thor::LLD::SYS::isDebuggerAttached;
    registry.desc_About             = Description::about;
    registry.desc_BackendDriverName = Description::backendDriverName;
    registry.desc_DocumentationLink = Description::documentationLink;
    registry.version_AsString       = Version::asString;
    registry.version_Major          = Version::major;
    registry.version_Minor          = Version::minor;
    registry.version_Patch          = Version::patch;
    return Chimera::Status::OK;
  }

}    // namespace Chimera::System::Backend
