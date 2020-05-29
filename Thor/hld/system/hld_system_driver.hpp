/********************************************************************************
 *  File Name:
 *    system.hpp
 *
 *  Description:
 *    Implements system interface functionality for Thor
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_SYSTEM_HPP
#define THOR_SYSTEM_HPP

/* C++ Includes */
#include <cstdint>
#include <string>

/* Chimera Includes */
#include <Chimera/system>


namespace Thor::System
{
  Chimera::Status_t initialize();
  Chimera::Status_t reset();
  Chimera::Status_t systemStartup();
  Chimera::System::InterruptMask disableInterrupts();
  void enableInterrupts( Chimera::System::InterruptMask &interruptMask );
  int maxConcurrentThreads();
  Chimera::System::ResetEvent getResetReason();
  void getSystemInformation( Chimera::System::Information *&info );

  namespace Version
  {
    /**
     *  Gets the Chimera driver version information as a string
     *  @return std::string_view
     */
    std::string_view asString();

    /**
     *  Gets the major version number of the backend driver
     *  @return size_t
     */
    size_t major();

    /**
     *  Gets the minor version number of the backend driver
     *  @return size_t
     */
    size_t minor();

    /**
     *  Gets the patch version number of the backend driver
     *  @return size_t
     */
    size_t patch();
  }

  namespace Description
  {
    /**
     *  Describes the backend driver with whatever information is desired
     *  @return std::string_view
     */
    std::string_view about();

    /**
     *  Name of the backend driver
     *  @return std::string_view
     */
    std::string_view backendDriverName();

    /**
     *  Link to where documentation for the backend driver should be
     *  @return std::string_view
     */
    std::string_view documentationLink();
  }
}

#endif
