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
}

#endif
