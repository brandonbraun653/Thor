/********************************************************************************
 *  File Name:
 *    hld_system_chimera.hpp
 *
 *  Description:
 *    Chimera hooks into Thor System
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_SYSTEM_CHIMERA_HPP
#define THOR_SYSTEM_CHIMERA_HPP

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/system>

namespace Chimera::System::Backend
{
  Chimera::Status_t initialize();
  Chimera::Status_t reset();
  Chimera::Status_t systemStartup();
  Chimera::System::InterruptMask disableInterrupts();
  void enableInterrupts( Chimera::System::InterruptMask &interruptMask );
  int maxConcurrentThreads();
  Chimera::System::ResetEvent getResetReason();
  void getSystemInformation( Chimera::System::Information *&info );
  void softwareReset();
  bool inISR();
}    // namespace Chimera::System::Backend

#endif /* !THOR_SYSTEM_CHIMERA_HPP */
