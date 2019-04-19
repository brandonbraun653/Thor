/********************************************************************************
 *   File Name:
 *    power.hpp
 *
 *   Description:
 *    Models the system power interface 
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_SYSTEM_POWER_HPP
#define THOR_SYSTEM_POWER_HPP

/* Chimera Includes */
#include <Chimera/interface/power_intf.hpp>

namespace Thor
{
  namespace Power
  {
    class SystemPower : public Chimera::Power::InfoInterface
    {
    public:
      SystemPower();
      ~SystemPower();

      float systemVCC() final override;
    };
  }    // namespace Power
}

#endif /* !THOR_SYSTEM_POWER_HPP */