/********************************************************************************
 *  File Name:
 *    ethernet.hpp
 *
 *  Description:
 *    Ethernet hardware driver interface for all STM32 chips
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_ETHERNET_DRIVER_HPP
#define THOR_ETHERNET_DRIVER_HPP

/* C++ Includes */


/* Chimera Includes */
#include <Chimera/interface/ethernet_intf.hpp>

/* Thor Includes */
#include <Thor/drivers/Ethernet.hpp>

namespace Thor::Ethernet
{
  class Driver : public Chimera::Ethernet::Interface
  {
  
  };
}    // namespace Thor::Ethernet


#endif /* !THOR_ETHERNET_DRIVER_HPP */
