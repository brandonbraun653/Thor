/********************************************************************************
 *  File Name:
 *    hld_usb_driver.hpp
 *
 *  Description:
 *    Thor USB high level driver
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HLD_USB_HPP
#define THOR_HLD_USB_HPP

/* C++ Includes */
#include <cstdint>
#include <cstdlib>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/usb>
#include <Chimera/thread>

/* Thor Includes */
#include <Thor/hld/usb/hld_usb_types.hpp>

namespace Thor::USB
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize();
  Chimera::Status_t reset();
  Driver_rPtr getDriver( const Chimera::USB::Channel ch );


  /*-------------------------------------------------------------------------------
  Classes
  -------------------------------------------------------------------------------*/
  /**
   *  USB Peripheral Driver
   *  Methods here at a minimum implement the interface specified in Chimera.
   *  Inheritance is avoided to minimize cost of virtual function lookup table.
   */
  class Driver : public Chimera::Threading::LockableCRTP<Driver>
  {
  public:
    Driver();
    ~Driver();

    /*-------------------------------------------------
    Interface: Hardware
    -------------------------------------------------*/
    Chimera::Status_t open( const Chimera::USB::PeriphConfig &cfg );
    void close();

  private:
    friend Chimera::Threading::LockableCRTP<Driver>;
    Chimera::Threading::RecursiveTimedMutex mClsMutex;

    Chimera::USB::Channel mChannel;
  };
}    // namespace Thor::USB

#endif /* THOR_HLD_USB_HPP */
