/********************************************************************************
 *  File Name:
 *    hld_can_driver.hpp
 *
 *  Description:
 *    Thor CAN high level driver
 *
 *  2020-2022 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HLD_CAN_HPP
#define THOR_HLD_CAN_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/can>
#include <Chimera/event>
#include <Chimera/gpio>
#include <Chimera/thread>
#include <Thor/hld/can/hld_can_types.hpp>
#include <cstdint>

namespace Thor::CAN
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  Chimera::Status_t initialize();
  Chimera::Status_t reset();
  Driver_rPtr getDriver( const Chimera::CAN::Channel channel );

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  class Driver : public Chimera::Thread::Lockable<Driver>
  {
  public:
    Driver();
    ~Driver();

    /*-------------------------------------------------------------------------
    Interface: Hardware
    -------------------------------------------------------------------------*/
    Chimera::Status_t open( const Chimera::CAN::DriverConfig &cfg );
    Chimera::Status_t close();
    Chimera::CAN::CANStatus getStatus();
    Chimera::Status_t send( const Chimera::CAN::BasicFrame &frame );
    Chimera::Status_t receive( Chimera::CAN::BasicFrame &frame );
    Chimera::Status_t filter( const Chimera::CAN::Filter *const list, const size_t size );
    Chimera::Status_t flush( Chimera::CAN::BufferType buffer );
    size_t available();
    void postISRProcessing();

  private:
    friend Chimera::Thread::Lockable<Driver>;

    Chimera::CAN::DriverConfig mConfig;
    Chimera::Event::ActionableList eventListeners;

    void ProcessISREvent_TX();
    void ProcessISREvent_RX();
    void ProcessISREvent_Error();
    void ProcessISREvent_StatusChange();
  };

}    // namespace Thor::CAN

#endif /* THOR_HLD_CAN_HPP */
