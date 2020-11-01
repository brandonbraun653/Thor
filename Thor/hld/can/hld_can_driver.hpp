/********************************************************************************
 *  File Name:
 *    hld_can_driver.hpp
 *
 *  Description:
 *    Thor CAN high level driver
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HLD_CAN_HPP
#define THOR_HLD_CAN_HPP

/* C/C++ Includes */
#include <cstdint>

/* Chimera Includes */
#include <Chimera/event>
#include <Chimera/can>
#include <Chimera/gpio>
#include <Chimera/thread>

/* Thor Includes */
#include <Thor/hld/can/hld_can_types.hpp>

namespace Thor::CAN
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize();
  Chimera::Status_t reset();
  Driver_rPtr getDriver( const Chimera::CAN::Channel channel );
  Driver_sPtr getDriverShared( const Chimera::CAN::Channel channel );

  /*-------------------------------------------------------------------------------
  Classes
  -------------------------------------------------------------------------------*/
  class Driver : public Chimera::Threading::Lockable
  {
  public:
    /*------------------------------------------------
    Class Specific Functions
    ------------------------------------------------*/
    Driver();
    ~Driver();

    /*-------------------------------------------------
    Interface: Hardware
    -------------------------------------------------*/
    Chimera::Status_t open( const Chimera::CAN::DriverConfig &cfg );
    Chimera::Status_t close();
    Chimera::CAN::CANStatus getStatus();
    Chimera::Status_t send( const Chimera::CAN::BasicFrame &frame );
    Chimera::Status_t receive( Chimera::CAN::BasicFrame &frame );
    Chimera::Status_t filter( const Chimera::CAN::Filter *const list, const size_t size );
    Chimera::Status_t flush( Chimera::CAN::BufferType buffer );
    size_t available();

    /*-------------------------------------------------
    Interface: Listener
    -------------------------------------------------*/
    Chimera::Status_t registerListener( Chimera::Event::Actionable &listener, const size_t timeout, size_t &registrationID );
    Chimera::Status_t removeListener( const size_t registrationID, const size_t timeout );

    /*-------------------------------------------------
    Interface: AsyncIO
    -------------------------------------------------*/
    Chimera::Status_t await( const Chimera::Event::Trigger event, const size_t timeout );
    Chimera::Status_t await( const Chimera::Event::Trigger event, Chimera::Threading::BinarySemaphore &notifier,
                             const size_t timeout );

    /*-------------------------------------------------
    ISR Event Handlers
    -------------------------------------------------*/
    void ProcessISREvent_TX();
    void ProcessISREvent_RX();
    void ProcessISREvent_Error();
    void ProcessISREvent_StatusChange();

  private:
    /*-------------------------------------------------
    Cached configuration settings
    -------------------------------------------------*/
    Chimera::CAN::DriverConfig mConfig;

    /*-------------------------------------------------
    Callbacks that can be executed upon an event
    -------------------------------------------------*/
    Chimera::Event::ActionableList eventListeners;
  };

}    // namespace Thor::CAN

#endif /* THOR_HLD_CAN_HPP */
