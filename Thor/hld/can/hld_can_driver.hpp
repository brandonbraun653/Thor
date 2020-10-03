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

    void postISRProcessing();

    /*-------------------------------------------------
    Interface: Hardware
    -------------------------------------------------*/
    Chimera::Status_t open( const Chimera::CAN::DriverConfig &cfg );
    Chimera::Status_t close();
    Chimera::CAN::CANStatus getStatus();
    Chimera::Status_t send( const Chimera::CAN::BasicFrame &frame );
    Chimera::Status_t receive( Chimera::CAN::BasicFrame &frame, const size_t timeout );
    Chimera::Status_t subscribe( const Chimera::CAN::Identifier_t id, Chimera::CAN::FrameCallback_t callback );
    Chimera::Status_t unsubscribe( const Chimera::CAN::Identifier_t id );
    Chimera::Status_t filter( const Chimera::CAN::Filter *const list, const size_t size );
    Chimera::Status_t flush( Chimera::CAN::BufferType buffer );

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

  private:
    Chimera::CAN::DriverConfig mConfig; /**< Configuration used to set up the class */
    Chimera::GPIO::Driver_sPtr mPinTX;  /**< CAN TX Pin */
    Chimera::GPIO::Driver_sPtr mPinRX;  /**< CAN RX Pin */

    Chimera::Event::ActionableList eventListeners;
    Chimera::Threading::BinarySemaphore awaitTransferComplete; /**< Internal signal for current transfer completed */
  };

}    // namespace Thor::CAN

#endif /* THOR_HLD_CAN_HPP */
