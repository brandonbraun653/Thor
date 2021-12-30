/******************************************************************************
 *  File Name:
 *    hld_i2c_driver.hpp
 *
 *  Description:
 *    Thor HLD I2C driver
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_HLD_I2C_DRIVER_HPP
#define THOR_HLD_I2C_DRIVER_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/event>
#include <Chimera/gpio>
#include <Chimera/i2c>
#include <Chimera/thread>
#include <Thor/hld/i2c/hld_i2c_types.hpp>
#include <cstdint>

namespace Thor::I2C
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  Chimera::Status_t initialize();
  Chimera::Status_t reset();
  Driver_rPtr getDriver( const Chimera::I2C::Channel channel );

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
    Chimera::Status_t open( const Chimera::I2C::DriverConfig &cfg );
    Chimera::Status_t close();
    Chimera::Status_t read( const uint16_t address, void *const data, const size_t length );
    Chimera::Status_t write( const uint16_t address, const void *const data, const size_t length );
    Chimera::Status_t transfer( const uint16_t address, const void *const tx_data, void *const rx_data, const size_t length );
    Chimera::Status_t stop();
    Chimera::Status_t start();

    /*-------------------------------------------------------------------------
    Interface: Listener
    -------------------------------------------------------------------------*/
    Chimera::Status_t registerListener( Chimera::Event::Actionable &listener, const size_t timeout, size_t &registrationID );
    Chimera::Status_t removeListener( const size_t registrationID, const size_t timeout );

    /*-------------------------------------------------------------------------
    Interface: AsyncIO
    -------------------------------------------------------------------------*/
    Chimera::Status_t await( const Chimera::Event::Trigger event, const size_t timeout );
    Chimera::Status_t await( const Chimera::Event::Trigger event, Chimera::Thread::BinarySemaphore &notifier,
                             const size_t timeout );

    /*-------------------------------------------------------------------------
    ISR Event Handlers
    -------------------------------------------------------------------------*/
    void postISRProcessing();

  private:
    friend Chimera::Thread::Lockable<Driver>;

    Chimera::I2C::DriverConfig mConfig;
    Chimera::Event::ActionableList eventListeners;
  };
}  // namespace Thor::I2C

#endif  /* !THOR_HLD_I2C_DRIVER_HPP */
