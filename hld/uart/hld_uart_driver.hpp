/********************************************************************************
 *  File Name:
 *    uart.hpp
 *
 *  Description:
 *    UART interface for Thor
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_UART_HPP
#define THOR_UART_HPP

/* C++ Includes */
#include <cstdlib>
#include <cstdint>
#include <memory>

/* Boost Includes */
#include <boost/circular_buffer_fwd.hpp>

/* Chimera Includes */
#include <Chimera/buffer>
#include <Chimera/thread>
#include <Chimera/event>
#include <Chimera/serial>
#include <Chimera/uart>

/* Thor Includes */
#include <Thor/gpio>

namespace Thor::UART
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize();
  Chimera::Status_t reset();
  bool isChannelUART( const Chimera::Serial::Channel channel );
  Chimera::UART::Driver_rPtr getDriver( const Chimera::Serial::Channel channel );


  /*-------------------------------------------------------------------------------
  Classes
  -------------------------------------------------------------------------------*/
  class Driver : public Chimera::Threading::Lockable<Driver>
  {
  public:
    Driver();
    ~Driver();
    Chimera::Status_t assignHW( const Chimera::Serial::Channel channel, const Chimera::Serial::IOPins &pins );
    Chimera::Status_t begin( const Chimera::Hardware::PeripheralMode txMode, const Chimera::Hardware::PeripheralMode rxMode );
    Chimera::Status_t end();
    Chimera::Status_t configure( const Chimera::Serial::Config &config );
    Chimera::Status_t setBaud( const uint32_t baud );
    Chimera::Status_t setMode( const Chimera::Hardware::SubPeripheral periph, const Chimera::Hardware::PeripheralMode mode );
    Chimera::Status_t write( const void *const buffer, const size_t length );
    Chimera::Status_t read( void *const buffer, const size_t length );
    Chimera::Status_t flush( const Chimera::Hardware::SubPeripheral periph );
    Chimera::Status_t toggleAsyncListening( const bool state );
    Chimera::Status_t readAsync( uint8_t *const buffer, const size_t len );
    Chimera::Status_t enableBuffering( const Chimera::Hardware::SubPeripheral periph,
                                       boost::circular_buffer<uint8_t> *const userBuffer, uint8_t *const hwBuffer,
                                       const size_t hwBufferSize );
    Chimera::Status_t disableBuffering( const Chimera::Hardware::SubPeripheral periph );
    Chimera::Status_t registerListener( Chimera::Event::Actionable &listener, const size_t timeout, size_t &registrationID );
    Chimera::Status_t removeListener( const size_t registrationID, const size_t timeout );
    bool available( size_t *const bytes = nullptr );
    Chimera::Status_t await( const Chimera::Event::Trigger event, const size_t timeout );
    Chimera::Status_t await( const Chimera::Event::Trigger event, Chimera::Threading::BinarySemaphore &notifier,
                             const size_t timeout );
    void postISRProcessing();

  private:
    friend Chimera::Threading::Lockable<Driver>;

  };
}    // namespace Thor::UART

#endif /* !THOR_UART_HPP */
