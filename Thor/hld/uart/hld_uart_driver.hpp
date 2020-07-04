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
  /**
   *  Initialize the driver
   *
   *  @return Chimera::Status_t
   */
  Chimera::Status_t initialize();

  class Driver : virtual public Chimera::UART::IUART,
                 public Chimera::Threading::Lockable
  {
  public:
    Driver();
    ~Driver();

    Chimera::Status_t assignHW( const Chimera::Serial::Channel channel, const Chimera::Serial::IOPins &pins ) final override;

    Chimera::Status_t begin( const Chimera::Hardware::PeripheralMode txMode,
                             const Chimera::Hardware::PeripheralMode rxMode ) final override;

    Chimera::Status_t end() final override;

    Chimera::Status_t configure( const Chimera::Serial::Config &config ) final override;

    Chimera::Status_t setBaud( const uint32_t baud ) final override;

    Chimera::Status_t setMode( const Chimera::Hardware::SubPeripheral periph,
                               const Chimera::Hardware::PeripheralMode mode ) final override;

    Chimera::Status_t write( const uint8_t *const buffer, const size_t length, const uint32_t timeout_mS = 500 ) final override;

    Chimera::Status_t read( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS = 500 ) final override;

    Chimera::Status_t flush( const Chimera::Hardware::SubPeripheral periph ) final override;

    Chimera::Status_t toggleAsyncListening( const bool state ) final override;
    Chimera::Status_t readAsync( uint8_t *const buffer, const size_t len ) final override;

    Chimera::Status_t enableBuffering( const Chimera::Hardware::SubPeripheral periph,
                                       boost::circular_buffer<uint8_t> *const userBuffer, uint8_t *const hwBuffer,
                                       const size_t hwBufferSize ) final override;

    Chimera::Status_t disableBuffering( const Chimera::Hardware::SubPeripheral periph ) final override;

    Chimera::Status_t registerListener( Chimera::Event::Actionable &listener, const size_t timeout, size_t &registrationID ) final override;

    Chimera::Status_t removeListener( const size_t registrationID, const size_t timeout ) final override;

    bool available( size_t *const bytes = nullptr ) final override;

    Chimera::Status_t await( const Chimera::Event::Trigger event, const size_t timeout ) final override;

    Chimera::Status_t await( const Chimera::Event::Trigger event, Chimera::Threading::BinarySemaphore &notifier,
                             const size_t timeout ) final override;

    void postISRProcessing() final override;
  };
}    // namespace Thor::UART

#endif /* !THOR_UART_HPP */
