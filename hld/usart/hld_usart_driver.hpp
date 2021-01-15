/********************************************************************************
 *  File Name:
 *    hld_usart_driver.hpp
 *
 *  Description:
 *    USART interface for Thor. This file supports the top level interface layer
 *    that all drivers for the underlying hardware must conform to.
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_USART_HPP
#define THOR_USART_HPP

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
#include <Chimera/usart>

/* Thor Includes */
#include <Thor/gpio>
#include <Thor/hld/usart/hld_usart_types.hpp>

namespace Thor::USART
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize();
  Chimera::Status_t reset();
  bool isChannelUSART( const Chimera::Serial::Channel channel );
  Driver_rPtr getDriver( const Chimera::Serial::Channel channel );


  /*-------------------------------------------------------------------------------
  Classes
  -------------------------------------------------------------------------------*/
  class Driver : public Chimera::Threading::LockableCRTP<Driver>
  {
  public:
    Driver();
    ~Driver();

    Chimera::Status_t assignHW( const Chimera::Serial::Channel channel, const Chimera::Serial::IOPins &pins );
    Chimera::Status_t begin( const Chimera::Hardware::PeripheralMode txMode,
                             const Chimera::Hardware::PeripheralMode rxMode );
    Chimera::Status_t end();
    Chimera::Status_t configure( const Chimera::Serial::Config &config );
    Chimera::Status_t setBaud( const uint32_t baud );
    Chimera::Status_t setMode( const Chimera::Hardware::SubPeripheral periph,
                               const Chimera::Hardware::PeripheralMode mode );
    Chimera::Status_t write( const void *const buffer, const size_t length );
    Chimera::Status_t read( void *const buffer, const size_t length );
    Chimera::Status_t flush( const Chimera::Hardware::SubPeripheral periph );
    Chimera::Status_t toggleAsyncListening( const bool state );
    Chimera::Status_t readAsync( uint8_t *const buffer, const size_t len );
    Chimera::Status_t enableBuffering( const Chimera::Hardware::SubPeripheral periph,
                                       boost::circular_buffer<uint8_t> *const userBuffer, uint8_t *const hwBuffer,
                                       const size_t hwBufferSize );
    Chimera::Status_t disableBuffering( const Chimera::Hardware::SubPeripheral periph );
    bool available( size_t *const bytes = nullptr );
    Chimera::Status_t await( const Chimera::Event::Trigger event, const size_t timeout );
    Chimera::Status_t await( const Chimera::Event::Trigger event, Chimera::Threading::BinarySemaphore &notifier,
                             const size_t timeout );
    void postISRProcessing();

  private:
    friend Chimera::Threading::LockableCRTP<Driver>;
    Chimera::Threading::RecursiveTimedMutex mClsMutex;

    Chimera::GPIO::Driver_rPtr rxPin;
    Chimera::GPIO::Driver_rPtr txPin;

    bool enabled;
    Chimera::Serial::Channel channel; /**< Hardware channel associated with this driver */
    size_t resourceIndex;             /**< Lookup table index for USART resources */


    Chimera::Threading::BinarySemaphore awaitRXComplete;
    Chimera::Threading::BinarySemaphore awaitTXComplete;
    Chimera::Threading::BinarySemaphore rxLock;
    Chimera::Threading::BinarySemaphore txLock;

    Chimera::Buffer::PeripheralBuffer txBuffers;
    Chimera::Buffer::PeripheralBuffer rxBuffers;

    Chimera::Hardware::PeripheralMode txMode;
    Chimera::Hardware::PeripheralMode rxMode;

    Chimera::Status_t readBlocking( void *const buffer, const size_t length );
    Chimera::Status_t readInterrupt( void *const buffer, const size_t length );
    Chimera::Status_t readDMA( void *const buffer, const size_t length );

    Chimera::Status_t writeBlocking( const void *const buffer, const size_t length );
    Chimera::Status_t writeInterrupt( const void *const buffer, const size_t length );
    Chimera::Status_t writeDMA( const void *const buffer, const size_t length );
  };
}    // namespace Thor::USART

#endif /* !THOR_USART_HPP */
