/********************************************************************************
 *  File Name:
 *    hld_usart_driver.hpp
 *
 *  Description:
 *    USART interface for Thor. This file supports the top level interface layer
 *    that all drivers for the underlying hardware must conform to.
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
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

namespace Thor::USART
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize();
  Chimera::Status_t reset();
  bool isChannelUSART( const Chimera::Serial::Channel channel );
  Chimera::USART::IUSART_sPtr getDriver( const Chimera::Serial::Channel channel );


  /*-------------------------------------------------------------------------------
  Classes
  -------------------------------------------------------------------------------*/
  class Driver : virtual public Chimera::USART::IUSART,
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

  private:
    Thor::GPIO::Driver_sPtr rxPin;
    Thor::GPIO::Driver_sPtr txPin;

    Chimera::Serial::Channel channel; /**< Hardware channel associated with this driver */
    size_t resourceIndex;             /**< Lookup table index for USART resources */


    size_t listenerIDCount;
    Chimera::Event::ActionableList eventListeners;

    Chimera::Threading::BinarySemaphore awaitRXComplete;
    Chimera::Threading::BinarySemaphore awaitTXComplete;
    Chimera::Threading::BinarySemaphore rxLock;
    Chimera::Threading::BinarySemaphore txLock;

    Chimera::Buffer::PeripheralBuffer txBuffers;
    Chimera::Buffer::PeripheralBuffer rxBuffers;

    Chimera::Hardware::PeripheralMode txMode;
    Chimera::Hardware::PeripheralMode rxMode;

    void processListeners( const Chimera::Event::Trigger event );

    std::array<Chimera::Status_t (Thor::USART::Driver::*)( uint8_t *const, const size_t, const uint32_t ), 3> readFuncPtrs;
    Chimera::Status_t readBlocking( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS );
    Chimera::Status_t readInterrupt( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS );
    Chimera::Status_t readDMA( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS );

    std::array<Chimera::Status_t (Thor::USART::Driver::*)( const uint8_t *const, const size_t, const uint32_t ), 3> writeFuncPtrs;
    Chimera::Status_t writeBlocking( const uint8_t *const buffer, const size_t length, const uint32_t timeout_mS );
    Chimera::Status_t writeInterrupt( const uint8_t *const buffer, const size_t length, const uint32_t timeout_mS );
    Chimera::Status_t writeDMA( const uint8_t *const buffer, const size_t length, const uint32_t timeout_mS );
  };

  using USARTClass_sPtr = std::shared_ptr<Driver>;
  using USARTClass_uPtr = std::unique_ptr<Driver>;
}    // namespace Thor::USART

#endif /* !THOR_USART_HPP */
