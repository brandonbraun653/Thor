/********************************************************************************
 * File Name:
 *    usart.hpp
 *
 * Description:
 *    USART interface for Thor. This file supports the top level interface layer
 *    that all drivers for the underlying hardware must conform to.
 *
 * 2019 | Brandon Braun | brandonbraun653@gmail.com
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
#include <Chimera/buffer.hpp>
#include <Chimera/threading.hpp>
#include <Chimera/interface/serial_intf.hpp>
#include <Chimera/types/event_types.hpp>

/* Thor Includes */
#include <Thor/drivers/usart.hpp>
#include <Thor/gpio.hpp>
#include <Thor/types/interrupt_types.hpp>


namespace Thor::USART
{
#if ( THOR_CUSTOM_DRIVERS == 1 ) && ( THOR_DRIVER_USART == 1 )

  class USARTClass : public Chimera::Serial::Interface, public Chimera::Threading::Lockable
  {
  public:
    USARTClass();
    ~USARTClass();

    Chimera::Status_t assignHW( const uint8_t channel, const Chimera::Serial::IOPins &pins ) final override;

    Chimera::Status_t begin( const Chimera::Hardware::SubPeripheralMode txMode,
                             const Chimera::Hardware::SubPeripheralMode rxMode ) final override;

    Chimera::Status_t end() final override;

    Chimera::Status_t configure( const Chimera::Serial::Config &config ) final override;

    Chimera::Status_t setBaud( const uint32_t baud ) final override;

    Chimera::Status_t setMode( const Chimera::Hardware::SubPeripheral periph,
                               const Chimera::Hardware::SubPeripheralMode mode ) final override;

    Chimera::Status_t write( const uint8_t *const buffer, const size_t length, const uint32_t timeout_mS = 500 ) final override;

    Chimera::Status_t read( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS = 500 ) final override;

    Chimera::Status_t flush( const Chimera::Hardware::SubPeripheral periph ) final override;

    Chimera::Status_t readAsync( uint8_t *const buffer, const size_t len ) final override;

    Chimera::Status_t enableBuffering( const Chimera::Hardware::SubPeripheral periph,
                                       boost::circular_buffer<uint8_t> *const userBuffer, uint8_t *const hwBuffer,
                                       const uint32_t hwBufferSize ) final override;

    Chimera::Status_t disableBuffering( const Chimera::Hardware::SubPeripheral periph ) final override;

    Chimera::Status_t registerListener( Chimera::Event::Actionable &listener, const size_t timeout, size_t &registrationID ) final override;

    Chimera::Status_t removeListener( const size_t registrationID, const size_t timeout ) final override;

    bool available( size_t *const bytes = nullptr ) final override;

    Chimera::Status_t await( const Chimera::Event::Trigger event, const size_t timeout ) final override;

    Chimera::Status_t await( const Chimera::Event::Trigger event, SemaphoreHandle_t notifier,
                             const size_t timeout ) final override;

    void postISRProcessing() final override;

  private:
    Thor::Driver::USART::Driver_uPtr hwDriver;
    Thor::GPIO::GPIOClass_uPtr rxPin;
    Thor::GPIO::GPIOClass_uPtr txPin;

    uint8_t channel;      /**< Hardware channel associated with this driver */
    size_t resourceIndex; /**< Lookup table index for USART resources */


    size_t listenerIDCount;
    std::vector<Chimera::Event::Actionable> eventListeners;

    SemaphoreHandle_t awaitRXComplete;
    SemaphoreHandle_t awaitTXComplete;
    SemaphoreHandle_t rxLock;
    SemaphoreHandle_t txLock;

    Chimera::Buffer::PeripheralBuffer txBuffers;
    Chimera::Buffer::PeripheralBuffer rxBuffers;

    Chimera::Hardware::SubPeripheralMode txMode;
    Chimera::Hardware::SubPeripheralMode rxMode;

    void processListeners( const Chimera::Event::Trigger event );

    std::array<Chimera::Status_t ( USARTClass::* )( uint8_t *const, const size_t, const uint32_t ), 3> readFuncPtrs;
    Chimera::Status_t readBlocking( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS );
    Chimera::Status_t readInterrupt( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS );
    Chimera::Status_t readDMA( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS );

    std::array<Chimera::Status_t ( USARTClass::* )( const uint8_t *const, const size_t, const uint32_t ), 3> writeFuncPtrs;
    Chimera::Status_t writeBlocking( const uint8_t *const buffer, const size_t length, const uint32_t timeout_mS );
    Chimera::Status_t writeInterrupt( const uint8_t *const buffer, const size_t length, const uint32_t timeout_mS );
    Chimera::Status_t writeDMA( const uint8_t *const buffer, const size_t length, const uint32_t timeout_mS );
  };

  using USARTClass_sPtr = std::shared_ptr<USARTClass>;
  using USARTClass_uPtr = std::unique_ptr<USARTClass>;

#endif /* THOR_CUSTOM_DRIVERS && THOR_DRIVER_USART */

}    // namespace Thor::USART
#endif /* !THOR_USART_HPP */
