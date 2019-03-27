/********************************************************************************
* File Name:
*   serial.hpp
*
* Description:
*   Wrapper around the Thor UART and USART drivers to provide merge the hardware
*   drivers into a common interface for the Chimera serial library.
*
* 2019 | Brandon Braun | brandonbraun653@gmail.com
********************************************************************************/

#pragma once
#ifndef THOR_SERIAL_HPP
#define THOR_SERIAL_HPP

/* C++ Includes */
#include <cstdlib>
#include <cstdint>
#include <memory>

/* Thor Includes */
#include <Thor/config.hpp>
#include <Thor/definitions.hpp>
#include <Thor/uart.hpp>
#include <Thor/usart.hpp>
#include <Thor/threads.hpp>

namespace Thor
{
  namespace Serial
  {
    class SerialClass;
    using SerialClass_sPtr = std::shared_ptr<SerialClass>;
    using SerialClass_uPtr = std::unique_ptr<SerialClass>;

    class SerialClass : public Chimera::Serial::Interface
    {
    public:
      SerialClass() = default;
      ~SerialClass() = default;

      SerialClass( const size_t bufferSize ) : bSize( bufferSize ) {}

      Chimera::Status_t assignHW(const uint8_t channel, const Chimera::Serial::IOPins &pins) final override;

      Chimera::Status_t begin( const Chimera::Serial::Modes txMode, const Chimera::Serial::Modes rxMode ) final override;

      Chimera::Status_t end() final override;

      Chimera::Status_t configure( const uint32_t baud, const Chimera::Serial::CharWid width,
                                   const Chimera::Serial::Parity parity, const Chimera::Serial::StopBits stop,
                                   const Chimera::Serial::FlowControl flow ) final override;

      Chimera::Status_t setBaud( const uint32_t baud ) final override;

      Chimera::Status_t setMode( const Chimera::Serial::SubPeripheral periph, const Chimera::Serial::Modes mode ) final override;

      Chimera::Status_t write( const uint8_t *const buffer, const size_t length,
                               const uint32_t timeout_mS = 500 ) final override;

      Chimera::Status_t read( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS = 500 ) final override;

      Chimera::Status_t flush( const Chimera::Serial::SubPeripheral periph ) final override;

      Chimera::Status_t readAsync( uint8_t *const buffer, const size_t len ) final override;

#if defined( USING_FREERTOS )
      Chimera::Status_t attachEventNotifier( const Chimera::Serial::Event event, SemaphoreHandle_t *const semphr ) final override;

      Chimera::Status_t removeEventNotifier( const Chimera::Serial::Event event, SemaphoreHandle_t *const semphr ) final override;
#endif

      Chimera::Status_t enableBuffering(const Chimera::Serial::SubPeripheral periph, boost::circular_buffer<uint8_t> *const buffer) final override;

      Chimera::Status_t disableBuffering(const Chimera::Serial::SubPeripheral periph) final override;


    private:
      uint8_t serialChannel = 0;
      size_t bSize = 1;

      Chimera::Serial::Interface_sPtr serialObject;
    };


  }    // namespace Serial
}    // namespace Thor
#endif /* !SERIAL_H_ */
