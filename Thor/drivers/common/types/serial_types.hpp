/********************************************************************************
 *   File Name:
 *    serial_types.hpp
 *
 *   Description:
 *    STM32 Serial Driver Types
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_DRIVER_TYPES_SERIAL_HPP
#define THOR_DRIVER_TYPES_SERIAL_HPP

/* C++ Includes */
#include <cstdint>

/* Driver Includes */
#include <Thor/drivers/common/types/interrupt_types.hpp>

namespace Thor::Driver::Serial
{
  struct Config
  {
    /*------------------------------------------------
    This member configures the Usart communication baud rate.
    The baud rate is computed using the following formula:
      - IntegerDivider = ((PCLKx) / (8 * (husart->Init.BaudRate)))
      - FractionalDivider = ((IntegerDivider - ((size_t) IntegerDivider)) * 8) + 0.5 
    ------------------------------------------------*/
    size_t BaudRate; 

    /*------------------------------------------------
    Specifies the number of data bits transmitted or received in a frame
    ------------------------------------------------*/
    size_t WordLength;
                              
    /*------------------------------------------------
    Specifies the number of stop bits transmitted
    ------------------------------------------------*/
    size_t StopBits; 

    /*------------------------------------------------
    Specifies the parity mode
    ------------------------------------------------*/
    size_t Parity;

    /*------------------------------------------------
    Specifies whether the Receive or Transmit mode is enabled or disabled
    ------------------------------------------------*/
    size_t Mode; 

    /*------------------------------------------------
    Specifies the steady state of the serial clock
    ------------------------------------------------*/
    size_t CLKPolarity;

    /*------------------------------------------------
    Specifies the clock transition on which the bit capture is made
    ------------------------------------------------*/
    size_t CLKPhase; 

    /*------------------------------------------------
    Specifies whether the clock pulse corresponding to the last transmitted
    data bit (MSB) has to be output on the SCLK pin in synchronous mode
    ------------------------------------------------*/
    size_t CLKLastBit;
  };

  class ITSigUSART
  {
  public:
    static constexpr InterruptSignal_t TRANSMIT_DATA_REGISTER_EMPTY = USARTSigOffset + 0;
    static constexpr InterruptSignal_t RECEIVED_DATA_READ_TO_READ   = USARTSigOffset + 1;
    static constexpr InterruptSignal_t TRANSMISSION_COMPLETE        = USARTSigOffset + 2;
    static constexpr InterruptSignal_t IDLE_LINE_DETECTED           = USARTSigOffset + 3;
    static constexpr InterruptSignal_t PARITY_ERROR                 = USARTSigOffset + 4;
    static constexpr InterruptSignal_t OVERRUN_ERROR                = USARTSigOffset + 5;
    static constexpr InterruptSignal_t FRAMING_ERROR                = USARTSigOffset + 6;
    static constexpr InterruptSignal_t BREAK_FLAG                   = USARTSigOffset + 7;
    static constexpr InterruptSignal_t NOISE_FLAG                   = USARTSigOffset + 8;
    static constexpr InterruptSignal_t CTS_FLAG                     = USARTSigOffset + 9;

    static constexpr InterruptSignal_t NUM_INTERRUPT_SIGNALS = CTS_FLAG - TRANSMIT_DATA_REGISTER_EMPTY + 1;

    static_assert( NUM_INTERRUPT_SIGNALS <= USARTMaxSig, "Too many interrupt signals for USART" );
  };

}    // namespace Thor::Driver::Serial


#endif /* !THOR_DRIVER_TYPES_SERIAL_HPP */