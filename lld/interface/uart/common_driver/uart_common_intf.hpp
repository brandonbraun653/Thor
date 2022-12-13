/******************************************************************************
 *  File Name:
 *    uart_common_intf.hpp
 *
 *  Description:
 *    Shared low level driver interface for accessing register level functions
 *    that may differ across microcontrollers
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_UART_COMMON_DRIVER_INTF_HPP
#define THOR_LLD_UART_COMMON_DRIVER_INTF_HPP

/* STL Includes */
#include <cstdint>

/* Thor Includes */
#include <Thor/lld/interface/uart/uart_detail.hpp>
#include <Thor/lld/interface/uart/uart_types.hpp>

namespace Thor::LLD::UART
{
  /**
   *  Enables the hardware transmitter
   *
   *  @param[in]  periph    Peripheral to act on
   *  @return void
   */
  void prjEnableTransmitter( RegisterMap *const periph );

  /**
   *  Disables the hardware transmitter
   *
   *  @param[in]  periph    Peripheral to act on
   *  @return void
   */
  void prjDisableTransmitter( RegisterMap *const periph );

  /**
   *  Enables the hardware receiver
   *
   *  @param[in]  periph    Peripheral to act on
   *  @return void
   */
  void prjEnableReceiver( RegisterMap *const periph );

  /**
   *  Disables the hardware receiver
   *
   *  @param[in]  periph    Peripheral to act on
   *  @return void
   */
  void prjDisableReceiver( RegisterMap *const periph );

  /**
   *  Writes the given data character onto the output. The amount of
   *  data actually written to the output depends on the config.
   *
   *  @param[in]  periph    Peripheral to act on
   *  @param[in]  data      Character to write
   *  @return void
   */
  void prjWriteDataRegister( RegisterMap *const periph, const uint16_t data );

  /**
   *  Reads the latest character off the input. the actual size
   *  is determined by the peripheral config.
   *
   *  @param[in]  periph    Peripheral to act on
   *  @return uint16_t
   */
  uint16_t prjReadDataRegister( RegisterMap *const periph );

  /**
   *  Enables an ISR signal
   *
   *  @param[in]  periph    Peripheral to act on
   *  @param[in]  signal    Which signal to act on
   *  @return void
   */
  void prjEnableISRSignal( RegisterMap *const periph, const ISRSignal signal );

  /**
   *  Disables an ISR signal
   *
   *  @param[in]  periph    Peripheral to act on
   *  @return void
   */
  void prjDisableISRSignal( RegisterMap *const periph, const ISRSignal signal );

  /**
   *  Checks if an ISR flag has been set
   *
   *  @param[in]  periph    Peripheral to act on
   *  @param[in]  signal    Which signal to act on
   *  @return bool
   */
  bool prjGetISRSignal( RegisterMap *const periph, const ISRSignal signal );

  /**
   *  Manually sets an ISR flag
   *
   *  @param[in]  periph    Peripheral to act on
   *  @param[in]  signal    Which signal to act on
   *  @return void
   */
  void prjSetISRSignal( RegisterMap *const periph, const ISRSignal signal );

  /**
   *  Clears an ISR flag
   *
   *  @param[in]  periph    Peripheral to act on
   *  @param[in]  signal    Which signal to act on
   *  @return void
   */
  void prjClrISRSignal( RegisterMap *const periph, const ISRSignal signal );

}  // namespace Thor::LLD::UART

#endif  /* !THOR_LLD_UART_COMMON_DRIVER_INTF_HPP */
