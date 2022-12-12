/********************************************************************************
 *  File Name:
 *    serial_intf.hpp
 *
 *  Description:
 *    STM32 Driver Model for Serial Communication
 *
 *  2019-2022 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_DRIVER_MODEL_SERIAL_HPP
#define THOR_DRIVER_MODEL_SERIAL_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <Chimera/serial>
#include <Thor/lld/interface/serial/lld_intf_serial_types.hpp>
#include <cstdint>
#include <cstdlib>


namespace Thor::LLD::Serial
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Initializes the low level driver
   * @return Chimera::Status_t
   */
  Chimera::Status_t initialize();

  /**
   * @brief Maps a UART/USART HW interface to a serial channel
   *
   * Expects to be called by the U(S)ART peripherals to let this driver know what kind
   * of mappings are available.
   *
   * @param channel   Serial channel to map to
   * @param intf      Hardware interface
   */
  void registerInterface( const Chimera::Serial::Channel channel, HwInterface *const intf );

  /**
   * @brief Gets a raw pointer to the driver for a particular channel
   *
   * @param channel   The channel to grab
   * @return Driver_rPtr
   */
  Driver_rPtr getDriver( const Chimera::Serial::Channel channel );

  /**
   * @brief Checks if the given hardware channel is supported on this device.
   *
   * @param channel   The channel number to be checked
   * @return bool
   */
  bool isSupported( const Chimera::Serial::Channel channel );


  /*---------------------------------------------------------------------------
  Driver Interface
  ---------------------------------------------------------------------------*/
  class Driver
  {
  public:
    HwInterface *mHWIntf;

    Driver();
    ~Driver();
    Chimera::Status_t open( const Chimera::Serial::Config &config );
    Chimera::Status_t close();
    int write( const Chimera::Serial::TxfrMode mode, const void *const buffer, const size_t length );
    int read( const Chimera::Serial::TxfrMode mode, void *const buffer, const size_t length );
    Chimera::Status_t txStatus();
    Chimera::Status_t rxStatus();
    Flag_t getFlags();
    void clearFlags( const uint32_t flagBits );
    CDTCB *getTCB_TX();
    MDTCB *getTCB_RX();
  };

}    // namespace Thor::LLD::Serial


#endif /* !THOR_DRIVER_MODEL_SERIAL_HPP */