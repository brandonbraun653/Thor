/******************************************************************************
 *  File Name:
 *    i2c_types.hpp
 *
 *  Description:
 *    Thor I2C LLD interface types
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_I2C_TYPES_HPP
#define THOR_LLD_I2C_TYPES_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/peripheral>


namespace Thor::LLD::I2C
{
  /*---------------------------------------------------------------------------
  Forward Declarations
  ---------------------------------------------------------------------------*/
  class Driver;
  struct RegisterMap;

  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  using Driver_rPtr = Driver *;

  /*-----------------------------------------------------------------------------
  Enumerations
  -----------------------------------------------------------------------------*/
  enum class DMADirection : uint8_t
  {
    TX,
    RX,

    NUM_OPTIONS,
    UNKNOWN
  };


  enum class IRQHandlerIndex : uint8_t
  {
    EVENT,
    ERROR,

    NUM_OPTIONS,
    UNKNOWN,
  };


  enum class TxfrState : uint8_t
  {
    IDLE,       /**< Nothing happening */
    ADDRESS,    /**< Device address needs to be sent */
    DATA,       /**< Data payload being sent */
    COMPLETE,   /**< Transfer complete */
    ERROR,      /**< An error occurred */

    NUM_OPTIONS,
    UNKNOWN
  };

  enum TxfrError : uint8_t
  {
    ERR_NACK,
    ERR_BUS,
    ERR_ARBITRATION_LOSS,
    ERR_OVER_UNDER_RUN,
    ERR_PACKET_ERROR_CHECK,
    ERR_TIMEOUT,
    ERR_SMBUS_ALERT,

    ERR_NUM_OPTIONS,
    ERR_UNKNOWN
  };
  static_assert( ERR_NUM_OPTIONS < CHAR_BIT );

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct TxfrCB
  {
    volatile bool                     inProgress;    /**< Transfer in progress? */
    uint16_t                          slave_address; /**< Address being transferred to/from */
    const uint8_t                    *txData;        /**< Buffer for transmit data */
    uint8_t                          *rxData;        /**< Buffer for received data */
    size_t                            offset;        /**< Byte index into the tx/rx buffers */
    size_t                            bytes_left;    /**< Number of bytes being transferred */
    TxfrState                         state;         /**< Current state of the transfer */
    uint32_t                          errorBF;       /**< Error bitfield (1u << TxfrError::value ) */
    Chimera::Peripheral::TransferMode txfrMode;      /**< Mode of transfer being used */

    void clear()
    {
      inProgress    = false;
      slave_address = 0;
      txData        = nullptr;
      rxData        = nullptr;
      offset        = 0;
      state         = TxfrState::IDLE;
      txfrMode      = Chimera::Peripheral::TransferMode::UNKNOWN;
      errorBF       = 0;
    }
  };
}    // namespace Thor::LLD::I2C

#endif /* !THOR_LLD_I2C_TYPES_HPP */
