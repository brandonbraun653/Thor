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

  };

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct TxfrCB
  {
    volatile bool inProgress;               /**< Transfer in progress? */
    uint16_t address;                       /**< Address being transferred to/from */
    const void *txData;                     /**< Buffer for transmit data */
    void *rxData;                           /**< Buffer for received data */
    size_t index;                           /**< Byte index into the tx/rx buffers */
    size_t length;                          /**< Number of bytes being transferred */
    TxfrState state;                        /**< Current state of the transfer */
    uint32_t errorBF;                       /**< Error bitfield (1u << TxfrError::value ) */
    Chimera::Peripheral::TransferMode mode; /**< Mode of transfer being used */

    void clear()
    {
      inProgress = false;
      address    = 0;
      txData     = nullptr;
      rxData     = nullptr;
      length     = 0;
      state      = TxfrState::IDLE;
      mode       = Chimera::Peripheral::TransferMode::UNKNOWN;
      errorBF    = 0;
    }
  };
}    // namespace Thor::LLD::I2C

#endif /* !THOR_LLD_I2C_TYPES_HPP */
