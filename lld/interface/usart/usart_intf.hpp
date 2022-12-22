/******************************************************************************
 *  File Name:
 *    usart_intf.hpp
 *
 *  Description:
 *    LLD interface to the USART module
 *
 *  2020-2022 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef LLD_USART_INTERFACE_HPP
#define LLD_USART_INTERFACE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <Chimera/dma>
#include <Chimera/thread>
#include <Chimera/usart>
#include <Thor/lld/common/interrupts/usart_interrupt_vectors.hpp>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/serial/lld_intf_serial_types.hpp>
#include <Thor/lld/interface/usart/usart_types.hpp>

namespace Thor::LLD::USART
{
  /*---------------------------------------------------------------------------
  Public Functions (Implemented by the project)
  ---------------------------------------------------------------------------*/
  /**
   *  Initializes the low level driver
   *
   *  @return Chimera::Status_t
   */
  Chimera::Status_t initialize();

  /**
   *  Gets a raw pointer to the driver for a particular channel
   *
   *  @param[in] channel        The channel to grab
   *  @return Driver_rPtr      Instance of the driver for the requested channel
   */
  Driver_rPtr getDriver( const Chimera::Serial::Channel channel );


  /*---------------------------------------------------------------------------
  Public Functions (Implemented by the interface layer)
  ---------------------------------------------------------------------------*/
  /**
   *  Checks if the given hardware channel is supported on this device.
   *
   *  @param[in]  channel       The channel number to be checked
   *  @return bool
   */
  bool isSupported( const Chimera::Serial::Channel channel );

  /**
   *  Gets the resource index associated with a particular channel. If not
   *  supported, will return INVALID_RESOURCE_INDEX
   *
   *  @param[in]  channel       The channel number to be checked
   *  @return RIndex_t
   */
  RIndex_t getResourceIndex( const Chimera::Serial::Channel channel );

  /**
   *  Looks up a resource index based on a raw peripheral instance. If not
   *  supported, will return INVALID_RESOURCE_INDEX
   *
   *  @param[in]  address       The peripheral address
   *  @return RIndex_t
   */
  RIndex_t getResourceIndex( const std::uintptr_t address );

  /**
   *  Gets the channel associated with a peripheral address
   *
   *  @param[in]  address       Memory address the peripheral is mapped to
   *  @return Chimera::Serial::Channel
   */
  Chimera::Serial::Channel getChannel( const std::uintptr_t address );

  /**
   *  Initializes the drivers by attaching the appropriate peripheral
   *
   *  @param[in]  driverList    List of driver objects to be initialized
   *  @param[in]  numDrivers    How many drivers are in driverList
   *  @return bool
   */
  bool attachDriverInstances( Driver *const driverList, const size_t numDrivers );

  /**
   *  Registers a handler for the given peripheral interrupt signal
   *
   *  @param[in]  signal        Peripheral specific ISR signal
   *  @param[in]  callback      Callback data to register
   *  @return Chimera::Status_t
   */
  Chimera::Status_t registerHandler( const Chimera::Interrupt::Signal_t        signal,
                                     const Chimera::Interrupt::SignalCallback &callback );

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  class Driver : public virtual Thor::LLD::Serial::HwInterface
  {
  public:
    Driver();
    ~Driver();

    /**
     *  Attaches a peripheral instance to the interaction model
     *
     *  @param[in]  peripheral    Memory mapped struct of the desired peripheral
     *  @return void
     */
    Chimera::Status_t attach( RegisterMap *const peripheral );
    Chimera::Status_t reset();
    Chimera::Status_t enableIT( const Chimera::Hardware::SubPeripheral periph );
    Chimera::Status_t disableIT( const Chimera::Hardware::SubPeripheral periph );
    Chimera::Status_t txInterrupt( etl::span<uint8_t> &buffer );
    Chimera::Status_t receiveIT( etl::span<uint8_t> &buffer );
    Chimera::Status_t initDMA();
    Chimera::Status_t deinitDMA();
    Chimera::Status_t enableDMA_IT( const Chimera::Hardware::SubPeripheral periph );
    Chimera::Status_t disableDMA_IT( const Chimera::Hardware::SubPeripheral periph );
    Chimera::Status_t txDMA( etl::span<uint8_t> &buffer );
    Chimera::Status_t receiveDMA( etl::span<uint8_t> &buffer );

    /*-------------------------------------------------------------------------
    Virtual Interface
    -------------------------------------------------------------------------*/
    Chimera::Peripheral::Type periphType() final override
    {
      return Chimera::Peripheral::Type::PERIPH_USART;
    }

    Chimera::Status_t init( const Thor::LLD::Serial::RegConfig &cfg ) final override;
    Chimera::Status_t deinit() final override;
    int transmit( const Chimera::Serial::TxfrMode mode, etl::span<uint8_t> &buffer ) final override;
    int receive( const Chimera::Serial::TxfrMode mode, etl::span<uint8_t> &buffer ) final override;
    Chimera::Status_t         txTransferStatus() final override;
    Chimera::Status_t         rxTransferStatus() final override;
    uint32_t                  getFlags() final override;
    void                      clearFlags( const uint32_t flagBits ) final override;
    Thor::LLD::Serial::CDTCB *getTCB_TX() final override;
    Thor::LLD::Serial::MDTCB *getTCB_RX() final override;
    void                      killTransfer( Chimera::Hardware::SubPeripheral periph ) final override;


  protected:
    friend void( ::USART1_IRQHandler )();
    friend void( ::USART2_IRQHandler )();
    friend void( ::USART3_IRQHandler )();
    friend void( ::USART6_IRQHandler )();

    Chimera::Status_t txBlocking( etl::span<uint8_t> &buffer );

    /**
     *  Generic interrupt handler for all USART specific ISR signals
     *
     *  @return void
     */
    void IRQHandler();

  private:
    /*-------------------------------------------------------------------------
    Misc Driver State Variables
    -------------------------------------------------------------------------*/
    RegisterMap            *mPeriph;         /**< Points to the hardware registers for this instance */
    size_t                  mResourceIndex;  /**< Derived lookup table index for resource access */
    volatile Reg32_t        mRuntimeFlags;   /**< Error/process flags set at runtime to indicate state */
    Chimera::DMA::RequestId mTXDMARequestId; /**< Request id of the TX DMA pipe for the driver */
    Chimera::DMA::RequestId mRXDMARequestId; /**< Request id of the RX DMA pipe for the driver */
    bool                    mDMAPipesReady;  /**< Track if the DMA pipes have been established */

    /*-------------------------------------------------------------------------
    Transfer Control Blocks
    -------------------------------------------------------------------------*/
    Thor::LLD::Serial::CDTCB mTXTCB;
    Thor::LLD::Serial::MDTCB mRXTCB;

    /**
     *  Calculates the appropriate configuration value for the Baud Rate Register
     *  given a desired baud rate.
     *
     *  @param[in]  desiredBaud   The baud rate to be configured
     *  @return uint32_t
     */
    uint32_t calculateBRR( const size_t desiredBaud );

    inline void disableUSARTInterrupts();
    inline void enableUSARTInterrupts();
    void        onDMATXComplete( const Chimera::DMA::TransferStats &stats );
    void        onDMARXComplete( const Chimera::DMA::TransferStats &stats );
  };
}    // namespace Thor::LLD::USART

#endif /* !LLD_USART_INTERFACE_HPP */
