/********************************************************************************
 *  File Name:
 *    hw_usart_driver.hpp
 *
 *  Description:
 *    Declares the LLD interface to the STM32L4 series USART hardware.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_USART_DRIVER_STM32L4_HPP
#define THOR_HW_USART_DRIVER_STM32L4_HPP

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/usart>
#include <Chimera/thread>

/* Thor Includes */
#include <Thor/lld/common/interrupts/usart_interrupt_vectors.hpp>
#include <Thor/lld/stm32l4x/usart/hw_usart_types.hpp>
#include <Thor/lld/stm32l4x/usart/hw_usart_mapping.hpp>
#include <Thor/lld/interface/usart/usart_intf.hpp>
#include <Thor/lld/interface/interrupt/interrupt_intf.hpp>
#include <Thor/lld/interface/serial/serial_intf.hpp>
#include <Thor/lld/interface/serial/serial_types.hpp>

namespace Thor::LLD::USART
{
  class Driver : public Thor::LLD::Serial::Basic,
                 public Thor::LLD::Serial::Extended
  {
  public:
    Driver( RegisterMap *peripheral );
    ~Driver();

    Chimera::Status_t init( const Thor::LLD::Serial::Config &cfg ) final override;

    Chimera::Status_t deinit() final override;

    Chimera::Status_t reset() final override;

    Chimera::Status_t transmit( const uint8_t *const data, const size_t size, const size_t timeout ) final override;

    Chimera::Status_t receive( uint8_t *const data, const size_t size, const size_t timeout ) final override;

    Chimera::Status_t enableIT( const Chimera::Hardware::SubPeripheral periph ) final override;

    Chimera::Status_t disableIT( const Chimera::Hardware::SubPeripheral periph ) final override;

    Chimera::Status_t transmitIT( const uint8_t *const data, const size_t size, const size_t timeout ) final override;

    Chimera::Status_t receiveIT( uint8_t *const data, const size_t size, const size_t timeout ) final override;

    Chimera::Status_t initDMA() final override;

    Chimera::Status_t deinitDMA() final override;

    Chimera::Status_t enableDMA_IT( const Chimera::Hardware::SubPeripheral periph ) final override;

    Chimera::Status_t disableDMA_IT( const Chimera::Hardware::SubPeripheral periph ) final override;

    Chimera::Status_t transmitDMA( const void *const data, const size_t size, const size_t timeout ) final override;

    Chimera::Status_t receiveDMA( void *const data, const size_t size, const size_t timeout ) final override;

    Chimera::Status_t txTransferStatus() final override;

    Chimera::Status_t rxTransferStatus() final override;

    uint32_t getFlags() final override;

    void clearFlags( const uint32_t flagBits ) final override;

    void killTransmit() final override;

    void killReceive() final override;

    void attachISRWakeup( Chimera::Threading::BinarySemaphore *const wakeup );

    Thor::LLD::Serial::CDTCB getTCB_TX();
    Thor::LLD::Serial::MDTCB getTCB_RX();
    Thor::LLD::Serial::Config getConfiguration();

  protected:
    friend void(::USART1_IRQHandler )();
    friend void(::USART2_IRQHandler )();
    friend void(::USART3_IRQHandler )();

    /**
     *  Generic interrupt handler for all USART specific ISR signals
     *
     *  @return void
     */
    void IRQHandler();

  private:
    RegisterMap *periph;                      /**< Points to the hardware registers for this instance */
    size_t resourceIndex;                     /**< Derived lookup table index for resource access */
    uint32_t dmaTXSignal;                     /**< DMA request signal ID for TX operations */
    uint32_t dmaRXSignal;                     /**< DMA request signal ID for RX operations */
    IRQn_Type periphIRQn;                     /**< Instance interrupt request signal number */
    volatile Runtime::Flag_t runtimeFlags;    /**< Error/process flags set at runtime to indicate state */
    Chimera::Peripheral::Type peripheralType; /**< What kind of peripheral this is */

    /*------------------------------------------------
    Asynchronous Event Listeners
    ------------------------------------------------*/
    Chimera::Threading::BinarySemaphore * ISRWakeup_external;

    /*------------------------------------------------
    Transfer Control Blocks
    ------------------------------------------------*/
    Thor::LLD::Serial::CDTCB txTCB;
    Thor::LLD::Serial::MDTCB rxTCB;

    /**
     *  Calculates the appropriate configuration value for the Baud Rate Register
     *  given a desired baud rate.
     *
     *  @param[in]  desiredBaud   The baud rate to be configured
     */
    uint32_t calculateBRR( const size_t desiredBaud );

    /**
     *  Disables the USART interrupts
     */
    inline void enterCriticalSection();

    /**
     *  Enables the USART interrupts
     */
    inline void exitCriticalSection();
  };

  using Driver_sPtr = std::shared_ptr<Driver>;
  using Driver_uPtr = std::unique_ptr<Driver>;
}    // namespace Thor::LLD::USART

#endif /* !THOR_HW_USART_DRIVER_STM32L4_HPP */