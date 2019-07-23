/********************************************************************************
 *   File Name:
 *    hw_usart_driver.hpp
 *
 *   Description:
 *    STM32 Driver for the USART Peripheral
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_USART_DRIVER_HPP
#define THOR_HW_USART_DRIVER_HPP

/* C++ Includes */
#include <memory>
#include <vector>

/* Chimera Includes */
#include <Chimera/threading.hpp>
#include <Chimera/types/common_types.hpp>
#include <Chimera/types/peripheral_types.hpp>

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/common/interrupts/usart_interrupt_vectors.hpp>
#include <Thor/drivers/common/types/serial_types.hpp>
#include <Thor/drivers/f4/common/types.hpp>
#include <Thor/drivers/f4/interrupt/hw_it_prj.hpp>
#include <Thor/drivers/f4/usart/hw_usart_types.hpp>
#include <Thor/drivers/model/interrupt_model.hpp>
#include <Thor/drivers/model/serial_model.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_USART == 1 )

namespace Thor::Driver::USART
{
  class Driver : public Thor::Driver::Serial::Basic,
                 public Thor::Driver::Serial::Extended
  {
  public:
    Driver( RegisterMap *const peripheral );
    ~Driver();

    Chimera::Status_t init( const Thor::Driver::Serial::Config &cfg ) final override;

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

    Chimera::Status_t transmitDMA( uint8_t *const data, const size_t size, const size_t timeout ) final override;

    Chimera::Status_t receiveDMA( uint8_t *const data, const size_t size, const size_t timeout ) final override;

    Chimera::Status_t txTransferStatus() final override;

    Chimera::Status_t rxTransferStatus() final override;

    uint32_t getFlags() final override;

    void clearFlags( const uint32_t flagBits ) final override;

    void killTransmit() final override;

    void killReceive() final override;

    void attachISRWakeup( SemaphoreHandle_t wakeup );

  protected:
    friend void(::USART1_IRQHandler )();
    friend void(::USART2_IRQHandler )();
    friend void(::USART3_IRQHandler )();
    friend void(::USART6_IRQHandler )();

    /**
     *  Generic interrupt handler for all USART specific ISR signals
     *
     *  @return void
     */
    void IRQHandler();

  private:
    RegisterMap *const periph;                /**< Points to the hardware registers for this instance */
    size_t resourceIndex;                     /**< Derived lookup table index for resource access */
    IRQn_Type periphIRQn;                     /**< Instance interrupt request signal number */
    volatile Runtime::Flag_t runtimeFlags;    /**< Error/process flags set at runtime to indicate state */
    Chimera::Peripheral::Type peripheralType; /**< What kind of peripheral this is */

    /*------------------------------------------------
    Asynchronous Event Listeners
    ------------------------------------------------*/
    SemaphoreHandle_t isrWakeup;

    /*------------------------------------------------
    Transfer Control Blocks
    ------------------------------------------------*/
    CDTCB txTCB;
    MDTCB rxTCB;

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

}    // namespace Thor::Driver::USART

#endif /* TARGET_STM32F4 && THOR_DRIVER_USART */
#endif /* !THOR_HW_USART_DRIVER_HPP */
