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
#include <vector>

/* Boost Includes */
#include <boost/function.hpp>

/* Chimera Includes */
#include <Chimera/threading.hpp>
#include <Chimera/types/common_types.hpp>
#include <Chimera/types/peripheral_types.hpp>

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/common/interrupts/usart_interrupt_vectors.hpp>
#include <Thor/drivers/common/types/peripheral_event_types.hpp>
#include <Thor/drivers/common/types/serial_types.hpp>
#include <Thor/drivers/f4/common/types.hpp>
#include <Thor/drivers/f4/usart/hw_usart_types.hpp>
#include <Thor/drivers/model/event_model.hpp>
#include <Thor/drivers/model/interrupt_model.hpp>
#include <Thor/drivers/model/serial_model.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_USART == 1 )

namespace Thor::Driver::USART
{

  class Driver : public Thor::Driver::Serial::Basic,
                 public Thor::Driver::Serial::Extended,
                 public Thor::Driver::SignalModel,
                 public Thor::Driver::EventListener
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

    Chimera::Status_t transmitIT( uint8_t *const data, const size_t size, const size_t timeout ) final override;

    Chimera::Status_t receiveIT( uint8_t *const data, const size_t size, const size_t timeout ) final override;

    Chimera::Status_t initDMA() final override;

    Chimera::Status_t deinitDMA() final override;

    Chimera::Status_t enableDMA_IT( const Chimera::Hardware::SubPeripheral periph ) final override;

    Chimera::Status_t disableDMA_IT( const Chimera::Hardware::SubPeripheral periph ) final override;

    Chimera::Status_t transmitDMA( uint8_t *const data, const size_t size, const size_t timeout ) final override;

    Chimera::Status_t receiveDMA( uint8_t *const data, const size_t size, const size_t timeout ) final override;

    Chimera::Status_t enableSignal( const InterruptSignal_t sig ) final override;

    Chimera::Status_t disableSignal( const InterruptSignal_t sig ) final override;

    Chimera::Status_t registerEventListener( const Chimera::Event::Trigger event,
                                             SemaphoreHandle_t *const listener ) final override;

    Chimera::Status_t removeEventListener( const Chimera::Event::Trigger event,
                                           SemaphoreHandle_t *const listener ) final override;

    Chimera::Hardware::Status pollTransferStatus() final override;

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
    RegisterMap *const periph;
    size_t resourceIndex;
    Chimera::Peripheral::Type peripheralType;

    EventResponders rxCompleteActors;
    EventResponders txCompleteActors;

    /**
     *  Blocking wait on the particular flag in the status register to become set
     *
     *  @param[in]  flag      The flag to wait upon
     *  @param[in]  timeout   How long to wait for the flag to become set
     *  @return bool
     */
    bool waitUntilSet( const uint32_t flag, const size_t timeout );

    /**
     *  Calculates the appropriate configuration value for the Baud Rate Register
     *  given a desired baud rate.
     *
     *  @param[in]  desiredBaud   The baud rate to be configured
     */
    uint32_t calculateBRR( const size_t desiredBaud );

    void enterCriticalSection();

    void exitCriticalSection();
  };
}    // namespace Thor::Driver::USART

#endif /* TARGET_STM32F4 && THOR_DRIVER_USART */
#endif /* !THOR_HW_USART_DRIVER_HPP */
