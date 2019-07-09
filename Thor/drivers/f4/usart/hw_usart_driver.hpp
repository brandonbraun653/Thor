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

/* Boost Includes */
#include <boost/function.hpp>

/* Chimera Includes */
#include <Chimera/types/common_types.hpp>
#include <Chimera/threading.hpp>

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/common/types/serial_types.hpp>
#include <Thor/drivers/f4/common/types.hpp>
#include <Thor/drivers/f4/usart/hw_usart_types.hpp>
#include <Thor/drivers/model/callback_model.hpp>
#include <Thor/drivers/model/interrupt_model.hpp>
#include <Thor/drivers/model/serial_model.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_USART == 1 )

namespace Thor::Driver::USART
{
  class Driver : public Thor::Driver::Serial::Basic,
                 public Thor::Driver::Serial::Extended,
                 public Thor::Driver::SignalModel,
                 public Thor::Driver::BasicCallback,
                 public Chimera::Threading::Lockable
  {
  public:
    Driver( RegisterMap *const peripheral );
    ~Driver();

    Chimera::Status_t init( const Thor::Driver::Serial::Config &cfg ) final override;

    Chimera::Status_t deinit() final override;

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

    Chimera::Status_t registerCallback( const CallbackEvent type, const VoidCallback &func ) final override;

    Chimera::Hardware::Status pollTransferStatus() final override;

    Chimera::Status_t enableSignal( const InterruptSignal_t sig ) final override;

    Chimera::Status_t disableSignal( const InterruptSignal_t sig ) final override;

  private:
    RegisterMap *const periph;
  };
}    // namespace Thor::Driver::USART

#endif /* TARGET_STM32F4 && THOR_DRIVER_USART */
#endif /* !THOR_HW_USART_DRIVER_HPP */
