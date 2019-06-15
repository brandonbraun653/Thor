/********************************************************************************
 *   File Name:
 *    hw_usart_driver_stm32f4.cpp
 *
 *   Description:
 *    STM32F4 specific driver implementation for the UART/USART driver. Both drivers
 *    are merged into one as the datasheet does not make a distinction between the
 *    two. In practice with the STM32HAL this was also found to be true.
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */

/* Driver Includes */
#include <Thor/drivers/f4/usart/hw_usart_driver.hpp>
#include <Thor/drivers/f4/usart/hw_usart_prj.hpp>
#include <Thor/drivers/f4/usart/hw_usart_types.hpp>

namespace Thor::Driver::USART
{
  Driver::Driver( RegisterMap *const peripheral ) : periph( peripheral )
  {
  }

  Driver::~Driver()
  {
  }

  Chimera::Status_t Driver::init( const Thor::Driver::Serial::Config &cfg )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::deinit()
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::transmit( const uint8_t *const data, const size_t size, const size_t timeout )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::receive( uint8_t *const data, const size_t size, const size_t timeout )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::enableIT( const Chimera::Hardware::SubPeripheral periph )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::disableIT( const Chimera::Hardware::SubPeripheral periph )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::transmitIT( uint8_t *const data, const size_t size, const size_t timeout )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::receiveIT( uint8_t *const data, const size_t size, const size_t timeout )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::initDMA()
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::deinitDMA()
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::enableDMA_IT( const Chimera::Hardware::SubPeripheral periph )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::disableDMA_IT( const Chimera::Hardware::SubPeripheral periph )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::transmitDMA( uint8_t *const data, const size_t size, const size_t timeout )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::receiveDMA( uint8_t *const data, const size_t size, const size_t timeout )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::registerCallback( const CallbackEvent type, const VoidCallback &func )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Hardware::Status Driver::pollTransferStatus()
  {
    return Chimera::Hardware::Status::PERIPHERAL_FREE;
  }

  Chimera::Status_t Driver::enableSignal( const InterruptSignal_t sig )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::disableSignal( const InterruptSignal_t sig )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

}    // namespace Thor::Driver::USART
