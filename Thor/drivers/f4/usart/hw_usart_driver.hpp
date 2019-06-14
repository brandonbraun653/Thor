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

/* Chimera Includes */
#include <Chimera/types/common_types.hpp>

/* Driver Includes */
#include <Thor/drivers/f4/usart/hw_usart_types.hpp>

namespace Thor::Driver::USART
{
  class Driver
  {
  public:
    Driver( RegisterMap *const peripheral );
    ~Driver();

    Chimera::Status_t init( const Config &cfg );

    Chimera::Status_t deinit();

  private:
    RegisterMap *const periph;
  };
}    // namespace Thor::Driver::USART

extern void USART1_IRQHandler();
extern void USART2_IRQHandler();
extern void USART3_IRQHandler();
extern void USART6_IRQHandler();

#endif /* !THOR_HW_USART_DRIVER_HPP */
