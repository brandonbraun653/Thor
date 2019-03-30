#pragma once
#ifndef INTERRUPT_H_
#define INTERRUPT_H_

/* Thor Includes */
#include <Thor/macro.hpp>
#include <Thor/config.hpp>
#include <Thor/definitions.hpp>
#include <Thor/types.hpp>

#ifdef USING_FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#endif

namespace Thor
{
  namespace Interrupt
  {
  }    // namespace Interrupt
}    // namespace Thor

#ifdef __cplusplus
extern "C"
{
#endif
  void USART1_IRQHandler();
  void USART2_IRQHandler();
  void USART3_IRQHandler();
  void USART6_IRQHandler();

  void SPI1_IRQHandler();
  void SPI2_IRQHandler();
  void SPI3_IRQHandler();
  void SPI4_IRQHandler();
  void SPI5_IRQHandler();
  void SPI6_IRQHandler();

#ifdef __cplusplus
}
#endif

#endif
