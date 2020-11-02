/********************************************************************************
 *  File Name:
 *    hw_exti_mapping.cpp
 *
 *  Description:
 *    Provides structures for conversion and mapping between data types for fast
 *    runtime performance of driver code.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/common/macros.hpp>
#include <Thor/lld/interface/exti/exti_types.hpp>
#include <Thor/lld/interface/interrupt/interrupt_detail.hpp>
#include <Thor/lld/stm32l4x/exti/hw_exti_types.hpp>

#if defined( TARGET_STM32L4 ) && defined( THOR_LLD_EXTI )

namespace Thor::LLD::EXTI
{
  /*-------------------------------------------------------------------------------
  Peripheral Memory Maps
  -------------------------------------------------------------------------------*/
#if defined( STM32_EXTI1_PERIPH_AVAILABLE )
  RegisterMap *EXTI1_PERIPH = reinterpret_cast<RegisterMap *>( EXTI1_BASE_ADDR );
#endif

  /*-------------------------------------------------------------------------------
  Configuration Maps
  -------------------------------------------------------------------------------*/
  namespace Config
  {
    LLD_CONST LineConfig lineConfig[ NUM_EXTI_LINES ] = {
      /* clang-format off */
      // 0
      { .supported = true, .NVIC_IRQNumber = EXTI0_IRQn,     .lineType = LineType::INTERRUPT },
      { .supported = true, .NVIC_IRQNumber = EXTI1_IRQn,     .lineType = LineType::INTERRUPT },
      { .supported = true, .NVIC_IRQNumber = EXTI2_IRQn,     .lineType = LineType::INTERRUPT },
      { .supported = true, .NVIC_IRQNumber = EXTI3_IRQn,     .lineType = LineType::INTERRUPT },
      { .supported = true, .NVIC_IRQNumber = EXTI4_IRQn,     .lineType = LineType::INTERRUPT },
      { .supported = true, .NVIC_IRQNumber = EXTI9_5_IRQn,   .lineType = LineType::INTERRUPT },
      { .supported = true, .NVIC_IRQNumber = EXTI9_5_IRQn,   .lineType = LineType::INTERRUPT },
      { .supported = true, .NVIC_IRQNumber = EXTI9_5_IRQn,   .lineType = LineType::INTERRUPT },
      { .supported = true, .NVIC_IRQNumber = EXTI9_5_IRQn,   .lineType = LineType::INTERRUPT },
      { .supported = true, .NVIC_IRQNumber = EXTI9_5_IRQn,   .lineType = LineType::INTERRUPT },
      // 10
      { .supported = true, .NVIC_IRQNumber = EXTI15_10_IRQn, .lineType = LineType::INTERRUPT },
      { .supported = true, .NVIC_IRQNumber = EXTI15_10_IRQn, .lineType = LineType::INTERRUPT },
      { .supported = true, .NVIC_IRQNumber = EXTI15_10_IRQn, .lineType = LineType::INTERRUPT },
      { .supported = true, .NVIC_IRQNumber = EXTI15_10_IRQn, .lineType = LineType::INTERRUPT },
      { .supported = true, .NVIC_IRQNumber = EXTI15_10_IRQn, .lineType = LineType::INTERRUPT },
      { .supported = true, .NVIC_IRQNumber = EXTI15_10_IRQn, .lineType = LineType::INTERRUPT },
      {},
      {},
      {},
      {},
      // 20
      {},
      {},
      {},
      {},
      {},
      {},
      {},
      {},
      {},
      {},
      // 30
      {},
      {},
      {},
      {},
      {},
      {},
      {},
      {}
      /* clang-format on */
    };
  }
}    // namespace Thor::LLD::EXTI

#endif /* TARGET_STM32L4 && THOR_LLD_EXTI */
