/********************************************************************************
 *  File Name:
 *    hw_exti_mapping.cpp
 *
 *  Description:
 *    Provides structures for conversion and mapping between data types for fast
 *    runtime performance of driver code.
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/common/macros.hpp>
#include <Thor/lld/interface/exti/exti_types.hpp>
#include <Thor/lld/interface/interrupt/interrupt_detail.hpp>
#include <Thor/lld/stm32f4x/exti/hw_exti_types.hpp>

#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_EXTI )

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
    extern LLD_CONST LineConfig lineConfig[ NUM_EXTI_LINES ] = {
      /* clang-format off */
      // 0
      { true,  EXTI0_IRQn,     LineType::INTERRUPT },
      { true,  EXTI1_IRQn,     LineType::INTERRUPT },
      { true,  EXTI2_IRQn,     LineType::INTERRUPT },
      { true,  EXTI3_IRQn,     LineType::INTERRUPT },
      { true,  EXTI4_IRQn,     LineType::INTERRUPT },
      { true,  EXTI9_5_IRQn,   LineType::INTERRUPT },
      { true,  EXTI9_5_IRQn,   LineType::INTERRUPT },
      { true,  EXTI9_5_IRQn,   LineType::INTERRUPT },
      { true,  EXTI9_5_IRQn,   LineType::INTERRUPT },
      { true,  EXTI9_5_IRQn,   LineType::INTERRUPT },
      // 10
      { true,  EXTI15_10_IRQn, LineType::INTERRUPT },
      { true,  EXTI15_10_IRQn, LineType::INTERRUPT },
      { true,  EXTI15_10_IRQn, LineType::INTERRUPT },
      { true,  EXTI15_10_IRQn, LineType::INTERRUPT },
      { true,  EXTI15_10_IRQn, LineType::INTERRUPT },
      { true,  EXTI15_10_IRQn, LineType::INTERRUPT },
      { false, -100, LineType::UNKNOWN },
      { false, -100, LineType::UNKNOWN },
      { false, -100, LineType::UNKNOWN },
      { false, -100, LineType::UNKNOWN },
      // 20
      { false, -100, LineType::UNKNOWN },
      { false, -100, LineType::UNKNOWN },
      { false, -100, LineType::UNKNOWN }
      /* clang-format on */
    };
  }
}    // namespace Thor::LLD::EXTI

#endif /* TARGET_STM32F4 && THOR_LLD_EXTI */
