/********************************************************************************
 *   File Name:
 *    hw_nvic_driver_stm32f4.cpp
 *
 *   Description:
 *    Implements the STM32F4 NVIC driver
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/lld/stm32f4x/nvic/hw_nvic_driver.hpp>
#include <Thor/lld/stm32f4x/nvic/hw_nvic_prj.hpp>
#include <Thor/lld/stm32f4x/nvic/hw_nvic_types.hpp>

#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_NVIC )

namespace Thor::Driver::Interrupt
{
  void setPriorityGrouping( const PriorityGrouping_t priorityGroup )
  {
    NVIC_SetPriorityGrouping( priorityGroup );
  }

  PriorityGrouping_t getPriorityGrouping()
  {
    return static_cast<PriorityGrouping_t>( NVIC_GetPriorityGrouping() );
  }

  void setPriority( const IRQn_Type IRQn, const uint32_t preemptPriority, const uint32_t subPriority )
  {
    uint32_t prioritygroup = NVIC_GetPriorityGrouping();
    NVIC_SetPriority( IRQn, NVIC_EncodePriority( prioritygroup, preemptPriority, subPriority ) );
  }

  void getPriority( const IRQn_Type IRQn, const uint32_t priorityGroup, uint32_t *const preemptPriority,
                    uint32_t *const subPriority )
  {
    NVIC_DecodePriority( NVIC_GetPriority( IRQn ), priorityGroup, preemptPriority, subPriority );
  }

  void enableIRQ( const IRQn_Type IRQn )
  {
    NVIC_EnableIRQ( IRQn );
  }

  void disableIRQ( const IRQn_Type IRQn )
  {
    NVIC_DisableIRQ( IRQn );
  }

  void setPendingIRQ( const IRQn_Type IRQn )
  {
    NVIC_SetPendingIRQ( IRQn );
  }

  void clearPendingIRQ( const IRQn_Type IRQn )
  {
    NVIC_ClearPendingIRQ( IRQn );
  }

  uint32_t getPendingIRQ( const IRQn_Type IRQn )
  {
    return NVIC_GetPendingIRQ( IRQn );
  }

  uint32_t getActive( const IRQn_Type IRQn )
  {
    return NVIC_GetActive( IRQn );
  }

  void SystemReset()
  {
    NVIC_SystemReset();
  }
}    // namespace Thor::Driver::Interrupt

#endif /* TARGET_STM32F4 && THOR_DRIVER_NVIC */