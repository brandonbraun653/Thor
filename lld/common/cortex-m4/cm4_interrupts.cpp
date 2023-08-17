/******************************************************************************
 *  File Name:
 *    cm4_interrupts.cpp
 *
 *  Description:
 *    Implementation of common interface to CM4 NVIC
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/* STL Includes */
#include <cstdint>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/common/cmsis/configuration.hpp>
#include <Thor/lld/common/cmsis/core/include/core_cm4.h>

#if defined( CORTEX_M4 )

namespace Thor::LLD::INT
{
  void setPriorityGrouping( const uint32_t priorityGroup )
  {
    NVIC_SetPriorityGrouping( priorityGroup );
  }

  uint32_t getPriorityGrouping()
  {
    return NVIC_GetPriorityGrouping();
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

  bool isEnabledIRQ( const IRQn_Type IRQn )
  {
    return( NVIC_GetEnableIRQ( IRQn ) == 1UL );
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
}    // namespace Thor::LLD::INT

#endif /* CORTEX_M4 */