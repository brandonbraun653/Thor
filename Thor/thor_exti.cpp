#include <Thor/exti.hpp>
#include <Thor/definitions.hpp>

using namespace Thor::Serial;
using namespace Thor::SPI;
using namespace Thor::Interrupt;
using namespace Thor::Interrupt;


#if defined( USING_FREERTOS )
TaskTrigger *trigger_obj          = nullptr;
SemaphoreHandle_t *trigger_semphr = nullptr;

boost::circular_buffer<TaskTrigger *> TriggerBuffer;

void setupEXTI0_Interrupt()
{
  TriggerBuffer.set_capacity( 10 );

  LL_EXTI_InitTypeDef exti_init;

  exti_init.LineCommand = ENABLE;
  exti_init.Line_0_31   = LL_EXTI_LINE_0;
  exti_init.Mode        = LL_EXTI_MODE_IT;
  exti_init.Trigger     = LL_EXTI_TRIGGER_RISING;

  LL_EXTI_Init( &exti_init );

  /** Ensures that the EXTI0 interrupt will always be able to preempt most other
   *	threads running on the kernel. This is important for quick response times.
   **/
  NVIC_SetPriority( EXTI0_IRQn, EXTI0_MAX_IRQn_PRIORITY );
  NVIC_EnableIRQ( EXTI0_IRQn );
}

void EXTI0_IRQHandler()
{
  /* Clear the flag to prevent infinite looping */
  LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_0 );

  /* Address all queued triggers */
  while ( !TriggerBuffer.empty() )
  {
    trigger_obj    = TriggerBuffer.front();
    trigger_semphr = trigger_obj->getEventSemaphore();

    /* Give a semaphore to the waiting task and possibly request a context switch */
    if ( trigger_semphr )
    {
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      xSemaphoreGiveFromISR( *trigger_semphr, &xHigherPriorityTaskWoken );

      portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }

    TriggerBuffer.pop_front();
  }
}
#else
void EXTI0_IRQHandler()
{
}
#endif

void EXTI1_IRQHandler()
{
}

void EXTI2_IRQHandler()
{
}

void EXTI3_IRQHandler()
{
}

void EXTI4_IRQHandler()
{
}

void EXTI9_5_IRQHandler()
{
}

void EXTI15_10_IRQHandler()
{
}
