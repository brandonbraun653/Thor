/******************************************************************************
 *  File Name:
 *    timer_isr.cpp
 *
 *  Description:
 *    Interrupt handlers for TIMER peripherals.
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/function>
#include <Chimera/peripheral>
#include <Thor/lld/common/interrupts/timer_interrupt_vectors.hpp>
#include <Thor/lld/interface/inc/interrupt>
#include <Thor/lld/interface/inc/timer>

/*-----------------------------------------------------------------------------
Static Data
-----------------------------------------------------------------------------*/
static Chimera::DeviceManager<Chimera::Function::Opaque, IRQn_Type, 10> s_isr_callbacks;

namespace Thor::LLD::TIMER
{
  Chimera::Status_t attachISR( const IRQn_Type irq, Chimera::Function::Opaque func )
  {

  }


  void detachISR( const IRQn_Type irq )
  {

  }
}

/*-----------------------------------------------------------------------------
ISR Handlers
-----------------------------------------------------------------------------*/
void TIM1_BRK_TIM9_IRQHandler()
{
}


void TIM1_UP_TIM10_IRQHandler()
{
}


void TIM1_TRG_COM_TIM11_IRQHandler()
{
}


void TIM1_CC_IRQHandler()
{
}


void TIM2_IRQHandler()
{
}


void TIM3_IRQHandler()
{
}


void TIM6_IRQHandler()
{
}


void TIM7_IRQHandler()
{
}


void LPTIM1_IRQHandler()
{
}


void LPTIM2_IRQHandler()
{
}
