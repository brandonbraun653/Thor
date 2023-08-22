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
#include <Aurora/utility>
#include <Chimera/common>
#include <Chimera/function>
#include <Chimera/peripheral>
#include <Chimera/timer>
#include <Thor/lld/common/interrupts/timer_interrupt_vectors.hpp>
#include <Thor/lld/interface/inc/interrupt>
#include <Thor/lld/interface/inc/timer>

/*-----------------------------------------------------------------------------
Constants
-----------------------------------------------------------------------------*/
static constexpr size_t TIMER_ISR_COUNT =
    EnumValue( Chimera::Timer::Instance::NUM_OPTIONS ) + EnumValue( Thor::LLD::TIMER::ISRExtended::NUM_OPTIONS );

/*-----------------------------------------------------------------------------
Static Data
-----------------------------------------------------------------------------*/
static std::array<Chimera::Function::Opaque, TIMER_ISR_COUNT> s_isr_callbacks;

/*-----------------------------------------------------------------------------
Static Functions
-----------------------------------------------------------------------------*/
static inline constexpr size_t getISRIndex( const Chimera::Timer::Instance inst, const Thor::LLD::TIMER::ISRExtended ext )
{
  size_t index = EnumValue( inst );
  if ( ext != Thor::LLD::TIMER::ISRExtended::NONE )
  {
    index += EnumValue( ext );
  }

  return index;
}

/*-----------------------------------------------------------------------------
Public Functions
-----------------------------------------------------------------------------*/
namespace Thor::LLD::TIMER
{
  IRQn_Type getHWISRIndex( const Chimera::Timer::Instance inst, const ISRExtended ext )
  {
    IRQn_Type isrIndex = NonMaskableInt_IRQn;
    switch ( inst )
    {
#if defined( STM32_TIMER1_PERIPH_AVAILABLE )
      case Chimera::Timer::Instance::TIMER1:
        switch ( ext )
        {
          case ISRExtended::NONE:
            isrIndex = TIM1_CC_IRQn;
            break;

          case ISRExtended::TIM1_TRG_COM_TIM11:
            isrIndex = TIM1_TRG_COM_TIM11_IRQn;
            break;

          case ISRExtended::TIM1_UP_TIM10:
            isrIndex = TIM1_UP_TIM10_IRQn;
            break;

          case ISRExtended::TIM1_BRK_TIM9:
            isrIndex = TIM1_BRK_TIM9_IRQn;
            break;

          default:
            // Do nothing
            break;
        };
        break;
#endif    // STM32_TIMER1_PERIPH_AVAILABLE

#if defined( STM32_TIMER2_PERIPH_AVAILABLE )
      case Chimera::Timer::Instance::TIMER2:
        isrIndex = TIM2_IRQn;
        break;
#endif    // STM32_TIMER2_PERIPH_AVAILABLE

#if defined( STM32_TIMER3_PERIPH_AVAILABLE )
      case Chimera::Timer::Instance::TIMER3:
        isrIndex = TIM3_IRQn;
        break;
#endif    // STM32_TIMER3_PERIPH_AVAILABLE

#if defined( STM32_TIMER6_PERIPH_AVAILABLE )
      case Chimera::Timer::Instance::TIMER6:
        isrIndex = TIM6_DAC_IRQn;
        break;
#endif    // STM32_TIMER6_PERIPH_AVAILABLE

#if defined( STM32_TIMER7_PERIPH_AVAILABLE )
      case Chimera::Timer::Instance::TIMER7:
        isrIndex = TIM7_IRQn;
        break;
#endif    // STM32_TIMER7_PERIPH_AVAILABLE

#if defined( STM32_TIMER8_PERIPH_AVAILABLE )
      case Chimera::Timer::Instance::TIMER8:
        isrIndex = TIM8_CC_IRQn;
        break;
#endif    // STM32_TIMER8_PERIPH_AVAILABLE

      default:
        // Do nothing
        break;
    };

    RT_DBG_ASSERT( isrIndex != NonMaskableInt_IRQn );
    return isrIndex;
  }


  Chimera::Status_t attachISR( const Chimera::Timer::Instance inst, Chimera::Function::Opaque func, const ISRExtended ext )
  {
    const size_t index = getISRIndex( inst, ext );
    RT_DBG_ASSERT( index < s_isr_callbacks.size() );

    s_isr_callbacks[ index ] = func;
    return Chimera::Status::OK;
  }


  void detachISR( const Chimera::Timer::Instance inst, const ISRExtended ext )
  {
    const size_t index = getISRIndex( inst, ext );
    RT_DBG_ASSERT( index < s_isr_callbacks.size() );

    s_isr_callbacks[ index ] = {};
  }


  void enableISR( Handle_rPtr timer, const ISRSource source )
  {
    DIER_ALL::setbit( timer->registers, EnumValue( source ) );
  }


  void disableISR( Handle_rPtr timer, const ISRSource source )
  {
    DIER_ALL::clear( timer->registers, EnumValue( source ) );
  }
}    // namespace Thor::LLD::TIMER

/*-----------------------------------------------------------------------------
ISR Handlers
-----------------------------------------------------------------------------*/
void TIM1_BRK_TIM9_IRQHandler()
{
  static constexpr size_t idx = getISRIndex( Chimera::Timer::Instance::TIMER1, Thor::LLD::TIMER::ISRExtended::TIM1_BRK_TIM9 );
  if ( s_isr_callbacks[ idx ] )
  {
    s_isr_callbacks[ idx ]();
  }
}


void TIM1_UP_TIM10_IRQHandler()
{
  static constexpr size_t idx = getISRIndex( Chimera::Timer::Instance::TIMER1, Thor::LLD::TIMER::ISRExtended::TIM1_UP_TIM10 );
  if ( s_isr_callbacks[ idx ] )
  {
    s_isr_callbacks[ idx ]();
  }
}


void TIM1_TRG_COM_TIM11_IRQHandler()
{
  static constexpr size_t idx =
      getISRIndex( Chimera::Timer::Instance::TIMER1, Thor::LLD::TIMER::ISRExtended::TIM1_TRG_COM_TIM11 );
  if ( s_isr_callbacks[ idx ] )
  {
    s_isr_callbacks[ idx ]();
  }
}


void TIM1_CC_IRQHandler()
{
  static constexpr size_t idx = getISRIndex( Chimera::Timer::Instance::TIMER1, Thor::LLD::TIMER::ISRExtended::NONE );
  if ( s_isr_callbacks[ idx ] )
  {
    s_isr_callbacks[ idx ]();
  }
}


void TIM2_IRQHandler()
{
  static constexpr size_t idx = getISRIndex( Chimera::Timer::Instance::TIMER2, Thor::LLD::TIMER::ISRExtended::NONE );
  if ( s_isr_callbacks[ idx ] )
  {
    s_isr_callbacks[ idx ]();
  }
}


void TIM3_IRQHandler()
{
  static constexpr size_t idx = getISRIndex( Chimera::Timer::Instance::TIMER3, Thor::LLD::TIMER::ISRExtended::NONE );
  if ( s_isr_callbacks[ idx ] )
  {
    s_isr_callbacks[ idx ]();
  }
}


void TIM6_IRQHandler()
{
  static constexpr size_t idx = getISRIndex( Chimera::Timer::Instance::TIMER6, Thor::LLD::TIMER::ISRExtended::NONE );
  if ( s_isr_callbacks[ idx ] )
  {
    s_isr_callbacks[ idx ]();
  }
}


void TIM7_IRQHandler()
{
  static constexpr size_t idx = getISRIndex( Chimera::Timer::Instance::TIMER7, Thor::LLD::TIMER::ISRExtended::NONE );
  if ( s_isr_callbacks[ idx ] )
  {
    s_isr_callbacks[ idx ]();
  }
}
