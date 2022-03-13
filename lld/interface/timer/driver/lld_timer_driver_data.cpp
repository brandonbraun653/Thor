/******************************************************************************
 *  File Name:
 *    lld_timer_driver_data.cpp
 *
 *  Description:
 *    Data resources needed for the Timer drivers
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Thor/lld/interface/inc/timer>


namespace Thor::LLD::TIMER
{
  /*---------------------------------------------------------------------------
  Peripheral Instances
  ---------------------------------------------------------------------------*/
#if defined( STM32_TIMER1_PERIPH_AVAILABLE )
  RegisterMap *TIMER1_PERIPH = reinterpret_cast<RegisterMap *>( TIMER1_BASE_ADDR );
#endif
#if defined( STM32_TIMER2_PERIPH_AVAILABLE )
  RegisterMap *TIMER2_PERIPH = reinterpret_cast<RegisterMap *>( TIMER2_BASE_ADDR );
#endif
#if defined( STM32_TIMER3_PERIPH_AVAILABLE )
  RegisterMap *TIMER3_PERIPH = reinterpret_cast<RegisterMap *>( TIMER3_BASE_ADDR );
#endif
#if defined( STM32_TIMER6_PERIPH_AVAILABLE )
  RegisterMap *TIMER6_PERIPH = reinterpret_cast<RegisterMap *>( TIMER6_BASE_ADDR );
#endif
#if defined( STM32_TIMER7_PERIPH_AVAILABLE )
  RegisterMap *TIMER7_PERIPH = reinterpret_cast<RegisterMap *>( TIMER7_BASE_ADDR );
#endif
#if defined( STM32_TIMER15_PERIPH_AVAILABLE )
  RegisterMap *TIMER15_PERIPH = reinterpret_cast<RegisterMap *>( TIMER15_BASE_ADDR );
#endif
#if defined( STM32_TIMER16_PERIPH_AVAILABLE )
  RegisterMap *TIMER16_PERIPH = reinterpret_cast<RegisterMap *>( TIMER16_BASE_ADDR );
#endif
}    // namespace Thor::LLD::TIMER
