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

  /*---------------------------------------------------------------------------
  Map peripheral instances at a channel level
  ---------------------------------------------------------------------------*/
  extern RegisterMap *const PeriphRegisterBlock[ EnumValue( Chimera::Timer::Instance::NUM_OPTIONS ) ] = {
#if defined( STM32_TIMER1_PERIPH_AVAILABLE )
    TIMER1_PERIPH,
#else
    nullptr,
#endif
#if defined( STM32_TIMER2_PERIPH_AVAILABLE )
    TIMER2_PERIPH,
#else
    nullptr,
#endif
#if defined( STM32_TIMER3_PERIPH_AVAILABLE )
    TIMER3_PERIPH,
#else
    nullptr,
#endif
#if defined( STM32_TIMER4_PERIPH_AVAILABLE )
    TIMER4_PERIPH,
#else
    nullptr,
#endif
#if defined( STM32_TIMER5_PERIPH_AVAILABLE )
    TIMER5_PERIPH,
#else
    nullptr,
#endif
#if defined( STM32_TIMER6_PERIPH_AVAILABLE )
    TIMER6_PERIPH,
#else
    nullptr,
#endif
#if defined( STM32_TIMER7_PERIPH_AVAILABLE )
    TIMER7_PERIPH,
#else
    nullptr,
#endif
#if defined( STM32_TIMER8_PERIPH_AVAILABLE )
    TIMER8_PERIPH,
#else
    nullptr,
#endif
#if defined( STM32_TIMER9_PERIPH_AVAILABLE )
    TIMER9_PERIPH,
#else
    nullptr,
#endif
#if defined( STM32_TIMER10_PERIPH_AVAILABLE )
    TIMER10_PERIPH,
#else
    nullptr,
#endif
#if defined( STM32_TIMER11_PERIPH_AVAILABLE )
    TIMER11_PERIPH,
#else
    nullptr,
#endif
#if defined( STM32_TIMER12_PERIPH_AVAILABLE )
    TIMER12_PERIPH,
#else
    nullptr,
#endif
#if defined( STM32_TIMER13_PERIPH_AVAILABLE )
    TIMER13_PERIPH,
#else
    nullptr,
#endif
#if defined( STM32_TIMER14_PERIPH_AVAILABLE )
    TIMER14_PERIPH,
#else
    nullptr,
#endif
#if defined( STM32_TIMER15_PERIPH_AVAILABLE )
    TIMER15_PERIPH,
#else
    nullptr,
#endif
#if defined( STM32_TIMER16_PERIPH_AVAILABLE )
    TIMER16_PERIPH,
#else
    nullptr,
#endif
#if defined( STM32_LPTIMER1_PERIPH_AVAILABLE )
    LPTIMER1_PERIPH,
#else
    nullptr,
#endif
#if defined( STM32_LPTIMER2_PERIPH_AVAILABLE )
    LPTIMER2_PERIPH,
#else
    nullptr,
#endif
  };
}    // namespace Thor::LLD::TIMER
