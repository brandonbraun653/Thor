/********************************************************************************
 *  File Name:
 *    hw_timer_register_stm32l432kc.cpp
 *
 *  Description:
 *    TIMER register definitions for the STM32L432KC series chips.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* STL Incldues */
#include <cstdint>

/* Chimera Includes */
#include <Chimera/timer>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/stm32l4x/timer/hw_timer_driver.hpp>
#include <Thor/lld/stm32l4x/timer/hw_timer_mapping.hpp>
#include <Thor/lld/stm32l4x/timer/variant/hw_timer_register_stm32l4xxxx.hpp>
#include <Thor/lld/stm32l4x/timer/hw_timer_types.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_mapping.hpp>

#if defined( STM32L432xx ) && defined( THOR_LLD_TIMER )

namespace Thor::LLD::TIMER
{
  /* clang-format off */
  const std::array<Chimera::Timer::Peripheral, NUM_TIMER_PERIPHS> supportedChannels = {

    Chimera::Timer::Peripheral::TIMER1,   
    Chimera::Timer::Peripheral::TIMER2,
    Chimera::Timer::Peripheral::TIMER6,
    Chimera::Timer::Peripheral::TIMER15,  
    Chimera::Timer::Peripheral::TIMER16,
    Chimera::Timer::Peripheral::LPTIMER1, 
    Chimera::Timer::Peripheral::LPTIMER2,

#if defined( STM32_TIMER3_PERIPH_AVAILABLE )
    Chimera::Timer::Peripheral::TIMER3,
#else
    Chimera::Timer::Peripheral::NOT_SUPPORTED,
#endif

#if defined( STM32_TIMER7_PERIPH_AVAILABLE )
    Chimera::Timer::Peripheral::TIMER7,
#else
    Chimera::Timer::Peripheral::NOT_SUPPORTED,
#endif
  };

#if defined( EMBEDDED )
  /*-------------------------------------------------
  Memory Mapped Structs to Peripherals
  -------------------------------------------------*/
  RegisterMap *TIMER1_PERIPH     = reinterpret_cast<RegisterMap *>( TIMER1_BASE_ADDR );
  RegisterMap *TIMER2_PERIPH     = reinterpret_cast<RegisterMap *>( TIMER2_BASE_ADDR );
  RegisterMap *TIMER3_PERIPH     = reinterpret_cast<RegisterMap *>( TIMER3_BASE_ADDR );
  RegisterMap *TIMER6_PERIPH     = reinterpret_cast<RegisterMap *>( TIMER6_BASE_ADDR );
  RegisterMap *TIMER7_PERIPH     = reinterpret_cast<RegisterMap *>( TIMER7_BASE_ADDR );
  RegisterMap *TIMER15_PERIPH    = reinterpret_cast<RegisterMap *>( TIMER15_BASE_ADDR );
  RegisterMap *TIMER16_PERIPH    = reinterpret_cast<RegisterMap *>( TIMER16_BASE_ADDR );
  LPRegisterMap *LPTIMER1_PERIPH = reinterpret_cast<LPRegisterMap *>( LPTIMER1_BASE_ADDR );
  LPRegisterMap *LPTIMER2_PERIPH = reinterpret_cast<LPRegisterMap *>( LPTIMER2_BASE_ADDR );

  /*-------------------------------------------------
  Lookup Tables Defintions
  -------------------------------------------------*/
  ITRIMap InstanceToResourceIndex{ 
    { reinterpret_cast<std::uintptr_t>( TIMER1_PERIPH ),    TIMER1_RESOURCE_INDEX },
    { reinterpret_cast<std::uintptr_t>( TIMER2_PERIPH ),    TIMER2_RESOURCE_INDEX },
    { reinterpret_cast<std::uintptr_t>( TIMER3_PERIPH ),    TIMER3_RESOURCE_INDEX },
    { reinterpret_cast<std::uintptr_t>( TIMER6_PERIPH ),    TIMER6_RESOURCE_INDEX },
    { reinterpret_cast<std::uintptr_t>( TIMER7_PERIPH ),    TIMER7_RESOURCE_INDEX },
    { reinterpret_cast<std::uintptr_t>( TIMER15_PERIPH ),   TIMER15_RESOURCE_INDEX },
    { reinterpret_cast<std::uintptr_t>( TIMER16_PERIPH ),   TIMER16_RESOURCE_INDEX },
    { reinterpret_cast<std::uintptr_t>( LPTIMER1_PERIPH ),  LPTIMER1_RESOURCE_INDEX },
    { reinterpret_cast<std::uintptr_t>( LPTIMER2_PERIPH ),  LPTIMER2_RESOURCE_INDEX }
  };

#elif defined( _SIM )
  /*-------------------------------------------------
  Memory Mapped Structs to Virtual Peripherals
  -------------------------------------------------*/
  RegisterMap *TIMER1_PERIPH = nullptr;
  RegisterMap *TIMER2_PERIPH = nullptr;
  RegisterMap *TIMER3_PERIPH = nullptr;
  RegisterMap *TIMER6_PERIPH = nullptr;
  RegisterMap *TIMER7_PERIPH = nullptr;
  RegisterMap *TIMER15_PERIPH = nullptr;
  RegisterMap *TIMER16_PERIPH = nullptr;
  RegisterMap *LPTIMER1_PERIPH = nullptr;
  RegisterMap *LPTIMER2_PERIPH = nullptr;

  /*-------------------------------------------------
  Lookup Tables Definitions
  -------------------------------------------------*/
  Chimera::Container::LightFlatMap<std::uintptr_t, size_t> InstanceToResourceIndex;
#endif

  /* clang-format on */

  void initializeRegisters()
  {
#if defined( _SIM )
    /*------------------------------------------------
    Allocate some memory to simulate the register blocks
    ------------------------------------------------*/
    TIMER1_PERIPH   = new RegisterMap;
    TIMER2_PERIPH   = new RegisterMap;
    TIMER3_PERIPH   = new RegisterMap;
    TIMER6_PERIPH   = new RegisterMap;
    TIMER7_PERIPH   = new RegisterMap;
    TIMER15_PERIPH  = new RegisterMap;
    TIMER16_PERIPH  = new RegisterMap;
    LPTIMER1_PERIPH = new LPRegisterMap;
    LPTIMER2_PERIPH = new LPRegisterMap;

    /*------------------------------------------------
    Update the resource indexer now that the registers actually exist
    ------------------------------------------------*/
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( TIMER1_PERIPH ), TIMER1_RESOURCE_INDEX );
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( TIMER2_PERIPH ), TIMER2_RESOURCE_INDEX );
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( TIMER3_PERIPH ), TIMER3_RESOURCE_INDEX );
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( TIMER6_PERIPH ), TIMER6_RESOURCE_INDEX );
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( TIMER7_PERIPH ), TIMER7_RESOURCE_INDEX );
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( TIMER15_PERIPH ), TIMER15_RESOURCE_INDEX );
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( TIMER16_PERIPH ), TIMER16_RESOURCE_INDEX );
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( LPTIMER1_PERIPH ), LPTIMER1_RESOURCE_INDEX );
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( LPTIMER2_PERIPH ), LPTIMER2_RESOURCE_INDEX );
#endif
  }
}    // namespace Thor::LLD::TIMER

namespace Thor::LLD::RCC::LookupTables
{
  /*------------------------------------------------
  Lookup tables for register access on a peripheral by peripheral basis.
  Indexing must match the lookup table hw_timer_mapping.hpp
  ------------------------------------------------*/
  RegisterConfig TIMER_ClockConfig[ Thor::LLD::TIMER::NUM_TIMER_PERIPHS ];
  RegisterConfig TIMER_ResetConfig[ Thor::LLD::TIMER::NUM_TIMER_PERIPHS ];
  Chimera::Clock::Bus TIMER_SourceClock[ Thor::LLD::TIMER::NUM_TIMER_PERIPHS ];

  PCC TIMERLookup = { TIMER_ClockConfig,
                      nullptr,
                      TIMER_ResetConfig,
                      TIMER_SourceClock,
                      &Thor::LLD::TIMER::InstanceToResourceIndex,
                      Thor::LLD::TIMER::NUM_TIMER_PERIPHS };

  void TIMERInit()
  {
    using namespace Thor::LLD::TIMER;

    /*------------------------------------------------
    TIMER clock enable register access lookup table
    ------------------------------------------------*/
    TIMER_ClockConfig[ TIMER1_RESOURCE_INDEX ].mask = APB2ENR_TIM1EN;
    TIMER_ClockConfig[ TIMER1_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB2ENR;

    TIMER_ClockConfig[ TIMER2_RESOURCE_INDEX ].mask = APB1ENR1_TIM2EN;
    TIMER_ClockConfig[ TIMER2_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB1ENR1;

#if defined( STM32_TIMER3_PERIPH_AVAILABLE )
    TIMER_ClockConfig[ TIMER3_RESOURCE_INDEX ].mask = APB1ENR1_TIM3EN;
    TIMER_ClockConfig[ TIMER3_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB1ENR1;
#endif

    TIMER_ClockConfig[ TIMER6_RESOURCE_INDEX ].mask = APB1ENR1_TIM6EN;
    TIMER_ClockConfig[ TIMER6_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB1ENR1;

    TIMER_ClockConfig[ TIMER7_RESOURCE_INDEX ].mask = APB1ENR1_TIM7EN;
    TIMER_ClockConfig[ TIMER7_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB1ENR1;

    TIMER_ClockConfig[ TIMER15_RESOURCE_INDEX ].mask = APB2ENR_TIM15EN;
    TIMER_ClockConfig[ TIMER15_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB2ENR;

    TIMER_ClockConfig[ TIMER16_RESOURCE_INDEX ].mask = APB2ENR_TIM16EN;
    TIMER_ClockConfig[ TIMER16_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB2ENR;

    TIMER_ClockConfig[ LPTIMER1_RESOURCE_INDEX ].mask = APB1ENR1_LPTIM1EN;
    TIMER_ClockConfig[ LPTIMER1_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB1ENR1;

    TIMER_ClockConfig[ LPTIMER2_RESOURCE_INDEX ].mask = APB1ENR2_LPTIM2EN;
    TIMER_ClockConfig[ LPTIMER2_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB1ENR2;

    /*------------------------------------------------
    TIMER reset register access lookup table
    ------------------------------------------------*/
    TIMER_ResetConfig[ TIMER1_RESOURCE_INDEX ].mask = APB2RSTR_TIM1RST;
    TIMER_ResetConfig[ TIMER1_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB2RSTR;

    TIMER_ResetConfig[ TIMER2_RESOURCE_INDEX ].mask = APB1RSTR1_TIM2RST;
    TIMER_ResetConfig[ TIMER2_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB1RSTR1;

#if defined( STM32_TIMER3_PERIPH_AVAILABLE )
    TIMER_ResetConfig[ TIMER3_RESOURCE_INDEX ].mask = APB1RSTR1_TIM3RST;
    TIMER_ResetConfig[ TIMER3_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB1RSTR1;
#endif

    TIMER_ResetConfig[ TIMER6_RESOURCE_INDEX ].mask = APB1RSTR1_TIM6RST;
    TIMER_ResetConfig[ TIMER6_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB1RSTR1;

    TIMER_ResetConfig[ TIMER7_RESOURCE_INDEX ].mask = APB1RSTR1_TIM7RST;
    TIMER_ResetConfig[ TIMER7_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB1RSTR1;

    TIMER_ResetConfig[ TIMER15_RESOURCE_INDEX ].mask = APB2RSTR_TIM15RST;
    TIMER_ResetConfig[ TIMER15_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB2RSTR;

    TIMER_ResetConfig[ TIMER16_RESOURCE_INDEX ].mask = APB2RSTR_TIM16RST;
    TIMER_ResetConfig[ TIMER16_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB2RSTR;

    TIMER_ResetConfig[ LPTIMER1_RESOURCE_INDEX ].mask = APB1RSTR1_LPTIM1RST;
    TIMER_ResetConfig[ LPTIMER1_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB1RSTR1;

    TIMER_ResetConfig[ LPTIMER2_RESOURCE_INDEX ].mask = APB1RSTR2_LPTIM2RST;
    TIMER_ResetConfig[ LPTIMER2_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB1RSTR2;

    /*------------------------------------------------
    TIMER clocking bus source identifier
    ------------------------------------------------*/
    TIMER_SourceClock[ TIMER1_RESOURCE_INDEX ] = Chimera::Clock::Bus::APB2;
    TIMER_SourceClock[ TIMER2_RESOURCE_INDEX ] = Chimera::Clock::Bus::APB1;

#if defined( STM32_TIMER3_PERIPH_AVAILABLE )
    TIMER_SourceClock[ TIMER3_RESOURCE_INDEX ] = Chimera::Clock::Bus::APB1;
#endif

    TIMER_SourceClock[ TIMER6_RESOURCE_INDEX ]   = Chimera::Clock::Bus::APB1;
    TIMER_SourceClock[ TIMER7_RESOURCE_INDEX ]   = Chimera::Clock::Bus::APB1;
    TIMER_SourceClock[ TIMER15_RESOURCE_INDEX ]  = Chimera::Clock::Bus::APB2;
    TIMER_SourceClock[ TIMER16_RESOURCE_INDEX ]  = Chimera::Clock::Bus::APB2;
    TIMER_SourceClock[ LPTIMER1_RESOURCE_INDEX ] = Chimera::Clock::Bus::APB1;
    TIMER_SourceClock[ LPTIMER2_RESOURCE_INDEX ] = Chimera::Clock::Bus::APB1;
  };

}    // namespace Thor::LLD::RCC::LookupTables

#endif /* STM32L432xx && THOR_LLD_TIMER */
