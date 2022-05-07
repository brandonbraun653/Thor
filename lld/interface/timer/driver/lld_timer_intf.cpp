/******************************************************************************
 *  File Name:
 *    lld_timer_driver_intf.cpp
 *
 *  Description:
 *    Interface implementation details for the timer module
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/peripheral>
#include <Thor/cfg>
#include <Thor/lld/common/cortex-m4/system_time.hpp>
#include <Thor/lld/interface/inc/rcc>
#include <Thor/lld/interface/inc/timer>
#include <cstddef>

namespace Thor::LLD::TIMER
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  size_t millis()
  {
#if defined( CORTEX_M4 )
    return CortexM4::SYSTick::getMilliseconds();
#else
#error Missing millis() definition
#endif
  }


  size_t micros()
  {
#if defined( CORTEX_M4 )
    return CortexM4::SYSTick::getMicroseconds();
#else
#error Missing micros() definition
#endif
  }


  void delayMilliseconds( const size_t ms )
  {
#if defined( USING_FREERTOS ) || defined( USING_FREERTOS_THREADS )
    vTaskDelay( pdMS_TO_TICKS( ms ) );
#else
#pragma message( "delayMilliseconds() has no implementation" )
#endif
  }


  void delayMicroseconds( const size_t us )
  {
#if defined( USING_FREERTOS ) || defined( USING_FREERTOS_THREADS )
    vTaskDelay( pdMS_TO_TICKS( us / 1000 ) );
#else
#pragma message( "delayMicroseconds() has no implementation" )
#endif
  }


  void blockDelayMillis( const size_t ms )
  {
    size_t startTick = millis();
    while ( ( millis() - startTick ) < ms )
    {
      asm volatile( "nop" );
    }
  }


  void blockDelayMicros( const size_t us )
  {
    size_t startTick = micros();
    while ( ( micros() - startTick ) < us )
    {
      asm volatile( "nop" );
    }

#if defined( DEBUG )
    volatile size_t actualDiff = micros() - startTick;
    volatile int    error      = static_cast<int>( us ) - static_cast<int>( actualDiff );
    ( void )error;
#endif
  }


  Chimera::Status_t initializeModule()
  {
    return Chimera::Status::OK;
  }


  RIndex_t getGlobalResourceIndex( const std::uintptr_t address )
  {
#if defined( STM32_TIMER1_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( TIMER1_PERIPH ) )
    {
      return TIMER1_GLOBAL_RESOURCE_INDEX;
    }
#endif
#if defined( STM32_TIMER2_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( TIMER2_PERIPH ) )
    {
      return TIMER2_GLOBAL_RESOURCE_INDEX;
    }
#endif
#if defined( STM32_TIMER3_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( TIMER3_PERIPH ) )
    {
      return TIMER3_GLOBAL_RESOURCE_INDEX;
    }
#endif
#if defined( STM32_TIMER6_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( TIMER6_PERIPH ) )
    {
      return TIMER6_GLOBAL_RESOURCE_INDEX;
    }
#endif
#if defined( STM32_TIMER7_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( TIMER7_PERIPH ) )
    {
      return TIMER7_GLOBAL_RESOURCE_INDEX;
    }
#endif
#if defined( STM32_TIMER15_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( TIMER15_PERIPH ) )
    {
      return TIMER15_GLOBAL_RESOURCE_INDEX;
    }
#endif
#if defined( STM32_TIMER16_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( TIMER16_PERIPH ) )
    {
      return TIMER16_GLOBAL_RESOURCE_INDEX;
    }
#endif

    return INVALID_RESOURCE_INDEX;
  }


  RIndex_t getTypeResourceIndex( const std::uintptr_t address )
  {
#if defined( STM32_TIMER1_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( TIMER1_PERIPH ) )
    {
      return TIMER1_TYPE_RESOURCE_INDEX;
    }
#endif
#if defined( STM32_TIMER2_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( TIMER2_PERIPH ) )
    {
      return TIMER2_TYPE_RESOURCE_INDEX;
    }
#endif
#if defined( STM32_TIMER3_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( TIMER3_PERIPH ) )
    {
      return TIMER3_TYPE_RESOURCE_INDEX;
    }
#endif
#if defined( STM32_TIMER6_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( TIMER6_PERIPH ) )
    {
      return TIMER6_TYPE_RESOURCE_INDEX;
    }
#endif
#if defined( STM32_TIMER7_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( TIMER7_PERIPH ) )
    {
      return TIMER7_TYPE_RESOURCE_INDEX;
    }
#endif
#if defined( STM32_TIMER15_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( TIMER15_PERIPH ) )
    {
      return TIMER15_TYPE_RESOURCE_INDEX;
    }
#endif
#if defined( STM32_TIMER16_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( TIMER16_PERIPH ) )
    {
      return TIMER16_TYPE_RESOURCE_INDEX;
    }
#endif

    return INVALID_RESOURCE_INDEX;
  }


  HardwareType getHardwareType( const std::uintptr_t address )
  {
#if defined( STM32_TIMER1_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( TIMER1_PERIPH ) )
    {
      return HardwareType::TIMER_HW_ADVANCED;
    }
#endif
#if defined( STM32_TIMER2_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( TIMER2_PERIPH ) )
    {
      return HardwareType::TIMER_HW_GENERAL;
    }
#endif
#if defined( STM32_TIMER3_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( TIMER3_PERIPH ) )
    {
      return HardwareType::TIMER_HW_GENERAL;
    }
#endif
#if defined( STM32_TIMER6_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( TIMER6_PERIPH ) )
    {
      return HardwareType::TIMER_HW_BASIC;
    }
#endif
#if defined( STM32_TIMER7_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( TIMER7_PERIPH ) )
    {
      return HardwareType::TIMER_HW_BASIC;
    }
#endif
#if defined( STM32_TIMER15_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( TIMER15_PERIPH ) )
    {
      return HardwareType::TIMER_HW_GENERAL;
    }
#endif
#if defined( STM32_TIMER16_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( TIMER16_PERIPH ) )
    {
      return HardwareType::TIMER_HW_GENERAL;
    }
#endif

    return TIMER_HW_INVALID;
  }


  HardwareType getHardwareType( const Chimera::Timer::Instance &instance )
  {
    switch ( instance )
    {
#if defined( STM32_TIMER1_PERIPH_AVAILABLE )
      case Chimera::Timer::Instance::TIMER1:
        return HardwareType::TIMER_HW_ADVANCED;
#endif
#if defined( STM32_TIMER2_PERIPH_AVAILABLE )
      case Chimera::Timer::Instance::TIMER2:
        return HardwareType::TIMER_HW_GENERAL;
#endif
#if defined( STM32_TIMER3_PERIPH_AVAILABLE )
      case Chimera::Timer::Instance::TIMER3:
        return HardwareType::TIMER_HW_GENERAL;
#endif
#if defined( STM32_TIMER6_PERIPH_AVAILABLE )
      case Chimera::Timer::Instance::TIMER6:
        return HardwareType::TIMER_HW_BASIC;
#endif
#if defined( STM32_TIMER7_PERIPH_AVAILABLE )
      case Chimera::Timer::Instance::TIMER7:
        return HardwareType::TIMER_HW_BASIC;
#endif
#if defined( STM32_TIMER15_PERIPH_AVAILABLE )
      case Chimera::Timer::Instance::TIMER15:
        return HardwareType::TIMER_HW_GENERAL;
#endif
#if defined( STM32_TIMER16_PERIPH_AVAILABLE )
      case Chimera::Timer::Instance::TIMER16:
        return HardwareType::TIMER_HW_GENERAL;
#endif

      default:
        return TIMER_HW_INVALID;
    };
  }


  UnifiedDriver getUnifiedDriver( const Chimera::Timer::Instance &instance )
  {
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if ( instance >= Chimera::Timer::Instance::NUM_OPTIONS )
    {
      return {};
    }

    /*-------------------------------------------------------------------------
    Build the unified driver object
    -------------------------------------------------------------------------*/
    UnifiedDriver output;
    output.type = getHardwareType( reinterpret_cast<std::uintptr_t>( PeriphRegisterBlock[ EnumValue( instance ) ] ) );

    switch ( output.type )
    {
      case HardwareType::TIMER_HW_ADVANCED:
        output.driver.advanced = getAdvancedDriver( instance );
        break;

      case HardwareType::TIMER_HW_BASIC:
        output.driver.basic = getBasicDriver( instance );
        break;

      case HardwareType::TIMER_HW_GENERAL:
        output.driver.general = getGeneralDriver( instance );
        break;

      default:
        // Do nothing
        break;
    };

    return output;
  }
}    // namespace Thor::LLD::TIMER
