/********************************************************************************
 *  File Name:
 *    hw_timer_driver_STM32L4.cpp
 *
 *  Description:
 *    Implements the LLD interface to the STM32L4 series TIMER hardware.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/thread>
#include <Chimera/timer>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/common/cortex-m4/system_time.hpp>
#include <Thor/lld/interface/rcc/rcc_intf.hpp>
#include <Thor/lld/interface/timer/timer_intf.hpp>
#include <Thor/lld/stm32l4x/timer/hw_timer_driver.hpp>
#include <Thor/lld/stm32l4x/timer/hw_timer_mapping.hpp>
#include <Thor/lld/stm32l4x/timer/hw_timer_prj.hpp>
#include <Thor/lld/stm32l4x/timer/hw_timer_types.hpp>

#if defined( TARGET_STM32L4 )

namespace Thor::LLD::TIMER
{
  /*-------------------------------------------------------------------------------
  Chimera Required Free Functions
  -------------------------------------------------------------------------------*/
  size_t millis()
  {
    return CortexM4::SYSTick::getMilliseconds();
  }


  size_t micros()
  {
    return CortexM4::SYSTick::getMicroseconds();
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
    while( ( millis() - startTick ) < ms )
    {
      asm volatile("nop");
    }
  }


  void blockDelayMicros( const size_t us )
  {
    size_t startTick = micros();
    while( ( micros() - startTick ) < us )
    {
      asm volatile("nop");
    }

#if defined( DEBUG )
    volatile size_t actualDiff = micros() - startTick;
    volatile int error         = static_cast<int>( us ) - static_cast<int>( actualDiff );
#endif
  }

  /*-------------------------------------------------------------------------------
  LLD Public Free Functions
  -------------------------------------------------------------------------------*/
#if defined( THOR_LLD_TIMER )

  Chimera::Status_t initializeModule()
  {

    initializeMapping();

    return Chimera::Status::OK;
  }

  IAdvancedDriver_rPtr getAdvancedDriver( const Thor::HLD::RIndex channel )
  {
    return nullptr;
  }

  IBasicDriver_rPtr getBasicDriver( const Thor::HLD::RIndex channel )
  {
    return nullptr;
  }

  ILowPowerDriver_rPtr getLowPowerDriver( const Thor::HLD::RIndex channel )
  {
    return nullptr;
  }


  RIndex_t getResourceIndex( const std::uintptr_t address )
  {
    return InstanceToResourceIndex.at( address ).second;
  }

  /*-------------------------------------------------------------------------------
  LLD Private Free Functions
  -------------------------------------------------------------------------------*/
  bool isTIMER( const std::uintptr_t address )
  {
    bool result = false;

    for ( auto &val : periphAddressList )
    {
      if ( val == address )
      {
        result = true;
      }
    }

    return result;
  }

  /*-------------------------------------------------------------------------------
  Advanced Low Level Driver Implementation
  -------------------------------------------------------------------------------*/
  AdvancedDriver::AdvancedDriver() : periph( nullptr )
  {
  }

  AdvancedDriver::~AdvancedDriver()
  {
  }

  Chimera::Status_t AdvancedDriver::attach( RegisterMap *const peripheral )
  {
    return Chimera::Status::OK;
  }


  /*-------------------------------------------------------------------------------
  Basic Low Level Driver Implementation
  -------------------------------------------------------------------------------*/
  BasicDriver::BasicDriver() : periph( nullptr )
  {
  }

  BasicDriver::~BasicDriver()
  {
  }

  Chimera::Status_t BasicDriver::attach( RegisterMap *const peripheral )
  {
    return Chimera::Status::OK;
  }

  /*-------------------------------------------------------------------------------
  Low Power Low Level Driver Implementation
  -------------------------------------------------------------------------------*/
  LowPowerDriver::LowPowerDriver() : periph( nullptr )
  {
  }

  LowPowerDriver::~LowPowerDriver()
  {
  }

  Chimera::Status_t LowPowerDriver::attach( LPRegisterMap *const peripheral )
  {
    return Chimera::Status::OK;
  }
#endif

}    // namespace Thor::LLD::TIMER

#endif /* TARGET_STM32L4 && THOR_DRIVER_TIMER */
