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
#include <Thor/lld/stm32l4x/timer/hw_timer_driver.hpp>
#include <Thor/lld/stm32l4x/timer/hw_timer_mapping.hpp>
#include <Thor/lld/stm32l4x/timer/hw_timer_prj.hpp>
#include <Thor/lld/stm32l4x/timer/hw_timer_types.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_driver.hpp>

#if defined( TARGET_STM32L4 ) && defined( THOR_LLD_TIMER )

namespace Thor::LLD::TIMER
{
  /*-------------------------------------------------
  LLD->HLD Interface Implementation
  -------------------------------------------------*/
  static size_t systemTick = 0u;

  Chimera::Status_t initialize()
  {
    initializeRegisters();
    initializeMapping();

    return Chimera::CommonStatusCodes::OK;
  }

  void incrementSystemTick()
  {
    systemTick++;
  }

  size_t millis()
  {
    return systemTick;
  }

  void delayMilliseconds( const size_t ms )
  {
    #if defined( USING_FREERTOS ) || defined( USING_FREERTOS_THREADS )
    vTaskDelay( pdMS_TO_TICKS( ms ) );
    #else
    #pragma message("delayMilliseconds() has no implementation")
    #endif
  }

  void delayMicroseconds( const size_t us )
  {
    #if defined( USING_FREERTOS ) || defined( USING_FREERTOS_THREADS )
    vTaskDelay( pdMS_TO_TICKS( us * 1000 ) );
    #else
    #pragma message("delayMicroseconds() has no implementation")
    #endif 
  }

  /*-------------------------------------------------
  Private LLD Function Implementation
  -------------------------------------------------*/
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

  /*-----------------------------------------------------
  Advanced Low Level Driver Implementation
  -----------------------------------------------------*/
  AdvancedDriver::AdvancedDriver() : periph( nullptr )
  {
  }

  AdvancedDriver::~AdvancedDriver()
  {
  }

  Chimera::Status_t AdvancedDriver::attach( RegisterMap *const peripheral )
  {
    return Chimera::CommonStatusCodes::OK;
  }


  /*-----------------------------------------------------
  Basic Low Level Driver Implementation
  -----------------------------------------------------*/
  BasicDriver::BasicDriver() : periph( nullptr )
  {
  }

  BasicDriver::~BasicDriver()
  {
  }

  Chimera::Status_t BasicDriver::attach( RegisterMap *const peripheral )
  {
    return Chimera::CommonStatusCodes::OK;
  }


  /*-----------------------------------------------------
  General Low Level Driver Implementation
  -----------------------------------------------------*/
  GeneralDriver::GeneralDriver() : periph( nullptr )
  {
  }

  GeneralDriver::~GeneralDriver()
  {
  }

  Chimera::Status_t GeneralDriver::attach( RegisterMap *const peripheral )
  {
    return Chimera::CommonStatusCodes::OK;
  }


  /*-----------------------------------------------------
  LowPower Low Level Driver Implementation
  -----------------------------------------------------*/
  LowPowerDriver::LowPowerDriver() : periph( nullptr )
  {
  }

  LowPowerDriver::~LowPowerDriver()
  {
  }

  Chimera::Status_t LowPowerDriver::attach( LPRegisterMap *const peripheral )
  {
    return Chimera::CommonStatusCodes::OK;
  }

}    // namespace Thor::LLD::TIMER

#endif /* TARGET_STM32L4 && THOR_DRIVER_TIMER */
