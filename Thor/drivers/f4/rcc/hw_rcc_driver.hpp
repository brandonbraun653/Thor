/********************************************************************************
 *   File Name:
 *    hw_rcc_driver.hpp
 *
 *   Description:
 *    STM32F4 RCC driver interface
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_DRIVER_RCC_HPP
#define THOR_HW_DRIVER_RCC_HPP

/* C++ Includes */
#include <cstdlib>

/* Chimera Includes */
#include <Chimera/types/common_types.hpp>
#include <Chimera/types/peripheral_types.hpp>

/* Driver Includes */
#include <Thor/drivers/model/rcc_model.hpp>
#include <Thor/types/clock_types.hpp>

namespace Thor::Driver::RCC
{
  /**
   *  Initializes all system resources to their default state.
   *  
   *  @return void 
   */
  void init();

  class SystemClock : public ClockTree
  {
  public:
    /**
     *  Gets the singleton instance to the system clock driver
     *
     *  @return SystemClock *
     */
    SystemClock *const get();

    Chimera::Status_t setPeriphClock( const Chimera::Peripheral::Type periph, const size_t freqHz ) final override;

    Chimera::Status_t enableClock( const Chimera::Peripheral::Type periph, const size_t instance ) final override;

    Chimera::Status_t disableClock( const Chimera::Peripheral::Type periph, const size_t instance ) final override;

    Chimera::Status_t setCoreClock( const size_t freqHz ) final override;

    Chimera::Status_t setCoreClockSource( const Thor::Clock::Source src ) final override;

    size_t getCoreClock() final override;

    Thor::Clock::Source getCoreClockSource() final override;

    size_t getPeriphClock( const Chimera::Peripheral::Type periph ) final override;

  private:
    SystemClock();
  };

  class GPIOPeriph : public Peripheral
  {
  public:
    ~GPIOPeriph();

    static Peripheral *const get();

    Chimera::Status_t init() final override;

    Chimera::Status_t reset( const size_t instance ) final override;

    Chimera::Peripheral::Type getType() final override;

    Chimera::Status_t enableClock( const size_t instance ) final override;

    Chimera::Status_t disableClock( const size_t instance ) final override;

    Chimera::Status_t enableClockLowPower( const size_t instance ) final override;

    Chimera::Status_t disableClockLowPower( const size_t instance ) final override;

  private:
    GPIOPeriph();

    uint8_t iterator;
    bool initialized;
    static const Chimera::Peripheral::Type sPeriphType = Chimera::Peripheral::Type::GPIO;
    ;
  };
}

#endif /* !THOR_HW_DRIVER_RCC_HPP */