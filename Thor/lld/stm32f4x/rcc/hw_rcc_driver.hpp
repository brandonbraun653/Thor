/********************************************************************************
 *  File Name:
 *    hw_rcc_driver.hpp
 *
 *  Description:
 *    STM32F4 RCC driver interface
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_DRIVER_RCC_HPP
#define THOR_HW_DRIVER_RCC_HPP

/* C++ Includes */
#include <cstdlib>
#include <memory>

/* Chimera Includes */
#include <Chimera/common>

/* Driver Includes */
#include <Thor/lld/stm32f4x/rcc/hw_rcc_types.hpp>
#include <Thor/lld/interface/rcc/rcc_intf.hpp>

namespace Thor::LLD::RCC
{
  class SystemClock
  {
  public:
    ~SystemClock();
    void enableClock( const Chimera::Clock::Bus clock );
    void disableClock( const Chimera::Clock::Bus clock );
    Chimera::Status_t configureProjectClocks();
    Chimera::Status_t setCoreClockSource( const Chimera::Clock::Bus src );
    Chimera::Clock::Bus getCoreClockSource();
    Chimera::Status_t setClockFrequency( const Chimera::Clock::Bus clock, const size_t freq, const bool enable );
    size_t getClockFrequency( const Chimera::Clock::Bus clock );
    size_t getPeriphClock( const Chimera::Peripheral::Type periph, const std::uintptr_t address );

  private:
    friend SystemClock *getCoreClock();
    SystemClock();
  };


  class PeripheralController
  {
  public:
    ~PeripheralController();

    Chimera::Status_t reset( const Chimera::Peripheral::Type type, const size_t index );
    Chimera::Status_t enableClock( const Chimera::Peripheral::Type type, const size_t index );
    Chimera::Status_t disableClock( const Chimera::Peripheral::Type type, const size_t index );
    Chimera::Status_t enableClockLowPower( const Chimera::Peripheral::Type type, const size_t index );
    Chimera::Status_t disableClockLowPower( const Chimera::Peripheral::Type type, const size_t index );

  private:
    friend PeripheralController *getPeripheralClock();
    PeripheralController();
  };

}    // namespace Thor::LLD::RCC

#endif /* !THOR_HW_DRIVER_RCC_HPP */
