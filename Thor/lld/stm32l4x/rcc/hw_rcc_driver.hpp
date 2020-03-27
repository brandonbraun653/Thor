/********************************************************************************
 *  File Name:
 *    hw_rcc_driver.hpp
 *
 *  Description:
 *    Low level RCC driver for STM32L4
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_RCC_DRIVER_HPP
#define THOR_LLD_RCC_DRIVER_HPP

/* C++ Includes */
#include <cstdlib>
#include <memory>

/* Chimera Includes */
#include <Chimera/common>

/* Driver Includes */
#include <Thor/hld/clock/clock_types.hpp>
#include <Thor/lld/interface/rcc/rcc_intf.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_types.hpp>

namespace Thor::LLD::RCC
{

  class SystemClock : virtual public IClockTree
  {
  public:
    ~SystemClock();

    Chimera::Status_t configureProjectClocks() final override;
    Chimera::Status_t setPeriphClock( const Chimera::Peripheral::Type periph, const size_t freqHz ) final override;
    Chimera::Status_t setCoreClock( const size_t freqHz ) final override;
    Chimera::Status_t setCoreClockSource( const Thor::Clock::Source src ) final override;
    Chimera::Status_t getClockFrequency( const ClockType_t clock, size_t *const freqHz ) final override;
    Chimera::Status_t getPeriphClock( const Chimera::Peripheral::Type periph, const std::uintptr_t address, size_t *const freqHz ) final override;

  private:
    friend IClockTree *getSystemClockController();
    SystemClock();
  };


  class PeripheralController : virtual public IPeripheralController
  {
  public:
    ~PeripheralController();

    Chimera::Status_t reset( const Chimera::Peripheral::Type type, const size_t index ) final override;
    Chimera::Status_t enableClock( const Chimera::Peripheral::Type type, const size_t index ) final override;
    Chimera::Status_t disableClock( const Chimera::Peripheral::Type type, const size_t index ) final override;
    Chimera::Status_t enableClockLowPower( const Chimera::Peripheral::Type type, const size_t index ) final override;
    Chimera::Status_t disableClockLowPower( const Chimera::Peripheral::Type type, const size_t index ) final override;

  private:
    friend IPeripheralController *getSystemPeripheralController();
    PeripheralController();
  };
}

#endif  /* !THOR_LLD_RCC_DRIVER_HPP */
