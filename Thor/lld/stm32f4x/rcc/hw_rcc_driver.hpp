/********************************************************************************
 *  File Name:
 *    hw_rcc_driver.hpp
 *
 *  Description:
 *    STM32F4 RCC driver interface
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
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
#include <Thor/hld/clock/clock_types.hpp>

namespace Thor::LLD::RCC
{
  /**
   *  Project specific declaration of the default HSI oscillator frequency in Hz
   *
   *  @note Default implementation declared weak so projects can override
   *
   *  @param[out] projectValue    Will contain the project's value upon function exit
   *  @return Chimera::Status_t
   */
  Chimera::Status_t prjGetHSIValue( size_t *const projectValue );

  /**
   *  Project specific declaration of the default HSE oscillator frequency in Hz
   *
   *  @note Default implementation declared weak so projects can override
   *
   *  @param[out] projectValue    Will contain the project's value upon function exit
   *  @return Chimera::Status_t
   */
  Chimera::Status_t prjGetHSEValue( size_t *const projectValue );

  /**
   *  Project specific declaration of the default LSI oscillator frequency in Hz
   *
   *  @note Default implementation declared weak so projects can override
   *
   *  @param[out] projectValue    Will contain the project's value upon function exit
   *  @return Chimera::Status_t
   */
  Chimera::Status_t prjGetLSIValue( size_t *const projectValue );

  /**
   *  Project specific declaration of the algorithm used to determine what
   *  is the current system clock frequency.
   *
   *  @note Default implementation declared weak so projects can override
   *
   *  @param[out] projectValue    Will contain the project's value upon function exit
   *  @return Chimera::Status_t
   */
  Chimera::Status_t prjGetSysClockFreq( size_t *const projectValue );

  Chimera::Status_t prjGetHCLKFreq( size_t *const projectValue );

  Chimera::Status_t prjGetPCLK1Freq( size_t *const projectValue );

  Chimera::Status_t prjGetPCLK2Freq( size_t *const projectValue );

  /**
   *  Project specific declaration of the oscillator configuration settings.
   *
   *  @note Default implementation declared weak so projects can override
   *
   *  @param[out] projectValue    Will contain the project's value upon function exit
   *  @return Chimera::Status_t
   */
  Chimera::Status_t prjGetOscillatorConfig( OscillatorInit *const projectValue );

  /**
   *  Project specific declaration of the clock configuration settings.
   *
   *  @note Default implementation declared weak so projects can override
   *
   *  @param[out] projectValue    Will contain the project's value upon function exit
   *  @return Chimera::Status_t
   */
  Chimera::Status_t prjGetClockConfig( ClockInit *const projectValue );

  /**
   *  Singleton that allows the user to configure a chip's clock at a very high level.
   */
  class SystemClock : virtual public ICoreClock
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
    friend ICoreClock *getCoreClock();
    SystemClock();
  };


  class PeripheralController : virtual public IPeripheralClock
  {
  public:
    ~PeripheralController();

    Chimera::Status_t reset( const Chimera::Peripheral::Type type, const size_t index ) final override;
    Chimera::Status_t enableClock( const Chimera::Peripheral::Type type, const size_t index ) final override;
    Chimera::Status_t disableClock( const Chimera::Peripheral::Type type, const size_t index ) final override;
    Chimera::Status_t enableClockLowPower( const Chimera::Peripheral::Type type, const size_t index ) final override;
    Chimera::Status_t disableClockLowPower( const Chimera::Peripheral::Type type, const size_t index ) final override;

  private:
    friend IPeripheralClock *getPeripheralClock();
    PeripheralController();
  };

  using PeripheralController_sPtr = std::shared_ptr<PeripheralController>;

}    // namespace Thor::LLD::RCC

#endif /* !THOR_HW_DRIVER_RCC_HPP */