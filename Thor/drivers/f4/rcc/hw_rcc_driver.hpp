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
#include <Thor/headers.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_types.hpp>
#include <Thor/drivers/model/rcc_model.hpp>
#include <Thor/types/clock_types.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_RCC == 1 )

namespace Thor::Driver::RCC
{
  /**
   *  Initializes all system resources to their default state.
   *  
   *  @return void 
   */
  void init();

  /**
   *  Project specific declaration of the default HSI oscillator frequency in Hz
   *
   *  @note Default implementation declared weak so projects can override
   *  @return size_t
   */
  size_t prjGetHSIValue();

  /**
   *  Project specific declaration of the default HSE oscillator frequency in Hz
   *
   *  @note Default implementation declared weak so projects can override
   *  @return size_t
   */
  size_t prjGetHSEValue();

  /**
   *  Project specific declaration of the default LSI oscillator frequency in Hz
   *
   *  @note Default implementation declared weak so projects can override
   *  @return size_t
   */
  size_t prjGetLSIValue();

  /**
   *  Project specific declaration of the algorithm used to determine what
   *  is the current system clock frequency.
   *
   *  @note Default implementation declared weak so projects can override
   *  @return size_t
   */
  size_t prjGetSysClockFreq();

  /**
   *  Project specific declaration of the oscillator configuration settings.
   *
   *  @note Default implementation declared weak so projects can override
   *  @return OscillatorInit
   */
  OscillatorInit prjGetOscillatorConfig();

  /**
   *  Project specific declaration of the clock configuration settings.
   *
   *  @note Default implementation declared weak so projects can override
   *  @return ClockInit
   */
  ClockInit prjGetClockConfig();

  /**
   *  Singleton that allows the user to configure a chip's clock at a very high level.
   */
  class SystemClock : public ClockTree
  {
  public:
    ~SystemClock();

    /**
     *  Gets the singleton instance to the system clock driver
     *
     *  @return SystemClock *const
     */
    static SystemClock *const get();

    Chimera::Status_t configureProjectClocks() final override;

    Chimera::Status_t setPeriphClock( const Chimera::Peripheral::Type periph, const size_t freqHz ) final override;

    Chimera::Status_t setCoreClock( const size_t freqHz ) final override;

    Chimera::Status_t setCoreClockSource( const Thor::Clock::Source src ) final override;

    size_t getCoreClock() final override;

    Thor::Clock::Source getCoreClockSource() final override;

    size_t getPeriphClock( const Chimera::Peripheral::Type periph ) final override;

  private:
    SystemClock();
  };

  /**
   *  Singleton that interacts with the GPIO reset and clock control registers
   */
  class GPIOPeriph : public Peripheral
  {
  public:
    ~GPIOPeriph();

    static Peripheral *const get();

    Chimera::Peripheral::Type getType() final override;

    size_t getPeriphIndex( const void *const peripheralAddress ) final override;

    Chimera::Status_t init() final override;

    Chimera::Status_t reset( const size_t periphIndex ) final override;

    Chimera::Status_t enableClock( const size_t periphIndex ) final override;

    Chimera::Status_t disableClock( const size_t periphIndex ) final override;

    Chimera::Status_t enableClockLowPower( const size_t periphIndex ) final override;

    Chimera::Status_t disableClockLowPower( const size_t periphIndex ) final override;

  private:
    GPIOPeriph();

    uint8_t iterator;
    static const Chimera::Peripheral::Type sPeriphType = Chimera::Peripheral::Type::GPIO;
  };

  /**
   *  Singleton that interacts with the UART reset and clock control registers
   */
  class UARTPeriph : public Peripheral
  {
  public:
    ~UARTPeriph();

    static Peripheral *const get();

    Chimera::Peripheral::Type getType() final override;

    size_t getPeriphIndex( const void *const peripheralAddress ) final override;

    Chimera::Status_t init() final override;

    Chimera::Status_t reset( const size_t periphIndex ) final override;

    Chimera::Status_t enableClock( const size_t periphIndex ) final override;

    Chimera::Status_t disableClock( const size_t periphIndex ) final override;

    Chimera::Status_t enableClockLowPower( const size_t periphIndex ) final override;

    Chimera::Status_t disableClockLowPower( const size_t periphIndex ) final override;

  private:
    UARTPeriph();

    uint8_t iterator;
    static const Chimera::Peripheral::Type sPeriphType = Chimera::Peripheral::Type::UART;
  };

  /**
   *  Singleton that interacts with the USART reset and clock control registers
   */
  class USARTPeriph : public Peripheral
  {
  public:
    ~USARTPeriph();

    static Peripheral *const get();

    Chimera::Peripheral::Type getType() final override;

    size_t getPeriphIndex( const void *const peripheralAddress ) final override;

    Chimera::Status_t init() final override;

    Chimera::Status_t reset( const size_t periphIndex ) final override;

    Chimera::Status_t enableClock( const size_t periphIndex ) final override;

    Chimera::Status_t disableClock( const size_t periphIndex ) final override;

    Chimera::Status_t enableClockLowPower( const size_t periphIndex ) final override;

    Chimera::Status_t disableClockLowPower( const size_t periphIndex ) final override;

  private:
    USARTPeriph();

    uint8_t iterator;
    static const Chimera::Peripheral::Type sPeriphType = Chimera::Peripheral::Type::USART;
  };
}    // namespace Thor::Driver::RCC

#endif /* TARGET_STM32F4 && THOR_DRIVER_RCC */
#endif /* !THOR_HW_DRIVER_RCC_HPP */