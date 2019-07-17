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
#include <memory>

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

    Chimera::Status_t getClockFrequency( const Configuration::ClockType_t clock, size_t *const freqHz ) final override;

    Chimera::Status_t getPeriphClock( const Chimera::Peripheral::Type periph, const std::uintptr_t address, size_t *const freqHz ) final override;

  private:
    SystemClock();
  };


  class PeripheralController
  {
  public:
    ~PeripheralController();

    static std::shared_ptr<PeripheralController> get();

    /**
     *  Resets the peripheral using RCC reset registers
     *
     *  @param[in]  address        Indicates which peripheral instance should be accessed
     *  @return Chimera::Status_t
     */
    Chimera::Status_t reset( const Chimera::Peripheral::Type type, const size_t index );

    /**
     *  Enables the peripheral clock
     *
     *  @param[in]  address        Indicates which peripheral instance should be accessed
     *  @return Chimera::Status_t
     */
    Chimera::Status_t enableClock( const Chimera::Peripheral::Type type, const size_t index );

    /**
     *  Disables the peripheral clock
     *
     *  @param[in]  address        Indicates which peripheral instance should be accessed
     *  @return Chimera::Status_t
     */
    Chimera::Status_t disableClock( const Chimera::Peripheral::Type type, const size_t index );

    /**
     *  Enables the peripheral clock in low power mode
     *
     *  @param[in]  address        Indicates which peripheral instance should be accessed
     *  @return Chimera::Status_t
     */
    Chimera::Status_t enableClockLowPower( const Chimera::Peripheral::Type type, const size_t index );

    /**
     *  Disables the peripheral clock in low power mode
     *
     *  @param[in]  address        Indicates which peripheral instance should be accessed
     *  @return Chimera::Status_t
     */
    Chimera::Status_t disableClockLowPower( const Chimera::Peripheral::Type type, const size_t index );

  private:
    PeripheralController();
  };

  using PeripheralController_sPtr = std::shared_ptr<PeripheralController>;

}    // namespace Thor::Driver::RCC

#endif /* TARGET_STM32F4 && THOR_DRIVER_RCC */
#endif /* !THOR_HW_DRIVER_RCC_HPP */