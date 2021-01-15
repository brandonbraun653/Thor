/********************************************************************************
 *  File Name:
 *    hw_rcc_sys_ctrl.cpp
 *
 *  Description:
 *    System clock controller implementation
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/clock>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/common/cortex-m4/system_time.hpp>
#include <Thor/lld/common/mapping/peripheral_mapping.hpp>
#include <Thor/lld/interface/inc/rcc>
#include <Thor/lld/interface/inc/power>

namespace Thor::LLD::RCC
{
  /*-------------------------------------------------------------------------------
  Static Data
  -------------------------------------------------------------------------------*/
  static SystemClock s_system_clock;

  /*-------------------------------------------------------------------------------
  SystemClock Class Implementation
  -------------------------------------------------------------------------------*/
  SystemClock *getCoreClock()
  {
    return &s_system_clock;
  }


  SystemClock::SystemClock()
  {
    initialize();
  }


  SystemClock::~SystemClock()
  {
  }


  Chimera::Status_t SystemClock::configureProjectClocks()
  {
    using namespace Thor::Driver;

    Chimera::Status_t result          = Chimera::Status::FAIL;
    const Chimera::Status_t prjResult = Chimera::Status::OK;

    /*------------------------------------------------
    Turn on the main internal regulator output voltage
    ------------------------------------------------*/
    PWREN::set( RCC1_PERIPH, APB1ENR_PWREN );

    /*------------------------------------------------
    Set the voltage scaling to allow us to achieve max clock
    ------------------------------------------------*/
    PWR::VOS::set( Thor::LLD::PWR::PWR1_PERIPH, PWR::VOLTAGE_SCALE_1 );

    /*------------------------------------------------
    Configure the system clocks
    ------------------------------------------------*/
    ClockInit clkCfg;
    OscillatorInit oscCfg;

    if ( ( prjGetOscillatorConfig( &oscCfg ) == prjResult ) && ( prjGetClockConfig( &clkCfg ) == prjResult ) )
    {
      /*------------------------------------------------
      Initialize the oscillators which drive the system clocks
      ------------------------------------------------*/
      result = OscillatorConfig( &oscCfg );

      /*------------------------------------------------
      Initializes the CPU, AHB, and APB bus clocks
      ------------------------------------------------*/
      result = ClockConfig( &clkCfg );
    }

    return result;
  }


  Chimera::Status_t SystemClock::setCoreClockSource( const Chimera::Clock::Bus src )
  {
    Chimera::Status_t result = Chimera::Status::NOT_SUPPORTED;
    return result;
  }

  Chimera::Status_t SystemClock::getClockFrequency( const ClockType_t clock, size_t *const freqHz )
  {
    Chimera::Status_t result = Chimera::Status::FAIL;

    if ( freqHz )
    {
      switch ( clock )
      {
        case Configuration::ClockType::HCLK:
          result = prjGetHCLKFreq( freqHz );
          break;

        case Configuration::ClockType::PCLK1:
          result = prjGetPCLK1Freq( freqHz );
          break;

        case Configuration::ClockType::PCLK2:
          result = prjGetPCLK2Freq( freqHz );
          break;

        case Configuration::ClockType::SYSCLK:
          result = prjGetSysClockFreq( freqHz );
          break;

        default:
          // result = Chimera::Status::FAIL;
          break;
      }
    }

    return result;
  }


  size_t SystemClock::getPeriphClock( const Chimera::Peripheral::Type periph, const std::uintptr_t address )
  {
    /*-------------------------------------------------
    Input protection
    -------------------------------------------------*/
    auto registry = getPCCRegistry( periph );
    if ( !registry || !registry->clockSource || !registry->getResourceIndex )
    {
      return INVALID_CLOCK;
    }

    /*-------------------------------------------------
    Perform the lookup
    -------------------------------------------------*/
    size_t idx = registry->getResourceIndex( address );
    if( idx == INVALID_RESOURCE_INDEX )
    {
      return INVALID_CLOCK;
    }

    return getClockFrequency( registry->clockSource[ idx ] );
  }

}  // namespace Thor::LLD::RCC
