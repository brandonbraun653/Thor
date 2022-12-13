/******************************************************************************
 *  File Name:
 *    hw_rcc_sys_ctrl.cpp
 *
 *  Description:
 *    System clock controller implementation
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/clock>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/common/cortex-m4/system_time.hpp>
#include <Thor/lld/interface/inc/rcc>
#include <Thor/lld/interface/inc/power>
#include <Thor/lld/interface/inc/interrupt>
#include <Thor/lld/interface/inc/flash>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_prv.hpp>

namespace Thor::LLD::RCC
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static SystemClock s_system_clock;

  /*---------------------------------------------------------------------------
  Public Data
  ---------------------------------------------------------------------------*/
  OscillatorSettings sOscillatorSettings;
  DerivedClockSettings sDerivedClockSettings;

  /*---------------------------------------------------------------------------
  SystemClock Class Implementation
  ---------------------------------------------------------------------------*/
  SystemClock *getCoreClockCtrl()
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


  void SystemClock::enableClock( const Chimera::Clock::Bus clock )
  {
    auto isrMask = Thor::LLD::INT::disableInterrupts();

    switch ( clock )
    {
      case Chimera::Clock::Bus::HSE:
        HSEON::set( RCC1_PERIPH, CR_HSEON );
        while ( !HSERDY::get( RCC1_PERIPH ) )
        {
          ;
        }
        break;

      case Chimera::Clock::Bus::HSI16:
        HSION::set( RCC1_PERIPH, CR_HSION );
        while ( !HSIRDY::get( RCC1_PERIPH ) )
        {
          ;
        }
        break;

      case Chimera::Clock::Bus::MSI:
        MSION::set( RCC1_PERIPH, CR_MSION );
        while ( !MSIRDY::get( RCC1_PERIPH ) )
        {
          ;
        }
        break;

      case Chimera::Clock::Bus::PLLP:
        PLLON::set( RCC1_PERIPH, CR_PLLON );
        while ( !PLLRDY::get( RCC1_PERIPH ) )
        {
          ;
        }
        break;

      case Chimera::Clock::Bus::RC48:
        HSI48ON::set( RCC1_PERIPH, CRRCR_HSI48ON );
        while ( !HSI48RDY::get( RCC1_PERIPH ) )
        {
          ;
        }
        break;

      default:
        break;
    }

    Thor::LLD::INT::enableInterrupts( isrMask );
  }


  void SystemClock::disableClock( const Chimera::Clock::Bus clock )
  {
    auto isrMask = Thor::LLD::INT::disableInterrupts();

    switch ( clock )
    {
      case Chimera::Clock::Bus::HSE:
        HSEON::clear( RCC1_PERIPH, CR_HSEON );
        while ( HSERDY::get( RCC1_PERIPH ) )
        {
          ;
        }
        break;

      case Chimera::Clock::Bus::HSI16:
        HSION::clear( RCC1_PERIPH, CR_HSION );
        while ( HSIRDY::get( RCC1_PERIPH ) )
        {
          ;
        }
        break;

      case Chimera::Clock::Bus::MSI:
        MSION::clear( RCC1_PERIPH, CR_MSION );
        while ( MSIRDY::get( RCC1_PERIPH ) )
        {
          ;
        }
        break;

      case Chimera::Clock::Bus::PLLP:
        PLLON::clear( RCC1_PERIPH, CR_PLLON );
        while ( PLLRDY::get( RCC1_PERIPH ) )
        {
          ;
        }
        break;

      case Chimera::Clock::Bus::RC48:
        HSI48ON::clear( RCC1_PERIPH, CRRCR_HSI48ON );
        while ( HSI48RDY::get( RCC1_PERIPH ) )
        {
          ;
        }
        break;

      default:
        break;
    }

    Thor::LLD::INT::enableInterrupts( isrMask );
  }


  Chimera::Status_t SystemClock::configureProjectClocks()
  {
    using namespace Thor::LLD::PWR;

    /*-------------------------------------------------------------------------
    Turn on the clock to the power controller peripheral and
    set the voltage scaling to allow us to achieve max clock
    -------------------------------------------------------------------------*/
    PWREN::set( RCC1_PERIPH, APB1ENR1_PWREN );
    VOS::set( PWR_PERIPH, CR1::VOS_SCALE_1 );

    /*-------------------------------------------------------------------------
    Configure the source clocks (oscillators)
      PLL P: off
      PLL Q: 40 MHz
      PLL R: 80 MHz
    -------------------------------------------------------------------------*/
    sOscillatorSettings = {};

    /* Disable configuration of the unnecessary clocks */
    sOscillatorSettings.HSEConfig.configure = false;
    sOscillatorSettings.HSIConfig.configure = false;
    sOscillatorSettings.LSEConfig.configure = false;
    sOscillatorSettings.LSIConfig.configure = false;
    sOscillatorSettings.MSIConfig.configure = false;

    /* Set up the PLL */
    sOscillatorSettings.PLLConfig.configure   = true;
    sOscillatorSettings.PLLConfig.inputSource = Chimera::Clock::Bus::HSI16;
    sOscillatorSettings.PLLConfig.divM        = Config::PLLCFGR::M_DIV_4;
    sOscillatorSettings.PLLConfig.divN        = ( 40 << PLLCFGR_PLLN_Pos ) & PLLCFGR_PLLN_Msk;
    sOscillatorSettings.PLLConfig.P.configure = false;
    sOscillatorSettings.PLLConfig.Q.configure = true;
    sOscillatorSettings.PLLConfig.Q.divisor   = Config::PLLCFGR::Q_DIV_4;
    sOscillatorSettings.PLLConfig.R.configure = true;
    sOscillatorSettings.PLLConfig.R.divisor   = Config::PLLCFGR::R_DIV_2;

    sOscillatorSettings.valid = true;

    /*-------------------------------------------------------------------------
    Configure the derived clocks
    -------------------------------------------------------------------------*/
    sDerivedClockSettings = {};

    sDerivedClockSettings.HCLKConfig.configure      = true;
    sDerivedClockSettings.HCLKConfig.AHBPrescaler   = Config::CFGR::AHB_SYS_DIV_1;
    sDerivedClockSettings.PCLK1Config.configure     = true;
    sDerivedClockSettings.PCLK1Config.APB1Prescaler = Config::CFGR::APB1_AHB_DIV_1;
    sDerivedClockSettings.PCLK2Config.configure     = true;
    sDerivedClockSettings.PCLK2Config.APB2Prescaler = Config::CFGR::APB2_AHB_DIV_1;

    sDerivedClockSettings.valid = true;

    /*-------------------------------------------------------------------------
    Apply the settings
    -------------------------------------------------------------------------*/
    if ( configureOscillators( sOscillatorSettings ) && configureClocks( sDerivedClockSettings ) )
    {
      setCoreClockSource( Chimera::Clock::Bus::PLLP );
      CortexM4::Clock::updateCoreClockCache( getSysClockFreq() );

      return Chimera::Status::OK;
    }
    else
    {
      return Chimera::Status::FAIL;
    }
  }


  Chimera::Status_t SystemClock::setCoreClockSource( const Chimera::Clock::Bus src )
  {
    /*-------------------------------------------------------------------------
    Prevent the clock selection update from being interrupted
    -------------------------------------------------------------------------*/
    auto itMask = Thor::LLD::INT::disableInterrupts();

    /*-------------------------------------------------------------------------
    Figure out the configuration bits that should be set. Go
    ahead and update the SystemCoreClock value. If the update
    fails, we'll be stuck in the while loop anyways and it won't
    matter that the variable has been set to a bad value.
    -------------------------------------------------------------------------*/
    Reg32_t cfgOption = 0;
    Reg32_t expStatus = 0;

    switch ( src )
    {
      case Chimera::Clock::Bus::HSE:
        cfgOption = Config::SystemClockSelect::SYSCLK_HSE;
        expStatus = Config::SystemClockStatus::SYSCLK_HSE;

        CortexM4::Clock::updateCoreClockCache( getHSEFreq() );
        break;

      case Chimera::Clock::Bus::MSI:
        cfgOption = Config::SystemClockSelect::SYSCLK_MSI;
        expStatus = Config::SystemClockStatus::SYSCLK_MSI;

        CortexM4::Clock::updateCoreClockCache( getMSIFreq() );
        break;

      case Chimera::Clock::Bus::HSI16:
        cfgOption = Config::SystemClockSelect::SYSCLK_HSI16;
        expStatus = Config::SystemClockStatus::SYSCLK_HSI16;

        CortexM4::Clock::updateCoreClockCache( getHSIFreq() );
        break;

      case Chimera::Clock::Bus::PLLP:
        cfgOption = Config::SystemClockSelect::SYSCLK_PLL;
        expStatus = Config::SystemClockStatus::SYSCLK_PLL;

        CortexM4::Clock::updateCoreClockCache( getPLLCLKFreq( PLLCFGR_PLLR ) );
        break;

      default:
        return Chimera::Status::FAIL;
        break;
    }

    /*-------------------------------------------------------------------------
    Adjust the flash read access latency (Section 3.3.3 of RM0394)

    Note: Currently hardcoded to assume a clock increase, but once
    I have the processor brought up and have some free time, this
    needs to adjust for a decrease too. Can calculate the desired
    clock frequency from the registers in the config structure.
    -------------------------------------------------------------------------*/
    using namespace Thor::LLD::FLASH;
    LATENCY::set( FLASH_PERIPH, ACR_LATENCY_4WS );
    DCEN::set( FLASH_PERIPH, ACR_DCEN );     /* Enable data cache */
    ICEN::set( FLASH_PERIPH, ACR_ICEN );     /* Enable instruction cache */
    PRFTEN::set( FLASH_PERIPH, ACR_PRFTEN ); /* Enable ART prefetch */

    /*-------------------------------------------------------------------------
    Apply the clock selection setting, then wait for the
    hardware to indicate it has stabilized.
    -------------------------------------------------------------------------*/
    SW::set( RCC1_PERIPH, cfgOption );
    while ( ( SWS::get( RCC1_PERIPH ) & expStatus ) != expStatus )
    {
      ;
    }

    /*-------------------------------------------------------------------------
    The clock is stable now, allow normal program execution
    -------------------------------------------------------------------------*/
    Thor::LLD::INT::enableInterrupts( itMask );
    return Chimera::Status::OK;
  }


  Chimera::Clock::Bus SystemClock::getCoreClockSource()
  {
    switch ( SWS::get( RCC1_PERIPH ) )
    {
      case Config::SystemClockStatus::SYSCLK_HSE:
        return Chimera::Clock::Bus::HSE;
        break;

      case Config::SystemClockSelect::SYSCLK_HSI16:
        return Chimera::Clock::Bus::HSI16;
        break;

      case Config::SystemClockSelect::SYSCLK_MSI:
        return Chimera::Clock::Bus::MSI;
        break;

      case Config::SystemClockSelect::SYSCLK_PLL:
        return Chimera::Clock::Bus::PLLP;
        break;

      default:
        return Chimera::Clock::Bus::UNKNOWN_BUS;
        break;
    }
  }


  Chimera::Status_t SystemClock::setClockFrequency( const Chimera::Clock::Bus clock, const size_t freq, const bool enable )
  {
    switch ( clock )
    {
      case Chimera::Clock::Bus::HSE:
        Thor::LLD::RCC::setHSEFreq( freq );
        return Chimera::Status::OK;
        break;

      case Chimera::Clock::Bus::LSE:
        Thor::LLD::RCC::setLSEFreq( freq );
        return Chimera::Status::OK;
        break;

        // TODO: Handle the more complex cases next

      default:
        return Chimera::Status::FAIL;
        break;
    };
  }


  size_t SystemClock::getClockFrequency( const Chimera::Clock::Bus clock )
  {
    switch ( clock )
    {
      case Chimera::Clock::Bus::HCLK:
        return getHCLKFreq();
        break;

      case Chimera::Clock::Bus::PCLK1:
        return getPCLK1Freq();
        break;

      case Chimera::Clock::Bus::PCLK2:
        return getPCLK2Freq();
        break;

      case Chimera::Clock::Bus::SYSCLK:
        return getSysClockFreq();
        break;

      default:
        return INVALID_CLOCK;
        break;
    }
  }


  size_t SystemClock::getPeriphClock( const Chimera::Peripheral::Type periph, const std::uintptr_t address )
  {
    /*-------------------------------------------------------------------------
    Input protection
    -------------------------------------------------------------------------*/
    auto registry = getPCCRegistry( periph );
    if ( !registry || !registry->clockSource || !registry->getResourceIndex )
    {
      return INVALID_CLOCK;
    }

    /*-------------------------------------------------------------------------
    Perform the lookup
    -------------------------------------------------------------------------*/
    size_t idx = registry->getResourceIndex( address );
    if( idx == INVALID_RESOURCE_INDEX )
    {
      return INVALID_CLOCK;
    }

    return getClockFrequency( registry->clockSource[ idx ] );
  }


}  // namespace
