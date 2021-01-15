/********************************************************************************
 *  File Name:
 *    hw_rcc_driver.cpp
 *
 *  Description:
 *    Implements the low level driver for the Reset and Clock Control peripheral
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <array>
#include <cstring>
#include <cstdlib>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/clock>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/common/cortex-m4/system_time.hpp>
#include <Thor/lld/common/mapping/peripheral_mapping.hpp>
#include <Thor/lld/interface/inc/rcc>
#include <Thor/lld/interface/inc/power>

#include <Thor/lld/stm32f4x/rcc/hw_rcc_types.hpp>

namespace Thor::LLD::RCC
{
  /*------------------------------------------------
  Local Function Declarations
  ------------------------------------------------*/
  static Chimera::Status_t HSEOscillatorConfig( const OscillatorInit *const init );
  static Chimera::Status_t HSIOscillatorConfig( const OscillatorInit *const init );
  static Chimera::Status_t LSIOscillatorConfig( const OscillatorInit *const init );
  static Chimera::Status_t LSEOsciallatorConfig( const OscillatorInit *const init );
  static Chimera::Status_t PLLOscillatorConfig( const OscillatorInit *const init );
  static void HCLKConfig( const ClockInit *const init );
  static Chimera::Status_t SYSCLKConfig( const ClockInit *const init );
  static Chimera::Status_t PCLK1Config( const ClockInit *init );
  static Chimera::Status_t PCLK2Config( const ClockInit *init );
  static Chimera::Status_t UpdateFlashLatency( const uint32_t value );
  static Chimera::Status_t OscillatorConfig( OscillatorInit *const init );
  static Chimera::Status_t ClockConfig( const ClockInit *const init );

  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  void initialize()
  {
    using namespace Chimera::Peripheral;
    using namespace Thor::LLD::RCC;

    static bool initialized = false;

    if ( !initialized )
    {
      initializeRegistry();
      initialized = true;
    }
  }


  Chimera::Status_t prjGetHSIValue( size_t *const projectValue )
  {
    Chimera::Status_t result = Chimera::Status::FAIL;

    /*------------------------------------------------
    Typical value of the high speed internal oscillator in Hz
    ------------------------------------------------*/
    if ( projectValue )
    {
      *projectValue = 16000000u;
      result        = Chimera::Status::OK;
    }

    return result;
  }


  Chimera::Status_t prjGetHSEValue( size_t *const projectValue )
  {
    Chimera::Status_t result = Chimera::Status::FAIL;

    /*------------------------------------------------
    Typical value of the high speed internal oscillator in Hz
    ------------------------------------------------*/
    if ( projectValue )
    {
      *projectValue = 25000000u;
      result        = Chimera::Status::OK;
    }

    return result;
  }


  Chimera::Status_t prjGetLSIValue( size_t *const projectValue )
  {
    Chimera::Status_t result = Chimera::Status::FAIL;

    /*------------------------------------------------
    Typical value of the high speed internal oscillator in Hz
    ------------------------------------------------*/
    if ( projectValue )
    {
      *projectValue = 32000u;
      result        = Chimera::Status::OK;
    }

    return result;
  }


  Chimera::Status_t prjGetSysClockFreq( size_t *const projectValue )
  {
    const Chimera::Status_t prjResult = Chimera::Status::OK;
    Chimera::Status_t result          = Chimera::Status::FAIL;
    size_t pllm                       = 0u;
    size_t pllvco                     = 0u;
    size_t pllp                       = 0u;
    size_t sysclockfreq               = 0u;
    size_t hsiValue                   = 0u;
    size_t hseValue                   = 0u;

    if ( projectValue && ( prjGetHSIValue( &hsiValue ) == prjResult ) && ( prjGetHSEValue( &hseValue ) == prjResult ) )
    {
      switch ( SWS::get( RCC1_PERIPH ) )
      {
        case SWS::HSI:
          sysclockfreq = hsiValue;
          break;

        case SWS::HSE:
          sysclockfreq = hseValue;
          break;

        case SWS::PLL:
          pllm = PLLM::get( RCC1_PERIPH );

          if ( PLLSRC::get( RCC1_PERIPH ) == PLLSRC::HSE )
          {
            pllvco = static_cast<size_t>(
                ( ( static_cast<uint64_t>( hseValue ) * ( static_cast<uint64_t>( PLLN::get( RCC1_PERIPH ) >> PLLCFGR_PLLN_Pos ) ) ) ) /
                static_cast<uint64_t>( pllm ) );
          }
          else
          {
            pllvco = static_cast<size_t>(
                ( ( static_cast<uint64_t>( hsiValue ) * ( static_cast<uint64_t>( PLLN::get( RCC1_PERIPH ) >> PLLCFGR_PLLN_Pos ) ) ) ) /
                static_cast<uint64_t>( pllm ) );
          }

          pllp = ( ( ( PLLP::get( RCC1_PERIPH ) >> PLLCFGR_PLLP_Pos ) + 1U ) * 2U );

          sysclockfreq = pllvco / pllp;
          break;

        default:
          sysclockfreq = hsiValue;
          break;
      }

      *projectValue = sysclockfreq;
      result        = Chimera::Status::OK;
    }

    return result;
  }


  Chimera::Status_t prjGetHCLKFreq( size_t *const projectValue )
  {
    Chimera::Status_t result = Chimera::Status::FAIL;

    if ( projectValue )
    {
      *projectValue = SystemCoreClock;
      result        = Chimera::Status::OK;
    }

    return result;
  }


  Chimera::Status_t prjGetPCLK1Freq( size_t *const projectValue )
  {
    Chimera::Status_t result = Chimera::Status::FAIL;

    if ( projectValue )
    {
      size_t hclk = 0u;
      prjGetHCLKFreq( &hclk );

      *projectValue = hclk >> APBPrescTable[ ( RCC1_PERIPH->CFGR & CFGR_PPRE1 ) >> CFGR_PPRE1_Pos ];
      result        = Chimera::Status::OK;
    }

    return result;
  }


  Chimera::Status_t prjGetPCLK2Freq( size_t *const projectValue )
  {
    Chimera::Status_t result = Chimera::Status::FAIL;

    if ( projectValue )
    {
      size_t hclk = 0u;
      prjGetHCLKFreq( &hclk );

      *projectValue = hclk >> APBPrescTable[ ( RCC1_PERIPH->CFGR & CFGR_PPRE2 ) >> CFGR_PPRE2_Pos ];
      result        = Chimera::Status::OK;
    }

    return result;
  }


  Chimera::Status_t prjGetOscillatorConfig( OscillatorInit *const projectValue )
  {
    Chimera::Status_t result = Chimera::Status::FAIL;

    if ( projectValue )
    {
      OscillatorInit config;
      memset( &config, 0, sizeof( OscillatorInit ) );

      /*------------------------------------------------
      Let the initialization function know we are sending
      configuration data for the HSI & PLL clocks.
      ------------------------------------------------*/
      config.source = Configuration::OscillatorType::HSI | Configuration::OscillatorType::PLLCLK;

      /*------------------------------------------------
      We don't care about these clocks
      ------------------------------------------------*/
      config.HSEState = CR::HSEConfig::OFF;
      config.LSIState = CSR::LSIConfig::OFF;
      config.LSEState = BDCR::LSEConfig::OFF;

      /*------------------------------------------------
      Turn on the internal high speed RC oscillator
      ------------------------------------------------*/
      config.HSIState            = CR::HSIConfig::ON;
      config.HSICalibrationValue = 16;

      /*------------------------------------------------
      Turn on the PLL and give it an input clock from the HSI.
      These settings will configure the PLL to output a
      128 MHz clock signal.
      ------------------------------------------------*/
      config.PLL.State  = CR::PLLConfig::ON;
      config.PLL.Source = PLLSRC::HSI;
      config.PLL.M      = 8;
      config.PLL.N      = 128;
      config.PLL.P      = CR::PLLDiv::DIV2;
      config.PLL.Q      = 2;
      config.PLL.R      = 2;

      memcpy( projectValue, &config, sizeof( OscillatorInit ) );
      result = Chimera::Status::OK;
    }

    return result;
  }


  Chimera::Status_t prjGetClockConfig( ClockInit *const projectValue )
  {
    Chimera::Status_t result = Chimera::Status::FAIL;

    if ( projectValue )
    {
      ClockInit config;
      memset( &config, 0, sizeof( ClockInit ) );

      config.clock = Configuration::ClockType::HCLK | Configuration::ClockType::SYSCLK | Configuration::ClockType::PCLK1 |
                     Configuration::ClockType::PCLK2;
      config.SYSCLKSource   = SW::PLLCLK;
      config.AHBCLKDivider  = HPRE::DIV1;
      config.APB1CLKDivider = PPRE1::DIV4;
      config.APB2CLKDivider = PPRE2::DIV2;
      config.FlashLatency   = 4;

      memcpy( projectValue, &config, sizeof( ClockInit ) );
      result = Chimera::Status::OK;
    }

    return result;
  }


  /*-------------------------------------------------------------------------------
  Static Functions
  -------------------------------------------------------------------------------*/
  static inline Chimera::Status_t HSEOscillatorConfig( const OscillatorInit *const init )
  {
    const auto clockSource = SWS::get( RCC1_PERIPH );
    const auto pllSource   = SRC::get( RCC1_PERIPH );

    if ( ( clockSource == SWS::HSE ) || ( ( clockSource == SWS::PLL ) && ( pllSource == SRC::HSE ) ) )
    {
      /*------------------------------------------------
      When HSE is used as system clock it will not be disabled.
      ------------------------------------------------*/
      if ( ( ( HSERDY::get( RCC1_PERIPH ) == locked ) || ( HSEON::get( RCC1_PERIPH ) == enabled ) ) && ( init->HSEState == HSEConfig::OFF ) )
      {
        return Chimera::Status::FAIL;
      }
    }
    else
    {
      /*------------------------------------------------
      Set the appropriate flags to change the HSE state
      ------------------------------------------------*/
      switch ( init->HSEState )
      {
        case HSEConfig::ON:
          RCC1_PERIPH->CR |= CR_HSEON;
          break;

        case HSEConfig::BYPASS:
          RCC1_PERIPH->CR |= CR_HSEBYP;
          RCC1_PERIPH->CR |= CR_HSEON;

        case HSEConfig::OFF:
        default:
          RCC1_PERIPH->CR &= ~CR_HSEON;
          RCC1_PERIPH->CR &= ~CR_HSEBYP;
          break;
      }

      /*------------------------------------------------
      Wait for the oscillator to achieve the desired state
      ------------------------------------------------*/

      if ( init->HSEState != HSEConfig::OFF )
      {
        while ( ( HSERDY::get( RCC1_PERIPH ) == locked ) || ( HSEON::get( RCC1_PERIPH ) == disabled ) )
        {
        }
      }
      else
      {
        while ( HSEON::get( RCC1_PERIPH ) != disabled )
        {
        }
      }
    }

    return Chimera::Status::OK;
  }


  static inline Chimera::Status_t HSIOscillatorConfig( const OscillatorInit *const init )
  {
    using namespace CFGR;
    using namespace PLLCFGR;
    using namespace CR;

    const auto clockSource = SWS::get( RCC1_PERIPH );
    const auto pllSource   = SRC::get( RCC1_PERIPH );

    /*------------------------------------------------
    Check if HSI is used as system clock or as PLL source when PLL is selected as system clock
    ------------------------------------------------*/
    if ( ( clockSource == SWS::HSI ) || ( ( clockSource == SWS::PLL ) && ( pllSource == SRC::HSI ) ) )
    {
      /*------------------------------------------------
      When HSI is used as system clock it will not be disabled.
      ------------------------------------------------*/
      if ( ( ( HSIRDY::get( RCC1_PERIPH ) == locked ) || ( HSION::get( RCC1_PERIPH ) == enabled ) ) && ( init->HSIState != HSIConfig::ON ) )
      {
        return Chimera::Status::FAIL;
      }
      else
      {
        HSITRIM::set( RCC1_PERIPH, init->HSICalibrationValue );
      }
    }
    else
    {
      /*------------------------------------------------
      Try to enter the desired configuration state
      ------------------------------------------------*/
      if ( init->HSIState != HSIConfig::OFF )
      {
        /*------------------------------------------------
        Enable the Internal High Speed oscillator
        ------------------------------------------------*/
        HSION::set( RCC1_PERIPH, HSIConfig::ON );

        /*------------------------------------------------
        Wait till HSI has achieved the desired state
        ------------------------------------------------*/
        while ( HSIRDY::get( RCC1_PERIPH ) == unlocked )
        {
        }

        /*------------------------------------------------
        Adjusts the Internal High Speed oscillator calibration value.
        ------------------------------------------------*/
        HSITRIM::set( RCC1_PERIPH, init->HSICalibrationValue );
      }
      else
      {
        /*------------------------------------------------
        Disable the Internal High Speed oscillator
        ------------------------------------------------*/
        HSION::set( RCC1_PERIPH, HSIConfig::OFF );

        /*------------------------------------------------
        Wait till HSI has achieved the desired state
        ------------------------------------------------*/
        while ( HSIRDY::get( RCC1_PERIPH ) == locked )
        {
        }
      }
    }

    return Chimera::Status::OK;
  }


  static inline Chimera::Status_t LSIOscillatorConfig( const OscillatorInit *const init )
  {
    using namespace CSR;

    /*------------------------------------------------
    Set the clock configuration to the desired state
    ------------------------------------------------*/
    if ( init->LSIState != LSIConfig::OFF )
    {
      LSION::set( RCC1_PERIPH, LSIConfig::ON );

      /* Wait till LSI is ready */
      while ( LSION::get( RCC1_PERIPH ) == unlocked )
      {
      }
    }
    else
    {
      LSION::set( RCC1_PERIPH, LSIConfig::OFF );

      /* Wait till LSI is ready */
      while ( LSION::get( RCC1_PERIPH ) == locked )
      {
      }
    }

    return Chimera::Status::OK;
  }


  static inline Chimera::Status_t LSEOsciallatorConfig( const OscillatorInit *const init )
  {
    using namespace APB1ENR;
    using namespace BDCR;

    bool pwrclkchanged = false;

    /*------------------------------------------------
    Updating the LSE configuration requires write access
    ------------------------------------------------*/
    if ( PWREN::get( RCC1_PERIPH ) == PWRENConfig::OFF )
    {
      PWREN::set( RCC1_PERIPH, PWRENConfig::ON );
      pwrclkchanged = true;
    }

    /*------------------------------------------------
    Enable write access to RTC and RTC Backup registers
    ------------------------------------------------*/
    if ( !Thor::LLD::PWR::CR::DBP::get( Thor::LLD::PWR::PWR_PERIPH ) )
    {
      PWR::CR::DBP::set( Thor::LLD::PWR::PWR_PERIPH, Thor::LLD::PWR::CR_DBP );

      /* Wait for Backup domain Write protection disable */
      while ( !Thor::LLD::PWR::CR::DBP::get( Thor::LLD::PWR::PWR_PERIPH ) )
      {
      }
    }

    /*------------------------------------------------
    Set the new oscillator configuration
    ------------------------------------------------*/
    switch ( init->LSEState )
    {
      case LSEConfig::ON:
        RCC1_PERIPH->BDCR |= BDCR_LSEON;
        break;

      case LSEConfig::BYPASS:
        RCC1_PERIPH->BDCR |= BDCR_LSEBYP;
        RCC1_PERIPH->BDCR |= BDCR_LSEON;

      case LSEConfig::OFF:
      default:
        RCC1_PERIPH->BDCR &= ~BDCR_LSEON;
        RCC1_PERIPH->BDCR &= ~BDCR_LSEBYP;
        break;
    }

    if ( init->LSEState == LSEConfig::OFF )
    {
      /*------------------------------------------------
      Wait until the flag goes low to indicate the clock has been disabled
      ------------------------------------------------*/
      while ( LSERDY::get( RCC1_PERIPH ) )
      {
      }
    }
    else
    {
      /*------------------------------------------------
      Wait until the flag goes high to indicate the clock is ready
      ------------------------------------------------*/
      while ( !LSERDY::get( RCC1_PERIPH ) )
      {
      }
    }

    /*------------------------------------------------
    Restore the power clock
    ------------------------------------------------*/
    if ( pwrclkchanged )
    {
      PWREN::set( RCC1_PERIPH, PWRENConfig::OFF );
    }

    return Chimera::Status::OK;
  }


  static Chimera::Status_t PLLOscillatorConfig( const OscillatorInit *const init )
  {
    using namespace CR;
    using namespace CFGR;

    Chimera::Status_t result = Chimera::Status::OK;

    if ( init->PLL.State == PLLConfig::NONE )
    {
      result = Chimera::Status::FAIL;
    }
    else
    {
      /*------------------------------------------------
      Turn off the PLL and wait for ready signal
      ------------------------------------------------*/
      PLLON::set( RCC1_PERIPH, PLLConfig::OFF );

      while ( PLLRDY::get( RCC1_PERIPH ) )
      {
      }

      /*------------------------------------------------
      Configure the main PLL clock source, multiplication and division factors
      ------------------------------------------------*/
      if ( init->PLL.State == PLLConfig::ON )
      {
        const auto src = init->PLL.Source;
        const auto M   = init->PLL.M;
        const auto N   = init->PLL.N << PLLCFGR_PLLN_Pos;
        const auto P   = ( ( init->PLL.P >> 1U ) - 1U ) << PLLCFGR_PLLP_Pos;
        const auto Q   = init->PLL.Q << PLLCFGR_PLLQ_Pos;
        const auto R   = init->PLL.R << PLLCFGR_PLLR_Pos;

        uint32_t tmp = RCC1_PERIPH->PLLCFGR;
        tmp &= ~( PLLCFGR_PLLSRC | PLLCFGR_PLLM | PLLCFGR_PLLN | PLLCFGR_PLLP | PLLCFGR_PLLQ | PLLCFGR_PLLR );
        tmp |= ( src | M | N | P | Q | R );
        RCC1_PERIPH->PLLCFGR = tmp;

        /*------------------------------------------------
        Turn on the PLL and wait for ready signal
        ------------------------------------------------*/
        PLLON::set( RCC1_PERIPH, PLLConfig::ON );

        #if defined( EMBEDDED )
        while ( !PLLRDY::get( RCC1_PERIPH ) )
        {
        }
        #endif
      }
    }

    return result;
  }


  static void HCLKConfig( const ClockInit *const init )
  {
    /*------------------------------------------------
    If also configuring the PCLKs, set the highest APBx dividers
    in order to ensure that we do not go through a non-spec phase
    whenever we decrease or increase HCLK.
    ------------------------------------------------*/
    if ( ( init->clock & Configuration::ClockType::PCLK1 ) == Configuration::ClockType::PCLK1 )
    {
      PPRE1::set( RCC1_PERIPH, PPRE1::DIV16 );
    }

    if ( ( init->clock & Configuration::ClockType::PCLK2 ) == Configuration::ClockType::PCLK2 )
    {
      PPRE2::set( RCC1_PERIPH, PPRE2::DIV16 );
    }

    HPRE::set( RCC1_PERIPH, init->AHBCLKDivider );
  }


  static Chimera::Status_t SYSCLKConfig( const ClockInit *const init )
  {
    /*------------------------------------------------
    Verify that the selected clock source indicates it is ready
    ------------------------------------------------*/
    auto sysClkSrc = init->SYSCLKSource;

    if ( ( sysClkSrc == SW::HSE ) && !CR::HSERDY::get( RCC1_PERIPH ) )
    {
      return Chimera::Status::FAIL;
    }
    else if ( ( ( sysClkSrc == SW::PLLCLK ) || ( sysClkSrc == SW::PLLRCLK ) ) && !CR::PLLRDY::get( RCC1_PERIPH ) )
    {
      return Chimera::Status::FAIL;
    }
    else if ( ( sysClkSrc == SW::HSI ) && !CR::HSIRDY::get( RCC1_PERIPH ) )
    {
      return Chimera::Status::FAIL;
    }

    /*------------------------------------------------
    Configure the system clock and wait for the ready signal
    ------------------------------------------------*/
    SW::set( RCC1_PERIPH, init->SYSCLKSource );

    /* Assumes SW and SWS config bits mean the same thing */
    while ( SWS::getRightShifted( RCC1_PERIPH ) != init->SYSCLKSource )
    {
    }

    return Chimera::Status::OK;
  }


  static inline Chimera::Status_t PCLK1Config( const ClockInit *init )
  {
    PPRE1::set( RCC1_PERIPH, init->APB1CLKDivider );
    return Chimera::Status::OK;
  }


  static inline Chimera::Status_t PCLK2Config( const ClockInit *init )
  {
    PPRE2::set( RCC1_PERIPH, init->APB2CLKDivider );
    return Chimera::Status::OK;
  }


  static Chimera::Status_t UpdateFlashLatency( const uint32_t value )
  {
    using namespace Thor::LLD::FLASH;
    Chimera::Status_t result = Chimera::Status::OK;

    /*------------------------------------------------
    Validate latency configuration since this is such
    a critical operating parameter.
    ------------------------------------------------*/
    ACR::LATENCY::set( Thor::LLD::FLASH::FLASH_PERIPH, value );
    while ( ACR::LATENCY::get( Thor::LLD::FLASH::FLASH_PERIPH ) != value )
    {
      ;
    }

    return result;
  }


  static Chimera::Status_t OscillatorConfig( OscillatorInit *const init )
  {
    Chimera::Status_t result = Chimera::Status::OK;

    if ( !init )
    {
      result = Chimera::Status::INVAL_FUNC_PARAM;
    }
    else
    {
      if ( ( init->source & Configuration::OscillatorType::HSE ) == Configuration::OscillatorType::HSE )
      {
        HSEOscillatorConfig( init );
      }

      if ( ( init->source & Configuration::OscillatorType::HSI ) == Configuration::OscillatorType::HSI )
      {
        HSIOscillatorConfig( init );
      }

      if ( ( init->source & Configuration::OscillatorType::PLLCLK ) == Configuration::OscillatorType::PLLCLK )
      {
        PLLOscillatorConfig( init );
      }
    }

    return result;
  }


  static Chimera::Status_t ClockConfig( const ClockInit *const init )
  {
    Chimera::Status_t result = Chimera::Status::OK;

    if ( !init )
    {
      result = Chimera::Status::FAIL;
    }
    else
    {
      UpdateFlashLatency( init->FlashLatency );

      /*------------------------------------------------
      Handle multiple clock configuration possibilities in the correct order.
      It may be ugly and innefficient, but this code does not run often.
      ------------------------------------------------*/
      if ( ( init->clock & Configuration::ClockType::HCLK ) == Configuration::ClockType::HCLK )
      {
        HCLKConfig( init );
      }

      if ( ( init->clock & Configuration::ClockType::SYSCLK ) == Configuration::ClockType::SYSCLK )
      {
        SYSCLKConfig( init );
      }

      if ( ( init->clock & Configuration::ClockType::PCLK1 ) == Configuration::ClockType::PCLK1 )
      {
        PCLK1Config( init );
      }

      if ( ( init->clock & Configuration::ClockType::PCLK2 ) == Configuration::ClockType::PCLK2 )
      {
        PCLK2Config( init );
      }

      prjGetSysClockFreq( reinterpret_cast<size_t *>( &SystemCoreClock ) );
    }

    return result;
  }
}    // namespace Thor::LLD::RCC
