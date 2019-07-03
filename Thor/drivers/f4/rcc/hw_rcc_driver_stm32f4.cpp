/********************************************************************************
 *   File Name:
 *    hw_rcc_driver_stm32f4.cpp
 *
 *   Description:
 *    Implements the low level driver for the Reset and Clock Control peripheral
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <array>
#include <cstring>

/* Chimera Includes */
#include <Chimera/chimera.hpp>
#include <Chimera/interface/compiler_intf.hpp>
#include <Chimera/types/peripheral_types.hpp>

/* Driver Includes */
#include <Thor/drivers/common/mapping/peripheral_mapping.hpp>
#include <Thor/drivers/f4/power/hw_power_types.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_driver.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_driver_prv.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_prj.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_types.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_mapping.hpp>
#include <Thor/drivers/model/rcc_model.hpp>

static constexpr uint8_t numPeriphs = static_cast<uint8_t>( Chimera::Peripheral::Type::NUM_SUPPORTED_TYPES );
static std::array<Thor::Driver::RCC::Peripheral *, numPeriphs> periphSingletonInstances;

namespace Thor::Driver::RCC
{
  static Peripheral *const getPeriphInstance( const Chimera::Peripheral::Type periph )
  {
    using namespace Thor::Driver::Mapping;

    /*------------------------------------------------
    Find the iterator associated with the requested peripheral
    ------------------------------------------------*/
    auto iterator = PeriphTypeToIterator.find( periph );
    if ( iterator == PeriphTypeToIterator.end() )
    {
      return nullptr;
    }
    else
    {
      return periphSingletonInstances[ iterator->second ];
    }
  }

  void init()
  {
    static bool initialized = false;

    if ( !initialized )
    {
      periphSingletonInstances.fill( nullptr );
    }
  }

  WEAKDECL uint32_t prjGetHSIValue()
  {
    /*------------------------------------------------
    Typical value of the high speed internal oscillator in Hz
    ------------------------------------------------*/
    return 16000000u;
  }

  WEAKDECL uint32_t prjGetHSEValue()
  {
    /*------------------------------------------------
    Typical value of the high speed external oscillator in Hz
    ------------------------------------------------*/
    return 25000000u;
  }

  WEAKDECL uint32_t prjGetLSIValue()
  {
    /*------------------------------------------------
    Typical value of the low speed internal oscillator in Hz
    ------------------------------------------------*/
    return 32000u;
  }


  WEAKDECL uint32_t HAL_RCC_GetSysClockFreq(void)
  {
    uint32_t pllm = 0U, pllvco = 0U, pllp = 0U;
    uint32_t sysclockfreq = 0U;

    const auto HSI_VALUE = prjGetHSIValue();
    const auto HSE_VALUE = prjGetHSEValue();

    switch (CFGR::SWS::get() )
    {
      case CFGR::SWS::HSI:
        sysclockfreq = HSI_VALUE;
        break;

      case CFGR::SWS::HSE:
        sysclockfreq = HSE_VALUE;
        break;

      case CFGR::SWS::PLL:
        pllm = PLLCFGR::M::get();

        if ( PLLCFGR::SRC::get() == PLLCFGR::SRC::HSE )
        {
          pllvco = static_cast<uint32_t>(((static_cast<uint64_t>( HSE_VALUE) * ( static_cast<uint64_t>(PLLCFGR::N::get() >> PLLCFGR_PLLN_Pos)))) / static_cast<uint64_t>(pllm));
        }
        else
        {
          pllvco = static_cast<uint32_t>(((static_cast<uint64_t>( HSI_VALUE ) * (static_cast<uint64_t>( PLLCFGR::N::get() >> PLLCFGR_PLLN_Pos)))) / static_cast<uint64_t>(pllm));
        }

        pllp = (((PLLCFGR::P::get() >> PLLCFGR_PLLP_Pos) + 1U) * 2U);

        sysclockfreq = pllvco/pllp;
        break;

      default:
        sysclockfreq = HSI_VALUE;
        break;
    }

    return sysclockfreq;
  }


  /**
   *  Configures the high speed external oscillator clock selection
   *
   *  @param[in]  init    Initialization configuration struct
   *  @return Chimera::Status_t
   */
  static inline Chimera::Status_t HSEOscillatorConfig( const OscInit *init )
  {
    using namespace CFGR;
    using namespace PLLCFGR;
    using namespace CR;

    const auto clockSource = SWS::get();
    const auto pllSource   = SRC::get();

    if ( ( clockSource == SWS::HSE ) || ( ( clockSource == SWS::PLL ) && ( pllSource == SRC::HSE ) ) )
    {
      /*------------------------------------------------
      When HSE is used as system clock it will not be disabled.
      ------------------------------------------------*/
      if ( ( ( HSERDY::get() == locked ) || ( HSEON::get() == enabled ) ) && ( init->HSEState == HSEConfig::OFF ) )
      {
        return Chimera::CommonStatusCodes::FAIL;
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
          RCC_PERIPH->CR |= CR_HSEON;
          break;

        case HSEConfig::BYPASS:
          RCC_PERIPH->CR |= CR_HSEBYP;
          RCC_PERIPH->CR |= CR_HSEON;

        case HSEConfig::OFF:
        default:
          RCC_PERIPH->CR &= ~CR_HSEON;
          RCC_PERIPH->CR &= ~CR_HSEBYP;
          break;
      }

      /*------------------------------------------------
      Wait for the oscillator to achieve the desired state
      ------------------------------------------------*/
      auto tickstart = Chimera::millis();

      if ( init->HSEState != HSEConfig::OFF )
      {
        while ( ( HSERDY::get() == locked ) || ( HSEON::get() == disabled ) )
        {
          if ( ( Chimera::millis() - tickstart ) > HSE_TIMEOUT_VALUE_MS )
          {
            return Chimera::CommonStatusCodes::TIMEOUT;
          }
        }
      }
      else
      {
        while ( HSEON::get() != disabled )
        {
          if ( ( Chimera::millis() - tickstart ) > HSE_TIMEOUT_VALUE_MS )
          {
            return Chimera::CommonStatusCodes::TIMEOUT;
          }
        }
      }
    }

    return Chimera::CommonStatusCodes::OK;
  }

  /**
   *  Configures the internal high speed oscillator clock selection
   *
   *  @param[in]  init    Initialization configuration struct
   *  @return Chimera::Status_t
   */
  static inline Chimera::Status_t HSIOscillatorConfig( const OscInit *init )
  {
    using namespace CFGR;
    using namespace PLLCFGR;
    using namespace CR;

    const auto clockSource = SWS::get();
    const auto pllSource   = SRC::get();

    /*------------------------------------------------
    Check if HSI is used as system clock or as PLL source when PLL is selected as system clock
    ------------------------------------------------*/
    if ( ( clockSource == SWS::HSI ) || ( ( clockSource == SWS::PLL ) && ( pllSource == SRC::HSI ) ) )
    {
      /*------------------------------------------------
      When HSI is used as system clock it will not be disabled.
      ------------------------------------------------*/
      if ( ( ( HSIRDY::get() == locked ) || ( HSION::get() == enabled ) ) && ( init->HSIState != HSIConfig::ON ) )
      {
        return Chimera::CommonStatusCodes::FAIL;
      }
      else
      {
        HSITRIM::set( init->HSICalibrationValue );
      }
    }
    else
    {
      /*------------------------------------------------
      Try to enter the desired configuration state
      ------------------------------------------------*/
      auto tickstart = Chimera::millis();

      if ( init->HSIState  != HSIConfig::OFF )
      {
        /*------------------------------------------------
        Enable the Internal High Speed oscillator
        ------------------------------------------------*/
        HSION::set( HSIConfig::ON );

        /*------------------------------------------------
        Wait till HSI has achieved the desired state
        ------------------------------------------------*/
        while ( HSIRDY::get() == unlocked )
        {
          if ( ( Chimera::millis() - tickstart ) > HSI_TIMEOUT_VALUE_MS )
          {
            return Chimera::CommonStatusCodes::TIMEOUT;
          }
        }

        /*------------------------------------------------
        Adjusts the Internal High Speed oscillator calibration value.
        ------------------------------------------------*/
        HSITRIM::set( init->HSICalibrationValue );
      }
      else
      {
        /*------------------------------------------------
        Disable the Internal High Speed oscillator
        ------------------------------------------------*/
        HSION::set( HSIConfig::OFF );

        /*------------------------------------------------
        Wait till HSI has achieved the desired state
        ------------------------------------------------*/
        while ( HSIRDY::get() == locked )
        {
          if ( ( Chimera::millis() - tickstart ) > HSI_TIMEOUT_VALUE_MS )
          {
            return Chimera::CommonStatusCodes::TIMEOUT;
          }
        }
      }
    }

    return Chimera::CommonStatusCodes::OK;
  }

  /**
   *  Configures the low speed internal oscillator clock selection
   *
   *  @param[in]  init    Initialization configuration struct
   *  @return Chimera::Status_t
   */
  static inline Chimera::Status_t LSIOscillatorConfig( const OscInit *init )
  {
    using namespace CSR;

    /*------------------------------------------------
    Set the clock configuration to the desired state
    ------------------------------------------------*/
    auto tickstart = Chimera::millis();

    if ( init->LSIState != LSIConfig::OFF )
    {
      LSION::set( LSIConfig::ON );

      /* Wait till LSI is ready */
      while ( LSION::get() == unlocked )
      {
        if ( ( Chimera::millis() - tickstart ) > LSI_TIMEOUT_VALUE_MS )
        {
          return Chimera::CommonStatusCodes::TIMEOUT;
        }
      }
    }
    else
    {
      LSION::set( LSIConfig::OFF );

      /* Wait till LSI is ready */
      while ( LSION::get() == locked )
      {
        if ( ( Chimera::millis() - tickstart ) > LSI_TIMEOUT_VALUE_MS )
        {
          return Chimera::CommonStatusCodes::TIMEOUT;
        }
      }
    }

    return Chimera::CommonStatusCodes::OK;
  }

  /**
   *  Configures the low speed external oscillator clock selection
   *
   *  @param[in]  init    Initialization configuration struct
   *  @return Chimera::Status_t
   */
  static inline Chimera::Status_t LSEOsciallatorConfig( const OscInit *init )
  {
    using namespace APB1ENR;
    using namespace BDCR;

    bool pwrclkchanged = false;
    uint32_t tickstart = 0u;

    /*------------------------------------------------
    Updating the LSE configuration requires write access
    ------------------------------------------------*/
    if ( PWREN::get() == PWRENConfig::OFF )
    {
      PWREN::set( PWRENConfig::ON );
      pwrclkchanged = true;
    }

    /*------------------------------------------------
    Enable write access to RTC and RTC Backup registers
    ------------------------------------------------*/
    if ( !PWR::CR::DBP::get() )
    {
      PWR::CR::DBP::set( PWR::CR_DBP );

      /* Wait for Backup domain Write protection disable */
      tickstart = Chimera::millis();

      while ( !PWR::CR::DBP::get() )
      {
        if ( ( Chimera::millis() - tickstart ) > DBP_TIMEOUT_VALUE_MS )
        {
          return Chimera::CommonStatusCodes::TIMEOUT;
        }
      }
    }

    /*------------------------------------------------
    Set the new oscillator configuration
    ------------------------------------------------*/
    switch ( init->LSEState )
    {
      case LSEConfig::ON:
        RCC_PERIPH->BDCR |= BDCR_LSEON;
        break;

      case LSEConfig::BYPASS:
        RCC_PERIPH->BDCR |= BDCR_LSEBYP;
        RCC_PERIPH->BDCR |= BDCR_LSEON;

      case LSEConfig::OFF:
      default:
        RCC_PERIPH->BDCR &= ~BDCR_LSEON;
        RCC_PERIPH->BDCR &= ~BDCR_LSEBYP;
        break;
    }

    if ( init->LSEState == LSEConfig::OFF )
    {
      /*------------------------------------------------
      Wait until the flag goes low to indicate the clock has been disabled
      ------------------------------------------------*/
      tickstart = Chimera::millis();

      while ( LSERDY::get() )
      {
        if ( ( Chimera::millis() - tickstart ) > LSE_TIMEOUT_VALUE_MS )
        {
          return Chimera::CommonStatusCodes::TIMEOUT;
        }
      }
    }
    else
    {
      /*------------------------------------------------
      Wait until the flag goes high to indicate the clock is ready
      ------------------------------------------------*/
      tickstart = Chimera::millis();

      while ( !LSERDY::get() )
      {
        if ( ( Chimera::millis() - tickstart ) > LSE_TIMEOUT_VALUE_MS )
        {
          return Chimera::CommonStatusCodes::TIMEOUT;
        }
      }
    }

    /*------------------------------------------------
    Restore the power clock
    ------------------------------------------------*/
    if ( pwrclkchanged )
    {
      PWREN::set( PWRENConfig::OFF );
    }
  }

  /**
   *  Configures the PLL oscillator clock selection
   *
   *  @note Before calling this function, the system clock source
   *        must be set to use the PLL.
   *
   *  @param[in]  init    Initialization configuration struct
   *  @return Chimera::Status_t
   */
  static inline Chimera::Status_t PLLOscillatorConfig( const OscInit *init )
  {
    using namespace CR;
    using namespace CFGR;

    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;
    const auto clockSource = SWS::get();

    if ( ( init->PLL.State != PLLConfig::NONE ) && ( clockSource == SWS::PLL ) )
    {
      if ( init->PLL.State == PLLConfig::ON )
      {
        /*------------------------------------------------
        Turn off the PLL and wait for ready signal
        ------------------------------------------------*/
        auto tickstart = Chimera::millis();
        PLLON::set( PLLConfig::OFF ); 

        while ( PLLRDY::get() )
        {
          if ( ( Chimera::millis() - tickstart ) > PLL_TIMEOUT_VALUE_MS )
          {
            return Chimera::CommonStatusCodes::TIMEOUT;
          }
        }

        /*------------------------------------------------
        Configure the main PLL clock source, multiplication and division factors
        ------------------------------------------------*/
        const auto src = init->PLL.Source;
        const auto M   = init->PLL.M;
        const auto N   = init->PLL.N << PLLCFGR_PLLN_Pos;
        const auto P   = ( ( init->PLL.P >> 1U ) - 1U ) << PLLCFGR_PLLP_Pos;
        const auto Q   = init->PLL.Q << PLLCFGR_PLLQ_Pos;
        const auto R   = init->PLL.R << PLLCFGR_PLLR_Pos;

        uint32_t tmp = RCC_PERIPH->PLLCFGR;
        tmp &= ~( PLLCFGR_PLLSRC | PLLCFGR_PLLM | PLLCFGR_PLLN | PLLCFGR_PLLP | PLLCFGR_PLLQ | PLLCFGR_PLLR );
        tmp |= ( src | M | N | P | Q | R );
        RCC_PERIPH->PLLCFGR = tmp;

        /*------------------------------------------------
        Turn on the PLL and wait for ready signal
        ------------------------------------------------*/
        tickstart = Chimera::millis();
        PLLON::set( PLLConfig::ON );

        while ( !PLLRDY::get() )
        {
          if ( ( Chimera::millis() - tickstart ) > PLL_TIMEOUT_VALUE_MS )
          {
            return Chimera::CommonStatusCodes::TIMEOUT;
          }
        }
      }
      else
      {
        /*------------------------------------------------
        Turn off the PLL and wait for ready signal
        ------------------------------------------------*/
        auto tickstart = Chimera::millis();
        PLLON::set( PLLConfig::OFF );

        while ( PLLRDY::get() )
        {
          if ( ( Chimera::millis() - tickstart ) > PLL_TIMEOUT_VALUE_MS )
          {
            return Chimera::CommonStatusCodes::TIMEOUT;
          }
        }
      }
    }
    else
    {
      result = Chimera::CommonStatusCodes::FAIL;
    }

    return result;
  }

  /**
   *
   */
  static inline void HCLKConfig( const ClkInit *init )
  {
    /*------------------------------------------------
    If also configuring the PCLKs, set the highest APBx dividers 
    in order to ensure that we do not go through a non-spec phase 
    whatever we decrease or increase HCLK.
    ------------------------------------------------*/
    if((init->ClockType & ClockType::PCLK1) == ClockType::PCLK1)
    {
      CFGR::PPRE1::set( CFGR::PPRE1::AHB_DIV16 );
    }

    if((init->ClockType & ClockType::PCLK2) == ClockType::PCLK2)
    {
      CFGR::PPRE2::set( CFGR::PPRE2::AHB_DIV16 );
    }

    CFGR::HPRE::set( init->AHBCLKDivider );
  }

  /**
   *
   */
  static inline Chimera::Status_t SYSCLKConfig( const ClkInit *init )
  {
    /*------------------------------------------------
    Verify that the selected clock source indicates it is ready
    ------------------------------------------------*/
    if ( ( init->SYSCLKSource == SysClockSource::HSE ) && !CR::HSERDY::get() )
    {
      return Chimera::CommonStatusCodes::FAIL;
    }
    else if ( ( ( init->SYSCLKSource == SysClockSource::PLLCLK ) || ( init->SYSCLKSource == SysClockSource::PLLRCLK ) ) &&
              !CR::PLLRDY::get() )
    {
      return Chimera::CommonStatusCodes::FAIL;
    }
    else if ( ( init->SYSCLKSource == SysClockSource::HSI ) && !CR::HSIRDY::get() )
    {
      return Chimera::CommonStatusCodes::FAIL;
    }
    else
    {
      return Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
    }

    /*------------------------------------------------
    Configure the system clock and wait for the ready signal
    ------------------------------------------------*/
    CFGR::SW::set( init->SYSCLKSource );
    auto tickstart = Chimera::millis();

    while ( CFGR::SWS::get() != init->SYSCLKSource )
    {
      if ( ( Chimera::millis() - tickstart ) > CLOCKSWITCH_TIMEOUT_VALUE_MS )
      {
        return Chimera::CommonStatusCodes::TIMEOUT;
      }
    }

    return Chimera::CommonStatusCodes::OK;
  }

  /**
   *
   */
  static inline Chimera::Status_t PCLK1Config( const ClkInit *init )
  {
    CFGR::PPRE1::set( init->APB1CLKDivider );
    return Chimera::CommonStatusCodes::OK;
  }

  /**
   *
   */
  static inline Chimera::Status_t PCLK2Config( const ClkInit *init )
  {
    CFGR::PPRE2::set( init->APB2CLKDivider );
    return Chimera::CommonStatusCodes::OK;
  }

  /**
   *  
   */
  static inline Chimera::Status_t UpdateFlashLatency( const uint32_t value )
  {
    //    /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
    //    __HAL_FLASH_SET_LATENCY(FLatency);
    //
    //    /* Check that the new number of wait states is taken into account to access the Flash
    //    memory by reading the FLASH_ACR register */
    //    if(__HAL_FLASH_GET_LATENCY() != FLatency)
    //    {
    //      return HAL_ERROR;
    //    }

    // TODO!
    return 0;
  }

  Chimera::Status_t OscillatorConfig( OscInit *init )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    if ( !init )
    {
      result = Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
    }
    else
    {
      switch ( init->source )
      {
        case OscillatorSource::HSE:
          result = HSEOscillatorConfig( init );
          break;

        case OscillatorSource::HSI:
          result = HSIOscillatorConfig( init );
          break;

        case OscillatorSource::LSE:
          result = LSEOsciallatorConfig( init );
          break;

        case OscillatorSource::LSI:
          result = LSIOscillatorConfig( init );
          break;

        default:
          result = Chimera::CommonStatusCodes::FAIL;
          break;
      }

      if ( result == Chimera::CommonStatusCodes::OK )
      {
        result = PLLOscillatorConfig( init );
      }
    }

    return result;
  }

  Chimera::Status_t ClockConfig( ClkInit *init )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    if ( !init )
    {
      result = Chimera::CommonStatusCodes::FAIL;
    }
    else
    {
      /*------------------------------------------------
      Handle multiple clock configuration possibilities in the correct order.
      It may be ugly and innefficient, but this code does not run often.
      ------------------------------------------------*/
      if ( ( init->ClockType & ClockType::HCLK ) == ClockType::HCLK )
      {
        HCLKConfig( init );
      }

      if ( ( init->ClockType & ClockType::SYSCLK ) == ClockType::SYSCLK )
      {
        SYSCLKConfig( init );
      }

      if ( ( init->ClockType & ClockType::PCLK1 ) == ClockType::PCLK1 )
      {
        PCLK1Config( init );
      }

      if ( ( init->ClockType & ClockType::PCLK2 ) == ClockType::PCLK2 )
      {
        PCLK2Config( init );
      }

      UpdateFlashLatency( 0 );

      /* Update the SystemCoreClock global variable */
      SystemCoreClock = HAL_RCC_GetSysClockFreq() >> AHBPrescTable[ CFGR::HPRE::get() >> CFGR_HPRE_Pos ];

      /* Configure the source of time base considering new system clocks settings */
      //HAL_InitTick( TICK_INT_PRIORITY );
    }

    return result;
  }


  WEAKDECL OscInit prjGetOscConfig()
  {
    OscInit dummyConfig;
    memset( &dummyConfig, 0, sizeof( OscInit ) );
    return dummyConfig;
  }

  WEAKDECL ClkInit prjGetClkConfig()
  {
    ClkInit dummyConfig;
    memset( &dummyConfig, 0, sizeof( ClkInit ) );
    return dummyConfig;
  }

  /**
   *  SystemClock Class Implementation
   */
  SystemClock::SystemClock()
  {
  }

  SystemClock::~SystemClock()
  {
  }

  SystemClock *const SystemClock::get()
  {
    static SystemClock *ref = nullptr;

    if ( ref == nullptr )
    {
      ref = new SystemClock();
    }

    return ref;
  }

  Chimera::Status_t SystemClock::setPeriphClock( const Chimera::Peripheral::Type periph, const size_t freqHz )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::FAIL;

    return result;
  }

  Chimera::Status_t SystemClock::setCoreClock( const size_t freqHz )
  {
    using namespace Thor::Driver;

    Chimera::Status_t result = Chimera::CommonStatusCodes::FAIL;

    /*------------------------------------------------
    Turn on the main internal regulator output voltage
    ------------------------------------------------*/
    APB1ENR::PWREN::set( APB1ENR::PWRENConfig::ON );

    /*------------------------------------------------
    Set the voltage scaling to allow us to achieve max clock
    ------------------------------------------------*/
    PWR::CR::VOS::set( PWR::CR::VOS::VOLTAGE_SCALE_1 );

    /*------------------------------------------------
    Initialize the clock source drivers
    ------------------------------------------------*/
    auto oscCfg = prjGetOscConfig();

    oscCfg.source = OscillatorSource::HSI;
    oscCfg.HSIState = CR::HSIConfig::ON;
    oscCfg.HSICalibrationValue = 16;

    oscCfg.HSEState = CR::HSEConfig::OFF;
    oscCfg.LSIState = CSR::LSIConfig::OFF;
    oscCfg.LSEState = BDCR::LSEConfig::OFF;

    oscCfg.PLL.State = CR::PLLConfig::ON;
    oscCfg.PLL.Source = PLLCFGR::SRC::HSI;
    oscCfg.PLL.M      = 8;
    oscCfg.PLL.N      = 128;
    oscCfg.PLL.P      = CR::PLLDiv::DIV2;
    oscCfg.PLL.Q      = 2;
    oscCfg.PLL.R      = 2;

    result = OscillatorConfig( &oscCfg );

    /*------------------------------------------------
    Initializes the CPU, AHB, and APB bus clocks
    ------------------------------------------------*/
    auto clkCfg = prjGetClkConfig();

    clkCfg.ClockType = ClockType::HCLK | ClockType::SYSCLK | ClockType::PCLK1 | ClockType::PCLK2;
    clkCfg.SYSCLKSource = SysClockSource::PLLCLK;
    clkCfg.AHBCLKDivider = SysClockDiv::DIV1;
    clkCfg.APB1CLKDivider = HClkDiv::DIV4;
    clkCfg.APB2CLKDivider = HClkDiv::DIV2;

    result = ClockConfig( &clkCfg );

    /*------------------------------------------------
    Configure the SysTick interrupt and clock source
    ------------------------------------------------*/

    return result;
  }

  Chimera::Status_t SystemClock::setCoreClockSource( const Thor::Clock::Source src )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::FAIL;

    

    return result;
  }

  size_t SystemClock::getCoreClock()
  {
    return 0;
  }

  Thor::Clock::Source SystemClock::getCoreClockSource()
  {
    return Thor::Clock::Source::CSI;
  }

  size_t SystemClock::getPeriphClock( const Chimera::Peripheral::Type periph )
  {
    return 0;
  }


  /**
   *  GPIOPeriph Class Implementation
   */

  GPIOPeriph::GPIOPeriph() : iterator( 0 )
  {
    /*------------------------------------------------
    Register the singleton instance with the peripheral tracker
    ------------------------------------------------*/
    auto peripheralType                  = Chimera::Peripheral::Type::GPIO;
    iterator                             = Thor::Driver::Mapping::PeriphTypeToIterator.find( peripheralType )->second;
    periphSingletonInstances[ iterator ] = reinterpret_cast<Peripheral *>( this );
  }

  GPIOPeriph::~GPIOPeriph()
  {
    free( periphSingletonInstances[ iterator ] );
    periphSingletonInstances[ iterator ] = nullptr;
  }

  Peripheral *const GPIOPeriph::get()
  {
    using namespace Thor::Driver::Mapping;

    /*------------------------------------------------
    Lookup the index where the GPIO singleton is stored
    ------------------------------------------------*/
    auto iterator = PeriphTypeToIterator.find( Chimera::Peripheral::Type::GPIO );

    if ( iterator == PeriphTypeToIterator.end() )
    {
      /*------------------------------------------------
      This peripheral type is not supported
      ------------------------------------------------*/
      return nullptr;
    }
    else
    {
      /*------------------------------------------------
      Create the new object if non-existant
      ------------------------------------------------*/
      if ( periphSingletonInstances[ iterator->second ] == nullptr )
      {
        periphSingletonInstances[ iterator->second ] = new GPIOPeriph();
      }

      return periphSingletonInstances[ iterator->second ];
    }
  }

  Chimera::Status_t GPIOPeriph::init()
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    return result;
  }

  Chimera::Status_t GPIOPeriph::reset( const size_t instance )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    auto prrConfig = ResetConfig_GPIO[ iterator ];
    uint32_t tmp   = *prrConfig.reg;

    /*------------------------------------------------
    Begin the reset operation
    ------------------------------------------------*/
    tmp |= prrConfig.mask;
    *prrConfig.reg = tmp;

    /*------------------------------------------------
    Waste a few cycles to allow the reset to complete
    ------------------------------------------------*/
    // TODO

    /*------------------------------------------------
    Remove the reset flag as it is not cleared automatically by hardware
    ------------------------------------------------*/
    tmp &= ~prrConfig.mask;
    *prrConfig.reg = tmp;

    return result;
  }

  Chimera::Peripheral::Type GPIOPeriph::getType()
  {
    return Chimera::Peripheral::Type::GPIO;
  }

  Chimera::Status_t GPIOPeriph::enableClock( const size_t instance )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    /*------------------------------------------------
    Read-modify-write
    ------------------------------------------------*/
    auto ceConfig = ClockConfig_GPIO[ iterator ];
    *ceConfig.reg |= ceConfig.mask;

    return result;
  }

  Chimera::Status_t GPIOPeriph::disableClock( const size_t instance )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    /*------------------------------------------------
    Read-modify-write
    ------------------------------------------------*/
    auto ceConfig = ClockConfig_GPIO[ iterator ];
    *ceConfig.reg &= ~ceConfig.mask;

    return result;
  }

  Chimera::Status_t GPIOPeriph::enableClockLowPower( const size_t instance )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    /*------------------------------------------------
    Read-modify-write
    ------------------------------------------------*/
    auto celpConfig = ClockConfigLP_GPIO[ iterator ];
    *celpConfig.reg |= celpConfig.mask;

    return result;
  }

  Chimera::Status_t GPIOPeriph::disableClockLowPower( const size_t instance )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    /*------------------------------------------------
    Read-modify-write
    ------------------------------------------------*/
    auto celpConfig = ClockConfigLP_GPIO[ iterator ];
    *celpConfig.reg &= ~celpConfig.mask;

    return result;
  }
}    // namespace Thor::Driver::RCC
