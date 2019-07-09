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
#include <Thor/headers.hpp>
#include <Thor/drivers/common/mapping/peripheral_mapping.hpp>
#include <Thor/drivers/f4/flash/hw_flash_driver.hpp>
#include <Thor/drivers/f4/gpio/hw_gpio_mapping.hpp>
#include <Thor/drivers/f4/uart/hw_uart_mapping.hpp>
#include <Thor/drivers/f4/usart/hw_usart_mapping.hpp>
#include <Thor/drivers/f4/nvic/hw_nvic_driver.hpp>
#include <Thor/drivers/f4/power/hw_power_types.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_driver.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_driver_prv.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_prj.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_types.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_mapping.hpp>
#include <Thor/drivers/model/rcc_model.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_RCC == 1 )

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

  /**
   *  Configures the high speed external oscillator clock selection
   *
   *  @param[in]  init    Initialization configuration struct
   *  @return Chimera::Status_t
   */
  static inline Chimera::Status_t HSEOscillatorConfig( const OscillatorInit *const init )
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
  static inline Chimera::Status_t HSIOscillatorConfig( const OscillatorInit *const init )
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

      if ( init->HSIState != HSIConfig::OFF )
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
  static inline Chimera::Status_t LSIOscillatorConfig( const OscillatorInit *const init )
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
  static inline Chimera::Status_t LSEOsciallatorConfig( const OscillatorInit *const init )
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
   *  @param[in]  init    Initialization configuration struct
   *  @return Chimera::Status_t
   */
  static Chimera::Status_t PLLOscillatorConfig( const OscillatorInit *const init )
  {
    using namespace CR;
    using namespace CFGR;

    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    if ( init->PLL.State == PLLConfig::NONE )
    {
      result = Chimera::CommonStatusCodes::FAIL;
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
    }

    return result;
  }

  /**
   *  Configures the HCLK
   */
  static void HCLKConfig( const ClockInit *const init )
  {
    /*------------------------------------------------
    If also configuring the PCLKs, set the highest APBx dividers
    in order to ensure that we do not go through a non-spec phase
    whenever we decrease or increase HCLK.
    ------------------------------------------------*/
    if ( ( init->clock & ClockType::PCLK1 ) == ClockType::PCLK1 )
    {
      CFGR::PPRE1::set( CFGR::PPRE1::DIV16 );
    }

    if ( ( init->clock & ClockType::PCLK2 ) == ClockType::PCLK2 )
    {
      CFGR::PPRE2::set( CFGR::PPRE2::DIV16 );
    }

    CFGR::HPRE::set( init->AHBCLKDivider );
  }

  /**
   *
   */
  static Chimera::Status_t SYSCLKConfig( const ClockInit *const init )
  {
    /*------------------------------------------------
    Verify that the selected clock source indicates it is ready
    ------------------------------------------------*/
    auto sysClkSrc = init->SYSCLKSource;

    if ( ( sysClkSrc == CFGR::SW::HSE ) && !CR::HSERDY::get() )
    {
      return Chimera::CommonStatusCodes::FAIL;
    }
    else if ( ( ( sysClkSrc == CFGR::SW::PLLCLK ) || ( sysClkSrc == CFGR::SW::PLLRCLK ) ) && !CR::PLLRDY::get() )
    {
      return Chimera::CommonStatusCodes::FAIL;
    }
    else if ( ( sysClkSrc == CFGR::SW::HSI ) && !CR::HSIRDY::get() )
    {
      return Chimera::CommonStatusCodes::FAIL;
    }

    /*------------------------------------------------
    Configure the system clock and wait for the ready signal
    ------------------------------------------------*/
    auto tickstart = Chimera::millis();
    CFGR::SW::set( init->SYSCLKSource );

    /* Assumes SW and SWS config bits mean the same thing */
    while ( CFGR::SWS::getRS() != init->SYSCLKSource )
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
  static inline Chimera::Status_t PCLK1Config( const ClockInit *init )
  {
    CFGR::PPRE1::set( init->APB1CLKDivider );
    return Chimera::CommonStatusCodes::OK;
  }

  /**
   *
   */
  static inline Chimera::Status_t PCLK2Config( const ClockInit *init )
  {
    CFGR::PPRE2::set( init->APB2CLKDivider );
    return Chimera::CommonStatusCodes::OK;
  }

  /**
   *
   */
  static Chimera::Status_t UpdateFlashLatency( const uint32_t value )
  {
    using namespace Thor::Driver::Flash;
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    /*------------------------------------------------
    Validate latency configuration since this is such
    a critical operating parameter.
    ------------------------------------------------*/
    ACR::LATENCY::set( value );
    while ( ACR::LATENCY::get() != value )
    {
      ;
    }

    return result;
  }

  /**
   *
   */
  static Chimera::Status_t OscillatorConfig( OscillatorInit *const init )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    if ( !init )
    {
      result = Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
    }
    else
    {
      if ( ( init->source & OscillatorType::HSE ) == OscillatorType::HSE )
      {
        HSEOscillatorConfig( init );
      }

      if ( ( init->source & OscillatorType::HSI ) == OscillatorType::HSI )
      {
        HSIOscillatorConfig( init );
      }

      if ( ( init->source & OscillatorType::PLLCLK ) == OscillatorType::PLLCLK )
      {
        PLLOscillatorConfig( init );
      }
    }

    return result;
  }

  static Chimera::Status_t ClockConfig( const ClockInit *const init )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    if ( !init )
    {
      result = Chimera::CommonStatusCodes::FAIL;
    }
    else
    {
      UpdateFlashLatency( init->FlashLatency );

      /*------------------------------------------------
      Handle multiple clock configuration possibilities in the correct order.
      It may be ugly and innefficient, but this code does not run often.
      ------------------------------------------------*/
      if ( ( init->clock & ClockType::HCLK ) == ClockType::HCLK )
      {
        HCLKConfig( init );
      }

      if ( ( init->clock & ClockType::SYSCLK ) == ClockType::SYSCLK )
      {
        SYSCLKConfig( init );
      }

      if ( ( init->clock & ClockType::PCLK1 ) == ClockType::PCLK1 )
      {
        PCLK1Config( init );
      }

      if ( ( init->clock & ClockType::PCLK2 ) == ClockType::PCLK2 )
      {
        PCLK2Config( init );
      }

      /* Update the SystemCoreClock global variable */
      SystemCoreClock = prjGetSysClockFreq();
    }

    return result;
  }

  WEAKDECL size_t prjGetHSIValue()
  {
    /*------------------------------------------------
    Typical value of the high speed internal oscillator in Hz
    ------------------------------------------------*/
    return 16000000u;
  }

  WEAKDECL size_t prjGetHSEValue()
  {
    /*------------------------------------------------
    Typical value of the high speed external oscillator in Hz
    ------------------------------------------------*/
    return 25000000u;
  }

  WEAKDECL size_t prjGetLSIValue()
  {
    /*------------------------------------------------
    Typical value of the low speed internal oscillator in Hz
    ------------------------------------------------*/
    return 32000u;
  }

  WEAKDECL size_t prjGetSysClockFreq()
  {
    size_t pllm = 0U, pllvco = 0U, pllp = 0U;
    size_t sysclockfreq = 0U;

    const auto HSI_VALUE = prjGetHSIValue();
    const auto HSE_VALUE = prjGetHSEValue();

    switch ( CFGR::SWS::get() )
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
          pllvco = static_cast<size_t>(
              ( ( static_cast<uint64_t>( HSE_VALUE ) * ( static_cast<uint64_t>( PLLCFGR::N::get() >> PLLCFGR_PLLN_Pos ) ) ) ) /
              static_cast<uint64_t>( pllm ) );
        }
        else
        {
          pllvco = static_cast<size_t>(
              ( ( static_cast<uint64_t>( HSI_VALUE ) * ( static_cast<uint64_t>( PLLCFGR::N::get() >> PLLCFGR_PLLN_Pos ) ) ) ) /
              static_cast<uint64_t>( pllm ) );
        }

        pllp = ( ( ( PLLCFGR::P::get() >> PLLCFGR_PLLP_Pos ) + 1U ) * 2U );

        sysclockfreq = pllvco / pllp;
        break;

      default:
        sysclockfreq = HSI_VALUE;
        break;
    }

    return sysclockfreq;
  }

  WEAKDECL OscillatorInit prjGetOscillatorConfig()
  {
    OscillatorInit config;
    memset( &config, 0, sizeof( OscillatorInit ) );

    /*------------------------------------------------
    Let the initialization function know we are sending
    configuration data for the HSI & PLL clocks.
    ------------------------------------------------*/
    config.source = OscillatorType::HSI | OscillatorType::PLLCLK;

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
    config.PLL.Source = PLLCFGR::SRC::HSI;
    config.PLL.M      = 8;
    config.PLL.N      = 128;
    config.PLL.P      = CR::PLLDiv::DIV2;
    config.PLL.Q      = 2;
    config.PLL.R      = 2;

    return config;
  }

  WEAKDECL ClockInit prjGetClockConfig()
  {
    ClockInit config;
    memset( &config, 0, sizeof( ClockInit ) );

    config.clock          = ClockType::HCLK | ClockType::SYSCLK | ClockType::PCLK1 | ClockType::PCLK2;
    config.SYSCLKSource   = CFGR::SW::PLLCLK;
    config.AHBCLKDivider  = CFGR::HPRE::DIV1;
    config.APB1CLKDivider = CFGR::PPRE1::DIV4;
    config.APB2CLKDivider = CFGR::PPRE2::DIV2;
    config.FlashLatency   = 4;

    return config;
  }

  void init()
  {
    static bool initialized = false;

    if ( !initialized )
    {
      periphSingletonInstances.fill( nullptr );
    }
  }

/*------------------------------------------------
SystemClock Class Implementation
------------------------------------------------*/
#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_RCC == 1 )

  SystemClock::SystemClock()
  {
    init();
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

  Chimera::Status_t SystemClock::configureProjectClocks()
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
    auto oscCfg = prjGetOscillatorConfig();
    result      = OscillatorConfig( &oscCfg );

    /*------------------------------------------------
    Initializes the CPU, AHB, and APB bus clocks
    ------------------------------------------------*/
    auto clkCfg = prjGetClockConfig();
    result      = ClockConfig( &clkCfg );

    return result;
  }

  Chimera::Status_t SystemClock::setPeriphClock( const Chimera::Peripheral::Type periph, const size_t freqHz )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::NOT_SUPPORTED;
    return result;
  }

  Chimera::Status_t SystemClock::setCoreClock( const size_t freqHz )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::NOT_SUPPORTED;
    return result;
  }

  Chimera::Status_t SystemClock::setCoreClockSource( const Thor::Clock::Source src )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::NOT_SUPPORTED;
    return result;
  }

  size_t SystemClock::getCoreClock()
  {
    return prjGetSysClockFreq();
  }

  Thor::Clock::Source SystemClock::getCoreClockSource()
  {
    // TODO
    return Thor::Clock::Source::CSI;
  }

  size_t SystemClock::getPeriphClock( const Chimera::Peripheral::Type periph )
  {
    // TODO
    return 0;
  }

#endif /* TARGET_STM32F4 && THOR_DRIVER_RCC */

/*------------------------------------------------
GPIOPeriph Class Implementation
------------------------------------------------*/
#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_GPIO == 1 )

  GPIOPeriph::GPIOPeriph() : iterator( 0 )
  {
    /*------------------------------------------------
    Register the singleton periphIndex with the peripheral tracker
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
    Lookup the index where the singleton is stored
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

  Chimera::Peripheral::Type GPIOPeriph::getType()
  {
    return sPeriphType;
  }

  size_t GPIOPeriph::getPeriphIndex( const void *const peripheralAddress )
  {
    using namespace Thor::Driver::GPIO;

    auto port = InstanceToPortMap.find( reinterpret_cast<std::uintptr_t>( peripheralAddress ) )->second;
    return static_cast<size_t>( port );
  }

  Chimera::Status_t GPIOPeriph::init()
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    return result;
  }

  Chimera::Status_t GPIOPeriph::reset( const size_t periphIndex )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    auto prrConfig = ResetConfig_GPIO[ periphIndex ];
    uint32_t tmp   = *prrConfig.reg;

    /*------------------------------------------------
    Begin the reset operation
    ------------------------------------------------*/
    tmp |= prrConfig.mask;
    *prrConfig.reg = tmp;

    /*------------------------------------------------
    Remove the reset flag as it is not cleared automatically by hardware
    ------------------------------------------------*/
    tmp &= ~prrConfig.mask;
    *prrConfig.reg = tmp;

    return result;
  }

  Chimera::Status_t GPIOPeriph::enableClock( const size_t periphIndex )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    auto ceConfig = ClockConfig_GPIO[ periphIndex ];
    *ceConfig.reg |= ceConfig.mask;

    return result;
  }

  Chimera::Status_t GPIOPeriph::disableClock( const size_t periphIndex )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    auto ceConfig = ClockConfig_GPIO[ periphIndex ];
    *ceConfig.reg &= ~ceConfig.mask;

    return result;
  }

  Chimera::Status_t GPIOPeriph::enableClockLowPower( const size_t periphIndex )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    auto celpConfig = ClockConfigLP_GPIO[ periphIndex ];
    *celpConfig.reg |= celpConfig.mask;

    return result;
  }

  Chimera::Status_t GPIOPeriph::disableClockLowPower( const size_t periphIndex )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    auto celpConfig = ClockConfigLP_GPIO[ periphIndex ];
    *celpConfig.reg &= ~celpConfig.mask;

    return result;
  }

#endif /* TARGET_STM32F4 && THOR_DRIVER_GPIO */

/*------------------------------------------------
UARTPeriph Class Implementation
------------------------------------------------*/
#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_UART == 1 )

  UARTPeriph::UARTPeriph() : iterator( 0 )
  {
    /*------------------------------------------------
    Register the singleton periphIndex with the peripheral tracker
    ------------------------------------------------*/
    auto peripheralType                  = Chimera::Peripheral::Type::UART;
    iterator                             = Thor::Driver::Mapping::PeriphTypeToIterator.find( peripheralType )->second;
    periphSingletonInstances[ iterator ] = reinterpret_cast<Peripheral *>( this );
  }

  UARTPeriph::~UARTPeriph()
  {
    free( periphSingletonInstances[ iterator ] );
    periphSingletonInstances[ iterator ] = nullptr;
  }

  Peripheral *const UARTPeriph::get()
  {
    using namespace Thor::Driver::Mapping;

    /*------------------------------------------------
    Lookup the index where the singleton is stored
    ------------------------------------------------*/
    auto iterator = PeriphTypeToIterator.find( Chimera::Peripheral::Type::UART );

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
        periphSingletonInstances[ iterator->second ] = new UARTPeriph();
      }

      return periphSingletonInstances[ iterator->second ];
    }
  }

  Chimera::Peripheral::Type UARTPeriph::getType()
  {
    return sPeriphType;
  }

  size_t UARTPeriph::getPeriphIndex( const void *const peripheralAddress )
  {
    auto lookupIndex = reinterpret_cast<std::uintptr_t>( peripheralAddress );
    return Thor::Driver::UART::InstanceToResourceIndex.find( lookupIndex )->second;
  }

  Chimera::Status_t UARTPeriph::init()
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;
    return result;
  }

  Chimera::Status_t UARTPeriph::reset( const size_t periphIndex )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    auto prrConfig = ResetConfig_UART[ periphIndex ];
    uint32_t tmp   = *prrConfig.reg;

    /*------------------------------------------------
    Begin the reset operation
    ------------------------------------------------*/
    tmp |= prrConfig.mask;
    *prrConfig.reg = tmp;

    /*------------------------------------------------
    Remove the reset flag as it is not cleared automatically by hardware
    ------------------------------------------------*/
    tmp &= ~prrConfig.mask;
    *prrConfig.reg = tmp;

    return result;
  }

  Chimera::Status_t UARTPeriph::enableClock( const size_t periphIndex )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    auto ceConfig = ClockConfig_UART[ periphIndex ];
    *ceConfig.reg |= ceConfig.mask;

    return result;
  }

  Chimera::Status_t UARTPeriph::disableClock( const size_t periphIndex )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    auto ceConfig = ClockConfig_UART[ periphIndex ];
    *ceConfig.reg &= ~ceConfig.mask;

    return result;
  }

  Chimera::Status_t UARTPeriph::enableClockLowPower( const size_t periphIndex )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    auto celpConfig = ClockConfigLP_UART[ periphIndex ];
    *celpConfig.reg |= celpConfig.mask;

    return result;
  }

  Chimera::Status_t UARTPeriph::disableClockLowPower( const size_t periphIndex )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    auto celpConfig = ClockConfigLP_UART[ periphIndex ];
    *celpConfig.reg &= ~celpConfig.mask;

    return result;
  }

#endif /* TARGET_STM32F4 && THOR_DRIVER_UART */

/*------------------------------------------------
USARTPeriph Class Implementation
------------------------------------------------*/
#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_USART == 1 )

  USARTPeriph::USARTPeriph() : iterator( 0 )
  {
    /*------------------------------------------------
    Register the singleton periphIndex with the peripheral tracker
    ------------------------------------------------*/
    auto peripheralType                  = Chimera::Peripheral::Type::USART;
    iterator                             = Thor::Driver::Mapping::PeriphTypeToIterator.find( peripheralType )->second;
    periphSingletonInstances[ iterator ] = reinterpret_cast<Peripheral *>( this );
  }

  USARTPeriph::~USARTPeriph()
  {
    free( periphSingletonInstances[ iterator ] );
    periphSingletonInstances[ iterator ] = nullptr;
  }

  Peripheral *const USARTPeriph::get()
  {
    using namespace Thor::Driver::Mapping;

    /*------------------------------------------------
    Lookup the index where the singleton is stored
    ------------------------------------------------*/
    auto iterator = PeriphTypeToIterator.find( Chimera::Peripheral::Type::USART );

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
        periphSingletonInstances[ iterator->second ] = new USARTPeriph();
      }

      return periphSingletonInstances[ iterator->second ];
    }
  }

  Chimera::Peripheral::Type USARTPeriph::getType()
  {
    return sPeriphType;
  }

  size_t USARTPeriph::getPeriphIndex( const void *const peripheralAddress )
  {
    auto lookupIndex = reinterpret_cast<std::uintptr_t>( peripheralAddress );
    return Thor::Driver::USART::InstanceToResourceIndex.find( lookupIndex )->second;
  }

  Chimera::Status_t USARTPeriph::init()
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;
    return result;
  }

  Chimera::Status_t USARTPeriph::reset( const size_t periphIndex )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    auto prrConfig = ResetConfig_USART[ periphIndex ];
    uint32_t tmp   = *prrConfig.reg;

    /*------------------------------------------------
    Begin the reset operation
    ------------------------------------------------*/
    tmp |= prrConfig.mask;
    *prrConfig.reg = tmp;

    /*------------------------------------------------
    Remove the reset flag as it is not cleared automatically by hardware
    ------------------------------------------------*/
    tmp &= ~prrConfig.mask;
    *prrConfig.reg = tmp;

    return result;
  }

  Chimera::Status_t USARTPeriph::enableClock( const size_t periphIndex )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    auto ceConfig = ClockConfig_USART[ periphIndex ];
    *ceConfig.reg |= ceConfig.mask;

    return result;
  }

  Chimera::Status_t USARTPeriph::disableClock( const size_t periphIndex )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    auto ceConfig = ClockConfig_USART[ periphIndex ];
    *ceConfig.reg &= ~ceConfig.mask;

    return result;
  }

  Chimera::Status_t USARTPeriph::enableClockLowPower( const size_t periphIndex )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    auto celpConfig = ClockConfigLP_USART[ periphIndex ];
    *celpConfig.reg |= celpConfig.mask;

    return result;
  }

  Chimera::Status_t USARTPeriph::disableClockLowPower( const size_t periphIndex )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    auto celpConfig = ClockConfigLP_USART[ periphIndex ];
    *celpConfig.reg &= ~celpConfig.mask;

    return result;
  }

#endif /* TARGET_STM32F4 && THOR_DRIVER_USART */

}    // namespace Thor::Driver::RCC

#endif /* TARGET_STM32F4 && THOR_DRIVER_RCC */