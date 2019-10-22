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
#include <Chimera/interface/clock_intf.hpp>
#include <Chimera/types/peripheral_types.hpp>

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/common/mapping/peripheral_mapping.hpp>

#include <Thor/drivers/f4/nvic/hw_nvic_driver.hpp>
#include <Thor/drivers/f4/power/hw_power_types.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_driver.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_driver_prv.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_prj.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_types.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_mapping.hpp>
#include <Thor/drivers/model/rcc_model.hpp>

#include <Thor/drivers/f4/flash/hw_flash_mapping.hpp>
#include <Thor/drivers/f4/gpio/hw_gpio_mapping.hpp>
#include <Thor/drivers/f4/power/hw_power_mapping.hpp>
#include <Thor/drivers/f4/uart/hw_uart_mapping.hpp>
#include <Thor/drivers/f4/usart/hw_usart_mapping.hpp>
#include <Thor/drivers/f4/wwdg/hw_wwdg_mapping.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_RCC == 1 )

namespace Thor::Driver::RCC
{
  /*------------------------------------------------
  Local Variables and Constants
  ------------------------------------------------*/
  static constexpr uint8_t numPeriphs = static_cast<uint8_t>( Chimera::Peripheral::Type::NUM_SUPPORTED_TYPES );

  /**
   *  Lookup table for all RCC peripheral control registers.
   */
  static std::array<const PCC *, numPeriphs> periphLookupTables;

  /*------------------------------------------------
  Local Function Declarations
  ------------------------------------------------*/
  static inline Chimera::Status_t HSEOscillatorConfig( const OscillatorInit *const init );
  static inline Chimera::Status_t HSIOscillatorConfig( const OscillatorInit *const init );
  static inline Chimera::Status_t LSIOscillatorConfig( const OscillatorInit *const init );
  static inline Chimera::Status_t LSEOsciallatorConfig( const OscillatorInit *const init );
  static Chimera::Status_t PLLOscillatorConfig( const OscillatorInit *const init );
  static void HCLKConfig( const ClockInit *const init );
  static Chimera::Status_t SYSCLKConfig( const ClockInit *const init );
  static inline Chimera::Status_t PCLK1Config( const ClockInit *init );
  static inline Chimera::Status_t PCLK2Config( const ClockInit *init );
  static Chimera::Status_t UpdateFlashLatency( const uint32_t value );
  static Chimera::Status_t OscillatorConfig( OscillatorInit *const init );
  static Chimera::Status_t ClockConfig( const ClockInit *const init );

  /*------------------------------------------------
  Default implementations of project level functions
  ------------------------------------------------*/
  WEAKDECL Chimera::Status_t prjGetHSIValue( size_t *const projectValue )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::FAIL;

    /*------------------------------------------------
    Typical value of the high speed internal oscillator in Hz
    ------------------------------------------------*/
    if ( projectValue )
    {
      *projectValue = 16000000u;
      result        = Chimera::CommonStatusCodes::OK;
    }

    return result;
  }

  WEAKDECL Chimera::Status_t prjGetHSEValue( size_t *const projectValue )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::FAIL;

    /*------------------------------------------------
    Typical value of the high speed internal oscillator in Hz
    ------------------------------------------------*/
    if ( projectValue )
    {
      *projectValue = 25000000u;
      result        = Chimera::CommonStatusCodes::OK;
    }

    return result;
  }

  WEAKDECL Chimera::Status_t prjGetLSIValue( size_t *const projectValue )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::FAIL;

    /*------------------------------------------------
    Typical value of the high speed internal oscillator in Hz
    ------------------------------------------------*/
    if ( projectValue )
    {
      *projectValue = 32000u;
      result        = Chimera::CommonStatusCodes::OK;
    }

    return result;
  }

  WEAKDECL Chimera::Status_t prjGetSysClockFreq( size_t *const projectValue )
  {
    const Chimera::Status_t prjResult = Chimera::CommonStatusCodes::OK;
    Chimera::Status_t result          = Chimera::CommonStatusCodes::FAIL;
    size_t pllm                       = 0u;
    size_t pllvco                     = 0u;
    size_t pllp                       = 0u;
    size_t sysclockfreq               = 0u;
    size_t hsiValue                   = 0u;
    size_t hseValue                   = 0u;

    if ( projectValue && ( prjGetHSIValue( &hsiValue ) == prjResult ) && ( prjGetHSEValue( &hseValue ) == prjResult ) )
    {
      switch ( CFGR::SWS::get( RCC1_PERIPH ) )
      {
        case CFGR::SWS::HSI:
          sysclockfreq = hsiValue;
          break;

        case CFGR::SWS::HSE:
          sysclockfreq = hseValue;
          break;

        case CFGR::SWS::PLL:
          pllm = PLLCFGR::M::get( RCC1_PERIPH );

          if ( PLLCFGR::SRC::get( RCC1_PERIPH ) == PLLCFGR::SRC::HSE )
          {
            pllvco = static_cast<size_t>(
                ( ( static_cast<uint64_t>( hseValue ) * ( static_cast<uint64_t>( PLLCFGR::N::get( RCC1_PERIPH ) >> PLLCFGR_PLLN_Pos ) ) ) ) /
                static_cast<uint64_t>( pllm ) );
          }
          else
          {
            pllvco = static_cast<size_t>(
                ( ( static_cast<uint64_t>( hsiValue ) * ( static_cast<uint64_t>( PLLCFGR::N::get( RCC1_PERIPH ) >> PLLCFGR_PLLN_Pos ) ) ) ) /
                static_cast<uint64_t>( pllm ) );
          }

          pllp = ( ( ( PLLCFGR::P::get( RCC1_PERIPH ) >> PLLCFGR_PLLP_Pos ) + 1U ) * 2U );

          sysclockfreq = pllvco / pllp;
          break;

        default:
          sysclockfreq = hsiValue;
          break;
      }

      *projectValue = sysclockfreq;
      result        = Chimera::CommonStatusCodes::OK;
    }

    return result;
  }

  WEAKDECL Chimera::Status_t prjGetHCLKFreq( size_t *const projectValue )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::FAIL;

    if ( projectValue )
    {
      *projectValue = SystemCoreClock;
      result        = Chimera::CommonStatusCodes::OK;
    }

    return result;
  }

  WEAKDECL Chimera::Status_t prjGetPCLK1Freq( size_t *const projectValue )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::FAIL;

    if ( projectValue )
    {
      size_t hclk = 0u;
      prjGetHCLKFreq( &hclk );

      *projectValue = hclk >> APBPrescTable[ ( RCC1_PERIPH->CFGR & CFGR_PPRE1 ) >> CFGR_PPRE1_Pos ];
      result        = Chimera::CommonStatusCodes::OK;
    }

    return result;
  }

  WEAKDECL Chimera::Status_t prjGetPCLK2Freq( size_t *const projectValue )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::FAIL;

    if ( projectValue )
    {
      size_t hclk = 0u;
      prjGetHCLKFreq( &hclk );

      *projectValue = hclk >> APBPrescTable[ ( RCC1_PERIPH->CFGR & CFGR_PPRE2 ) >> CFGR_PPRE2_Pos ];
      result        = Chimera::CommonStatusCodes::OK;
    }

    return result;
  }

  WEAKDECL Chimera::Status_t prjGetOscillatorConfig( OscillatorInit *const projectValue )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::FAIL;

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
      config.PLL.Source = PLLCFGR::SRC::HSI;
      config.PLL.M      = 8;
      config.PLL.N      = 128;
      config.PLL.P      = CR::PLLDiv::DIV2;
      config.PLL.Q      = 2;
      config.PLL.R      = 2;

      memcpy( projectValue, &config, sizeof( OscillatorInit ) );
      result = Chimera::CommonStatusCodes::OK;
    }

    return result;
  }

  WEAKDECL Chimera::Status_t prjGetClockConfig( ClockInit *const projectValue )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::FAIL;

    if ( projectValue )
    {
      ClockInit config;
      memset( &config, 0, sizeof( ClockInit ) );

      config.clock = Configuration::ClockType::HCLK | Configuration::ClockType::SYSCLK | Configuration::ClockType::PCLK1 |
                     Configuration::ClockType::PCLK2;
      config.SYSCLKSource   = CFGR::SW::PLLCLK;
      config.AHBCLKDivider  = CFGR::HPRE::DIV1;
      config.APB1CLKDivider = CFGR::PPRE1::DIV4;
      config.APB2CLKDivider = CFGR::PPRE2::DIV2;
      config.FlashLatency   = 4;

      memcpy( projectValue, &config, sizeof( ClockInit ) );
      result = Chimera::CommonStatusCodes::OK;
    }

    return result;
  }

  /*------------------------------------------------
  Standalone Functions
  ------------------------------------------------*/
  void initialize()
  {
    using namespace Chimera::Peripheral;
    using namespace Thor::Driver::RCC;

    static bool initialized = false;

    if ( !initialized )
    {
      initializeRegisters();
      initializeMapping();

      /*------------------------------------------------
      Register the lookup tables with the system
      ------------------------------------------------*/
      memset( periphLookupTables.data(), 0, sizeof( periphLookupTables ) );

#if defined( THOR_DRIVER_DMA ) && ( THOR_DRIVER_DMA == 1 )
      periphLookupTables[ static_cast<uint8_t>( Type::PERIPH_DMA ) ] = &LookupTables::DMALookup;
#endif

#if defined( THOR_DRIVER_GPIO ) && ( THOR_DRIVER_GPIO == 1 )
      periphLookupTables[ static_cast<uint8_t>( Type::PERIPH_GPIO ) ]  = &LookupTables::GPIOLookup;
#endif

#if defined( THOR_DRIVER_UART ) && ( THOR_DRIVER_UART == 1 )
      periphLookupTables[ static_cast<uint8_t>( Type::PERIPH_UART ) ] = &LookupTables::UARTLookup;
#endif

#if defined( THOR_DRIVER_USART ) && ( THOR_DRIVER_USART == 1 )
      periphLookupTables[ static_cast<uint8_t>( Type::PERIPH_USART ) ] = &LookupTables::USARTLookup;
#endif

#if defined( THOR_DRIVER_WWDG ) && ( THOR_DRIVER_WWDG == 1 )  
      periphLookupTables[ static_cast<uint8_t>( Type::PERIPH_WWDG ) ]  = &LookupTables::WWDGLookup;
#endif

      initialized = true;
    }
  }

  /*------------------------------------------------
  SystemClock Class Implementation
  ------------------------------------------------*/
  SystemClock::SystemClock()
  {
    initialize();
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

    Chimera::Status_t result          = Chimera::CommonStatusCodes::FAIL;
    const Chimera::Status_t prjResult = Chimera::CommonStatusCodes::OK;

    /*------------------------------------------------
    Turn on the main internal regulator output voltage
    ------------------------------------------------*/
    APB1ENR::PWREN::set( RCC1_PERIPH, APB1ENR::PWRENConfig::ON );

    /*------------------------------------------------
    Set the voltage scaling to allow us to achieve max clock
    ------------------------------------------------*/
    PWR::CR::VOS::set( Thor::Driver::PWR::PWR_PERIPH, PWR::CR::VOS::VOLTAGE_SCALE_1 );

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

  Chimera::Status_t SystemClock::getClockFrequency( const Configuration::ClockType_t clock, size_t *const freqHz )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::FAIL;

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
          // result = Chimera::CommonStatusCodes::FAIL;
          break;
      }
    }

    return result;
  }

  Chimera::Status_t SystemClock::getPeriphClock( const Chimera::Peripheral::Type periph, const std::uintptr_t address,
                                                 size_t *const freqHz )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::FAIL;

    auto clockLookupTable = periphLookupTables[ static_cast<uint8_t>( periph ) ]->clockSource;
    auto indexLookupTable = periphLookupTables[ static_cast<uint8_t>( periph ) ]->resourceIndexMap;

    //auto tmp = reinterpret_cast<Chimera::Container::LightFlatMap<std::uintptr_t, size_t, NUM_DMA_PERIPHS>

    if ( freqHz && clockLookupTable && indexLookupTable )
    {
      auto index       = indexLookupTable->find( address )->second;
      auto sourceClock = clockLookupTable[ index ];

      result = getClockFrequency( sourceClock, freqHz );
    }

    return result;
  }

  /*------------------------------------------------
  PeripheralController Class Implementation
  ------------------------------------------------*/
  std::shared_ptr<PeripheralController> PeripheralController::get()
  {
    struct make_shared_enabler : public PeripheralController
    {
    };
    static std::shared_ptr<make_shared_enabler> ref;

    if ( !ref )
    {
      ref = std::make_shared<make_shared_enabler>();
    }

    return ref;
  }

  PeripheralController::PeripheralController()
  {
    initialize();
  }

  PeripheralController::~PeripheralController()
  {
  }

  Chimera::Status_t PeripheralController::reset( const Chimera::Peripheral::Type type, const size_t index )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    auto lookupTable = periphLookupTables[ static_cast<uint8_t>( type ) ]->reset;
    auto config      = lookupTable[ index ];
    uint32_t tmp     = *config.reg;

    /*------------------------------------------------
    Begin the reset operation
    ------------------------------------------------*/
    tmp |= config.mask;
    *config.reg = tmp;

    /*------------------------------------------------
    Remove the reset flag as it is not cleared automatically by hardware
    ------------------------------------------------*/
    tmp &= ~config.mask;
    *config.reg = tmp;

    return result;
  }

  Chimera::Status_t PeripheralController::enableClock( const Chimera::Peripheral::Type type, const size_t index )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    auto lookupTable = periphLookupTables[ static_cast<uint8_t>( type ) ]->clock;
    auto config      = lookupTable[ index ];
    *config.reg |= config.mask;

    return result;
  }

  Chimera::Status_t PeripheralController::disableClock( const Chimera::Peripheral::Type type, const size_t index )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    auto lookupTable = periphLookupTables[ static_cast<uint8_t>( type ) ]->clock;
    auto config      = lookupTable[ index ];
    *config.reg &= ~config.mask;

    return result;
  }

  Chimera::Status_t PeripheralController::enableClockLowPower( const Chimera::Peripheral::Type type, const size_t index )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    auto lookupTable = periphLookupTables[ static_cast<uint8_t>( type ) ]->clockLP;
    auto config      = lookupTable[ index ];
    *config.reg |= config.mask;

    return result;
  }

  Chimera::Status_t PeripheralController::disableClockLowPower( const Chimera::Peripheral::Type type, const size_t index )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    auto lookupTable = periphLookupTables[ static_cast<uint8_t>( type ) ]->clockLP;
    auto config      = lookupTable[ index ];
    *config.reg &= ~config.mask;

    return result;
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

    const auto clockSource = SWS::get( RCC1_PERIPH );
    const auto pllSource   = SRC::get( RCC1_PERIPH );

    if ( ( clockSource == SWS::HSE ) || ( ( clockSource == SWS::PLL ) && ( pllSource == SRC::HSE ) ) )
    {
      /*------------------------------------------------
      When HSE is used as system clock it will not be disabled.
      ------------------------------------------------*/
      if ( ( ( HSERDY::get( RCC1_PERIPH ) == locked ) || ( HSEON::get( RCC1_PERIPH ) == enabled ) ) && ( init->HSEState == HSEConfig::OFF ) )
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
      auto tickstart = Chimera::millis();

      if ( init->HSEState != HSEConfig::OFF )
      {
        while ( ( HSERDY::get( RCC1_PERIPH ) == locked ) || ( HSEON::get( RCC1_PERIPH ) == disabled ) )
        {
          if ( ( Chimera::millis() - tickstart ) > HSE_TIMEOUT_VALUE_MS )
          {
            return Chimera::CommonStatusCodes::TIMEOUT;
          }
        }
      }
      else
      {
        while ( HSEON::get( RCC1_PERIPH ) != disabled )
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
        return Chimera::CommonStatusCodes::FAIL;
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
      auto tickstart = Chimera::millis();

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
          if ( ( Chimera::millis() - tickstart ) > HSI_TIMEOUT_VALUE_MS )
          {
            return Chimera::CommonStatusCodes::TIMEOUT;
          }
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
      LSION::set( RCC1_PERIPH, LSIConfig::ON );

      /* Wait till LSI is ready */
      while ( LSION::get( RCC1_PERIPH ) == unlocked )
      {
        if ( ( Chimera::millis() - tickstart ) > LSI_TIMEOUT_VALUE_MS )
        {
          return Chimera::CommonStatusCodes::TIMEOUT;
        }
      }
    }
    else
    {
      LSION::set( RCC1_PERIPH, LSIConfig::OFF );

      /* Wait till LSI is ready */
      while ( LSION::get( RCC1_PERIPH ) == locked )
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
    size_t tickstart = 0u;

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
    if ( !PWR::CR::DBP::get( Thor::Driver::PWR::PWR_PERIPH ) )
    {
      PWR::CR::DBP::set( Thor::Driver::PWR::PWR_PERIPH, PWR::CR_DBP );

      /* Wait for Backup domain Write protection disable */
      tickstart = Chimera::millis();

      while ( !PWR::CR::DBP::get( Thor::Driver::PWR::PWR_PERIPH ) )
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
      tickstart = Chimera::millis();

      while ( LSERDY::get( RCC1_PERIPH ) )
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

      while ( !LSERDY::get( RCC1_PERIPH ) )
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
      PWREN::set( RCC1_PERIPH, PWRENConfig::OFF );
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
      PLLON::set( RCC1_PERIPH, PLLConfig::OFF );

      while ( PLLRDY::get( RCC1_PERIPH ) )
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

        uint32_t tmp = RCC1_PERIPH->PLLCFGR;
        tmp &= ~( PLLCFGR_PLLSRC | PLLCFGR_PLLM | PLLCFGR_PLLN | PLLCFGR_PLLP | PLLCFGR_PLLQ | PLLCFGR_PLLR );
        tmp |= ( src | M | N | P | Q | R );
        RCC1_PERIPH->PLLCFGR = tmp;

        /*------------------------------------------------
        Turn on the PLL and wait for ready signal
        ------------------------------------------------*/
        tickstart = Chimera::millis();
        PLLON::set( RCC1_PERIPH, PLLConfig::ON );

        #if defined( _EMBEDDED )
        while ( !PLLRDY::get( RCC1_PERIPH ) )
        {
          if ( ( Chimera::millis() - tickstart ) > PLL_TIMEOUT_VALUE_MS )
          {
            return Chimera::CommonStatusCodes::TIMEOUT;
          }
        }
        #endif 
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
    if ( ( init->clock & Configuration::ClockType::PCLK1 ) == Configuration::ClockType::PCLK1 )
    {
      CFGR::PPRE1::set( RCC1_PERIPH, CFGR::PPRE1::DIV16 );
    }

    if ( ( init->clock & Configuration::ClockType::PCLK2 ) == Configuration::ClockType::PCLK2 )
    {
      CFGR::PPRE2::set( RCC1_PERIPH, CFGR::PPRE2::DIV16 );
    }

    CFGR::HPRE::set( RCC1_PERIPH, init->AHBCLKDivider );
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

    if ( ( sysClkSrc == CFGR::SW::HSE ) && !CR::HSERDY::get( RCC1_PERIPH ) )
    {
      return Chimera::CommonStatusCodes::FAIL;
    }
    else if ( ( ( sysClkSrc == CFGR::SW::PLLCLK ) || ( sysClkSrc == CFGR::SW::PLLRCLK ) ) && !CR::PLLRDY::get( RCC1_PERIPH ) )
    {
      return Chimera::CommonStatusCodes::FAIL;
    }
    else if ( ( sysClkSrc == CFGR::SW::HSI ) && !CR::HSIRDY::get( RCC1_PERIPH ) )
    {
      return Chimera::CommonStatusCodes::FAIL;
    }

    /*------------------------------------------------
    Configure the system clock and wait for the ready signal
    ------------------------------------------------*/
    auto tickstart = Chimera::millis();
    CFGR::SW::set( RCC1_PERIPH, init->SYSCLKSource );

    /* Assumes SW and SWS config bits mean the same thing */
    while ( CFGR::SWS::getRightShifted( RCC1_PERIPH ) != init->SYSCLKSource )
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
    CFGR::PPRE1::set( RCC1_PERIPH, init->APB1CLKDivider );
    return Chimera::CommonStatusCodes::OK;
  }

  /**
   *
   */
  static inline Chimera::Status_t PCLK2Config( const ClockInit *init )
  {
    CFGR::PPRE2::set( RCC1_PERIPH, init->APB2CLKDivider );
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
    ACR::LATENCY::set( Thor::Driver::Flash::FLASH_PERIPH, value );
    while ( ACR::LATENCY::get( Thor::Driver::Flash::FLASH_PERIPH ) != value )
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
}    // namespace Thor::Driver::RCC

#endif /* TARGET_STM32F4 && THOR_DRIVER_RCC */
