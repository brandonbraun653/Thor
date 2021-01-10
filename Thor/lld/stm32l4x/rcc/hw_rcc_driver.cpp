/********************************************************************************
 *  File Name:
 *    hw_rcc_driver_stm32l4.cpp
 *
 *  Description:
 *    RCC Low Level driver for the STM32L4 series chips
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* C++ Includes */
#include <array>
#include <cstring>
#include <cstdlib>
#include <limits>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/clock>
#include <Chimera/system>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/common/mapping/peripheral_mapping.hpp>
#include <Thor/lld/common/cortex-m4/system_time.hpp>
#include <Thor/lld/interface/interrupt/interrupt_intf.hpp>
#include <Thor/lld/interface/rcc/rcc_intf.hpp>
#include <Thor/lld/stm32l4x/flash/hw_flash_mapping.hpp>
#include <Thor/lld/stm32l4x/flash/hw_flash_prj.hpp>
#include <Thor/lld/stm32l4x/flash/hw_flash_types.hpp>
#include <Thor/lld/stm32l4x/power/hw_power_mapping.hpp>
#include <Thor/lld/stm32l4x/power/hw_power_prj.hpp>
#include <Thor/lld/stm32l4x/power/hw_power_types.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_driver.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_mapping.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_pll.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_prj.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_types.hpp>


#if defined( TARGET_STM32L4 ) && defined( THOR_LLD_RCC )

namespace Thor::LLD::RCC
{
  /*------------------------------------------------
  Local Variables and Constants
  ------------------------------------------------*/
  static constexpr uint8_t numPeriphs = static_cast<uint8_t>( Chimera::Peripheral::Type::NUM_OPTIONS );

  /* Default bad clock value. Is positive so that there aren't accidental div/0 errors at runtime. */
  static constexpr size_t BAD_CLOCK = static_cast<size_t>( 0xCCCCCCCC );

  /**
   *  Lookup table for all RCC peripheral control registers.
   */
  static std::array<PCC *, numPeriphs> periphLookupTables;

  /**
   *  Cached clock settings
   */
  static OscillatorSettings sOscillatorSettings;
  static DerivedClockSettings sDerivedClockSettings;

  /*------------------------------------------------
  Standalone Functions
  ------------------------------------------------*/
  void initialize()
  {
    using namespace Chimera::Peripheral;
    using namespace Thor::LLD::RCC;

    static bool initialized = false;

    if ( !initialized )
    {
      sOscillatorSettings.HSEConfig.frequency = std::numeric_limits<size_t>::max();
      sOscillatorSettings.LSEConfig.frequency = std::numeric_limits<size_t>::max();

      initializeRegisters();
      initializeMapping();

      /*------------------------------------------------
      Register the lookup tables with the system
      ------------------------------------------------*/
      periphLookupTables.fill( nullptr );

#if defined( THOR_LLD_ADC )
      periphLookupTables[ static_cast<uint8_t>( Type::PERIPH_ADC ) ] = &LookupTables::ADCLookup;
#endif

#if defined( THOR_LLD_CAN )
      periphLookupTables[ static_cast<uint8_t>( Type::PERIPH_CAN ) ] = &LookupTables::CANLookup;
#endif

#if defined( THOR_LLD_CRS )
      periphLookupTables[ static_cast<uint8_t>( Type::PERIPH_CRS ) ] = &LookupTables::CRSLookup;
#endif

#if defined( THOR_LLD_DMA )
      periphLookupTables[ static_cast<uint8_t>( Type::PERIPH_DMA ) ] = &LookupTables::DMALookup;
#endif

#if defined( THOR_LLD_FLASH )
      periphLookupTables[ static_cast<uint8_t>( Type::PERIPH_FLASH ) ] = &LookupTables::FLASHLookup;
#endif

#if defined( THOR_LLD_GPIO )
      periphLookupTables[ static_cast<uint8_t>( Type::PERIPH_GPIO ) ] = &LookupTables::GPIOLookup;
#endif

#if defined( THOR_LLD_IWDG )
      periphLookupTables[ static_cast<uint8_t>( Type::PERIPH_IWDG ) ] = &LookupTables::IWDGLookup;
#endif

#if defined( THOR_LLD_PWR )
      periphLookupTables[ static_cast<uint8_t>( Type::PERIPH_PWR ) ] = &LookupTables::PWRLookup;
#endif

#if defined( THOR_LLD_SPI )
      periphLookupTables[ static_cast<uint8_t>( Type::PERIPH_SPI ) ] = &LookupTables::SPILookup;
#endif

#if defined( THOR_LLD_SYSCFG )
      periphLookupTables[ static_cast<uint8_t>( Type::PERIPH_SYSCFG ) ] = &LookupTables::SYSCFGLookup;
#endif

#if defined( THOR_LLD_TIMER )
      periphLookupTables[ static_cast<uint8_t>( Type::PERIPH_TIM ) ] = &LookupTables::TIMERLookup;
#endif

#if defined( THOR_LLD_UART )
      periphLookupTables[ static_cast<uint8_t>( Type::PERIPH_UART ) ] = &LookupTables::UARTLookup;
#endif

#if defined( THOR_LLD_USART )
      periphLookupTables[ static_cast<uint8_t>( Type::PERIPH_USART ) ] = &LookupTables::USARTLookup;
#endif

#if defined( THOR_LLD_USB )
      periphLookupTables[ static_cast<uint8_t>( Type::PERIPH_USB ) ] = &LookupTables::USBLookup;
#endif

#if defined( THOR_LLD_WWDG )
      periphLookupTables[ static_cast<uint8_t>( Type::PERIPH_WWDG ) ] = &LookupTables::WWDGLookup;
#endif

      initialized = true;
    }
  }

  void clearResetReason()
  {
    RMVFRSTF::set( RCC1_PERIPH, CSR_RMVF );
  }

  Chimera::System::ResetEvent getResetReason()
  {
    /*------------------------------------------------
    Read out the flag bits and then clear them to ensure we
    get an accurate read the next time this function is called.
    ------------------------------------------------*/
    Reg32_t flags = RCC1_PERIPH->CSR & CSR_ResetFlags_Msk;
    clearResetReason();

    /*------------------------------------------------
    When debugging and powering on the board for the first time, usually there
    are two reset flags set. One is the brown out, the other is the pin reset.
    If more than just the brown out flag has been set, it's safe to mask it away
    as a false positive. This is known to happen on the STM32 development boards.
    ------------------------------------------------*/
    if ( ( flags & ResetFlags::BROWN_OUT ) && ( flags != ResetFlags::BROWN_OUT ) )
    {
      flags &= ~ResetFlags::BROWN_OUT;
    }

    switch ( flags )
    {
      case ResetFlags::CLEARED:
        return Chimera::System::ResetEvent::CLEARED;
        break;

      case ResetFlags::PIN_RESET:
        return Chimera::System::ResetEvent::HW_EXTERNAL_PIN;
        break;

      case ResetFlags::IWDG:
        return Chimera::System::ResetEvent::HW_INDEPENDENT_WATCHDOG_TIMEOUT;
        break;

      case ResetFlags::WWDG:
        return Chimera::System::ResetEvent::HW_WINDOW_WATCHDOG_TIMEOUT;
        break;

      case ResetFlags::SOFTWARE:
        return Chimera::System::ResetEvent::SOFTWARE;
        break;

      case ResetFlags::BROWN_OUT:
        return Chimera::System::ResetEvent::BROWN_OUT;
        break;

      case ResetFlags::LOW_POWER:
      case ResetFlags::OPTION_BYTE:
      case ResetFlags::FIREWALL:
        return Chimera::System::ResetEvent::UNKNOWN;
        break;

      default:
        return Chimera::System::ResetEvent::NOT_SUPPORTED;
        break;
    }
  }

  size_t getHSIFreq()
  {
    /* This value is constant across all L4 chips */
    sOscillatorSettings.HSIConfig.frequency = 16000000;
    return sOscillatorSettings.HSIConfig.frequency;
  }

  size_t getHSEFreq()
  {
    return sOscillatorSettings.HSEConfig.frequency;
  }

  void setHSEFreq( const size_t freq )
  {
    sOscillatorSettings.HSEConfig.frequency = freq;
    sOscillatorSettings.HSEConfig.applied   = true;
  }

  size_t getLSEFreq()
  {
    return sOscillatorSettings.LSEConfig.frequency;
  }

  void setLSEFreq( const size_t freq )
  {
    sOscillatorSettings.LSEConfig.frequency = freq;
    sOscillatorSettings.LSEConfig.applied   = true;
  }

  size_t getLSIFreq()
  {
    /* A 32kHz clock is common across all L4 variants */
    sOscillatorSettings.LSIConfig.frequency = 32000;
    return sOscillatorSettings.LSIConfig.frequency;
  }

  size_t getMSIFreq()
  {
    const Reg32_t msi_range_select = MSIRGSEL::get( RCC1_PERIPH );

    /*------------------------------------------------
    On powerup or reset, the MSI clock speed is configured
    from the RCC_CSR register instead of the RCC_CR register.
    ------------------------------------------------*/
    Reg32_t msiConfig = std::numeric_limits<Reg32_t>::max();
    if ( msi_range_select == CR_MSIRGSEL )
    {
      // Range is from RCC_CR
      msiConfig = MSIRANGE1::get( RCC1_PERIPH );
    }
    else
    {
      // Range is from RCC_CSR
      msiConfig = MSIRANGE2::get( RCC1_PERIPH );
    }

    /*------------------------------------------------
    Given the configured option, return the clock to the user.
    ------------------------------------------------*/
    using namespace Config;

    switch ( msiConfig )
    {
      case MSIClock::CLK_100KHZ:
        return 100000;
        break;

      case MSIClock::CLK_200KHZ:
        return 200000;
        break;

      case MSIClock::CLK_400KHZ:
        return 400000;
        break;

      case MSIClock::CLK_800KHZ:
        return 800000;
        break;

      case MSIClock::CLK_1MHZ:
        return 1000000;
        break;

      case MSIClock::CLK_2MHZ:
        return 2000000;
        break;

      case MSIClock::CLK_4MHZ:
        return 4000000;
        break;

      case MSIClock::CLK_8MHZ:
        return 8000000;
        break;

      case MSIClock::CLK_16MHZ:
        return 16000000;
        break;

      case MSIClock::CLK_24MHZ:
        return 24000000;
        break;

      case MSIClock::CLK_32MHZ:
        return 32000000;
        break;

      case MSIClock::CLK_48MHZ:
        return 48000000;
        break;
      default:
        return BAD_CLOCK;
        break;
    };
  }

  size_t getPLLCLKFreq( const uint32_t mask )
  {
    size_t outputClock = BAD_CLOCK;

    /*------------------------------------------------
    The system PLL clock is driven by PLL R. Make sure
    it's been turned on before going through all the math
    to figure out how it's been configured.
    ------------------------------------------------*/
    bool pllMainOn    = static_cast<bool>( PLLON::get( RCC1_PERIPH ) );
    bool pllR_Enabled = static_cast<bool>( PLLREN::get( RCC1_PERIPH ) );

    if ( !pllMainOn || !pllR_Enabled )
    {
      return outputClock;
    }

    /*------------------------------------------------
    Get the PLL source clock input frequency
    ------------------------------------------------*/
    size_t entryClock = 0;
    Reg32_t pllSrc    = PLLSRC::get( RCC1_PERIPH );

    switch ( pllSrc )
    {
      case PLLCFGR_PLLSRC_MSI:
        entryClock = getMSIFreq();
        break;

      case PLLCFGR_PLLSRC_HSI:
        entryClock = getHSIFreq();
        break;

      case PLLCFGR_PLLSRC_HSE:
        entryClock = getHSEFreq();
        break;

      default:
        return outputClock;
        break;
    };

    /*------------------------------------------------
    Cacluate the VCO frequency:
      VCO = PLL_Src_Clock * (PLLN / PLLM)
    ------------------------------------------------*/
    Reg32_t pdivM = ( PLLM::get( RCC1_PERIPH ) >> PLLCFGR_PLLM_Pos ) + 1;
    Reg32_t pdivN = PLLN::get( RCC1_PERIPH ) >> PLLCFGR_PLLN_Pos;

    Reg32_t VCO = entryClock * ( pdivN / pdivM );

    /*------------------------------------------------
    Calculate the output PLL clock
    ------------------------------------------------*/
    // PLL P divisor. Only two options are available, 17 or 7
    Reg32_t pdivP = PLLP::get( RCC1_PERIPH ) ? 17 : 7;

    // PLL Q or R divisor: Shifted to zero then quickly converted to the numerical
    //                     equivalents of the configuration bits as described in the
    //                     device datasheet. Pretty simple.
    Reg32_t pdivQ = ( ( PLLQ::get( RCC1_PERIPH ) >> PLLCFGR_PLLR_Pos ) + 1 ) * 2;
    Reg32_t pdivR = ( ( PLLR::get( RCC1_PERIPH ) >> PLLCFGR_PLLR_Pos ) + 1 ) * 2;

    // No need to do div/0 checks here as all possible values are positive.
    //  PLLP: Only two options are positive
    //  PLLQ/R: Even if the register value is zero, the +1 offset mitigates this
    switch ( mask )
    {
      case PLLCFGR_PLLP:
        outputClock = VCO / pdivP;
        break;

      case PLLCFGR_PLLQ:
        outputClock = VCO / pdivQ;
        break;

      case PLLCFGR_PLLR:
        outputClock = VCO / pdivR;
        break;

      default:
        outputClock = BAD_CLOCK;
        break;
    };

    return outputClock;
  }

  bool updatePLL( const uint32_t mask, OscillatorSettings &config )
  {
    auto rcc    = getCoreClock();
    auto result = false;

    /*------------------------------------------------
    Disable ISR handling so that other systems won't be
    totally corrupted by the clock switching.
    ------------------------------------------------*/
    auto isrMask = Thor::LLD::IT::disableInterrupts();

    /*------------------------------------------------
    If the current system clock is derived from the PLL, switch
    to the HSI16 clock so that something is driving the ARM core.
    ------------------------------------------------*/
    bool PLLIsSysClockSource = ( rcc->getCoreClockSource() == Chimera::Clock::Bus::PLLCLK );
    if ( PLLIsSysClockSource )
    {
      rcc->enableClock( Chimera::Clock::Bus::HSI16 );
      rcc->setCoreClockSource( Chimera::Clock::Bus::HSI16 );
    }

    /*------------------------------------------------
    Handle the configuration of each PLL output
    ------------------------------------------------*/
    switch ( mask )
    {
      case PLLCFGR_PLLP:
        result = PLLConfigureP( config );
        break;

      case PLLCFGR_PLLQ:
        result = PLLConfigureQ( config );
        break;

      case PLLCFGR_PLLR:
        result = PLLConfigureR( config );
        break;

      default:
        // Do nothing. The clocks will go back to their original settings.
        break;
    };

    /*------------------------------------------------
    Switch back to the PLL source if necessary. Turn off
    the HSI16 clock to save a bit of power.
    ------------------------------------------------*/
    if ( PLLIsSysClockSource )
    {
      rcc->enableClock( Chimera::Clock::Bus::PLLCLK );
      rcc->setCoreClockSource( Chimera::Clock::Bus::PLLCLK );
      rcc->disableClock( Chimera::Clock::Bus::HSI16 );
    }

    /*------------------------------------------------
    Re-enable the ISRs, allowing the system to update itself
    in response to the new clock configurations.
    ------------------------------------------------*/
    Thor::LLD::IT::enableInterrupts( isrMask );

    return result;
  }

  size_t getSysClockFreq()
  {
    /*------------------------------------------------
    From the clock tree diagram in RM0394 Fig. 13, there are
    only four possible clock sources for the System Clock.
    ------------------------------------------------*/
    switch ( SWS::get( RCC1_PERIPH ) )
    {
      case Config::SystemClockStatus::SYSCLK_HSE:
        return getHSEFreq();
        break;

      case Config::SystemClockStatus::SYSCLK_HSI16:
        return getHSIFreq();
        break;

      case Config::SystemClockStatus::SYSCLK_MSI:
        return getMSIFreq();
        break;

      case Config::SystemClockStatus::SYSCLK_PLL:
        return getPLLCLKFreq( PLLCFGR_PLLR );
        break;

      default:
        return std::numeric_limits<size_t>::max();
        break;
    };
  }

  size_t getHCLKFreq()
  {
    size_t hclkDiv                 = 1;
    size_t systemClock             = getSysClockFreq();
    Reg32_t ahbPrescalerConfigBits = HPRE::get( RCC1_PERIPH );

    switch ( ahbPrescalerConfigBits )
    {
      case CFGR_HPRE_DIV1:
        hclkDiv = 1;
        break;

      case CFGR_HPRE_DIV2:
        hclkDiv = 2;
        break;

      case CFGR_HPRE_DIV4:
        hclkDiv = 4;
        break;

      case CFGR_HPRE_DIV8:
        hclkDiv = 8;
        break;

      case CFGR_HPRE_DIV16:
        hclkDiv = 16;
        break;

      case CFGR_HPRE_DIV64:
        hclkDiv = 64;
        break;

      case CFGR_HPRE_DIV128:
        hclkDiv = 128;
        break;

      case CFGR_HPRE_DIV256:
        hclkDiv = 256;
        break;

      case CFGR_HPRE_DIV512:
        hclkDiv = 512;
        break;

      default:
        hclkDiv = 1;
        break;
    }

    return systemClock / hclkDiv;
  }

  size_t getPCLK1Freq()
  {
    /*------------------------------------------------
    According to the clock tree diagram, PCLK1 is derived
    from HCLK bus using the APB1 divisor.
    ------------------------------------------------*/
    size_t pclk1Div               = 1;
    size_t hclkFreq               = getHCLKFreq();
    size_t apbPrescalerConfigBits = PPRE1::get( RCC1_PERIPH );

    switch ( apbPrescalerConfigBits )
    {
      case CFGR_PPRE1_DIV1:
        pclk1Div = 1;
        break;

      case CFGR_PPRE1_DIV2:
        pclk1Div = 2;
        break;

      case CFGR_PPRE1_DIV4:
        pclk1Div = 4;
        break;

      case CFGR_PPRE1_DIV8:
        pclk1Div = 8;
        break;

      case CFGR_PPRE1_DIV16:
        pclk1Div = 16;
        break;

      default:
        pclk1Div = 1;
        break;
    }

    return hclkFreq / pclk1Div;
  }

  size_t getPCLK2Freq()
  {
    /*------------------------------------------------
    According to the clock tree diagram, PCLK1 is derived
    from HCLK bus using the APB1 divisor.
    ------------------------------------------------*/
    size_t pclk2Div               = 1;
    size_t hclkFreq               = getHCLKFreq();
    size_t apbPrescalerConfigBits = PPRE2::get( RCC1_PERIPH );

    switch ( apbPrescalerConfigBits )
    {
      case CFGR_PPRE2_DIV1:
        pclk2Div = 1;
        break;

      case CFGR_PPRE2_DIV2:
        pclk2Div = 2;
        break;

      case CFGR_PPRE2_DIV4:
        pclk2Div = 4;
        break;

      case CFGR_PPRE2_DIV8:
        pclk2Div = 8;
        break;

      case CFGR_PPRE2_DIV16:
        pclk2Div = 16;
        break;

      default:
        pclk2Div = 1;
        break;
    }

    return hclkFreq / pclk2Div;
  }


  /*------------------------------------------------
  SystemClock Class Implementation
  ------------------------------------------------*/
  SystemClock *getCoreClock()
  {
    static SystemClock *ref = nullptr;
    if ( ref == nullptr )
    {
      ref = new SystemClock();
    }

    return ref;
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
    auto isrMask = Thor::LLD::IT::disableInterrupts();

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

      case Chimera::Clock::Bus::PLLCLK:
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

    Thor::LLD::IT::enableInterrupts( isrMask );
  }

  void SystemClock::disableClock( const Chimera::Clock::Bus clock )
  {
    auto isrMask = Thor::LLD::IT::disableInterrupts();

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

      case Chimera::Clock::Bus::PLLCLK:
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

    Thor::LLD::IT::enableInterrupts( isrMask );
  }

  Chimera::Status_t SystemClock::configureProjectClocks()
  {
    using namespace Thor::LLD::PWR;

    /*------------------------------------------------
    Turn on the clock to the power controller peripheral and
    set the voltage scaling to allow us to achieve max clock
    ------------------------------------------------*/
    PWREN::set( RCC1_PERIPH, APB1ENR1_PWREN );
    VOS::set( POWER1_PERIPH, CR1::VOS_SCALE_1 );

    /*------------------------------------------------
    Configure the source clocks (oscillators)
      PLL P: off
      PLL Q: 40 MHz
      PLL R: 80 MHz
    ------------------------------------------------*/
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

    /*------------------------------------------------
    Configure the derived clocks
    ------------------------------------------------*/
    sDerivedClockSettings = {};

    sDerivedClockSettings.HCLKConfig.configure      = true;
    sDerivedClockSettings.HCLKConfig.AHBPrescaler   = Config::CFGR::AHB_SYS_DIV_1;
    sDerivedClockSettings.PCLK1Config.configure     = true;
    sDerivedClockSettings.PCLK1Config.APB1Prescaler = Config::CFGR::APB1_AHB_DIV_1;
    sDerivedClockSettings.PCLK2Config.configure     = true;
    sDerivedClockSettings.PCLK2Config.APB2Prescaler = Config::CFGR::APB2_AHB_DIV_1;

    sDerivedClockSettings.valid = true;

    /*------------------------------------------------
    Apply the settings
    ------------------------------------------------*/
    if ( configureOscillators( sOscillatorSettings ) && configureClocks( sDerivedClockSettings ) )
    {
      setCoreClockSource( Chimera::Clock::Bus::PLLCLK );
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
    /*------------------------------------------------
    Prevent the clock selection update from being interrupted
    ------------------------------------------------*/
    auto itMask = Thor::LLD::IT::disableInterrupts();

    /*------------------------------------------------
    Figure out the configuration bits that should be set. Go
    ahead and update the SystemCoreClock value. If the update
    fails, we'll be stuck in the while loop anyways and it won't
    matter that the variable has been set to a bad value.
    ------------------------------------------------*/
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

      case Chimera::Clock::Bus::PLLCLK:
        cfgOption = Config::SystemClockSelect::SYSCLK_PLL;
        expStatus = Config::SystemClockStatus::SYSCLK_PLL;

        CortexM4::Clock::updateCoreClockCache( getPLLCLKFreq( PLLCFGR_PLLR ) );
        break;

      default:
        return Chimera::Status::FAIL;
        break;
    }

    /*------------------------------------------------
    Adjust the flash read access latency (Section 3.3.3 of RM0394)

    Note: Currently harcoded to assume a clock increase, but once
    I have the processor brought up and have some free time, this
    needs to adjust for a decrease too. Can calculate the desired
    clock frequency from the registers in the config structure.
    ------------------------------------------------*/
    using namespace Thor::LLD::FLASH;
    LATENCY::set( FLASH_PERIPH, ACR_LATENCY_4WS );

    /*------------------------------------------------
    Apply the clock selection setting, then wait for the
    hardware to indicate it has stabilized.
    ------------------------------------------------*/
    SW::set( RCC1_PERIPH, cfgOption );
    while ( ( SWS::get( RCC1_PERIPH ) & expStatus ) != expStatus )
    {
      ;
    }

    /*------------------------------------------------
    The clock is stable now, allow normal program execution
    ------------------------------------------------*/
    Thor::LLD::IT::enableInterrupts( itMask );
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
        return Chimera::Clock::Bus::PLLCLK;
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
        return BAD_CLOCK;
        break;
    }
  }

  size_t SystemClock::getPeriphClock( const Chimera::Peripheral::Type periph, const std::uintptr_t address )
  {
    auto clockLookupTable = periphLookupTables[ static_cast<uint8_t>( periph ) ]->clockSource;
    auto indexLookupFunc  = periphLookupTables[ static_cast<uint8_t>( periph ) ]->getResourceIndex;

    if ( clockLookupTable && indexLookupFunc )
    {
      auto index       = indexLookupFunc( address );
      auto sourceClock = clockLookupTable[ index ];

      return getClockFrequency( sourceClock );
    }
    else
    {
      return BAD_CLOCK;
    }
  }

  bool SystemClock::configureOscillators( OscillatorSettings &cfg )
  {
    /*------------------------------------------------
    Make sure we are supposed to allow configuration of these settings
    ------------------------------------------------*/
    if ( !cfg.valid )
    {
      return false;
    }

    auto isrMask = Thor::LLD::IT::disableInterrupts();

    /*------------------------------------------------
    Configure the HSE oscillator
    ------------------------------------------------*/
    if ( cfg.HSEConfig.configure )
    {
      // TODO once needed
    }

    /*------------------------------------------------
    Configure the HSI oscillator
    ------------------------------------------------*/
    if ( cfg.HSIConfig.configure )
    {
      // TODO once needed
    }

    /*------------------------------------------------
    Configure the LSE oscillator
    ------------------------------------------------*/
    if ( cfg.LSEConfig.configure )
    {
      // TODO once needed
    }

    /*------------------------------------------------
    Configure the LSI oscillator
    ------------------------------------------------*/
    if ( cfg.LSIConfig.configure )
    {
      // TODO once needed
    }

    /*------------------------------------------------
    Configure the MSI oscillator
    ------------------------------------------------*/
    if ( cfg.MSIConfig.configure )
    {
      // TODO once needed
    }

    /*------------------------------------------------
    Configure the PLL oscillator
    ------------------------------------------------*/
    if ( cfg.PLLConfig.configure )
    {
      if ( cfg.PLLConfig.P.configure )
      {
        updatePLL( PLLCFGR_PLLP, cfg );
        cfg.PLLConfig.P.configure = false;
      }

      if ( cfg.PLLConfig.Q.configure )
      {
        updatePLL( PLLCFGR_PLLQ, cfg );
        cfg.PLLConfig.Q.configure = false;
      }

      if ( cfg.PLLConfig.R.configure )
      {
        updatePLL( PLLCFGR_PLLR, cfg );
        cfg.PLLConfig.R.configure = false;
      }
    }

    Thor::LLD::IT::enableInterrupts( isrMask );
    return true;
  }

  bool SystemClock::configureClocks( DerivedClockSettings &cfg )
  {
    /*------------------------------------------------
    Make sure we are supposed to allow configuration of these settings
    ------------------------------------------------*/
    if ( !cfg.valid )
    {
      return false;
    }

    auto isrMask = Thor::LLD::IT::disableInterrupts();

    /*------------------------------------------------
    Update the HCLK divisor
    ------------------------------------------------*/
    if ( cfg.HCLKConfig.configure )
    {
      HPRE::set( RCC1_PERIPH, cfg.HCLKConfig.AHBPrescaler );

      /* Wait until the value is set appropriately before continuing */
      while ( HPRE::get( RCC1_PERIPH ) != cfg.HCLKConfig.AHBPrescaler )
      {
        ;
      }

      cfg.HCLKConfig.applied   = true;
      cfg.HCLKConfig.configure = false;
    }

    /*------------------------------------------------
    Update the PCLK1 divisor
    ------------------------------------------------*/
    if ( cfg.PCLK1Config.configure )
    {
      PPRE1::set( RCC1_PERIPH, cfg.PCLK1Config.APB1Prescaler );

      /* Wait until the value is set appropriately before continuing */
      while ( PPRE1::get( RCC1_PERIPH ) != cfg.PCLK1Config.APB1Prescaler )
      {
        ;
      }

      cfg.PCLK1Config.applied   = true;
      cfg.PCLK1Config.configure = false;
    }

    /*------------------------------------------------
    Update the PCLK2 divisor
    ------------------------------------------------*/
    if ( cfg.PCLK2Config.configure )
    {
      PPRE2::set( RCC1_PERIPH, cfg.PCLK2Config.APB2Prescaler );

      /* Wait until the value is set appropriately before continuing */
      while ( PPRE2::get( RCC1_PERIPH ) != cfg.PCLK2Config.APB2Prescaler )
      {
        ;
      }

      cfg.PCLK2Config.applied   = true;
      cfg.PCLK2Config.configure = false;
    }

    Thor::LLD::IT::enableInterrupts( isrMask );
    return true;
  }


  /*------------------------------------------------
  PeripheralController Class Implementation
  ------------------------------------------------*/
  PeripheralController *getPeripheralClock()
  {
    static PeripheralController *ref = nullptr;

    if ( !ref )
    {
      ref = new PeripheralController();
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
    Chimera::Status_t result = Chimera::Status::OK;

    auto lookupTable = periphLookupTables[ static_cast<uint8_t>( type ) ]->reset;
    auto config      = lookupTable[ index ];

    if ( config.reg )
    {
      uint32_t tmp = *config.reg;

      /*------------------------------------------------
      Begin the reset operation
      ------------------------------------------------*/
      tmp |= config.mask;
      *config.reg = tmp;

      /*-------------------------------------------------
      Burn a few cycles to avoid accessing HW too early
      -------------------------------------------------*/
      for ( auto x = 0; x < 5; x++ )
      {
        asm( "nop" );
      }

      /*------------------------------------------------
      Remove the reset flag as it is not cleared automatically by hardware
      ------------------------------------------------*/
      tmp &= ~config.mask;
      *config.reg = tmp;
    }
    else
    {
      result = Chimera::Status::NOT_SUPPORTED;
    }

    return result;
  }

  Chimera::Status_t PeripheralController::enableClock( const Chimera::Peripheral::Type type, const size_t index )
  {
    Chimera::Status_t result = Chimera::Status::OK;

    auto lookupTable = periphLookupTables[ static_cast<uint8_t>( type ) ]->clock;
    auto config      = lookupTable[ index ];

    if( config.reg )
    {
      *config.reg |= config.mask;
    }
    else
    {
      result = Chimera::Status::NOT_SUPPORTED;
    }

    return result;
  }

  Chimera::Status_t PeripheralController::disableClock( const Chimera::Peripheral::Type type, const size_t index )
  {
    Chimera::Status_t result = Chimera::Status::OK;

    auto lookupTable = periphLookupTables[ static_cast<uint8_t>( type ) ]->clock;
    auto config      = lookupTable[ index ];

    if( config.reg )
    {
      *config.reg &= ~config.mask;
    }
    else
    {
      result = Chimera::Status::NOT_SUPPORTED;
    }

    return result;
  }

  Chimera::Status_t PeripheralController::enableClockLowPower( const Chimera::Peripheral::Type type, const size_t index )
  {
    Chimera::Status_t result = Chimera::Status::OK;

    auto lookupTable = periphLookupTables[ static_cast<uint8_t>( type ) ]->clockLP;
    auto config      = lookupTable[ index ];

    if( config.reg )
    {
      *config.reg |= config.mask;
    }
    else
    {
      result = Chimera::Status::NOT_SUPPORTED;
    }

    return result;
  }

  Chimera::Status_t PeripheralController::disableClockLowPower( const Chimera::Peripheral::Type type, const size_t index )
  {
    Chimera::Status_t result = Chimera::Status::OK;

    auto lookupTable = periphLookupTables[ static_cast<uint8_t>( type ) ]->clockLP;
    auto config      = lookupTable[ index ];

    if( config.reg )
    {
      *config.reg &= ~config.mask;
    }
    else
    {
      result = Chimera::Status::NOT_SUPPORTED;
    }

    return result;
  }
}    // namespace Thor::LLD::RCC

#endif /* TARGET_STM32L4 && THOR_LLD_RCC */
