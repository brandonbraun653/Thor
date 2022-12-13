/******************************************************************************
 *  File Name:
 *    hw_rcc_driver_stm32l4.cpp
 *
 *  Description:
 *    RCC Low Level driver for the STM32L4 series chips
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

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
#include <Thor/lld/common/cortex-m4/system_time.hpp>
#include <Thor/lld/interface/inc/rcc>
#include <Thor/lld/interface/inc/power>
#include <Thor/lld/interface/inc/interrupt>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_pll.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_prv.hpp>


#if defined( TARGET_STM32L4 )

namespace Thor::LLD::RCC
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void initialize()
  {
    using namespace Chimera::Peripheral;
    using namespace Thor::LLD::RCC;

    static bool initialized = false;

    if ( !initialized )
    {
      sOscillatorSettings.HSEConfig.frequency = std::numeric_limits<size_t>::max();
      sOscillatorSettings.LSEConfig.frequency = std::numeric_limits<size_t>::max();
      initializeRegistry();

      initialized = true;
    }
  }

  size_t getBusFrequency( const Chimera::Clock::Bus bus )
  {
    switch ( bus )
    {
      case Chimera::Clock::Bus::HSI16:
        return 16000000u;
        break;

      case Chimera::Clock::Bus::LSI:
        return 32000u;
        break;

      case Chimera::Clock::Bus::HSE:
      case Chimera::Clock::Bus::LSE:
        return getExtOscFreq( bus );
        break;

      default:
        return getCoreClockCtrl()->getClockFrequency( bus );
        break;
    }
  }


  void clearResetReason()
  {
    RMVFRSTF::set( RCC1_PERIPH, CSR_RMVF );
  }


  Chimera::System::ResetEvent getResetReason()
  {
    /*-------------------------------------------------------------------------
    Read out the flag bits and then clear them to ensure we
    get an accurate read the next time this function is called.
    -------------------------------------------------------------------------*/
    Reg32_t flags = RCC1_PERIPH->CSR & CSR_ResetFlags_Msk;
    clearResetReason();

    /*-------------------------------------------------------------------------
    When debugging and powering on the board for the first time, usually there
    are two reset flags set. One is the brown out, the other is the pin reset.
    If more than just the brown out flag has been set, it's safe to mask it away
    as a false positive. This is known to happen on the STM32 development boards.
    -------------------------------------------------------------------------*/
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

    /*-------------------------------------------------------------------------
    On powerup or reset, the MSI clock speed is configured
    from the RCC_CSR register instead of the RCC_CR register.
    -------------------------------------------------------------------------*/
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

    /*-------------------------------------------------------------------------
    Given the configured option, return the clock to the user.
    -------------------------------------------------------------------------*/
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
        return INVALID_CLOCK;
        break;
    };
  }


  size_t getPLLCLKFreq( const uint32_t mask )
  {
    size_t outputClock = INVALID_CLOCK;

    /*-------------------------------------------------------------------------
    The system PLL clock is driven by PLL R. Make sure
    it's been turned on before going through all the math
    to figure out how it's been configured.
    -------------------------------------------------------------------------*/
    bool pllMainOn    = static_cast<bool>( PLLON::get( RCC1_PERIPH ) );
    bool pllR_Enabled = static_cast<bool>( PLLREN::get( RCC1_PERIPH ) );

    if ( !pllMainOn || !pllR_Enabled )
    {
      return outputClock;
    }

    /*-------------------------------------------------------------------------
    Get the PLL source clock input frequency
    -------------------------------------------------------------------------*/
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

    /*-------------------------------------------------------------------------
    Cacluate the VCO frequency:
      VCO = PLL_Src_Clock * (PLLN / PLLM)
    -------------------------------------------------------------------------*/
    Reg32_t pdivM = ( PLLM::get( RCC1_PERIPH ) >> PLLCFGR_PLLM_Pos ) + 1;
    Reg32_t pdivN = PLLN::get( RCC1_PERIPH ) >> PLLCFGR_PLLN_Pos;

    Reg32_t VCO = entryClock * ( pdivN / pdivM );

    /*-------------------------------------------------------------------------
    Calculate the output PLL clock
    -------------------------------------------------------------------------*/
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
        outputClock = INVALID_CLOCK;
        break;
    };

    return outputClock;
  }


  size_t getSysClockFreq()
  {
    /*-------------------------------------------------------------------------
    From the clock tree diagram in RM0394 Fig. 13, there are
    only four possible clock sources for the System Clock.
    -------------------------------------------------------------------------*/
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
    /*-------------------------------------------------------------------------
    According to the clock tree diagram, PCLK1 is derived
    from HCLK bus using the APB1 divisor.
    -------------------------------------------------------------------------*/
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
    /*-------------------------------------------------------------------------
    According to the clock tree diagram, PCLK1 is derived
    from HCLK bus using the APB1 divisor.
    -------------------------------------------------------------------------*/
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

}    // namespace Thor::LLD::RCC

#endif /* TARGET_STM32L4 && THOR_LLD_RCC */
