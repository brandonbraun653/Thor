/********************************************************************************
 *  File Name:
 *    hw_rcc_prv.cpp
 *
 *  Description:
 *    Private function implementation
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Chimera Includes */
#include <Chimera/assert>
#include <Chimera/clock>
#include <Chimera/system>

/* Thor Includes */
#include <Thor/lld/interface/inc/rcc>
#include <Thor/lld/stm32f4x/rcc/hw_rcc_prv.hpp>

namespace Thor::LLD::RCC
{
  /*-------------------------------------------------------------------------------
  Private Functions
  -------------------------------------------------------------------------------*/
  bool enableHSI()
  {
    bool enabled = HSION::get( RCC1_PERIPH ) && HSIRDY::get( RCC1_PERIPH );

    if ( !enabled )
    {
      HSION::set( RCC1_PERIPH, CR_HSION );
      while ( !HSIRDY::get( RCC1_PERIPH ) )
      {
        continue;
      }

      enabled = true;
    }

    return enabled;
  }


  void disableHSI()
  {
    HSION::clear( RCC1_PERIPH, CR_HSION );
    while ( HSIRDY::get( RCC1_PERIPH ) )
    {
      continue;
    }
  }


  bool enableLSI()
  {
    bool enabled = LSION::get( RCC1_PERIPH ) && LSIRDY::get( RCC1_PERIPH );

    if ( !enabled )
    {
      LSION::set( RCC1_PERIPH, CSR_LSION );
      while ( !LSIRDY::get( RCC1_PERIPH ) )
      {
        continue;
      }

      enabled = true;
    }

    return enabled;
  }


  void disableLSI()
  {
    LSION::clear( RCC1_PERIPH, CSR_LSION );
    while ( LSIRDY::get( RCC1_PERIPH ) )
    {
      continue;
    }
  }


  bool select_system_clock_source( Chimera::Clock::Bus src )
  {
    using namespace Chimera::Clock;

    /*-------------------------------------------------
    Select the new clock source
    -------------------------------------------------*/
    uint32_t waitFlag = 0;
    uint32_t bitFlag  = 0;
    switch ( src )
    {
      case Bus::HSI16:
        bitFlag  = CFGR_SW_HSI;
        waitFlag = CFGR_SWS_HSI;
        break;

      case Bus::HSE:
        bitFlag  = CFGR_SW_HSE;
        waitFlag = CFGR_SWS_HSE;
        break;

      case Bus::PLLP:
        bitFlag  = CFGR_SW_PLL;
        waitFlag = CFGR_SWS_PLL;
        break;

      case Bus::PLLR:
        bitFlag  = CFGR_SW_PLLR;
        waitFlag = CFGR_SWS_PLLR;
        break;

      default:
        return false;
        break;
    };

    /*-------------------------------------------------
    Already set to the correct source?
    -------------------------------------------------*/
    if ( ( SW::get( RCC1_PERIPH ) == bitFlag ) && ( SWS::get( RCC1_PERIPH ) == waitFlag ) )
    {
      return true;
    }

    /*-------------------------------------------------
    Prevent execution of other system code while the
    core clock is changing.
    -------------------------------------------------*/
    auto isrMask = Chimera::System::disableInterrupts();
    SW::set( RCC1_PERIPH, bitFlag );

    /*-------------------------------------------------
    Wait for the selection to take effect
    -------------------------------------------------*/
    while ( ( SWS::get( RCC1_PERIPH ) & waitFlag ) != waitFlag )
    {
      continue;
    }

    /*-------------------------------------------------
    Re-allow external code to run
    -------------------------------------------------*/
    Chimera::System::enableInterrupts( isrMask );
    return true;
  }


  bool select_pll_clock_source( Chimera::Clock::Bus src )
  {
    using namespace Chimera::Clock;

    /*-------------------------------------------------
    Don't change the source if the PLL is running
    -------------------------------------------------*/
    if ( PLLRDY::get( RCC1_PERIPH ) )
    {
      return false;
    }

    /*-------------------------------------------------
    Decide the mask bits to set
    -------------------------------------------------*/
    uint32_t waitFlag = 0;
    switch ( src )
    {
      case Bus::HSI16:
        waitFlag = 0;    // PLLSRC is cleared
        break;

      case Bus::HSE:
        waitFlag = PLLCFGR_PLLSRC;
        break;

      default:
        return false;
        break;
    };

    /*-------------------------------------------------
    Prevent execution of other system code while the
    core clock is changing.
    -------------------------------------------------*/
    auto isrMask = Chimera::System::disableInterrupts();
    PLLSRC::set( RCC1_PERIPH, waitFlag );

    /*-------------------------------------------------
    Wait for the selection to take effect
    -------------------------------------------------*/
    while ( ( PLLSRC::get( RCC1_PERIPH ) & waitFlag ) != waitFlag )
    {
      continue;
    }

    /*-------------------------------------------------
    Re-allow external code to run
    -------------------------------------------------*/
    Chimera::System::enableInterrupts( isrMask );
    return true;
  }

  /*-------------------------------------------------------------------------------
  Clock Muxing
  -------------------------------------------------------------------------------*/
  Chimera::Clock::Bus getSysClockSource()
  {
    using namespace Chimera::Clock;

    switch ( SWS::get( RCC1_PERIPH ) )
    {
      case CFGR_SWS_HSE:
        return Bus::HSE;
        break;

      case CFGR_SWS_HSI:
        return Bus::HSI16;
        break;

      case CFGR_SWS_PLL:
        return Bus::PLLP;
        break;

      case CFGR_SWS_PLLR:
        return Bus::PLLR;
        break;

      default:
        return Bus::UNKNOWN_BUS;
        break;
    };
  }


  Chimera::Clock::Bus getPLLClockSource()
  {
    using namespace Chimera::Clock;

    switch ( PLLSRC::get( RCC1_PERIPH ) )
    {
      case PLLCFGR_PLLSRC_HSE:
        return Bus::HSE;
        break;

      case PLLCFGR_PLLSRC_HSI:
        return Bus::HSI16;
        break;

      default:
        return Bus::UNKNOWN_BUS;
        break;
    };
  }


  /*-------------------------------------------------------------------------------
  Runtime Bus Frequency Calculations
  -------------------------------------------------------------------------------*/
  size_t getSystemClock()
  {
    using namespace Chimera::Clock;

    switch ( getSysClockSource() )
    {
      case Bus::HSE:
        // Constant, so recursive lookup is ok
        return getBusFrequency( Bus::HSE );
        break;

      case Bus::HSI16:
        // Constant, so recursive lookup is ok
        return getBusFrequency( Bus::HSI16 );
        break;

      case Bus::PLLP:
        return getPLLClock( PLLOut::P );
        break;

      case Bus::PLLR:
        return getPLLClock( PLLOut::R );
        break;

      default:
        return INVALID_CLOCK;
        break;
    };
  }


  size_t getHCLKFreq()
  {
    size_t hclkDiv                 = 1;
    size_t systemClock             = getSystemClock();
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


  size_t getPLLClock( const PLLOut which )
  {
    using namespace Chimera::Clock;

    /*-------------------------------------------------
    Determine the PLL input base frequency
    -------------------------------------------------*/
    size_t input_freq;
    switch ( getPLLClockSource() )
    {
      case Bus::HSE:
        input_freq = getBusFrequency( Bus::HSE );
        break;

      case Bus::HSI16:
        input_freq = getBusFrequency( Bus::HSI16 );
        break;

      default:
        return INVALID_CLOCK;
        break;
    };

    /*-------------------------------------------------
    Calculate the VCO frequency
    -------------------------------------------------*/
    uint32_t m = PLLM::get( RCC1_PERIPH ) >> PLLCFGR_PLLM_Pos;
    RT_HARD_ASSERT( m );

    uint32_t n = PLLN::get( RCC1_PERIPH ) >> PLLCFGR_PLLN_Pos;
    RT_HARD_ASSERT( n );

    uint32_t vco_freq = ( input_freq / m ) * n;

    /*-------------------------------------------------
    Calculate the PLL output frequency
    -------------------------------------------------*/
    if ( which == PLLOut::P )
    {
      uint32_t p = PLLP::get( RCC1_PERIPH ) >> PLLCFGR_PLLP_Pos;
      switch( p )
      {
        case 0:
          p = 2;
          break;

        case 1:
          p = 4;
          break;

        case 2:
          p = 6;
          break;

        case 3:
          p = 8;
          break;

        default:
          RT_HARD_ASSERT( false );
          break;
      };

      RT_HARD_ASSERT( p );
      return vco_freq / p;
    }
    else if ( which == PLLOut::R )
    {
      uint32_t r = PLLR::get( RCC1_PERIPH ) >> PLLCFGR_PLLR_Pos;
      RT_HARD_ASSERT( r );
      return vco_freq / r;
    }
    else
    {
      return INVALID_CLOCK;
    }
  }

  /*-------------------------------------------------------------------------------
  Oscillator Configuration
  -------------------------------------------------------------------------------*/
  bool configureHSE( ClockTreeInit &cfg )
  {
    if ( cfg.enabled.hse )
    {
      HSEON::set( RCC1_PERIPH, CR_HSEON );
      while ( !HSERDY::get( RCC1_PERIPH ) )
      {
        continue;
      }
    }
    else
    {
      HSEON::clear( RCC1_PERIPH, CR_HSEON );
      while ( HSERDY::get( RCC1_PERIPH ) )
      {
        continue;
      }
    }

    return true;
  }


  bool configureHSI( ClockTreeInit &cfg )
  {
    if ( cfg.enabled.hsi )
    {
      return enableHSI();
    }
    else
    {
      disableHSI();
    }

    return true;
  }


  bool configureLSE( ClockTreeInit &cfg )
  {
    if ( cfg.enabled.lse )
    {
      LSEON::set( RCC1_PERIPH, BDCR_LSEON );
      while ( !LSERDY::get( RCC1_PERIPH ) )
      {
        continue;
      }
    }
    else
    {
      LSEON::clear( RCC1_PERIPH, BDCR_LSEON );
      while ( LSERDY::get( RCC1_PERIPH ) )
      {
        continue;
      }
    }

    return true;
  }


  bool configureLSI( ClockTreeInit &cfg )
  {
    if ( cfg.enabled.lsi )
    {
      return enableLSI();
    }
    else
    {
      disableLSI();
    }

    return true;
  }


  /*-------------------------------------------------------------------------------
  Clock Source Input Selection
  -------------------------------------------------------------------------------*/
  bool setSourcePLL( ClockTreeInit &cfg )
  {
    return select_pll_clock_source( cfg.mux.pll );
  }


  bool setSourceSYS( ClockTreeInit &cfg )
  {
    return select_system_clock_source( cfg.mux.sys );
  }


  bool setSourceSDIO( ClockTreeInit &cfg )
  {
    /*-------------------------------------------------
    Currently not implemented cause no use for SDIO clk
    -------------------------------------------------*/
    return true;
  }


  bool setSourceRTC( ClockTreeInit &cfg )
  {
    /*-------------------------------------------------
    Currently not implemented cause no use for RTC clk
    -------------------------------------------------*/
    return true;
  }


  bool setSourceUSB48( ClockTreeInit &cfg )
  {
    /*-------------------------------------------------
    Currently not implemented cause no use for USB clk
    -------------------------------------------------*/
    return true;
  }


  bool setSourceI2S( ClockTreeInit &cfg )
  {
    /*-------------------------------------------------
    Currently not implemented cause no use for I2S clk
    -------------------------------------------------*/
    if ( cfg.enabled.pll_i2s_p || cfg.enabled.pll_i2s_q || cfg.enabled.pll_i2s_r )
    {
      Chimera::insert_debug_breakpoint();
    }

    return true;
  }


  bool setSourceSAI( ClockTreeInit &cfg )
  {
    /*-------------------------------------------------
    Currently not implemented cause no use for SAI clk
    -------------------------------------------------*/
    if ( cfg.enabled.pll_sai_p || cfg.enabled.pll_sai_q )
    {
      Chimera::insert_debug_breakpoint();
    }

    return true;
  }


  /*-------------------------------------------------------------------------------
  Bus Prescalers
  -------------------------------------------------------------------------------*/
  bool setPrescaleAHB( ClockTreeInit &cfg )
  {
    /*-------------------------------------------------
    Input protection
    -------------------------------------------------*/
    if ( cfg.prescaler.ahb > 512 )
    {
      return false;
    }

    /*-------------------------------------------------
    Assign the prescaler. The voltage domain must match
    for this, else the clock setting may not match.
    -------------------------------------------------*/
    uint32_t expValue = 0;
    switch ( cfg.prescaler.ahb )
    {
      default:
        return false;
        break;

      case 0:
      case 1:
        expValue = 0u;
        break;

      case 2:
        expValue = ( 0x8 | 0u ) << CFGR_HPRE_Pos;
        break;

      case 4:
        expValue = ( 0x8 | 1u ) << CFGR_HPRE_Pos;
        break;

      case 8:
        expValue = ( 0x8 | 2u ) << CFGR_HPRE_Pos;
        break;

      case 16:
        expValue = ( 0x8 | 3u ) << CFGR_HPRE_Pos;
        break;

      case 64:
        expValue = ( 0x8 | 4u ) << CFGR_HPRE_Pos;
        break;

      case 128:
        expValue = ( 0x8 | 5u ) << CFGR_HPRE_Pos;
        break;

      case 256:
        expValue = ( 0x8 | 6u ) << CFGR_HPRE_Pos;
        break;

      case 512:
        expValue = ( 0x8 | 7u ) << CFGR_HPRE_Pos;
        break;
    };

    /*-------------------------------------------------
    Per datasheet, ensure value is applied
    -------------------------------------------------*/
    HPRE::set( RCC1_PERIPH, expValue );
    while ( HPRE::get( RCC1_PERIPH ) != expValue )
    {
      continue;
    }

    return true;
  }


  bool setPrescaleAPB1( ClockTreeInit &cfg )
  {
    /*-------------------------------------------------
    Input protection
    -------------------------------------------------*/
    if ( cfg.prescaler.apb1 > 16 )
    {
      return false;
    }

    /*-------------------------------------------------
    Assign the prescaler. The voltage domain must match
    for this, else the clock setting may not match.
    -------------------------------------------------*/
    uint32_t expValue = 0;
    switch ( cfg.prescaler.apb1 )
    {
      default:
        return false;
        break;

      case 0:
      case 1:
        expValue = 0u;
        break;

      case 2:
        expValue = ( 0x4 | 0u ) << CFGR_PPRE1_Pos;
        break;

      case 4:
        expValue = ( 0x4 | 1u ) << CFGR_PPRE1_Pos;
        break;

      case 8:
        expValue = ( 0x4 | 2u ) << CFGR_PPRE1_Pos;
        break;

      case 16:
        expValue = ( 0x4 | 3u ) << CFGR_PPRE1_Pos;
        break;
    };

    /*-------------------------------------------------
    Per datasheet, ensure value is applied
    -------------------------------------------------*/
    PPRE1::set( RCC1_PERIPH, expValue );
    while ( PPRE1::get( RCC1_PERIPH ) != expValue )
    {
      continue;
    }

    return true;
  }


  bool setPrescaleAPB2( ClockTreeInit &cfg )
  {
    /*-------------------------------------------------
    Input protection
    -------------------------------------------------*/
    if ( cfg.prescaler.apb1 > 16 )
    {
      return false;
    }

    /*-------------------------------------------------
    Assign the prescaler. The voltage domain must match
    for this, else the clock setting may not match.
    -------------------------------------------------*/
    uint32_t expValue = 0;
    switch ( cfg.prescaler.apb2 )
    {
      default:
        return false;
        break;

      case 0:
      case 1:
        expValue = 0u;
        break;

      case 2:
        expValue = ( 0x4 | 0u ) << CFGR_PPRE2_Pos;
        break;

      case 4:
        expValue = ( 0x4 | 1u ) << CFGR_PPRE2_Pos;
        break;

      case 8:
        expValue = ( 0x4 | 2u ) << CFGR_PPRE2_Pos;
        break;

      case 16:
        expValue = ( 0x4 | 3u ) << CFGR_PPRE2_Pos;
        break;
    };

    /*-------------------------------------------------
    Per datasheet, ensure value is applied
    -------------------------------------------------*/
    PPRE2::set( RCC1_PERIPH, expValue );
    while ( PPRE2::get( RCC1_PERIPH ) != expValue )
    {
      continue;
    }

    return true;
  }

}    // namespace Thor::LLD::RCC
