/******************************************************************************
 *  File Name:
 *    hw_rcc_prv.cpp
 *
 *  Description:
 *    Private function implementations
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/* Chimera Includes */
#include <Chimera/clock>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/inc/rcc>
#include <Thor/lld/interface/inc/interrupt>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_pll.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_prv.hpp>

namespace Thor::LLD::RCC
{
  /*---------------------------------------------------------------------------
  Internal Functions
  ---------------------------------------------------------------------------*/
  bool updatePLL( const uint32_t mask, OscillatorSettings &config )
  {
    auto rcc    = getCoreClockCtrl();
    auto result = false;

    /*-------------------------------------------------------------------------
    Disable ISR handling so that other systems won't be
    totally corrupted by the clock switching.
    -------------------------------------------------------------------------*/
    auto isrMask = Thor::LLD::INT::disableInterrupts();

    /*-------------------------------------------------------------------------
    If the current system clock is derived from the PLL, switch
    to the HSI16 clock so that something is driving the ARM core.
    -------------------------------------------------------------------------*/
    bool PLLIsSysClockSource = ( rcc->getCoreClockSource() == Chimera::Clock::Bus::PLLP );
    if ( PLLIsSysClockSource )
    {
      rcc->enableClock( Chimera::Clock::Bus::HSI16 );
      rcc->setCoreClockSource( Chimera::Clock::Bus::HSI16 );
    }

    /*-------------------------------------------------------------------------
    Handle the configuration of each PLL output
    -------------------------------------------------------------------------*/
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

    /*-------------------------------------------------------------------------
    Switch back to the PLL source if necessary. Turn off
    the HSI16 clock to save a bit of power.
    -------------------------------------------------------------------------*/
    if ( PLLIsSysClockSource )
    {
      rcc->enableClock( Chimera::Clock::Bus::PLLP );
      rcc->setCoreClockSource( Chimera::Clock::Bus::PLLP );
      rcc->disableClock( Chimera::Clock::Bus::HSI16 );
    }

    /*-------------------------------------------------------------------------
    Re-enable the ISRs, allowing the system to update itself
    in response to the new clock configurations.
    -------------------------------------------------------------------------*/
    Thor::LLD::INT::enableInterrupts( isrMask );

    return result;
  }


  bool configureOscillators( OscillatorSettings &cfg )
  {
    /*-------------------------------------------------------------------------
    Make sure we are supposed to allow configuration of these settings
    -------------------------------------------------------------------------*/
    if ( !cfg.valid )
    {
      return false;
    }

    auto isrMask = Thor::LLD::INT::disableInterrupts();

    /*-------------------------------------------------------------------------
    Configure the HSE oscillator
    -------------------------------------------------------------------------*/
    if ( cfg.HSEConfig.configure )
    {
      // TODO once needed
    }

    /*-------------------------------------------------------------------------
    Configure the HSI oscillator
    -------------------------------------------------------------------------*/
    if ( cfg.HSIConfig.configure )
    {
      // TODO once needed
    }

    /*-------------------------------------------------------------------------
    Configure the LSE oscillator
    -------------------------------------------------------------------------*/
    if ( cfg.LSEConfig.configure )
    {
      // TODO once needed
    }

    /*-------------------------------------------------------------------------
    Configure the LSI oscillator
    -------------------------------------------------------------------------*/
    if ( cfg.LSIConfig.configure )
    {
      // TODO once needed
    }

    /*-------------------------------------------------------------------------
    Configure the MSI oscillator
    -------------------------------------------------------------------------*/
    if ( cfg.MSIConfig.configure )
    {
      // TODO once needed
    }

    /*-------------------------------------------------------------------------
    Configure the PLL oscillator
    -------------------------------------------------------------------------*/
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

    Thor::LLD::INT::enableInterrupts( isrMask );
    return true;
  }


  bool configureClocks( DerivedClockSettings &cfg )
  {
    /*-------------------------------------------------------------------------
    Make sure we are supposed to allow configuration of these settings
    -------------------------------------------------------------------------*/
    if ( !cfg.valid )
    {
      return false;
    }

    auto isrMask = Thor::LLD::INT::disableInterrupts();

    /*-------------------------------------------------------------------------
    Update the HCLK divisor
    -------------------------------------------------------------------------*/
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

    /*-------------------------------------------------------------------------
    Update the PCLK1 divisor
    -------------------------------------------------------------------------*/
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

    /*-------------------------------------------------------------------------
    Update the PCLK2 divisor
    -------------------------------------------------------------------------*/
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

    Thor::LLD::INT::enableInterrupts( isrMask );
    return true;
  }

}  // namespace Thor::LLD::RCC
