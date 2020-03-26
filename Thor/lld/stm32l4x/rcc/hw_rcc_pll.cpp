/********************************************************************************
 *  File Name:
 *    hw_rcc_pll.cpp
 *
 *  Description:
 *    Implementation of PLL utilities for STM32L4 chips
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Chimera Includes */
#include <Chimera/algorithm>

/* Thor Includes */
#include <Thor/lld/stm32l4x/rcc/hw_rcc_mapping.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_pll.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_prj.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_types.hpp>

namespace Thor::LLD::RCC
{
  bool PLLConfigureP( OscillatorSettings &config )
  {
    // Haven't needed this functionality yet
    return false;
  }

  bool PLLConfigureQ( OscillatorSettings &config )
  {
    // Haven't needed this functionality yet
    return false;
  }

  bool PLLConfigureR( OscillatorSettings &config )
  {
    /*------------------------------------------------
    Disable the PLL and wait for the hardware to signal it's ready
    ------------------------------------------------*/
    PLLON::clear( RCC1_PERIPH, CR_PLLON );
    while ( PLLRDY::get( RCC1_PERIPH ) )
    {
      ; // Ready when cleared
    }

    /*------------------------------------------------
    Configure the VCO
    ------------------------------------------------*/
    // Select the input source
    switch ( config.PLLConfig.inputSource )
    {
      case Chimera::Clock::Bus::HSE:
        PLLSRC::set( RCC1_PERIPH, PLLCFGR_PLLSRC_HSE );
        break;

      case Chimera::Clock::Bus::HSI16:
        PLLSRC::set( RCC1_PERIPH, PLLCFGR_PLLSRC_HSI );
        break;

      case Chimera::Clock::Bus::MSI:
        PLLSRC::set( RCC1_PERIPH, PLLCFGR_PLLSRC_MSI );
        break;

      default:
        // Do nothing
        break;
    }

    // Apply the divisor to the input source to generate the VCO input
    PLLM::set( RCC1_PERIPH, config.PLLConfig.divM );

    // Apply the N divisor to generate the VCO clock
    PLLN::set( RCC1_PERIPH, config.PLLConfig.divN );

    // Configure the PLL R divisor
    PLLR::set( RCC1_PERIPH, config.PLLConfig.R.divisor );

    /*------------------------------------------------
    Turn on the PLL and wait for the hardware to signal it's ready
    ------------------------------------------------*/
    PLLON::set( RCC1_PERIPH, CR_PLLON );
    while ( ( PLLRDY::get( RCC1_PERIPH ) & CR_PLLRDY ) != CR_PLLRDY )
    {
      ; // Ready when set
    }

    /*------------------------------------------------
    Turn on the PLL R output
    ------------------------------------------------*/
    PLLREN::set( RCC1_PERIPH, PLLCFGR_PLLREN );
    config.PLLConfig.R.enabled = true;

    return true;
  }
}