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
#include <cstddef>

/* Chimera Includes */
#include <Chimera/assert>
#include <Chimera/common>
#include <Chimera/clock>
#include <Chimera/utility>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/common/cortex-m4/system_time.hpp>
#include <Thor/lld/interface/inc/rcc>
#include <Thor/lld/interface/inc/power>
#include <Thor/lld/stm32f4x/rcc/hw_rcc_prv.hpp>

namespace Thor::LLD::RCC
{
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
        return Chimera::System::ResetEvent::UNKNOWN;
        break;

      default:
        return Chimera::System::ResetEvent::NOT_SUPPORTED;
        break;
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

      case Chimera::Clock::Bus::SYSCLK:
        return getSystemClock();
        break;

      case Chimera::Clock::Bus::HCLK:
        return getHCLKFreq();
        break;

      case Chimera::Clock::Bus::PCLK1:
        return getPCLK1Freq();
        break;

      case Chimera::Clock::Bus::PCLK2:
        return getPCLK2Freq();
        break;

      case Chimera::Clock::Bus::HSE:
      case Chimera::Clock::Bus::LSE:
        return getExtOscFreq( bus );
        break;

      default:
        RT_HARD_ASSERT( false );
        return INVALID_CLOCK;
        break;
    }
  }


  bool configureClockTree( ClockTreeInit &config )
  {
    bool result = true;

    /*-------------------------------------------------
    Disable interrupts during reconfiguration
    -------------------------------------------------*/
    auto isrMask = Chimera::System::disableInterrupts();

    /*-------------------------------------------------
    Configure the oscillator sources
    -------------------------------------------------*/
    result |= configureHSE( config );
    result |= configureHSI( config );
    result |= configureLSE( config );
    result |= configureLSI( config );

    if ( config.enabled.pll_core_clk || config.enabled.pll_core_q || config.enabled.pll_core_r )
    {
      result |= configureCorePLL( config );
    }

    // if( config.enabled.pll_sai_p || config.enabled.pll_sai_q )
    // {
    //   result |= configureSAIPLL( config );
    // }

    /*-------------------------------------------------
    Configure the source mux's that aren't tied to
    oscillator inputs.
    -------------------------------------------------*/
    result |= setSourceSYS( config );
    result |= setSourceSDIO( config );
    result |= setSourceRTC( config );
    result |= setSourceUSB48( config );
    result |= setSourceI2S( config );
    result |= setSourceSAI( config );

    /*-------------------------------------------------
    Configure the system prescalers
    -------------------------------------------------*/
    result |= setPrescaleAHB( config );
    result |= setPrescaleAPB1( config );
    result |= setPrescaleAPB2( config );

    /*-------------------------------------------------
    Re-enable interrupts now that configuration is done
    -------------------------------------------------*/
    Chimera::System::enableInterrupts( isrMask );

    return result;
  }


  Chimera::Status_t calculatePLLBaseOscillator( const PLLType pll, const size_t inFreq, const size_t outFreq,
                                                ClockTreeInit &config )
  {
    switch ( pll )
    {
      case PLLType::CORE:
      case PLLType::SAI:
        return calcPLLCoreSettings( inFreq, outFreq, config );
        break;

      default:
        return Chimera::Status::NOT_SUPPORTED;
        break;
    };
  }


  Chimera::Status_t calculatePLLOuputOscillator( const PLLType pll, const PLLOut channel, const size_t inFreq,
                                                 const size_t outFreq, ClockTreeInit &config )
  {
    switch ( pll )
    {
      case PLLType::CORE:
      case PLLType::SAI:
        return calculatePLLOuputOscillator( channel, inFreq, outFreq, config );
        break;

      default:
        return Chimera::Status::NOT_SUPPORTED;
        break;
    };
  }

}    // namespace Thor::LLD::RCC
