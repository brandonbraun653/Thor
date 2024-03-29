/******************************************************************************
 *  File Name:
 *    hw_rcc_pll.cpp
 *
 *  Description:
 *    Functions for configuring the device PLLs
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/* STL Includes */
#include <cmath>

/* Chimera Includes */
#include <Chimera/assert>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/inc/rcc>
#include <Thor/lld/stm32f4x/rcc/hw_rcc_prv.hpp>

namespace Thor::LLD::RCC
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  /*-------------------------------------------------
  CORE PLL operating ranges from register spec
  -------------------------------------------------*/
  static constexpr size_t PLL_CORE_M_MIN        = 2;
  static constexpr size_t PLL_CORE_M_MAX        = 63;
  static constexpr size_t PLL_CORE_N_MIN        = 50;
  static constexpr size_t PLL_CORE_N_MAX        = 432;
  static constexpr size_t PLL_CORE_P_MIN        = 2;
  static constexpr size_t PLL_CORE_P_MAX        = 8;
  static constexpr size_t PLL_CORE_Q_MIN        = 2;
  static constexpr size_t PLL_CORE_Q_MAX        = 15;
  static constexpr size_t PLL_CORE_R_MIN        = 2;
  static constexpr size_t PLL_CORE_R_MAX        = 7;
  static constexpr size_t PLL_CORE_VCO_IN_FMIN  = 1000000;      // 1 MHz
  static constexpr size_t PLL_CORE_VCO_IN_FMAX  = 2000000;      // 2 MHz
  static constexpr size_t PLL_CORE_VCO_OUT_FMIN = 100000000;    // 100 MHz
  static constexpr size_t PLL_CORE_VCO_OUT_FMAX = 432000000;    // 432 MHz


  /*---------------------------------------------------------------------------
  Driver Functions
  ---------------------------------------------------------------------------*/
  bool configureCorePLL( ClockTreeInit &cfg )
  {
    using namespace Chimera::Clock;

    /*-------------------------------------------------------------------------
    Double check input parameters. If the VCO control
    params are out of bounds, reject the config. Else
    if the divisors are out of bounds, simply set them
    to the max division possible to reduce clock rate.
    -------------------------------------------------------------------------*/
    if ( ( cfg.PLLCore.M < PLL_CORE_M_MIN ) || ( cfg.PLLCore.M > PLL_CORE_M_MAX ) || ( cfg.PLLCore.N < PLL_CORE_N_MIN ) ||
         ( cfg.PLLCore.N > PLL_CORE_N_MAX ) )
    {
      return false;
    }

    if ( ( cfg.PLLCore.P < PLL_CORE_P_MIN ) || ( cfg.PLLCore.P > PLL_CORE_P_MAX ) )
    {
      cfg.PLLCore.P = PLL_CORE_P_MAX;
    }

    if ( ( cfg.PLLCore.Q < PLL_CORE_Q_MIN ) || ( cfg.PLLCore.Q > PLL_CORE_Q_MAX ) )
    {
      cfg.PLLCore.Q = PLL_CORE_Q_MAX;
    }

    if ( ( cfg.PLLCore.R < PLL_CORE_R_MIN ) || ( cfg.PLLCore.R > PLL_CORE_R_MAX ) )
    {
      cfg.PLLCore.R = PLL_CORE_R_MAX;
    }

    /*-------------------------------------------------------------------------
    Block all other code from execution
    -------------------------------------------------------------------------*/
    auto isrMask = Chimera::System::disableInterrupts();

    /*-------------------------------------------------------------------------
    Switch core clock to HSI if currently using PLL
    -------------------------------------------------------------------------*/
    auto clk_src          = SW::get( RCC1_PERIPH );
    bool hsi_prev_enabled = false;
    bool source_switched  = false;
    if ( ( ( clk_src & CFGR_SW_PLL ) == CFGR_SW_PLL ) || ( ( clk_src & CFGR_SW_PLLR ) == CFGR_SW_PLLR ) )
    {
      source_switched  = true;
      hsi_prev_enabled = enableHSI();
      select_system_clock_source( Chimera::Clock::Bus::HSI16 );
    }

    /*-------------------------------------------------------------------------
    Disable the PLL
    -------------------------------------------------------------------------*/
    PLLON::clear( RCC1_PERIPH, CR_PLLON );
    while ( PLLRDY::get( RCC1_PERIPH ) )
    {
      continue;
    }

    /*-------------------------------------------------------------------------
    Remap the P setting to the register value
    -------------------------------------------------------------------------*/
    switch( cfg.PLLCore.P )
    {
      case 2:
        cfg.PLLCore.P = 0;
        break;

      case 4:
        cfg.PLLCore.P = 1;
        break;

      case 6:
        cfg.PLLCore.P = 2;
        break;

      case 8:
        cfg.PLLCore.P = 3;
        break;

      default:
        RT_HARD_ASSERT( false );
        break;
    };

    /*-------------------------------------------------------------------------
    Configure the desired settings
    -------------------------------------------------------------------------*/
    PLLM::set( RCC1_PERIPH, cfg.PLLCore.M << PLLCFGR_PLLM_Pos );
    PLLN::set( RCC1_PERIPH, cfg.PLLCore.N << PLLCFGR_PLLN_Pos );
    PLLP::set( RCC1_PERIPH, cfg.PLLCore.P << PLLCFGR_PLLP_Pos );
    PLLQ::set( RCC1_PERIPH, cfg.PLLCore.Q << PLLCFGR_PLLQ_Pos );
    PLLR::set( RCC1_PERIPH, cfg.PLLCore.R << PLLCFGR_PLLR_Pos );

    // Input clock source selection
    switch ( cfg.mux.pll )
    {
      case Chimera::Clock::Bus::HSE:
        PLLSRC::set( RCC1_PERIPH, PLLCFGR_PLLSRC_HSE );
        break;

      default:    // HSI
        PLLSRC::clear( RCC1_PERIPH, PLLCFGR_PLLSRC_HSE );
        break;
    };

    /*-------------------------------------------------------------------------
    Enable the PLL
    -------------------------------------------------------------------------*/
    PLLON::set( RCC1_PERIPH, CR_PLLON );
    while ( !PLLRDY::get( RCC1_PERIPH ) )
    {
      continue;
    }

    /*-------------------------------------------------------------------------
    Switch back to the PLL source if used before
    -------------------------------------------------------------------------*/
    if ( source_switched && !hsi_prev_enabled )
    {
      disableHSI();
    }

    if ( ( cfg.mux.sys == Bus::PLLP ) || ( cfg.mux.sys == Bus::PLLR ) )
    {
      select_system_clock_source( cfg.mux.sys );
    }

    /*-------------------------------------------------------------------------
    Re-enable interrupts
    -------------------------------------------------------------------------*/
    Chimera::System::enableInterrupts( isrMask );
    return true;
  }


  bool configureSAIPLL( ClockTreeInit &cfg )
  {
    return false;
  }

  Chimera::Status_t calcPLLCoreSettings( const size_t inFreq, const size_t outFreq, ClockTreeInit &config )
  {
    /**
     *  From the datasheet, the higher VCO input frequency, the
     *  less jitter will be present. Use max as the target.
     */
    constexpr int _target_vco_in = PLL_CORE_VCO_IN_FMAX;

    /*-------------------------------------------------------------------------
    Input protection
    -------------------------------------------------------------------------*/
    if ( !inFreq || ( outFreq < PLL_CORE_VCO_OUT_FMIN ) || ( outFreq > PLL_CORE_VCO_OUT_FMAX ) )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }

    /*-------------------------------------------------------------------------
    Calculate the PLL input divisor: M
    -------------------------------------------------------------------------*/
    int error      = 0;
    size_t vco_in  = 0;
    uint32_t calcM = 0;

    for ( auto div = PLL_CORE_M_MIN; div < PLL_CORE_M_MAX; div++ )
    {
      vco_in = ( inFreq / div );
      error  = _target_vco_in - static_cast<int>( vco_in );

      /*-------------------------------------------------
      Once error crosses from negative to positive, that
      is when the VCO input is near the target frequency
      but not exceeding it.
      -------------------------------------------------*/
      if ( error >= 0 )
      {
        calcM = div;
        break;
      }
    }

    // Double check the VCO input range assumption
    RT_HARD_ASSERT( ( vco_in >= PLL_CORE_VCO_IN_FMIN ) && ( vco_in <= PLL_CORE_VCO_IN_FMAX ) );

    /*-------------------------------------------------------------------------
    Calculate the VCO input multiplier: N
    -------------------------------------------------------------------------*/
    size_t vco_out      = 0;                                       // Loop var for VCO calculation
    uint32_t calcN      = 0;                                       // Tracks best found multiplier value
    uint32_t lowestErr  = std::numeric_limits<uint32_t>::max();    // Error tracker
    int _target_vco_out = static_cast<int>( outFreq );

    for ( auto mul = PLL_CORE_N_MIN; mul < PLL_CORE_N_MAX; mul++ )
    {
      vco_out = vco_in * mul;
      error   = _target_vco_out - vco_out;

      if ( abs( error ) < lowestErr )
      {
        calcN = mul;
        if ( error == 0 )
        {
          break;
        }
      }
      else
      {
        /*---------------------------------------------------------------------
        Early stop condition. Due to initialization value
        of error tracker and linear VCO equation, any error
        increase means an inflection point was reached.
        ---------------------------------------------------------------------*/
        break;
      }
    }

    // Double check the VCO output range assumption
    RT_HARD_ASSERT( ( vco_out >= PLL_CORE_VCO_OUT_FMIN ) && ( vco_out <= PLL_CORE_VCO_OUT_FMAX ) );

    /*-------------------------------------------------------------------------
    Getting here means things are within spec. Assign
    the config data to the user data.
    -------------------------------------------------------------------------*/
    config.PLLCore.M = calcM;
    config.PLLCore.N = calcN;
    return Chimera::Status::OK;
  }


  Chimera::Status_t calculatePLLOuputOscillator( const PLLOut channel, const size_t inFreq, const size_t outFreq,
                                                 ClockTreeInit &config )
  {
    /*-------------------------------------------------------------------------
    Input protection
    -------------------------------------------------------------------------*/
    if ( ( inFreq < PLL_CORE_VCO_OUT_FMIN ) || ( inFreq > PLL_CORE_VCO_OUT_FMAX ) )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------------------------------
    Calculate the output divisors
    -------------------------------------------------------------------------*/
    size_t X_out        = 0;                                       // Loop var for VCO calculation
    uint32_t calcX      = 0;                                       // Tracks best found multiplier value
    uint32_t lowestErr  = std::numeric_limits<uint32_t>::max();    // Error tracker
    uint32_t currentErr = lowestErr;
    int _target_X_out   = static_cast<int>( outFreq );
    uint32_t _start     = 0;
    uint32_t _stop      = 0;
    uint32_t _incr      = 0;

    switch ( channel )
    {
      case PLLOut::P:
        _start = PLL_CORE_P_MIN;
        _stop  = PLL_CORE_P_MAX;
        _incr  = 2;
        break;

      case PLLOut::Q:
        _start = PLL_CORE_Q_MIN;
        _stop  = PLL_CORE_Q_MAX;
        _incr  = 1;
        break;

      case PLLOut::R:
        _start = PLL_CORE_R_MIN;
        _stop  = PLL_CORE_R_MAX;
        _incr  = 1;
        break;

      default:
        return Chimera::Status::NOT_SUPPORTED;
        break;
    };


    for ( auto div = _start; div <= _stop; div += _incr )
    {
      X_out      = inFreq / div;
      currentErr = abs( _target_X_out - static_cast<int>( X_out ) );

      if ( currentErr < lowestErr )
      {
        calcX     = div;
        lowestErr = currentErr;
        if ( currentErr == 0 )
        {
          break;
        }
      }
      else
      {
        /*---------------------------------------------------------------------
        Early stop condition. Due to initialization value
        of error tracker and linear VCO equation, any error
        increase means an inflection point was reached.
        ---------------------------------------------------------------------*/
        X_out = inFreq / calcX;
        break;
      }
    }

    /*-------------------------------------------------------------------------
    Verify the calculated settings
    -------------------------------------------------------------------------*/
    if ( channel == PLLOut::P )
    {
      // Drives the core, but can't exceed 180 MHz
      RT_HARD_ASSERT( X_out <= 180000000 );
    }
    else if ( channel == PLLOut::Q )
    {
      // Drives USB and SDIO, which require 48 MHz clock exactly.
      RT_HARD_ASSERT( X_out == 48000000 );
    }

    /*-------------------------------------------------------------------------
    Throw error if the clock deviates more than 5%
    -------------------------------------------------------------------------*/
    float exp = static_cast<float>( outFreq );
    float act = static_cast<float>( X_out );

    if ( 0.05f < abs( ( act - exp ) / exp ) )
    {
      return Chimera::Status::FAIL;
    }

    /*-------------------------------------------------------------------------
    Otherwise, update the config with the new data
    -------------------------------------------------------------------------*/
    switch ( channel )
    {
      case PLLOut::P:
        config.PLLCore.P = calcX;
        break;

      case PLLOut::Q:
        config.PLLCore.Q = calcX;
        break;

      case PLLOut::R:
        config.PLLCore.R = calcX;
        break;

      default:
        // Not possible to hit this. Just do nothing.
        break;
    };

    return Chimera::Status::OK;
  }

}    // namespace Thor::LLD::RCC
