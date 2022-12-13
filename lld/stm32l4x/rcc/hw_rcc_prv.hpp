/******************************************************************************
 *  File Name:
 *    hw_rcc_prv.hpp
 *
 *  Description:
 *    Private interface for the RCC driver
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_RCC_PRIVATE_HPP
#define THOR_LLD_RCC_PRIVATE_HPP

/* Driver Includes */
#include <Thor/lld/stm32l4x/rcc/hw_rcc_types.hpp>

namespace Thor::LLD::RCC
{
  /*---------------------------------------------------------------------------
  External Data
  ---------------------------------------------------------------------------*/
  extern OscillatorSettings sOscillatorSettings;
  extern DerivedClockSettings sDerivedClockSettings;

  /*---------------------------------------------------------------------------
  Internal Functions
  ---------------------------------------------------------------------------*/
  /**
   *  Sets the PLLCLK oscillator frequency in Hz
   *
   *  @param[in]  mask    The desired PLL output clock to set
   *  @return bool
   */
  bool updatePLL( const uint32_t mask, OscillatorSettings &config );
  bool configureOscillators( OscillatorSettings &cfg );
  bool configureClocks( DerivedClockSettings &cfg );

  void clearResetReason();
  Chimera::System::ResetEvent getResetReason();
  size_t getHSIFreq();
  size_t getHSEFreq();
  void setHSEFreq( const size_t freq );
  size_t getLSEFreq();
  void setLSEFreq( const size_t freq );
  size_t getLSIFreq();
  size_t getMSIFreq();
  size_t getPLLCLKFreq( const uint32_t mask );
  size_t getSysClockFreq();
  size_t getHCLKFreq();
  size_t getPCLK1Freq();
  size_t getPCLK2Freq();

}    // namespace Thor::LLD::RCC

#endif /* !THOR_LLD_RCC_PRIVATE_HPP */
