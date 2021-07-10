/********************************************************************************
 *  File Name:
 *    hw_rcc_prv.hpp
 *
 *  Description:
 *    Private methods and declarations
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_RCC_PRIVATE_HPP
#define THOR_LLD_RCC_PRIVATE_HPP

/* STL Includes */
#include <cstddef>

/* Thor Includes */
#include <Thor/lld/stm32f4x/rcc/hw_rcc_types.hpp>

namespace Thor::LLD::RCC
{
  /*-------------------------------------------------------------------------------
  External Data
  -------------------------------------------------------------------------------*/
  extern ClockTreeInit clock_cfg;

  /*-------------------------------------------------------------------------------
  Private Functions
  -------------------------------------------------------------------------------*/
  /*-------------------------------------------------
  Low level Functions
  -------------------------------------------------*/
  bool select_system_clock_source( Chimera::Clock::Bus src );
  bool select_pll_clock_source( Chimera::Clock::Bus src );

  bool enableHSI();
  void disableHSI();
  bool enableLSI();
  void disableLSI();

  Chimera::Status_t calcPLLCoreSettings( const size_t inFreq, const size_t outFreq, ClockTreeInit &config );
  Chimera::Status_t calculatePLLOuputOscillator( const PLLOut channel, const size_t inFreq, const size_t outFreq, ClockTreeInit &config );

  /*-------------------------------------------------
  Clock Mux
  -------------------------------------------------*/
  Chimera::Clock::Bus getSysClockSource();
  Chimera::Clock::Bus getPLLClockSource();

  /*-------------------------------------------------
  Runtime Bus Frequency Calculation
  -------------------------------------------------*/
  size_t getSystemClock();
  size_t getHCLKFreq();
  size_t getPCLK1Freq();
  size_t getPCLK2Freq();
  size_t getPLLClock( const PLLOut which );

  /*-------------------------------------------------
  Oscillator Configuration
  -------------------------------------------------*/
  bool configureHSE( ClockTreeInit &cfg );
  bool configureHSI( ClockTreeInit &cfg );
  bool configureLSE( ClockTreeInit &cfg );
  bool configureLSI( ClockTreeInit &cfg );
  bool configureCorePLL( ClockTreeInit &cfg );
  bool configureSAIPLL( ClockTreeInit &cfg );

  /*-------------------------------------------------
  Clock Source Selection
  -------------------------------------------------*/
  bool setSourcePLL( ClockTreeInit &cfg );
  bool setSourceSYS( ClockTreeInit &cfg );
  bool setSourceSDIO( ClockTreeInit &cfg );
  bool setSourceRTC( ClockTreeInit &cfg );
  bool setSourceUSB48( ClockTreeInit &cfg );
  bool setSourceI2S( ClockTreeInit &cfg );
  bool setSourceSAI( ClockTreeInit &cfg );

  /*-------------------------------------------------
  Bus Prescalers
  -------------------------------------------------*/
  bool setPrescaleAHB( ClockTreeInit &cfg );
  bool setPrescaleAPB1( ClockTreeInit &cfg );
  bool setPrescaleAPB2( ClockTreeInit &cfg );

}  // namespace Thor::LLD::RCC

#endif  /* !THOR_LLD_RCC_PRIVATE_HPP */
