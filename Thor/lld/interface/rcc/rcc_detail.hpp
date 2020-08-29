/********************************************************************************
 *  File Name:
 *    rcc_detail.hpp
 *
 *  Description:
 *    Includes the LLD specific headers for chip implementation details as well
 *    as defines various functions that are available on all chip types.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_RCC_DETAIL_HPP
#define THOR_LLD_RCC_DETAIL_HPP

#if defined( TARGET_LLD_MOCK )
#include <Thor/lld/interface/rcc/mock/rcc_mock.hpp>
#elif defined( TARGET_STM32L4 )
#include <Thor/lld/stm32l4x/rcc/hw_rcc_driver.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_mapping.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_prj.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_types.hpp>
#else
#pragma message( "rcc_detail.hpp: Unknown target for LLD" )
#endif

/* Chimera Includes */
#include <Chimera/common>

namespace Thor::LLD::RCC
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  /**
   *  Gets the HSI oscillator frequency in Hz
   *  @return size_t
   */
  size_t getHSIFreq();

  /**
   *  Gets the HSE oscillator frequency in Hz
   *  @return size_t
   */
  size_t getHSEFreq();

  /**
   *  Sets the HSE oscillator frequency in Hz. Given that this is
   *  an external clock, no configuration is performed.
   *
   *  @param[in]  freq    The project's external HSE clock frequency
   *  @return void
   */
  void setHSEFreq( const size_t freq );

  /**
   *  Gets the LSE oscillator frequency in Hz
   *  @return size_t
   */
  size_t getLSEFreq();

  /**
   *  Sets the LSE oscillator frequency in Hz. Given that this is
   *  an external clock, no configuration is performed.
   *
   *  @param[in]  freq    The project's external LSE clock frequency
   *  @return void
   */
  void setLSEFreq( const size_t freq );

  /**
   *  Gets the LSI oscillator frequency in Hz
   *  @return size_t
   */
  size_t getLSIFreq();

  /**
   *  Gets the MSI oscillator frequency in Hz
   *  @return size_t
   */
  size_t getMSIFreq();

  /**
   *  Gets the PLLCLK oscillator frequency in Hz
   *
   *  @note The function input expects a mask to be given that corresponds to
   *        one of the PLL clock outputs. On the L4 chips, if the PLLR clock
   *        is desired, pass in PLLCFGR_PLLR. PLLQ? -> PLLCFGR_PLLQ.
   *        These definitions are in hw_rcc_register_stm32l4<xxxx>.hpp
   *
   *  @param[in]  mask    The desired PLL output clock to get
   *  @return size_t
   */
  size_t getPLLCLKFreq( const uint32_t mask );

  /**
   *  Sets the PLLCLK oscillator frequency in Hz
   *
   *  @param[in]  mask    The desired PLL output clock to set
   *  @return bool
   */
  bool updatePLL( const uint32_t mask, OscillatorSettings &config );

  /**
   *  Get's the CPU core system clock frequency in Hz
   *  @return size_t
   */
  size_t getSysClockFreq();

  /**
   *  Gets the HSI oscillator frequency in Hz
   *  @return size_t
   */
  size_t getHCLKFreq();

  /**
   *  Gets the PCLK1 oscillator frequency in Hz
   *  @return size_t
   */
  size_t getPCLK1Freq();

  /**
   *  Gets the PCLK2 oscillator frequency in Hz
   *  @return size_t
   */
  size_t getPCLK2Freq();
}    // namespace Thor::LLD::RCC

#endif /* !THOR_LLD_RCC_DETAIL_HPP */
