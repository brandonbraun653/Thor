/******************************************************************************
 *  File Name:
 *    hw_crs_driver.hpp
 *
 *  Description:
 *    CRS driver interface for the STM32L432xx
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_CRS_DRIVER_HPP
#define THOR_LLD_CRS_DRIVER_HPP

/* Chimera Includes */
#include <Chimera/common>

/* Thor Includes */
#include <Thor/lld/stm32l4x/crs/hw_crs_types.hpp>

namespace Thor::LLD::CRS
{
  /**
   *  Enables the peripheral clock
   *  @return void
   */
  void enableClock();

  /**
   *  Disables the peripheral clock
   *  @return void
   */
  void disableClock();

  /**
   *  Initializes the peripheral. Other methods in the driver cannot
   *  be called until this method succeeds.
   *
   *  @return Chimera::Status_t
   */
  Chimera::Status_t initialize();

  /**
   *  Sets the input synchronization clock source
   *
   *  @param[in]  src             Desired source to set
   */
  Chimera::Status_t selectSyncSource( const SyncSource src );

  /**
   *  Sets the clock divisor on the Sync source
   *
   *  @param[in]  div             Desired divisor to set
   *  @return Chimera::Status_t
   */
  Chimera::Status_t configureDiv( const SyncDiv div );

  /**
   *  Configures the FELIM and RELOAD registers based on the equations
   *  found in section 7.4.3 in the device reference manual.
   *
   *  @param[in]  targetFreq      Target frequency in Hz
   *  @param[in]  syncFreq        Sync frequency in Hz
   *  @param[in]  stepSize        Trimming step size [1,100]
   *  @return Chimera::Status_t
   */
  Chimera::Status_t configureHWAlgo( const size_t targetFreq, const size_t syncFreq, const size_t stepSize );

  /**
   *  Enables/disables the automatic hardware trimming functionality
   *
   *  @param[in]  state           Enable/disable
   *  @return void
   */
  void toggleAutoTrim( const bool state );


  /**
   *  Enables/disables the frequency error counter
   *
   *  @param[in]  state           Enable/disable
   *  @return void
   */
  void toggleFEQ( const bool state );

}    // namespace Thor::LLD::CRS

#endif /* !THOR_LLD_CRS_DRIVER_HPP */
