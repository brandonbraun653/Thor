/********************************************************************************
 *  File Name:
 *    adc_types.hpp
 *
 *  Description:
 *    Common LLD ADC Types
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_LLD_ADC_COMMON_TYPES_HPP
#define THOR_LLD_ADC_COMMON_TYPES_HPP

/* C++ Includes */
#include <array>
#include <cstdint>

/* ETL Includes */
#include <etl/queue_spsc_locked.h>

/* Chimera Includes */
#include <Chimera/common>

namespace Thor::LLD::ADC
{
  /*-------------------------------------------------------------------------------
  Forward Declarations
  -------------------------------------------------------------------------------*/
  class Driver;
  struct RegisterMap;


  /*-------------------------------------------------------------------------------
  Structures
  -------------------------------------------------------------------------------*/
  struct Measurement
  {
    size_t timestamp_us;          /**< System time in microseconds */
    Chimera::ADC::Sample data;  /**< Channel data */
  };


  /*-------------------------------------------------------------------------------
  Aliases
  -------------------------------------------------------------------------------*/
  using Driver_rPtr = Driver *;

  template<size_t SIZE>
  using ChannelQueue = etl::queue_spsc_locked<Measurement, SIZE>;


  /*-------------------------------------------------------------------------------
  Enumerations
  -------------------------------------------------------------------------------*/
  /**
   *  How long an ADC channel should be sampled for. Values are directly
   *  converted to register settings once shifted into the proper position.
   */
  enum class SampleTime : uint8_t
  {
    SMP_2P5,   /**< 2.5 ADC Clock Cycles */
    SMP_6P5,   /**< 6.5 ADC Clock Cycles */
    SMP_12P5,  /**< 12.5 ADC Clock Cycles */
    SMP_24P5,  /**< 24.5 ADC Clock Cycles */
    SMP_47P5,  /**< 47.5 ADC Clock Cycles */
    SMP_92P5,  /**< 92.5 ADC Clock Cycles */
    SMP_247P5, /**< 247.5 ADC Clock Cycles */
    SMP_640P5, /**< 640.5 ADC Clock Cycles */

    NUM_OPTIONS,
    UNKNOWN
  };

}    // namespace Thor::LLD::ADC

#endif /* !THOR_LLD_ADC_COMMON_TYPES_HPP */
