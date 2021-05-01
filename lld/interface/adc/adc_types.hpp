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
#include <Chimera/adc>

namespace Thor::LLD::ADC
{
  /*-------------------------------------------------------------------------------
  Forward Declarations
  -------------------------------------------------------------------------------*/
  class Driver;
  struct RegisterMap;

  /*-------------------------------------------------------------------------------
  Aliases
  -------------------------------------------------------------------------------*/
  using Driver_rPtr = Driver *;

  template<size_t SIZE>
  using ChannelQueue = etl::queue_spsc_locked<Chimera::ADC::Sample, SIZE>;


  /*-------------------------------------------------------------------------------
  Enumerations
  -------------------------------------------------------------------------------*/
  /**
   *  How long an ADC channel should be sampled for. Values are directly
   *  converted to register settings once shifted into the proper position.
   */
  enum class SampleTime : uint8_t
  {
#if defined( STM32L432xx )
    SMP_2P5,   /**< 2.5 ADC Clock Cycles */
    SMP_6P5,   /**< 6.5 ADC Clock Cycles */
    SMP_12P5,  /**< 12.5 ADC Clock Cycles */
    SMP_24P5,  /**< 24.5 ADC Clock Cycles */
    SMP_47P5,  /**< 47.5 ADC Clock Cycles */
    SMP_92P5,  /**< 92.5 ADC Clock Cycles */
    SMP_247P5, /**< 247.5 ADC Clock Cycles */
    SMP_640P5, /**< 640.5 ADC Clock Cycles */

#elif defined( STM32F446xx )
    SMP_3,   /**< 3 ADC Clock Cycles */
    SMP_15,  /**< 15 ADC Clock Cycles */
    SMP_28,  /**< 28 ADC Clock Cycles */
    SMP_56,  /**< 56 ADC Clock Cycles */
    SMP_84,  /**< 84 ADC Clock Cycles */
    SMP_112, /**< 112 ADC Clock Cycles */
    SMP_144, /**< 144 ADC Clock Cycles */
    SMP_480, /**< 480 ADC Clock Cycles */
#endif

    NUM_OPTIONS,
    UNKNOWN
  };

}    // namespace Thor::LLD::ADC

#endif /* !THOR_LLD_ADC_COMMON_TYPES_HPP */
