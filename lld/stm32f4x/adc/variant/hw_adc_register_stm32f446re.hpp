/********************************************************************************
 *  File Name:
 *    hw_adc_register_stm32f446re.hpp
 *
 *  Description:
 *    ADC register definitions for the STM32F446RE series chips.
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_LLD_ADC_REGISTER_STM32F446RE_HPP
#define THOR_LLD_ADC_REGISTER_STM32F446RE_HPP

/* STL Includes */
#include <cstddef>

/* Thor Includes */
#include <Thor/lld/common/types.hpp>

/*-------------------------------------------------------------------------------
Macros
-------------------------------------------------------------------------------*/
#define STM32_ADC1_PERIPH_AVAILABLE


namespace Thor::LLD::ADC
{
  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  static constexpr size_t NUM_ADC_PERIPHS             = 1;
  static constexpr size_t NUM_ADC_IRQ_HANDLERS        = 1;
  static constexpr RIndex_t ADC1_RESOURCE_INDEX       = 0u;
  static constexpr size_t NUM_ADC_CHANNELS_PER_PERIPH = 19;

}    // namespace Thor::LLD::ADC

#endif /* !THOR_LLD_ADC_REGISTER_STM32F446RE_HPP */
