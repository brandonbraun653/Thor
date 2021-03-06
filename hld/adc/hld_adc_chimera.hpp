/********************************************************************************
 *  File Name:
 *    hld_adc_chimera.hpp
 *
 *  Description:
 *    Chimera hooks for implementing ADC
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_ADC_CHIMERA_HOOKS_HPP
#define THOR_ADC_CHIMERA_HOOKS_HPP

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/adc>

namespace Chimera::ADC::Backend
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize();
  Chimera::Status_t reset();
  Driver_rPtr getDriver( const Peripheral periph );
  bool featureSupported( const Peripheral periph, const Feature feature );
}    // namespace Chimera::ADC::Backend

#endif /* !THOR_ADC_CHIMERA_HOOKS_HPP */
