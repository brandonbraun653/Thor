/********************************************************************************
 *  File Name:
 *    hld_pwm_chimera.hpp
 *
 *	 Description:
 *    Chimera hooks for implementing PWM
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_PWM_CHIMERA_HOOKS_HPP
#define THOR_PWM_CHIMERA_HOOKS_HPP

/* STL Includes */
#include <memory>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/pwm>

namespace Chimera::PWM::Backend
{
  Chimera::Status_t initialize();
  Chimera::Status_t reset();
  Chimera::PWM::IPWM_sPtr getDriver( const size_t channel );
  size_t numSupportedChannels();
}    // namespace Chimera::PWM::Backend

#endif /* !THOR_PWM_CHIMERA_HOOKS_HPP */
