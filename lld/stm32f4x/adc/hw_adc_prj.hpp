/******************************************************************************
 *  File Name:
 *    hw_adc_prj.hpp
 *
 *  Description:
 *    Pulls in target specific definitions and resources used in the actual driver
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_HW_ADC_PROJECT_HPP
#define THOR_HW_ADC_PROJECT_HPP

/*------------------------------------------------
All STM32L4 devices
------------------------------------------------*/
#include <Thor/lld/stm32f4x/adc/variant/hw_adc_register_stm32f4xxxx.hpp>

/*------------------------------------------------
Chip specific STM32L4 devices
------------------------------------------------*/
#if defined( STM32F446xx )
#include <Thor/lld/stm32f4x/adc/variant/hw_adc_register_stm32f446re.hpp>
#endif

#endif /* !THOR_HW_ADC_PROJECT_HPP */
