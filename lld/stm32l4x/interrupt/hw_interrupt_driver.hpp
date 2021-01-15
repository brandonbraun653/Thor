/********************************************************************************
 *  File Name:
 *    hw_interrupt_driver.hpp
 *
 *  Description:
 *    Declares the LLD interface to the STM32L4 series INTERRUPT hardware.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_INTERRUPT_DRIVER_STM32L4_HPP
#define THOR_HW_INTERRUPT_DRIVER_STM32L4_HPP

/* Chimera Includes */
//#include <Chimera/common>

/* Thor Includes */
#include <Thor/lld/stm32l4x/interrupt/hw_interrupt_types.hpp>
#include <Thor/lld/stm32l4x/interrupt/hw_interrupt_mapping.hpp>
#include <Thor/lld/interface/interrupt/interrupt_intf.hpp>

namespace Thor::LLD::INT
{
}    // namespace Thor::LLD::INT

#endif /* !THOR_HW_INTERRUPT_DRIVER_STM32L4_HPP */
