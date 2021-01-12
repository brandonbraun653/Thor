/********************************************************************************
 *  File Name:
 *    hw_interrupt_mapping.hpp
 *
 *  Description:
 *    Provides structures for conversion and mapping between data types for fast
 *    runtime performance of driver code.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_INTERRUPT_MAPPING_HPP
#define THOR_HW_INTERRUPT_MAPPING_HPP

/* Driver Includes */
#include <Thor/lld/interface/interrupt/interrupt_types.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_types.hpp>
#include <Thor/lld/stm32l4x/interrupt/hw_interrupt_types.hpp>

namespace Thor::LLD::IT
{
}    // namespace Thor::LLD::IT

#endif /* !THOR_HW_INTERRUPT_MAPPING_HPP */
