/********************************************************************************
 *   File Name:
 *    hw_usart_mapping.hpp
 *
 *   Description:
 *    Provides structures for conversion and mapping between data types for fast
 *    runtime performance of driver code.
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_USART_MAPPING_HPP
#define THOR_HW_USART_MAPPING_HPP

/* C++ Includes */
#include <array>

/* Chimera Includes */
#include <Chimera/container.hpp>

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/f4/interrupt/hw_it_prj.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_types.hpp>
#include <Thor/drivers/f4/usart/hw_usart_types.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_USART == 1 )

namespace Thor::Driver::USART
{
#if defined( STM32_USART1_PERIPH_AVAILABLE )
  extern RegisterMap * USART1_PERIPH;
#endif 
 
#if defined( STM32_USART2_PERIPH_AVAILABLE )
  extern RegisterMap * USART2_PERIPH;
#endif 

#if defined( STM32_USART3_PERIPH_AVAILABLE )
  extern RegisterMap * USART3_PERIPH;
#endif 

#if defined( STM32_USART6_PERIPH_AVAILABLE )
  extern RegisterMap * USART6_PERIPH;
#endif 
  
  /*------------------------------------------------
  Peripheral Memory Mapping
  ------------------------------------------------*/
  extern PeriphRegisterList PeripheralList;
  extern Chimera::Container::LightFlatMap<std::uintptr_t, size_t> InstanceToResourceIndex;
  extern Chimera::Container::LightFlatMap<size_t, RegisterMap *const> ChanneltoInstance;

  /*------------------------------------------------
  Gets the interrupt request number tied to a USART instance.
  ------------------------------------------------*/
  extern const IRQn_Type USART_IRQn[ NUM_USART_PERIPHS ];

  /*------------------------------------------------
  Map Chimera options into Register configuration values
  ------------------------------------------------*/
  extern const std::array<uint32_t, NUM_USART_PERIPHS> CharWidToRegConfig;
  extern const std::array<uint32_t, NUM_USART_PERIPHS> ParityToRegConfig;
  extern const std::array<uint32_t, NUM_USART_PERIPHS> StopBitsToRegConfig;

  /**
   *  Initializes memory associated with mapping
   *  
   *  @return void
   */
  void initializeMapping();

  /**
   *  Checks if the given address belongs to a peripheral instance
   *
   *  @return bool
   */
  bool isUSART( const std::uintptr_t address );

}    // namespace Thor::Driver::USART

#endif /* TARGET_STM32F4 && THOR_DRIVER_USART */
#endif /* !THOR_HW_USART_MAPPING_HPP */