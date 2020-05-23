/********************************************************************************
 *  File Name:
 *    hw_usart_mapping.hpp
 *
 *  Description:
 *    Provides structures for conversion and mapping between data types for fast
 *    runtime performance of driver code.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_USART_MAPPING_HPP
#define THOR_HW_USART_MAPPING_HPP

/* STL Includes */
#include <array>

/* Chimera Includes */
#include <Chimera/container>
#include <Chimera/usart>

/* Driver Includes */
#include <Thor/lld/interface/usart/usart_types.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_types.hpp>
#include <Thor/lld/stm32l4x/usart/hw_usart_types.hpp>

namespace Thor::LLD::USART
{
  /*-------------------------------------------------
  Define Memory Mapped Structs for Peripheral
  -------------------------------------------------*/
#if defined( STM32_USART1_PERIPH_AVAILABLE )
  extern RegisterMap *USART1_PERIPH;
#endif

#if defined( STM32_USART2_PERIPH_AVAILABLE )
  extern RegisterMap *USART2_PERIPH;
#endif

#if defined( STM32_USART3_PERIPH_AVAILABLE )
  extern RegisterMap *USART3_PERIPH;
#endif

  /*------------------------------------------------
  Hardware Memory Mappings
  ------------------------------------------------*/
  extern PeriphRegisterList PeripheralList;
  extern DriverInstanceList usartObjects;
  extern DMASignalList RXDMASignals;
  extern DMASignalList TXDMASignals;
  extern IRQSignalList IRQSignals;
  extern Thor::LLD::RIndexMap InstanceToResourceIndex;
  extern Chimera::Container::LightFlatMap<Chimera::Serial::Channel, RegisterMap*> ChannelToInstance;

  /*------------------------------------------------
  Mappings from Chimera Config Options->Register Values
  ------------------------------------------------*/
  extern std::array<uint32_t, static_cast<size_t>( Chimera::Serial::CharWid::NUM_OPTIONS )> CharWidToRegConfig;
  extern std::array<uint32_t, static_cast<size_t>( Chimera::Serial::Parity::NUM_OPTIONS )> ParityToRegConfig;
  extern std::array<uint32_t, static_cast<size_t>( Chimera::Serial::StopBits::NUM_OPTIONS )> StopBitsToRegConfig;

  /*-------------------------------------------------
  Module Functions
  -------------------------------------------------*/
  /**
   *  Initializes memory associated with mapping
   *
   *  @return void
   */
  void initializeMapping();
  
}    // namespace Thor::LLD::USART

#endif /* !THOR_HW_USART_MAPPING_HPP */
