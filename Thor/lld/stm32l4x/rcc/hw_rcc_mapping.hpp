/********************************************************************************
 *  File Name:
 *    hw_rcc_mapping.hpp
 *
 *  Description:
 *    Defines structures used to map various resources needed in the RCC driver
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_RCC_MAPPING_HPP
#define THOR_LLD_RCC_MAPPING_HPP

/* C++ Includes */
#include <array>

/* Chimera Includes */
#include <Chimera/clock>
#include <Chimera/common>
#include <Chimera/container>

/* Driver Includes */
#include <Thor/lld/stm32l4x/flash/hw_flash_prj.hpp>
#include <Thor/lld/stm32l4x/gpio/hw_gpio_prj.hpp>
#include <Thor/lld/stm32l4x/power/hw_power_prj.hpp>
#include <Thor/lld/stm32l4x/spi/hw_spi_prj.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_prj.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_types.hpp>
#include <Thor/lld/stm32l4x/system/sys_memory_map_prj.hpp>

namespace Thor::LLD::RCC
{
#if defined( STM32_RCC1_PERIPH_AVAILABLE )
  extern RegisterMap *RCC1_PERIPH;
#endif

  /*------------------------------------------------
  Peripheral Memory Mapping
  ------------------------------------------------*/
  extern PeriphRegisterList PeripheralList;
  extern Chimera::Container::LightFlatMap<std::uintptr_t, size_t> InstanceToResourceIndex;

  /**
   *  Initializes memory associated with mapping
   *  
   *  @return void
   */
  extern void initializeMapping();

  /**
   *  Checks if the given address belongs to a peripheral instance
   *
   *  @return bool
   */
  extern bool isRCC( const std::uintptr_t address );


  namespace LookupTables
  {
#if defined( THOR_LLD_FLASH )
    /**
     *  Power Peripheral Config Lookup Tables
     */
    extern void FLASHInit();

    extern PCC FLASHLookup;
    extern RegisterConfig FLASH_ClockConfig[ Thor::LLD::FLASH::NUM_FLASH_PERIPHS ];
    extern RegisterConfig FLASH_ResetConfig[ Thor::LLD::FLASH::NUM_FLASH_PERIPHS ];
    extern Chimera::Clock::Bus FLASH_SourceClock[ Thor::LLD::FLASH::NUM_FLASH_PERIPHS ];
#endif /* THOR_LLD_FLASH */

#if defined( THOR_LLD_GPIO )
    /**
     *  GPIO Peripheral Config Lookup Tables
     */
    extern void GPIOInit();

    extern PCC GPIOLookup;
    extern RegisterConfig GPIO_ClockConfig[ Thor::LLD::GPIO::NUM_GPIO_PERIPHS ];
    extern RegisterConfig GPIO_ResetConfig[ Thor::LLD::GPIO::NUM_GPIO_PERIPHS ];
    extern Chimera::Clock::Bus GPIO_SourceClock[ Thor::LLD::GPIO::NUM_GPIO_PERIPHS ];
#endif /* THOR_LLD_GPIO */

#if defined( THOR_LLD_PWR )
    /**
     *  Power Peripheral Config Lookup Tables
     */
    extern void PWRInit();

    extern PCC PWRLookup;
    extern RegisterConfig PWR_ClockConfig[ Thor::LLD::PWR::NUM_POWER_PERIPHS ];
    extern RegisterConfig PWR_ResetConfig[ Thor::LLD::PWR::NUM_POWER_PERIPHS ];
    extern Chimera::Clock::Bus PWR_SourceClock[ Thor::LLD::PWR::NUM_POWER_PERIPHS ];
#endif /* THOR_LLD_PWR */

#if defined( THOR_LLD_SPI )
    /**
     *  SPI Peripheral Config Lookup Tables
     */
    extern void SPIInit();

    extern PCC SPILookup;
    extern RegisterConfig SPI_ClockConfig[ Thor::LLD::SPI::NUM_SPI_PERIPHS ];
    extern RegisterConfig SPI_ResetConfig[ Thor::LLD::SPI::NUM_SPI_PERIPHS ];
    extern Chimera::Clock::Bus SPI_SourceClock[ Thor::LLD::SPI::NUM_SPI_PERIPHS ];
#endif /* THOR_LLD_SPI */
  }
}

#endif  /* !THOR_LLD_RCC_MAPPING_HPP */
