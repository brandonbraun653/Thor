/********************************************************************************
 *  File Name:
 *    hw_timer_register_stm32l432kc.hpp
 *
 *  Description:
 *    TIMER register definitions for the STM32L432KC series chips.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_TIMER_REGISTER_STM32L432KC_HPP
#define THOR_HW_TIMER_REGISTER_STM32L432KC_HPP

/* C++ Includes */
#include <array>
#include <cstdint>
#include <cstddef>

/* Chimera Includes */
#include <Chimera/timer>

/* Driver Includes */
#include <Thor/hld/common/types.hpp>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/stm32l4x/system/variant/sys_memory_map_stm32l432kc.hpp>

/*-------------------------------------------------
Peripheral Availability
-------------------------------------------------*/
// Available on every STM32L4 device
#define STM32_TIMER1_PERIPH_AVAILABLE
#define STM32_TIMER2_PERIPH_AVAILABLE
#define STM32_TIMER6_PERIPH_AVAILABLE
#define STM32_TIMER15_PERIPH_AVAILABLE
#define STM32_TIMER16_PERIPH_AVAILABLE
#define STM32_LPTIMER1_PERIPH_AVAILABLE
#define STM32_LPTIMER2_PERIPH_AVAILABLE

// Only available on STM32L45xxx and STM32L46xxx devices (none supported currently)
//#define STM32_TIMER3_PERIPH_AVAILABLE

#if defined( STM32L432xx )
#define STM32_TIMER7_PERIPH_AVAILABLE
#endif


namespace Thor::LLD::TIMER
{
  /**
   *  Initializes the LLD register resources and memory
   *
   *  @return void
   */
  void initializeRegisters();

  /*------------------------------------------------
  Configuration Data
  ------------------------------------------------*/
  static constexpr size_t NUM_TIMER_PERIPHS     = 9;
  static constexpr size_t NUM_ADVANCED_PERIPHS  = 1;
  static constexpr size_t NUM_BASIC_PERIPHS     = 2;
  static constexpr size_t NUM_GENERAL_PERIPHS   = 4;
  static constexpr size_t NUM_LOW_POWER_PERIPHS = 2;

  /*-------------------------------------------------
  Peripheral Instance Memory Map Base
  -------------------------------------------------*/
  static constexpr uint32_t TIMER1_BASE_ADDR   = Thor::System::MemoryMap::TIMER1_PERIPH_START_ADDRESS;
  static constexpr uint32_t TIMER2_BASE_ADDR   = Thor::System::MemoryMap::TIMER2_PERIPH_START_ADDRESS;
  static constexpr uint32_t TIMER6_BASE_ADDR   = Thor::System::MemoryMap::TIMER6_PERIPH_START_ADDRESS;
  static constexpr uint32_t TIMER15_BASE_ADDR  = Thor::System::MemoryMap::TIMER15_PERIPH_START_ADDRESS;
  static constexpr uint32_t TIMER16_BASE_ADDR  = Thor::System::MemoryMap::TIMER16_PERIPH_START_ADDRESS;
  static constexpr uint32_t LPTIMER1_BASE_ADDR = Thor::System::MemoryMap::LPTIMER1_PERIPH_START_ADDRESS;
  static constexpr uint32_t LPTIMER2_BASE_ADDR = Thor::System::MemoryMap::LPTIMER2_PERIPH_START_ADDRESS;

#if defined( STM32_TIMER3_PERIPH_AVAILABLE )
  static constexpr uint32_t TIMER3_BASE_ADDR = Thor::System::MemoryMap::TIMER3_PERIPH_START_ADDRESS;
#else
  static constexpr uint32_t TIMER3_BASE_ADDR = std::numeric_limits<uint32_t>::max();
#endif

#if defined( STM32_TIMER7_PERIPH_AVAILABLE )
  static constexpr uint32_t TIMER7_BASE_ADDR = Thor::System::MemoryMap::TIMER7_PERIPH_START_ADDRESS;
#else
  static constexpr uint32_t TIMER7_BASE_ADDR = std::numeric_limits<uint32_t>::max();
#endif

  /*-------------------------------------------------
  Peripheral Resource Lookup Indices
  -------------------------------------------------*/
  // static constexpr Thor::HLD::RIndex ADVANCED_TIMER1_RESOURCE_INDEX = Thor::HLD::RIndex( 0u );;

  // static constexpr Thor::HLD::RIndex BASIC_TIMER1_RESOURCE_INDEX = Thor::HLD::RIndex( 0u );
  // static constexpr Thor::HLD::RIndex BASIC_TIMER2_RESOURCE_INDEX = Thor::HLD::RIndex( 1u );

  // static constexpr Thor::HLD::RIndex GENERAL_TIMER1_RESOURCE_INDEX = Thor::HLD::RIndex( 0u );
  // static constexpr Thor::HLD::RIndex GENERAL_TIMER2_RESOURCE_INDEX = Thor::HLD::RIndex( 1u );
  // static constexpr Thor::HLD::RIndex GENERAL_TIMER3_RESOURCE_INDEX = Thor::HLD::RIndex( 2u );
  // static constexpr Thor::HLD::RIndex GENERAL_TIMER4_RESOURCE_INDEX = Thor::HLD::RIndex( 3u );

  // static constexpr Thor::HLD::RIndex LOW_POWER_TIMER1_RESOURCE_INDEX = Thor::HLD::RIndex( 0u );
  // static constexpr Thor::HLD::RIndex LOW_POWER_TIMER2_RESOURCE_INDEX = Thor::HLD::RIndex( 1u );

  // static constexpr Thor::LLD::RIndex TIMER1_RESOURCE_INDEX   = Thor::LLD::RIndex( 0u );
  // static constexpr Thor::LLD::RIndex TIMER2_RESOURCE_INDEX   = Thor::LLD::RIndex( 1u );
  // static constexpr Thor::LLD::RIndex TIMER3_RESOURCE_INDEX   = Thor::LLD::RIndex( 2u );
  // static constexpr Thor::LLD::RIndex TIMER6_RESOURCE_INDEX   = Thor::LLD::RIndex( 3u );
  // static constexpr Thor::LLD::RIndex TIMER7_RESOURCE_INDEX   = Thor::LLD::RIndex( 4u );
  // static constexpr Thor::LLD::RIndex TIMER15_RESOURCE_INDEX  = Thor::LLD::RIndex( 5u );
  // static constexpr Thor::LLD::RIndex TIMER16_RESOURCE_INDEX  = Thor::LLD::RIndex( 6u );
  // static constexpr Thor::LLD::RIndex LPTIMER1_RESOURCE_INDEX = Thor::LLD::RIndex( 7u );
  // static constexpr Thor::LLD::RIndex LPTIMER2_RESOURCE_INDEX = Thor::LLD::RIndex( 8u );

  /*-------------------------------------------------
  Lookup addresses
  -------------------------------------------------*/

  static constexpr std::array<uint32_t, NUM_TIMER_PERIPHS> periphAddressList = {
    TIMER1_BASE_ADDR,  TIMER2_BASE_ADDR,  TIMER3_BASE_ADDR,   TIMER6_BASE_ADDR,  TIMER7_BASE_ADDR,
    TIMER15_BASE_ADDR, TIMER16_BASE_ADDR, LPTIMER1_BASE_ADDR, LPTIMER2_BASE_ADDR
  };

  /*------------------------------------------------
  Supported Hardware Channels
  ------------------------------------------------*/
  extern const std::array<Chimera::Timer::Peripheral, NUM_TIMER_PERIPHS> supportedChannels;

  /*-------------------------------------------------
  Peripheral Register Definitions
  -------------------------------------------------*/
  /*******************  Bit definition for TIM_CR1 register  ********************/
  static constexpr uint32_t TIM_CR1_CEN_Pos  = ( 0U );
  static constexpr uint32_t TIM_CR1_CEN_Msk  = ( 0x1UL << TIM_CR1_CEN_Pos );
  static constexpr uint32_t TIM_CR1_CEN      = TIM_CR1_CEN_Msk;
  static constexpr uint32_t TIM_CR1_UDIS_Pos = ( 1U );
  static constexpr uint32_t TIM_CR1_UDIS_Msk = ( 0x1UL << TIM_CR1_UDIS_Pos );
  static constexpr uint32_t TIM_CR1_UDIS     = TIM_CR1_UDIS_Msk;
  static constexpr uint32_t TIM_CR1_URS_Pos  = ( 2U );
  static constexpr uint32_t TIM_CR1_URS_Msk  = ( 0x1UL << TIM_CR1_URS_Pos );
  static constexpr uint32_t TIM_CR1_URS      = TIM_CR1_URS_Msk;
  static constexpr uint32_t TIM_CR1_OPM_Pos  = ( 3U );
  static constexpr uint32_t TIM_CR1_OPM_Msk  = ( 0x1UL << TIM_CR1_OPM_Pos );
  static constexpr uint32_t TIM_CR1_OPM      = TIM_CR1_OPM_Msk;
  static constexpr uint32_t TIM_CR1_DIR_Pos  = ( 4U );
  static constexpr uint32_t TIM_CR1_DIR_Msk  = ( 0x1UL << TIM_CR1_DIR_Pos );
  static constexpr uint32_t TIM_CR1_DIR      = TIM_CR1_DIR_Msk;

  static constexpr uint32_t TIM_CR1_CMS_Pos = ( 5U );
  static constexpr uint32_t TIM_CR1_CMS_Msk = ( 0x3UL << TIM_CR1_CMS_Pos );
  static constexpr uint32_t TIM_CR1_CMS     = TIM_CR1_CMS_Msk;
  static constexpr uint32_t TIM_CR1_CMS_0   = ( 0x1UL << TIM_CR1_CMS_Pos );
  static constexpr uint32_t TIM_CR1_CMS_1   = ( 0x2UL << TIM_CR1_CMS_Pos );

  static constexpr uint32_t TIM_CR1_ARPE_Pos = ( 7U );
  static constexpr uint32_t TIM_CR1_ARPE_Msk = ( 0x1UL << TIM_CR1_ARPE_Pos );
  static constexpr uint32_t TIM_CR1_ARPE     = TIM_CR1_ARPE_Msk;

  static constexpr uint32_t TIM_CR1_CKD_Pos = ( 8U );
  static constexpr uint32_t TIM_CR1_CKD_Msk = ( 0x3UL << TIM_CR1_CKD_Pos );
  static constexpr uint32_t TIM_CR1_CKD     = TIM_CR1_CKD_Msk;
  static constexpr uint32_t TIM_CR1_CKD_0   = ( 0x1UL << TIM_CR1_CKD_Pos );
  static constexpr uint32_t TIM_CR1_CKD_1   = ( 0x2UL << TIM_CR1_CKD_Pos );

  static constexpr uint32_t TIM_CR1_UIFREMAP_Pos = ( 11U );
  static constexpr uint32_t TIM_CR1_UIFREMAP_Msk = ( 0x1UL << TIM_CR1_UIFREMAP_Pos );
  static constexpr uint32_t TIM_CR1_UIFREMAP     = TIM_CR1_UIFREMAP_Msk;

  /*******************  Bit definition for TIM_CR2 register  ********************/
  static constexpr uint32_t TIM_CR2_CCPC_Pos = ( 0U );
  static constexpr uint32_t TIM_CR2_CCPC_Msk = ( 0x1UL << TIM_CR2_CCPC_Pos );
  static constexpr uint32_t TIM_CR2_CCPC     = TIM_CR2_CCPC_Msk;
  static constexpr uint32_t TIM_CR2_CCUS_Pos = ( 2U );
  static constexpr uint32_t TIM_CR2_CCUS_Msk = ( 0x1UL << TIM_CR2_CCUS_Pos );
  static constexpr uint32_t TIM_CR2_CCUS     = TIM_CR2_CCUS_Msk;
  static constexpr uint32_t TIM_CR2_CCDS_Pos = ( 3U );
  static constexpr uint32_t TIM_CR2_CCDS_Msk = ( 0x1UL << TIM_CR2_CCDS_Pos );
  static constexpr uint32_t TIM_CR2_CCDS     = TIM_CR2_CCDS_Msk;

  static constexpr uint32_t TIM_CR2_MMS_Pos = ( 4U );
  static constexpr uint32_t TIM_CR2_MMS_Msk = ( 0x7UL << TIM_CR2_MMS_Pos );
  static constexpr uint32_t TIM_CR2_MMS     = TIM_CR2_MMS_Msk;
  static constexpr uint32_t TIM_CR2_MMS_0   = ( 0x1UL << TIM_CR2_MMS_Pos );
  static constexpr uint32_t TIM_CR2_MMS_1   = ( 0x2UL << TIM_CR2_MMS_Pos );
  static constexpr uint32_t TIM_CR2_MMS_2   = ( 0x4UL << TIM_CR2_MMS_Pos );

  static constexpr uint32_t TIM_CR2_TI1S_Pos  = ( 7U );
  static constexpr uint32_t TIM_CR2_TI1S_Msk  = ( 0x1UL << TIM_CR2_TI1S_Pos );
  static constexpr uint32_t TIM_CR2_TI1S      = TIM_CR2_TI1S_Msk;
  static constexpr uint32_t TIM_CR2_OIS1_Pos  = ( 8U );
  static constexpr uint32_t TIM_CR2_OIS1_Msk  = ( 0x1UL << TIM_CR2_OIS1_Pos );
  static constexpr uint32_t TIM_CR2_OIS1      = TIM_CR2_OIS1_Msk;
  static constexpr uint32_t TIM_CR2_OIS1N_Pos = ( 9U );
  static constexpr uint32_t TIM_CR2_OIS1N_Msk = ( 0x1UL << TIM_CR2_OIS1N_Pos );
  static constexpr uint32_t TIM_CR2_OIS1N     = TIM_CR2_OIS1N_Msk;
  static constexpr uint32_t TIM_CR2_OIS2_Pos  = ( 10U );
  static constexpr uint32_t TIM_CR2_OIS2_Msk  = ( 0x1UL << TIM_CR2_OIS2_Pos );
  static constexpr uint32_t TIM_CR2_OIS2      = TIM_CR2_OIS2_Msk;
  static constexpr uint32_t TIM_CR2_OIS2N_Pos = ( 11U );
  static constexpr uint32_t TIM_CR2_OIS2N_Msk = ( 0x1UL << TIM_CR2_OIS2N_Pos );
  static constexpr uint32_t TIM_CR2_OIS2N     = TIM_CR2_OIS2N_Msk;
  static constexpr uint32_t TIM_CR2_OIS3_Pos  = ( 12U );
  static constexpr uint32_t TIM_CR2_OIS3_Msk  = ( 0x1UL << TIM_CR2_OIS3_Pos );
  static constexpr uint32_t TIM_CR2_OIS3      = TIM_CR2_OIS3_Msk;
  static constexpr uint32_t TIM_CR2_OIS3N_Pos = ( 13U );
  static constexpr uint32_t TIM_CR2_OIS3N_Msk = ( 0x1UL << TIM_CR2_OIS3N_Pos );
  static constexpr uint32_t TIM_CR2_OIS3N     = TIM_CR2_OIS3N_Msk;
  static constexpr uint32_t TIM_CR2_OIS4_Pos  = ( 14U );
  static constexpr uint32_t TIM_CR2_OIS4_Msk  = ( 0x1UL << TIM_CR2_OIS4_Pos );
  static constexpr uint32_t TIM_CR2_OIS4      = TIM_CR2_OIS4_Msk;
  static constexpr uint32_t TIM_CR2_OIS5_Pos  = ( 16U );
  static constexpr uint32_t TIM_CR2_OIS5_Msk  = ( 0x1UL << TIM_CR2_OIS5_Pos );
  static constexpr uint32_t TIM_CR2_OIS5      = TIM_CR2_OIS5_Msk;
  static constexpr uint32_t TIM_CR2_OIS6_Pos  = ( 18U );
  static constexpr uint32_t TIM_CR2_OIS6_Msk  = ( 0x1UL << TIM_CR2_OIS6_Pos );
  static constexpr uint32_t TIM_CR2_OIS6      = TIM_CR2_OIS6_Msk;

  static constexpr uint32_t TIM_CR2_MMS2_Pos = ( 20U );
  static constexpr uint32_t TIM_CR2_MMS2_Msk = ( 0xFUL << TIM_CR2_MMS2_Pos );
  static constexpr uint32_t TIM_CR2_MMS2     = TIM_CR2_MMS2_Msk;
  static constexpr uint32_t TIM_CR2_MMS2_0   = ( 0x1UL << TIM_CR2_MMS2_Pos );
  static constexpr uint32_t TIM_CR2_MMS2_1   = ( 0x2UL << TIM_CR2_MMS2_Pos );
  static constexpr uint32_t TIM_CR2_MMS2_2   = ( 0x4UL << TIM_CR2_MMS2_Pos );
  static constexpr uint32_t TIM_CR2_MMS2_3   = ( 0x8UL << TIM_CR2_MMS2_Pos );

  /*******************  Bit definition for TIM_SMCR register  *******************/
  static constexpr uint32_t TIM_SMCR_SMS_Pos = ( 0U );
  static constexpr uint32_t TIM_SMCR_SMS_Msk = ( 0x10007UL << TIM_SMCR_SMS_Pos );
  static constexpr uint32_t TIM_SMCR_SMS     = TIM_SMCR_SMS_Msk;
  static constexpr uint32_t TIM_SMCR_SMS_0   = ( 0x00001UL << TIM_SMCR_SMS_Pos );
  static constexpr uint32_t TIM_SMCR_SMS_1   = ( 0x00002UL << TIM_SMCR_SMS_Pos );
  static constexpr uint32_t TIM_SMCR_SMS_2   = ( 0x00004UL << TIM_SMCR_SMS_Pos );
  static constexpr uint32_t TIM_SMCR_SMS_3   = ( 0x10000UL << TIM_SMCR_SMS_Pos );

  static constexpr uint32_t TIM_SMCR_OCCS_Pos = ( 3U );
  static constexpr uint32_t TIM_SMCR_OCCS_Msk = ( 0x1UL << TIM_SMCR_OCCS_Pos );
  static constexpr uint32_t TIM_SMCR_OCCS     = TIM_SMCR_OCCS_Msk;

  static constexpr uint32_t TIM_SMCR_TS_Pos = ( 4U );
  static constexpr uint32_t TIM_SMCR_TS_Msk = ( 0x7UL << TIM_SMCR_TS_Pos );
  static constexpr uint32_t TIM_SMCR_TS     = TIM_SMCR_TS_Msk;
  static constexpr uint32_t TIM_SMCR_TS_0   = ( 0x1UL << TIM_SMCR_TS_Pos );
  static constexpr uint32_t TIM_SMCR_TS_1   = ( 0x2UL << TIM_SMCR_TS_Pos );
  static constexpr uint32_t TIM_SMCR_TS_2   = ( 0x4UL << TIM_SMCR_TS_Pos );

  static constexpr uint32_t TIM_SMCR_MSM_Pos = ( 7U );
  static constexpr uint32_t TIM_SMCR_MSM_Msk = ( 0x1UL << TIM_SMCR_MSM_Pos );
  static constexpr uint32_t TIM_SMCR_MSM     = TIM_SMCR_MSM_Msk;

  static constexpr uint32_t TIM_SMCR_ETF_Pos = ( 8U );
  static constexpr uint32_t TIM_SMCR_ETF_Msk = ( 0xFUL << TIM_SMCR_ETF_Pos );
  static constexpr uint32_t TIM_SMCR_ETF     = TIM_SMCR_ETF_Msk;
  static constexpr uint32_t TIM_SMCR_ETF_0   = ( 0x1UL << TIM_SMCR_ETF_Pos );
  static constexpr uint32_t TIM_SMCR_ETF_1   = ( 0x2UL << TIM_SMCR_ETF_Pos );
  static constexpr uint32_t TIM_SMCR_ETF_2   = ( 0x4UL << TIM_SMCR_ETF_Pos );
  static constexpr uint32_t TIM_SMCR_ETF_3   = ( 0x8UL << TIM_SMCR_ETF_Pos );

  static constexpr uint32_t TIM_SMCR_ETPS_Pos = ( 12U );
  static constexpr uint32_t TIM_SMCR_ETPS_Msk = ( 0x3UL << TIM_SMCR_ETPS_Pos );
  static constexpr uint32_t TIM_SMCR_ETPS     = TIM_SMCR_ETPS_Msk;
  static constexpr uint32_t TIM_SMCR_ETPS_0   = ( 0x1UL << TIM_SMCR_ETPS_Pos );
  static constexpr uint32_t TIM_SMCR_ETPS_1   = ( 0x2UL << TIM_SMCR_ETPS_Pos );

  static constexpr uint32_t TIM_SMCR_ECE_Pos = ( 14U );
  static constexpr uint32_t TIM_SMCR_ECE_Msk = ( 0x1UL << TIM_SMCR_ECE_Pos );
  static constexpr uint32_t TIM_SMCR_ECE     = TIM_SMCR_ECE_Msk;
  static constexpr uint32_t TIM_SMCR_ETP_Pos = ( 15U );
  static constexpr uint32_t TIM_SMCR_ETP_Msk = ( 0x1UL << TIM_SMCR_ETP_Pos );
  static constexpr uint32_t TIM_SMCR_ETP     = TIM_SMCR_ETP_Msk;

  /*******************  Bit definition for TIM_DIER register  *******************/
  static constexpr uint32_t TIM_DIER_UIE_Pos   = ( 0U );
  static constexpr uint32_t TIM_DIER_UIE_Msk   = ( 0x1UL << TIM_DIER_UIE_Pos );
  static constexpr uint32_t TIM_DIER_UIE       = TIM_DIER_UIE_Msk;
  static constexpr uint32_t TIM_DIER_CC1IE_Pos = ( 1U );
  static constexpr uint32_t TIM_DIER_CC1IE_Msk = ( 0x1UL << TIM_DIER_CC1IE_Pos );
  static constexpr uint32_t TIM_DIER_CC1IE     = TIM_DIER_CC1IE_Msk;
  static constexpr uint32_t TIM_DIER_CC2IE_Pos = ( 2U );
  static constexpr uint32_t TIM_DIER_CC2IE_Msk = ( 0x1UL << TIM_DIER_CC2IE_Pos );
  static constexpr uint32_t TIM_DIER_CC2IE     = TIM_DIER_CC2IE_Msk;
  static constexpr uint32_t TIM_DIER_CC3IE_Pos = ( 3U );
  static constexpr uint32_t TIM_DIER_CC3IE_Msk = ( 0x1UL << TIM_DIER_CC3IE_Pos );
  static constexpr uint32_t TIM_DIER_CC3IE     = TIM_DIER_CC3IE_Msk;
  static constexpr uint32_t TIM_DIER_CC4IE_Pos = ( 4U );
  static constexpr uint32_t TIM_DIER_CC4IE_Msk = ( 0x1UL << TIM_DIER_CC4IE_Pos );
  static constexpr uint32_t TIM_DIER_CC4IE     = TIM_DIER_CC4IE_Msk;
  static constexpr uint32_t TIM_DIER_COMIE_Pos = ( 5U );
  static constexpr uint32_t TIM_DIER_COMIE_Msk = ( 0x1UL << TIM_DIER_COMIE_Pos );
  static constexpr uint32_t TIM_DIER_COMIE     = TIM_DIER_COMIE_Msk;
  static constexpr uint32_t TIM_DIER_TIE_Pos   = ( 6U );
  static constexpr uint32_t TIM_DIER_TIE_Msk   = ( 0x1UL << TIM_DIER_TIE_Pos );
  static constexpr uint32_t TIM_DIER_TIE       = TIM_DIER_TIE_Msk;
  static constexpr uint32_t TIM_DIER_BIE_Pos   = ( 7U );
  static constexpr uint32_t TIM_DIER_BIE_Msk   = ( 0x1UL << TIM_DIER_BIE_Pos );
  static constexpr uint32_t TIM_DIER_BIE       = TIM_DIER_BIE_Msk;
  static constexpr uint32_t TIM_DIER_UDE_Pos   = ( 8U );
  static constexpr uint32_t TIM_DIER_UDE_Msk   = ( 0x1UL << TIM_DIER_UDE_Pos );
  static constexpr uint32_t TIM_DIER_UDE       = TIM_DIER_UDE_Msk;
  static constexpr uint32_t TIM_DIER_CC1DE_Pos = ( 9U );
  static constexpr uint32_t TIM_DIER_CC1DE_Msk = ( 0x1UL << TIM_DIER_CC1DE_Pos );
  static constexpr uint32_t TIM_DIER_CC1DE     = TIM_DIER_CC1DE_Msk;
  static constexpr uint32_t TIM_DIER_CC2DE_Pos = ( 10U );
  static constexpr uint32_t TIM_DIER_CC2DE_Msk = ( 0x1UL << TIM_DIER_CC2DE_Pos );
  static constexpr uint32_t TIM_DIER_CC2DE     = TIM_DIER_CC2DE_Msk;
  static constexpr uint32_t TIM_DIER_CC3DE_Pos = ( 11U );
  static constexpr uint32_t TIM_DIER_CC3DE_Msk = ( 0x1UL << TIM_DIER_CC3DE_Pos );
  static constexpr uint32_t TIM_DIER_CC3DE     = TIM_DIER_CC3DE_Msk;
  static constexpr uint32_t TIM_DIER_CC4DE_Pos = ( 12U );
  static constexpr uint32_t TIM_DIER_CC4DE_Msk = ( 0x1UL << TIM_DIER_CC4DE_Pos );
  static constexpr uint32_t TIM_DIER_CC4DE     = TIM_DIER_CC4DE_Msk;
  static constexpr uint32_t TIM_DIER_COMDE_Pos = ( 13U );
  static constexpr uint32_t TIM_DIER_COMDE_Msk = ( 0x1UL << TIM_DIER_COMDE_Pos );
  static constexpr uint32_t TIM_DIER_COMDE     = TIM_DIER_COMDE_Msk;
  static constexpr uint32_t TIM_DIER_TDE_Pos   = ( 14U );
  static constexpr uint32_t TIM_DIER_TDE_Msk   = ( 0x1UL << TIM_DIER_TDE_Pos );
  static constexpr uint32_t TIM_DIER_TDE       = TIM_DIER_TDE_Msk;

  /********************  Bit definition for TIM_SR register  ********************/
  static constexpr uint32_t TIM_SR_UIF_Pos   = ( 0U );
  static constexpr uint32_t TIM_SR_UIF_Msk   = ( 0x1UL << TIM_SR_UIF_Pos );
  static constexpr uint32_t TIM_SR_UIF       = TIM_SR_UIF_Msk;
  static constexpr uint32_t TIM_SR_CC1IF_Pos = ( 1U );
  static constexpr uint32_t TIM_SR_CC1IF_Msk = ( 0x1UL << TIM_SR_CC1IF_Pos );
  static constexpr uint32_t TIM_SR_CC1IF     = TIM_SR_CC1IF_Msk;
  static constexpr uint32_t TIM_SR_CC2IF_Pos = ( 2U );
  static constexpr uint32_t TIM_SR_CC2IF_Msk = ( 0x1UL << TIM_SR_CC2IF_Pos );
  static constexpr uint32_t TIM_SR_CC2IF     = TIM_SR_CC2IF_Msk;
  static constexpr uint32_t TIM_SR_CC3IF_Pos = ( 3U );
  static constexpr uint32_t TIM_SR_CC3IF_Msk = ( 0x1UL << TIM_SR_CC3IF_Pos );
  static constexpr uint32_t TIM_SR_CC3IF     = TIM_SR_CC3IF_Msk;
  static constexpr uint32_t TIM_SR_CC4IF_Pos = ( 4U );
  static constexpr uint32_t TIM_SR_CC4IF_Msk = ( 0x1UL << TIM_SR_CC4IF_Pos );
  static constexpr uint32_t TIM_SR_CC4IF     = TIM_SR_CC4IF_Msk;
  static constexpr uint32_t TIM_SR_COMIF_Pos = ( 5U );
  static constexpr uint32_t TIM_SR_COMIF_Msk = ( 0x1UL << TIM_SR_COMIF_Pos );
  static constexpr uint32_t TIM_SR_COMIF     = TIM_SR_COMIF_Msk;
  static constexpr uint32_t TIM_SR_TIF_Pos   = ( 6U );
  static constexpr uint32_t TIM_SR_TIF_Msk   = ( 0x1UL << TIM_SR_TIF_Pos );
  static constexpr uint32_t TIM_SR_TIF       = TIM_SR_TIF_Msk;
  static constexpr uint32_t TIM_SR_BIF_Pos   = ( 7U );
  static constexpr uint32_t TIM_SR_BIF_Msk   = ( 0x1UL << TIM_SR_BIF_Pos );
  static constexpr uint32_t TIM_SR_BIF       = TIM_SR_BIF_Msk;
  static constexpr uint32_t TIM_SR_B2IF_Pos  = ( 8U );
  static constexpr uint32_t TIM_SR_B2IF_Msk  = ( 0x1UL << TIM_SR_B2IF_Pos );
  static constexpr uint32_t TIM_SR_B2IF      = TIM_SR_B2IF_Msk;
  static constexpr uint32_t TIM_SR_CC1OF_Pos = ( 9U );
  static constexpr uint32_t TIM_SR_CC1OF_Msk = ( 0x1UL << TIM_SR_CC1OF_Pos );
  static constexpr uint32_t TIM_SR_CC1OF     = TIM_SR_CC1OF_Msk;
  static constexpr uint32_t TIM_SR_CC2OF_Pos = ( 10U );
  static constexpr uint32_t TIM_SR_CC2OF_Msk = ( 0x1UL << TIM_SR_CC2OF_Pos );
  static constexpr uint32_t TIM_SR_CC2OF     = TIM_SR_CC2OF_Msk;
  static constexpr uint32_t TIM_SR_CC3OF_Pos = ( 11U );
  static constexpr uint32_t TIM_SR_CC3OF_Msk = ( 0x1UL << TIM_SR_CC3OF_Pos );
  static constexpr uint32_t TIM_SR_CC3OF     = TIM_SR_CC3OF_Msk;
  static constexpr uint32_t TIM_SR_CC4OF_Pos = ( 12U );
  static constexpr uint32_t TIM_SR_CC4OF_Msk = ( 0x1UL << TIM_SR_CC4OF_Pos );
  static constexpr uint32_t TIM_SR_CC4OF     = TIM_SR_CC4OF_Msk;
  static constexpr uint32_t TIM_SR_SBIF_Pos  = ( 13U );
  static constexpr uint32_t TIM_SR_SBIF_Msk  = ( 0x1UL << TIM_SR_SBIF_Pos );
  static constexpr uint32_t TIM_SR_SBIF      = TIM_SR_SBIF_Msk;
  static constexpr uint32_t TIM_SR_CC5IF_Pos = ( 16U );
  static constexpr uint32_t TIM_SR_CC5IF_Msk = ( 0x1UL << TIM_SR_CC5IF_Pos );
  static constexpr uint32_t TIM_SR_CC5IF     = TIM_SR_CC5IF_Msk;
  static constexpr uint32_t TIM_SR_CC6IF_Pos = ( 17U );
  static constexpr uint32_t TIM_SR_CC6IF_Msk = ( 0x1UL << TIM_SR_CC6IF_Pos );
  static constexpr uint32_t TIM_SR_CC6IF     = TIM_SR_CC6IF_Msk;

  /*******************  Bit definition for TIM_EGR register  ********************/
  static constexpr uint32_t TIM_EGR_UG_Pos   = ( 0U );
  static constexpr uint32_t TIM_EGR_UG_Msk   = ( 0x1UL << TIM_EGR_UG_Pos );
  static constexpr uint32_t TIM_EGR_UG       = TIM_EGR_UG_Msk;
  static constexpr uint32_t TIM_EGR_CC1G_Pos = ( 1U );
  static constexpr uint32_t TIM_EGR_CC1G_Msk = ( 0x1UL << TIM_EGR_CC1G_Pos );
  static constexpr uint32_t TIM_EGR_CC1G     = TIM_EGR_CC1G_Msk;
  static constexpr uint32_t TIM_EGR_CC2G_Pos = ( 2U );
  static constexpr uint32_t TIM_EGR_CC2G_Msk = ( 0x1UL << TIM_EGR_CC2G_Pos );
  static constexpr uint32_t TIM_EGR_CC2G     = TIM_EGR_CC2G_Msk;
  static constexpr uint32_t TIM_EGR_CC3G_Pos = ( 3U );
  static constexpr uint32_t TIM_EGR_CC3G_Msk = ( 0x1UL << TIM_EGR_CC3G_Pos );
  static constexpr uint32_t TIM_EGR_CC3G     = TIM_EGR_CC3G_Msk;
  static constexpr uint32_t TIM_EGR_CC4G_Pos = ( 4U );
  static constexpr uint32_t TIM_EGR_CC4G_Msk = ( 0x1UL << TIM_EGR_CC4G_Pos );
  static constexpr uint32_t TIM_EGR_CC4G     = TIM_EGR_CC4G_Msk;
  static constexpr uint32_t TIM_EGR_COMG_Pos = ( 5U );
  static constexpr uint32_t TIM_EGR_COMG_Msk = ( 0x1UL << TIM_EGR_COMG_Pos );
  static constexpr uint32_t TIM_EGR_COMG     = TIM_EGR_COMG_Msk;
  static constexpr uint32_t TIM_EGR_TG_Pos   = ( 6U );
  static constexpr uint32_t TIM_EGR_TG_Msk   = ( 0x1UL << TIM_EGR_TG_Pos );
  static constexpr uint32_t TIM_EGR_TG       = TIM_EGR_TG_Msk;
  static constexpr uint32_t TIM_EGR_BG_Pos   = ( 7U );
  static constexpr uint32_t TIM_EGR_BG_Msk   = ( 0x1UL << TIM_EGR_BG_Pos );
  static constexpr uint32_t TIM_EGR_BG       = TIM_EGR_BG_Msk;
  static constexpr uint32_t TIM_EGR_B2G_Pos  = ( 8U );
  static constexpr uint32_t TIM_EGR_B2G_Msk  = ( 0x1UL << TIM_EGR_B2G_Pos );
  static constexpr uint32_t TIM_EGR_B2G      = TIM_EGR_B2G_Msk;

  /******************  Bit definition for TIM_CCMR1 register  *******************/
  static constexpr uint32_t TIM_CCMR1_CC1S_Pos = ( 0U );
  static constexpr uint32_t TIM_CCMR1_CC1S_Msk = ( 0x3UL << TIM_CCMR1_CC1S_Pos );
  static constexpr uint32_t TIM_CCMR1_CC1S     = TIM_CCMR1_CC1S_Msk;
  static constexpr uint32_t TIM_CCMR1_CC1S_0   = ( 0x1UL << TIM_CCMR1_CC1S_Pos );
  static constexpr uint32_t TIM_CCMR1_CC1S_1   = ( 0x2UL << TIM_CCMR1_CC1S_Pos );

  static constexpr uint32_t TIM_CCMR1_OC1FE_Pos = ( 2U );
  static constexpr uint32_t TIM_CCMR1_OC1FE_Msk = ( 0x1UL << TIM_CCMR1_OC1FE_Pos );
  static constexpr uint32_t TIM_CCMR1_OC1FE     = TIM_CCMR1_OC1FE_Msk;
  static constexpr uint32_t TIM_CCMR1_OC1PE_Pos = ( 3U );
  static constexpr uint32_t TIM_CCMR1_OC1PE_Msk = ( 0x1UL << TIM_CCMR1_OC1PE_Pos );
  static constexpr uint32_t TIM_CCMR1_OC1PE     = TIM_CCMR1_OC1PE_Msk;

  static constexpr uint32_t TIM_CCMR1_OC1M_Pos = ( 4U );
  static constexpr uint32_t TIM_CCMR1_OC1M_Msk = ( 0x1007UL << TIM_CCMR1_OC1M_Pos );
  static constexpr uint32_t TIM_CCMR1_OC1M     = TIM_CCMR1_OC1M_Msk;
  static constexpr uint32_t TIM_CCMR1_OC1M_0   = ( 0x0001UL << TIM_CCMR1_OC1M_Pos );
  static constexpr uint32_t TIM_CCMR1_OC1M_1   = ( 0x0002UL << TIM_CCMR1_OC1M_Pos );
  static constexpr uint32_t TIM_CCMR1_OC1M_2   = ( 0x0004UL << TIM_CCMR1_OC1M_Pos );
  static constexpr uint32_t TIM_CCMR1_OC1M_3   = ( 0x1000UL << TIM_CCMR1_OC1M_Pos );

  static constexpr uint32_t TIM_CCMR1_OC1CE_Pos = ( 7U );
  static constexpr uint32_t TIM_CCMR1_OC1CE_Msk = ( 0x1UL << TIM_CCMR1_OC1CE_Pos );
  static constexpr uint32_t TIM_CCMR1_OC1CE     = TIM_CCMR1_OC1CE_Msk;

  static constexpr uint32_t TIM_CCMR1_CC2S_Pos = ( 8U );
  static constexpr uint32_t TIM_CCMR1_CC2S_Msk = ( 0x3UL << TIM_CCMR1_CC2S_Pos );
  static constexpr uint32_t TIM_CCMR1_CC2S     = TIM_CCMR1_CC2S_Msk;
  static constexpr uint32_t TIM_CCMR1_CC2S_0   = ( 0x1UL << TIM_CCMR1_CC2S_Pos );
  static constexpr uint32_t TIM_CCMR1_CC2S_1   = ( 0x2UL << TIM_CCMR1_CC2S_Pos );

  static constexpr uint32_t TIM_CCMR1_OC2FE_Pos = ( 10U );
  static constexpr uint32_t TIM_CCMR1_OC2FE_Msk = ( 0x1UL << TIM_CCMR1_OC2FE_Pos );
  static constexpr uint32_t TIM_CCMR1_OC2FE     = TIM_CCMR1_OC2FE_Msk;
  static constexpr uint32_t TIM_CCMR1_OC2PE_Pos = ( 11U );
  static constexpr uint32_t TIM_CCMR1_OC2PE_Msk = ( 0x1UL << TIM_CCMR1_OC2PE_Pos );
  static constexpr uint32_t TIM_CCMR1_OC2PE     = TIM_CCMR1_OC2PE_Msk;

  static constexpr uint32_t TIM_CCMR1_OC2M_Pos = ( 12U );
  static constexpr uint32_t TIM_CCMR1_OC2M_Msk = ( 0x1007UL << TIM_CCMR1_OC2M_Pos );
  static constexpr uint32_t TIM_CCMR1_OC2M     = TIM_CCMR1_OC2M_Msk;
  static constexpr uint32_t TIM_CCMR1_OC2M_0   = ( 0x0001UL << TIM_CCMR1_OC2M_Pos );
  static constexpr uint32_t TIM_CCMR1_OC2M_1   = ( 0x0002UL << TIM_CCMR1_OC2M_Pos );
  static constexpr uint32_t TIM_CCMR1_OC2M_2   = ( 0x0004UL << TIM_CCMR1_OC2M_Pos );
  static constexpr uint32_t TIM_CCMR1_OC2M_3   = ( 0x1000UL << TIM_CCMR1_OC2M_Pos );

  static constexpr uint32_t TIM_CCMR1_OC2CE_Pos = ( 15U );
  static constexpr uint32_t TIM_CCMR1_OC2CE_Msk = ( 0x1UL << TIM_CCMR1_OC2CE_Pos );
  static constexpr uint32_t TIM_CCMR1_OC2CE     = TIM_CCMR1_OC2CE_Msk;

  static constexpr uint32_t TIM_CCMR1_IC1PSC_Pos = ( 2U );
  static constexpr uint32_t TIM_CCMR1_IC1PSC_Msk = ( 0x3UL << TIM_CCMR1_IC1PSC_Pos );
  static constexpr uint32_t TIM_CCMR1_IC1PSC     = TIM_CCMR1_IC1PSC_Msk;
  static constexpr uint32_t TIM_CCMR1_IC1PSC_0   = ( 0x1UL << TIM_CCMR1_IC1PSC_Pos );
  static constexpr uint32_t TIM_CCMR1_IC1PSC_1   = ( 0x2UL << TIM_CCMR1_IC1PSC_Pos );

  static constexpr uint32_t TIM_CCMR1_IC1F_Pos = ( 4U );
  static constexpr uint32_t TIM_CCMR1_IC1F_Msk = ( 0xFUL << TIM_CCMR1_IC1F_Pos );
  static constexpr uint32_t TIM_CCMR1_IC1F     = TIM_CCMR1_IC1F_Msk;
  static constexpr uint32_t TIM_CCMR1_IC1F_0   = ( 0x1UL << TIM_CCMR1_IC1F_Pos );
  static constexpr uint32_t TIM_CCMR1_IC1F_1   = ( 0x2UL << TIM_CCMR1_IC1F_Pos );
  static constexpr uint32_t TIM_CCMR1_IC1F_2   = ( 0x4UL << TIM_CCMR1_IC1F_Pos );
  static constexpr uint32_t TIM_CCMR1_IC1F_3   = ( 0x8UL << TIM_CCMR1_IC1F_Pos );

  static constexpr uint32_t TIM_CCMR1_IC2PSC_Pos = ( 10U );
  static constexpr uint32_t TIM_CCMR1_IC2PSC_Msk = ( 0x3UL << TIM_CCMR1_IC2PSC_Pos );
  static constexpr uint32_t TIM_CCMR1_IC2PSC     = TIM_CCMR1_IC2PSC_Msk;
  static constexpr uint32_t TIM_CCMR1_IC2PSC_0   = ( 0x1UL << TIM_CCMR1_IC2PSC_Pos );
  static constexpr uint32_t TIM_CCMR1_IC2PSC_1   = ( 0x2UL << TIM_CCMR1_IC2PSC_Pos );

  static constexpr uint32_t TIM_CCMR1_IC2F_Pos = ( 12U );
  static constexpr uint32_t TIM_CCMR1_IC2F_Msk = ( 0xFUL << TIM_CCMR1_IC2F_Pos );
  static constexpr uint32_t TIM_CCMR1_IC2F     = TIM_CCMR1_IC2F_Msk;
  static constexpr uint32_t TIM_CCMR1_IC2F_0   = ( 0x1UL << TIM_CCMR1_IC2F_Pos );
  static constexpr uint32_t TIM_CCMR1_IC2F_1   = ( 0x2UL << TIM_CCMR1_IC2F_Pos );
  static constexpr uint32_t TIM_CCMR1_IC2F_2   = ( 0x4UL << TIM_CCMR1_IC2F_Pos );
  static constexpr uint32_t TIM_CCMR1_IC2F_3   = ( 0x8UL << TIM_CCMR1_IC2F_Pos );

  /******************  Bit definition for TIM_CCMR2 register  *******************/
  static constexpr uint32_t TIM_CCMR2_CC3S_Pos = ( 0U );
  static constexpr uint32_t TIM_CCMR2_CC3S_Msk = ( 0x3UL << TIM_CCMR2_CC3S_Pos );
  static constexpr uint32_t TIM_CCMR2_CC3S     = TIM_CCMR2_CC3S_Msk;
  static constexpr uint32_t TIM_CCMR2_CC3S_0   = ( 0x1UL << TIM_CCMR2_CC3S_Pos );
  static constexpr uint32_t TIM_CCMR2_CC3S_1   = ( 0x2UL << TIM_CCMR2_CC3S_Pos );

  static constexpr uint32_t TIM_CCMR2_OC3FE_Pos = ( 2U );
  static constexpr uint32_t TIM_CCMR2_OC3FE_Msk = ( 0x1UL << TIM_CCMR2_OC3FE_Pos );
  static constexpr uint32_t TIM_CCMR2_OC3FE     = TIM_CCMR2_OC3FE_Msk;
  static constexpr uint32_t TIM_CCMR2_OC3PE_Pos = ( 3U );
  static constexpr uint32_t TIM_CCMR2_OC3PE_Msk = ( 0x1UL << TIM_CCMR2_OC3PE_Pos );
  static constexpr uint32_t TIM_CCMR2_OC3PE     = TIM_CCMR2_OC3PE_Msk;

  static constexpr uint32_t TIM_CCMR2_OC3M_Pos = ( 4U );
  static constexpr uint32_t TIM_CCMR2_OC3M_Msk = ( 0x1007UL << TIM_CCMR2_OC3M_Pos );
  static constexpr uint32_t TIM_CCMR2_OC3M     = TIM_CCMR2_OC3M_Msk;
  static constexpr uint32_t TIM_CCMR2_OC3M_0   = ( 0x0001UL << TIM_CCMR2_OC3M_Pos );
  static constexpr uint32_t TIM_CCMR2_OC3M_1   = ( 0x0002UL << TIM_CCMR2_OC3M_Pos );
  static constexpr uint32_t TIM_CCMR2_OC3M_2   = ( 0x0004UL << TIM_CCMR2_OC3M_Pos );
  static constexpr uint32_t TIM_CCMR2_OC3M_3   = ( 0x1000UL << TIM_CCMR2_OC3M_Pos );

  static constexpr uint32_t TIM_CCMR2_OC3CE_Pos = ( 7U );
  static constexpr uint32_t TIM_CCMR2_OC3CE_Msk = ( 0x1UL << TIM_CCMR2_OC3CE_Pos );
  static constexpr uint32_t TIM_CCMR2_OC3CE     = TIM_CCMR2_OC3CE_Msk;

  static constexpr uint32_t TIM_CCMR2_CC4S_Pos = ( 8U );
  static constexpr uint32_t TIM_CCMR2_CC4S_Msk = ( 0x3UL << TIM_CCMR2_CC4S_Pos );
  static constexpr uint32_t TIM_CCMR2_CC4S     = TIM_CCMR2_CC4S_Msk;
  static constexpr uint32_t TIM_CCMR2_CC4S_0   = ( 0x1UL << TIM_CCMR2_CC4S_Pos );
  static constexpr uint32_t TIM_CCMR2_CC4S_1   = ( 0x2UL << TIM_CCMR2_CC4S_Pos );

  static constexpr uint32_t TIM_CCMR2_OC4FE_Pos = ( 10U );
  static constexpr uint32_t TIM_CCMR2_OC4FE_Msk = ( 0x1UL << TIM_CCMR2_OC4FE_Pos );
  static constexpr uint32_t TIM_CCMR2_OC4FE     = TIM_CCMR2_OC4FE_Msk;
  static constexpr uint32_t TIM_CCMR2_OC4PE_Pos = ( 11U );
  static constexpr uint32_t TIM_CCMR2_OC4PE_Msk = ( 0x1UL << TIM_CCMR2_OC4PE_Pos );
  static constexpr uint32_t TIM_CCMR2_OC4PE     = TIM_CCMR2_OC4PE_Msk;

  static constexpr uint32_t TIM_CCMR2_OC4M_Pos = ( 12U );
  static constexpr uint32_t TIM_CCMR2_OC4M_Msk = ( 0x1007UL << TIM_CCMR2_OC4M_Pos );
  static constexpr uint32_t TIM_CCMR2_OC4M     = TIM_CCMR2_OC4M_Msk;
  static constexpr uint32_t TIM_CCMR2_OC4M_0   = ( 0x0001UL << TIM_CCMR2_OC4M_Pos );
  static constexpr uint32_t TIM_CCMR2_OC4M_1   = ( 0x0002UL << TIM_CCMR2_OC4M_Pos );
  static constexpr uint32_t TIM_CCMR2_OC4M_2   = ( 0x0004UL << TIM_CCMR2_OC4M_Pos );
  static constexpr uint32_t TIM_CCMR2_OC4M_3   = ( 0x1000UL << TIM_CCMR2_OC4M_Pos );

  static constexpr uint32_t TIM_CCMR2_OC4CE_Pos = ( 15U );
  static constexpr uint32_t TIM_CCMR2_OC4CE_Msk = ( 0x1UL << TIM_CCMR2_OC4CE_Pos );
  static constexpr uint32_t TIM_CCMR2_OC4CE     = TIM_CCMR2_OC4CE_Msk;

  static constexpr uint32_t TIM_CCMR2_IC3PSC_Pos = ( 2U );
  static constexpr uint32_t TIM_CCMR2_IC3PSC_Msk = ( 0x3UL << TIM_CCMR2_IC3PSC_Pos );
  static constexpr uint32_t TIM_CCMR2_IC3PSC     = TIM_CCMR2_IC3PSC_Msk;
  static constexpr uint32_t TIM_CCMR2_IC3PSC_0   = ( 0x1UL << TIM_CCMR2_IC3PSC_Pos );
  static constexpr uint32_t TIM_CCMR2_IC3PSC_1   = ( 0x2UL << TIM_CCMR2_IC3PSC_Pos );

  static constexpr uint32_t TIM_CCMR2_IC3F_Pos = ( 4U );
  static constexpr uint32_t TIM_CCMR2_IC3F_Msk = ( 0xFUL << TIM_CCMR2_IC3F_Pos );
  static constexpr uint32_t TIM_CCMR2_IC3F     = TIM_CCMR2_IC3F_Msk;
  static constexpr uint32_t TIM_CCMR2_IC3F_0   = ( 0x1UL << TIM_CCMR2_IC3F_Pos );
  static constexpr uint32_t TIM_CCMR2_IC3F_1   = ( 0x2UL << TIM_CCMR2_IC3F_Pos );
  static constexpr uint32_t TIM_CCMR2_IC3F_2   = ( 0x4UL << TIM_CCMR2_IC3F_Pos );
  static constexpr uint32_t TIM_CCMR2_IC3F_3   = ( 0x8UL << TIM_CCMR2_IC3F_Pos );

  static constexpr uint32_t TIM_CCMR2_IC4PSC_Pos = ( 10U );
  static constexpr uint32_t TIM_CCMR2_IC4PSC_Msk = ( 0x3UL << TIM_CCMR2_IC4PSC_Pos );
  static constexpr uint32_t TIM_CCMR2_IC4PSC     = TIM_CCMR2_IC4PSC_Msk;
  static constexpr uint32_t TIM_CCMR2_IC4PSC_0   = ( 0x1UL << TIM_CCMR2_IC4PSC_Pos );
  static constexpr uint32_t TIM_CCMR2_IC4PSC_1   = ( 0x2UL << TIM_CCMR2_IC4PSC_Pos );

  static constexpr uint32_t TIM_CCMR2_IC4F_Pos = ( 12U );
  static constexpr uint32_t TIM_CCMR2_IC4F_Msk = ( 0xFUL << TIM_CCMR2_IC4F_Pos );
  static constexpr uint32_t TIM_CCMR2_IC4F     = TIM_CCMR2_IC4F_Msk;
  static constexpr uint32_t TIM_CCMR2_IC4F_0   = ( 0x1UL << TIM_CCMR2_IC4F_Pos );
  static constexpr uint32_t TIM_CCMR2_IC4F_1   = ( 0x2UL << TIM_CCMR2_IC4F_Pos );
  static constexpr uint32_t TIM_CCMR2_IC4F_2   = ( 0x4UL << TIM_CCMR2_IC4F_Pos );
  static constexpr uint32_t TIM_CCMR2_IC4F_3   = ( 0x8UL << TIM_CCMR2_IC4F_Pos );

  /******************  Bit definition for TIM_CCMR3 register  *******************/
  static constexpr uint32_t TIM_CCMR3_OC5FE_Pos = ( 2U );
  static constexpr uint32_t TIM_CCMR3_OC5FE_Msk = ( 0x1UL << TIM_CCMR3_OC5FE_Pos );
  static constexpr uint32_t TIM_CCMR3_OC5FE     = TIM_CCMR3_OC5FE_Msk;
  static constexpr uint32_t TIM_CCMR3_OC5PE_Pos = ( 3U );
  static constexpr uint32_t TIM_CCMR3_OC5PE_Msk = ( 0x1UL << TIM_CCMR3_OC5PE_Pos );
  static constexpr uint32_t TIM_CCMR3_OC5PE     = TIM_CCMR3_OC5PE_Msk;

  static constexpr uint32_t TIM_CCMR3_OC5M_Pos = ( 4U );
  static constexpr uint32_t TIM_CCMR3_OC5M_Msk = ( 0x1007UL << TIM_CCMR3_OC5M_Pos );
  static constexpr uint32_t TIM_CCMR3_OC5M     = TIM_CCMR3_OC5M_Msk;
  static constexpr uint32_t TIM_CCMR3_OC5M_0   = ( 0x0001UL << TIM_CCMR3_OC5M_Pos );
  static constexpr uint32_t TIM_CCMR3_OC5M_1   = ( 0x0002UL << TIM_CCMR3_OC5M_Pos );
  static constexpr uint32_t TIM_CCMR3_OC5M_2   = ( 0x0004UL << TIM_CCMR3_OC5M_Pos );
  static constexpr uint32_t TIM_CCMR3_OC5M_3   = ( 0x1000UL << TIM_CCMR3_OC5M_Pos );

  static constexpr uint32_t TIM_CCMR3_OC5CE_Pos = ( 7U );
  static constexpr uint32_t TIM_CCMR3_OC5CE_Msk = ( 0x1UL << TIM_CCMR3_OC5CE_Pos );
  static constexpr uint32_t TIM_CCMR3_OC5CE     = TIM_CCMR3_OC5CE_Msk;

  static constexpr uint32_t TIM_CCMR3_OC6FE_Pos = ( 10U );
  static constexpr uint32_t TIM_CCMR3_OC6FE_Msk = ( 0x1UL << TIM_CCMR3_OC6FE_Pos );
  static constexpr uint32_t TIM_CCMR3_OC6FE     = TIM_CCMR3_OC6FE_Msk;
  static constexpr uint32_t TIM_CCMR3_OC6PE_Pos = ( 11U );
  static constexpr uint32_t TIM_CCMR3_OC6PE_Msk = ( 0x1UL << TIM_CCMR3_OC6PE_Pos );
  static constexpr uint32_t TIM_CCMR3_OC6PE     = TIM_CCMR3_OC6PE_Msk;

  static constexpr uint32_t TIM_CCMR3_OC6M_Pos = ( 12U );
  static constexpr uint32_t TIM_CCMR3_OC6M_Msk = ( 0x1007UL << TIM_CCMR3_OC6M_Pos );
  static constexpr uint32_t TIM_CCMR3_OC6M     = TIM_CCMR3_OC6M_Msk;
  static constexpr uint32_t TIM_CCMR3_OC6M_0   = ( 0x0001UL << TIM_CCMR3_OC6M_Pos );
  static constexpr uint32_t TIM_CCMR3_OC6M_1   = ( 0x0002UL << TIM_CCMR3_OC6M_Pos );
  static constexpr uint32_t TIM_CCMR3_OC6M_2   = ( 0x0004UL << TIM_CCMR3_OC6M_Pos );
  static constexpr uint32_t TIM_CCMR3_OC6M_3   = ( 0x1000UL << TIM_CCMR3_OC6M_Pos );

  static constexpr uint32_t TIM_CCMR3_OC6CE_Pos = ( 15U );
  static constexpr uint32_t TIM_CCMR3_OC6CE_Msk = ( 0x1UL << TIM_CCMR3_OC6CE_Pos );
  static constexpr uint32_t TIM_CCMR3_OC6CE     = TIM_CCMR3_OC6CE_Msk;

  /*******************  Bit definition for TIM_CCER register  *******************/
  static constexpr uint32_t TIM_CCER_CC1E_Pos  = ( 0U );
  static constexpr uint32_t TIM_CCER_CC1E_Msk  = ( 0x1UL << TIM_CCER_CC1E_Pos );
  static constexpr uint32_t TIM_CCER_CC1E      = TIM_CCER_CC1E_Msk;
  static constexpr uint32_t TIM_CCER_CC1P_Pos  = ( 1U );
  static constexpr uint32_t TIM_CCER_CC1P_Msk  = ( 0x1UL << TIM_CCER_CC1P_Pos );
  static constexpr uint32_t TIM_CCER_CC1P      = TIM_CCER_CC1P_Msk;
  static constexpr uint32_t TIM_CCER_CC1NE_Pos = ( 2U );
  static constexpr uint32_t TIM_CCER_CC1NE_Msk = ( 0x1UL << TIM_CCER_CC1NE_Pos );
  static constexpr uint32_t TIM_CCER_CC1NE     = TIM_CCER_CC1NE_Msk;
  static constexpr uint32_t TIM_CCER_CC1NP_Pos = ( 3U );
  static constexpr uint32_t TIM_CCER_CC1NP_Msk = ( 0x1UL << TIM_CCER_CC1NP_Pos );
  static constexpr uint32_t TIM_CCER_CC1NP     = TIM_CCER_CC1NP_Msk;
  static constexpr uint32_t TIM_CCER_CC2E_Pos  = ( 4U );
  static constexpr uint32_t TIM_CCER_CC2E_Msk  = ( 0x1UL << TIM_CCER_CC2E_Pos );
  static constexpr uint32_t TIM_CCER_CC2E      = TIM_CCER_CC2E_Msk;
  static constexpr uint32_t TIM_CCER_CC2P_Pos  = ( 5U );
  static constexpr uint32_t TIM_CCER_CC2P_Msk  = ( 0x1UL << TIM_CCER_CC2P_Pos );
  static constexpr uint32_t TIM_CCER_CC2P      = TIM_CCER_CC2P_Msk;
  static constexpr uint32_t TIM_CCER_CC2NE_Pos = ( 6U );
  static constexpr uint32_t TIM_CCER_CC2NE_Msk = ( 0x1UL << TIM_CCER_CC2NE_Pos );
  static constexpr uint32_t TIM_CCER_CC2NE     = TIM_CCER_CC2NE_Msk;
  static constexpr uint32_t TIM_CCER_CC2NP_Pos = ( 7U );
  static constexpr uint32_t TIM_CCER_CC2NP_Msk = ( 0x1UL << TIM_CCER_CC2NP_Pos );
  static constexpr uint32_t TIM_CCER_CC2NP     = TIM_CCER_CC2NP_Msk;
  static constexpr uint32_t TIM_CCER_CC3E_Pos  = ( 8U );
  static constexpr uint32_t TIM_CCER_CC3E_Msk  = ( 0x1UL << TIM_CCER_CC3E_Pos );
  static constexpr uint32_t TIM_CCER_CC3E      = TIM_CCER_CC3E_Msk;
  static constexpr uint32_t TIM_CCER_CC3P_Pos  = ( 9U );
  static constexpr uint32_t TIM_CCER_CC3P_Msk  = ( 0x1UL << TIM_CCER_CC3P_Pos );
  static constexpr uint32_t TIM_CCER_CC3P      = TIM_CCER_CC3P_Msk;
  static constexpr uint32_t TIM_CCER_CC3NE_Pos = ( 10U );
  static constexpr uint32_t TIM_CCER_CC3NE_Msk = ( 0x1UL << TIM_CCER_CC3NE_Pos );
  static constexpr uint32_t TIM_CCER_CC3NE     = TIM_CCER_CC3NE_Msk;
  static constexpr uint32_t TIM_CCER_CC3NP_Pos = ( 11U );
  static constexpr uint32_t TIM_CCER_CC3NP_Msk = ( 0x1UL << TIM_CCER_CC3NP_Pos );
  static constexpr uint32_t TIM_CCER_CC3NP     = TIM_CCER_CC3NP_Msk;
  static constexpr uint32_t TIM_CCER_CC4E_Pos  = ( 12U );
  static constexpr uint32_t TIM_CCER_CC4E_Msk  = ( 0x1UL << TIM_CCER_CC4E_Pos );
  static constexpr uint32_t TIM_CCER_CC4E      = TIM_CCER_CC4E_Msk;
  static constexpr uint32_t TIM_CCER_CC4P_Pos  = ( 13U );
  static constexpr uint32_t TIM_CCER_CC4P_Msk  = ( 0x1UL << TIM_CCER_CC4P_Pos );
  static constexpr uint32_t TIM_CCER_CC4P      = TIM_CCER_CC4P_Msk;
  static constexpr uint32_t TIM_CCER_CC4NP_Pos = ( 15U );
  static constexpr uint32_t TIM_CCER_CC4NP_Msk = ( 0x1UL << TIM_CCER_CC4NP_Pos );
  static constexpr uint32_t TIM_CCER_CC4NP     = TIM_CCER_CC4NP_Msk;
  static constexpr uint32_t TIM_CCER_CC5E_Pos  = ( 16U );
  static constexpr uint32_t TIM_CCER_CC5E_Msk  = ( 0x1UL << TIM_CCER_CC5E_Pos );
  static constexpr uint32_t TIM_CCER_CC5E      = TIM_CCER_CC5E_Msk;
  static constexpr uint32_t TIM_CCER_CC5P_Pos  = ( 17U );
  static constexpr uint32_t TIM_CCER_CC5P_Msk  = ( 0x1UL << TIM_CCER_CC5P_Pos );
  static constexpr uint32_t TIM_CCER_CC5P      = TIM_CCER_CC5P_Msk;
  static constexpr uint32_t TIM_CCER_CC6E_Pos  = ( 20U );
  static constexpr uint32_t TIM_CCER_CC6E_Msk  = ( 0x1UL << TIM_CCER_CC6E_Pos );
  static constexpr uint32_t TIM_CCER_CC6E      = TIM_CCER_CC6E_Msk;
  static constexpr uint32_t TIM_CCER_CC6P_Pos  = ( 21U );
  static constexpr uint32_t TIM_CCER_CC6P_Msk  = ( 0x1UL << TIM_CCER_CC6P_Pos );
  static constexpr uint32_t TIM_CCER_CC6P      = TIM_CCER_CC6P_Msk;

  /*******************  Bit definition for TIM_CNT register  ********************/
  static constexpr uint32_t TIM_CNT_CNT_Pos    = ( 0U );
  static constexpr uint32_t TIM_CNT_CNT_Msk    = ( 0xFFFFFFFFUL << TIM_CNT_CNT_Pos );
  static constexpr uint32_t TIM_CNT_CNT        = TIM_CNT_CNT_Msk;
  static constexpr uint32_t TIM_CNT_UIFCPY_Pos = ( 31U );
  static constexpr uint32_t TIM_CNT_UIFCPY_Msk = ( 0x1UL << TIM_CNT_UIFCPY_Pos );
  static constexpr uint32_t TIM_CNT_UIFCPY     = TIM_CNT_UIFCPY_Msk;

  /*******************  Bit definition for TIM_PSC register  ********************/
  static constexpr uint32_t TIM_PSC_PSC_Pos = ( 0U );
  static constexpr uint32_t TIM_PSC_PSC_Msk = ( 0xFFFFUL << TIM_PSC_PSC_Pos );
  static constexpr uint32_t TIM_PSC_PSC     = TIM_PSC_PSC_Msk;

  /*******************  Bit definition for TIM_ARR register  ********************/
  static constexpr uint32_t TIM_ARR_ARR_Pos = ( 0U );
  static constexpr uint32_t TIM_ARR_ARR_Msk = ( 0xFFFFFFFFUL << TIM_ARR_ARR_Pos );
  static constexpr uint32_t TIM_ARR_ARR     = TIM_ARR_ARR_Msk;

  /*******************  Bit definition for TIM_RCR register  ********************/
  static constexpr uint32_t TIM_RCR_REP_Pos = ( 0U );
  static constexpr uint32_t TIM_RCR_REP_Msk = ( 0xFFFFUL << TIM_RCR_REP_Pos );
  static constexpr uint32_t TIM_RCR_REP     = TIM_RCR_REP_Msk;

  /*******************  Bit definition for TIM_CCR1 register  *******************/
  static constexpr uint32_t TIM_CCR1_CCR1_Pos = ( 0U );
  static constexpr uint32_t TIM_CCR1_CCR1_Msk = ( 0xFFFFUL << TIM_CCR1_CCR1_Pos );
  static constexpr uint32_t TIM_CCR1_CCR1     = TIM_CCR1_CCR1_Msk;

  /*******************  Bit definition for TIM_CCR2 register  *******************/
  static constexpr uint32_t TIM_CCR2_CCR2_Pos = ( 0U );
  static constexpr uint32_t TIM_CCR2_CCR2_Msk = ( 0xFFFFUL << TIM_CCR2_CCR2_Pos );
  static constexpr uint32_t TIM_CCR2_CCR2     = TIM_CCR2_CCR2_Msk;

  /*******************  Bit definition for TIM_CCR3 register  *******************/
  static constexpr uint32_t TIM_CCR3_CCR3_Pos = ( 0U );
  static constexpr uint32_t TIM_CCR3_CCR3_Msk = ( 0xFFFFUL << TIM_CCR3_CCR3_Pos );
  static constexpr uint32_t TIM_CCR3_CCR3     = TIM_CCR3_CCR3_Msk;

  /*******************  Bit definition for TIM_CCR4 register  *******************/
  static constexpr uint32_t TIM_CCR4_CCR4_Pos = ( 0U );
  static constexpr uint32_t TIM_CCR4_CCR4_Msk = ( 0xFFFFUL << TIM_CCR4_CCR4_Pos );
  static constexpr uint32_t TIM_CCR4_CCR4     = TIM_CCR4_CCR4_Msk;

  /*******************  Bit definition for TIM_CCR5 register  *******************/
  static constexpr uint32_t TIM_CCR5_CCR5_Pos  = ( 0U );
  static constexpr uint32_t TIM_CCR5_CCR5_Msk  = ( 0xFFFFFFFFUL << TIM_CCR5_CCR5_Pos );
  static constexpr uint32_t TIM_CCR5_CCR5      = TIM_CCR5_CCR5_Msk;
  static constexpr uint32_t TIM_CCR5_GC5C1_Pos = ( 29U );
  static constexpr uint32_t TIM_CCR5_GC5C1_Msk = ( 0x1UL << TIM_CCR5_GC5C1_Pos );
  static constexpr uint32_t TIM_CCR5_GC5C1     = TIM_CCR5_GC5C1_Msk;
  static constexpr uint32_t TIM_CCR5_GC5C2_Pos = ( 30U );
  static constexpr uint32_t TIM_CCR5_GC5C2_Msk = ( 0x1UL << TIM_CCR5_GC5C2_Pos );
  static constexpr uint32_t TIM_CCR5_GC5C2     = TIM_CCR5_GC5C2_Msk;
  static constexpr uint32_t TIM_CCR5_GC5C3_Pos = ( 31U );
  static constexpr uint32_t TIM_CCR5_GC5C3_Msk = ( 0x1UL << TIM_CCR5_GC5C3_Pos );
  static constexpr uint32_t TIM_CCR5_GC5C3     = TIM_CCR5_GC5C3_Msk;

  /*******************  Bit definition for TIM_CCR6 register  *******************/
  static constexpr uint32_t TIM_CCR6_CCR6_Pos = ( 0U );
  static constexpr uint32_t TIM_CCR6_CCR6_Msk = ( 0xFFFFUL << TIM_CCR6_CCR6_Pos );
  static constexpr uint32_t TIM_CCR6_CCR6     = TIM_CCR6_CCR6_Msk;

  /*******************  Bit definition for TIM_BDTR register  *******************/
  static constexpr uint32_t TIM_BDTR_DTG_Pos = ( 0U );
  static constexpr uint32_t TIM_BDTR_DTG_Msk = ( 0xFFUL << TIM_BDTR_DTG_Pos );
  static constexpr uint32_t TIM_BDTR_DTG     = TIM_BDTR_DTG_Msk;
  static constexpr uint32_t TIM_BDTR_DTG_0   = ( 0x01UL << TIM_BDTR_DTG_Pos );
  static constexpr uint32_t TIM_BDTR_DTG_1   = ( 0x02UL << TIM_BDTR_DTG_Pos );
  static constexpr uint32_t TIM_BDTR_DTG_2   = ( 0x04UL << TIM_BDTR_DTG_Pos );
  static constexpr uint32_t TIM_BDTR_DTG_3   = ( 0x08UL << TIM_BDTR_DTG_Pos );
  static constexpr uint32_t TIM_BDTR_DTG_4   = ( 0x10UL << TIM_BDTR_DTG_Pos );
  static constexpr uint32_t TIM_BDTR_DTG_5   = ( 0x20UL << TIM_BDTR_DTG_Pos );
  static constexpr uint32_t TIM_BDTR_DTG_6   = ( 0x40UL << TIM_BDTR_DTG_Pos );
  static constexpr uint32_t TIM_BDTR_DTG_7   = ( 0x80UL << TIM_BDTR_DTG_Pos );

  static constexpr uint32_t TIM_BDTR_LOCK_Pos = ( 8U );
  static constexpr uint32_t TIM_BDTR_LOCK_Msk = ( 0x3UL << TIM_BDTR_LOCK_Pos );
  static constexpr uint32_t TIM_BDTR_LOCK     = TIM_BDTR_LOCK_Msk;
  static constexpr uint32_t TIM_BDTR_LOCK_0   = ( 0x1UL << TIM_BDTR_LOCK_Pos );
  static constexpr uint32_t TIM_BDTR_LOCK_1   = ( 0x2UL << TIM_BDTR_LOCK_Pos );

  static constexpr uint32_t TIM_BDTR_OSSI_Pos = ( 10U );
  static constexpr uint32_t TIM_BDTR_OSSI_Msk = ( 0x1UL << TIM_BDTR_OSSI_Pos );
  static constexpr uint32_t TIM_BDTR_OSSI     = TIM_BDTR_OSSI_Msk;
  static constexpr uint32_t TIM_BDTR_OSSR_Pos = ( 11U );
  static constexpr uint32_t TIM_BDTR_OSSR_Msk = ( 0x1UL << TIM_BDTR_OSSR_Pos );
  static constexpr uint32_t TIM_BDTR_OSSR     = TIM_BDTR_OSSR_Msk;
  static constexpr uint32_t TIM_BDTR_BKE_Pos  = ( 12U );
  static constexpr uint32_t TIM_BDTR_BKE_Msk  = ( 0x1UL << TIM_BDTR_BKE_Pos );
  static constexpr uint32_t TIM_BDTR_BKE      = TIM_BDTR_BKE_Msk;
  static constexpr uint32_t TIM_BDTR_BKP_Pos  = ( 13U );
  static constexpr uint32_t TIM_BDTR_BKP_Msk  = ( 0x1UL << TIM_BDTR_BKP_Pos );
  static constexpr uint32_t TIM_BDTR_BKP      = TIM_BDTR_BKP_Msk;
  static constexpr uint32_t TIM_BDTR_AOE_Pos  = ( 14U );
  static constexpr uint32_t TIM_BDTR_AOE_Msk  = ( 0x1UL << TIM_BDTR_AOE_Pos );
  static constexpr uint32_t TIM_BDTR_AOE      = TIM_BDTR_AOE_Msk;
  static constexpr uint32_t TIM_BDTR_MOE_Pos  = ( 15U );
  static constexpr uint32_t TIM_BDTR_MOE_Msk  = ( 0x1UL << TIM_BDTR_MOE_Pos );
  static constexpr uint32_t TIM_BDTR_MOE      = TIM_BDTR_MOE_Msk;

  static constexpr uint32_t TIM_BDTR_BKF_Pos  = ( 16U );
  static constexpr uint32_t TIM_BDTR_BKF_Msk  = ( 0xFUL << TIM_BDTR_BKF_Pos );
  static constexpr uint32_t TIM_BDTR_BKF      = TIM_BDTR_BKF_Msk;
  static constexpr uint32_t TIM_BDTR_BK2F_Pos = ( 20U );
  static constexpr uint32_t TIM_BDTR_BK2F_Msk = ( 0xFUL << TIM_BDTR_BK2F_Pos );
  static constexpr uint32_t TIM_BDTR_BK2F     = TIM_BDTR_BK2F_Msk;

  static constexpr uint32_t TIM_BDTR_BK2E_Pos = ( 24U );
  static constexpr uint32_t TIM_BDTR_BK2E_Msk = ( 0x1UL << TIM_BDTR_BK2E_Pos );
  static constexpr uint32_t TIM_BDTR_BK2E     = TIM_BDTR_BK2E_Msk;
  static constexpr uint32_t TIM_BDTR_BK2P_Pos = ( 25U );
  static constexpr uint32_t TIM_BDTR_BK2P_Msk = ( 0x1UL << TIM_BDTR_BK2P_Pos );
  static constexpr uint32_t TIM_BDTR_BK2P     = TIM_BDTR_BK2P_Msk;

  /*******************  Bit definition for TIM_DCR register  ********************/
  static constexpr uint32_t TIM_DCR_DBA_Pos = ( 0U );
  static constexpr uint32_t TIM_DCR_DBA_Msk = ( 0x1FUL << TIM_DCR_DBA_Pos );
  static constexpr uint32_t TIM_DCR_DBA     = TIM_DCR_DBA_Msk;
  static constexpr uint32_t TIM_DCR_DBA_0   = ( 0x01UL << TIM_DCR_DBA_Pos );
  static constexpr uint32_t TIM_DCR_DBA_1   = ( 0x02UL << TIM_DCR_DBA_Pos );
  static constexpr uint32_t TIM_DCR_DBA_2   = ( 0x04UL << TIM_DCR_DBA_Pos );
  static constexpr uint32_t TIM_DCR_DBA_3   = ( 0x08UL << TIM_DCR_DBA_Pos );
  static constexpr uint32_t TIM_DCR_DBA_4   = ( 0x10UL << TIM_DCR_DBA_Pos );

  static constexpr uint32_t TIM_DCR_DBL_Pos = ( 8U );
  static constexpr uint32_t TIM_DCR_DBL_Msk = ( 0x1FUL << TIM_DCR_DBL_Pos );
  static constexpr uint32_t TIM_DCR_DBL     = TIM_DCR_DBL_Msk;
  static constexpr uint32_t TIM_DCR_DBL_0   = ( 0x01UL << TIM_DCR_DBL_Pos );
  static constexpr uint32_t TIM_DCR_DBL_1   = ( 0x02UL << TIM_DCR_DBL_Pos );
  static constexpr uint32_t TIM_DCR_DBL_2   = ( 0x04UL << TIM_DCR_DBL_Pos );
  static constexpr uint32_t TIM_DCR_DBL_3   = ( 0x08UL << TIM_DCR_DBL_Pos );
  static constexpr uint32_t TIM_DCR_DBL_4   = ( 0x10UL << TIM_DCR_DBL_Pos );

  /*******************  Bit definition for TIM_DMAR register  *******************/
  static constexpr uint32_t TIM_DMAR_DMAB_Pos = ( 0U );
  static constexpr uint32_t TIM_DMAR_DMAB_Msk = ( 0xFFFFUL << TIM_DMAR_DMAB_Pos );
  static constexpr uint32_t TIM_DMAR_DMAB     = TIM_DMAR_DMAB_Msk;

  /*******************  Bit definition for TIM1_OR1 register  *******************/
  static constexpr uint32_t TIM1_OR1_ETR_ADC1_RMP_Pos = ( 0U );
  static constexpr uint32_t TIM1_OR1_ETR_ADC1_RMP_Msk = ( 0x3UL << TIM1_OR1_ETR_ADC1_RMP_Pos );
  static constexpr uint32_t TIM1_OR1_ETR_ADC1_RMP     = TIM1_OR1_ETR_ADC1_RMP_Msk;
  static constexpr uint32_t TIM1_OR1_ETR_ADC1_RMP_0   = ( 0x1UL << TIM1_OR1_ETR_ADC1_RMP_Pos );
  static constexpr uint32_t TIM1_OR1_ETR_ADC1_RMP_1   = ( 0x2UL << TIM1_OR1_ETR_ADC1_RMP_Pos );

  static constexpr uint32_t TIM1_OR1_TI1_RMP_Pos = ( 4U );
  static constexpr uint32_t TIM1_OR1_TI1_RMP_Msk = ( 0x1UL << TIM1_OR1_TI1_RMP_Pos );
  static constexpr uint32_t TIM1_OR1_TI1_RMP     = TIM1_OR1_TI1_RMP_Msk;

  /*******************  Bit definition for TIM1_OR2 register  *******************/
  static constexpr uint32_t TIM1_OR2_BKINE_Pos   = ( 0U );
  static constexpr uint32_t TIM1_OR2_BKINE_Msk   = ( 0x1UL << TIM1_OR2_BKINE_Pos );
  static constexpr uint32_t TIM1_OR2_BKINE       = TIM1_OR2_BKINE_Msk;
  static constexpr uint32_t TIM1_OR2_BKCMP1E_Pos = ( 1U );
  static constexpr uint32_t TIM1_OR2_BKCMP1E_Msk = ( 0x1UL << TIM1_OR2_BKCMP1E_Pos );
  static constexpr uint32_t TIM1_OR2_BKCMP1E     = TIM1_OR2_BKCMP1E_Msk;
  static constexpr uint32_t TIM1_OR2_BKCMP2E_Pos = ( 2U );
  static constexpr uint32_t TIM1_OR2_BKCMP2E_Msk = ( 0x1UL << TIM1_OR2_BKCMP2E_Pos );
  static constexpr uint32_t TIM1_OR2_BKCMP2E     = TIM1_OR2_BKCMP2E_Msk;
  static constexpr uint32_t TIM1_OR2_BKINP_Pos   = ( 9U );
  static constexpr uint32_t TIM1_OR2_BKINP_Msk   = ( 0x1UL << TIM1_OR2_BKINP_Pos );
  static constexpr uint32_t TIM1_OR2_BKINP       = TIM1_OR2_BKINP_Msk;
  static constexpr uint32_t TIM1_OR2_BKCMP1P_Pos = ( 10U );
  static constexpr uint32_t TIM1_OR2_BKCMP1P_Msk = ( 0x1UL << TIM1_OR2_BKCMP1P_Pos );
  static constexpr uint32_t TIM1_OR2_BKCMP1P     = TIM1_OR2_BKCMP1P_Msk;
  static constexpr uint32_t TIM1_OR2_BKCMP2P_Pos = ( 11U );
  static constexpr uint32_t TIM1_OR2_BKCMP2P_Msk = ( 0x1UL << TIM1_OR2_BKCMP2P_Pos );
  static constexpr uint32_t TIM1_OR2_BKCMP2P     = TIM1_OR2_BKCMP2P_Msk;

  static constexpr uint32_t TIM1_OR2_ETRSEL_Pos = ( 14U );
  static constexpr uint32_t TIM1_OR2_ETRSEL_Msk = ( 0x7UL << TIM1_OR2_ETRSEL_Pos );
  static constexpr uint32_t TIM1_OR2_ETRSEL     = TIM1_OR2_ETRSEL_Msk;
  static constexpr uint32_t TIM1_OR2_ETRSEL_0   = ( 0x1UL << TIM1_OR2_ETRSEL_Pos );
  static constexpr uint32_t TIM1_OR2_ETRSEL_1   = ( 0x2UL << TIM1_OR2_ETRSEL_Pos );
  static constexpr uint32_t TIM1_OR2_ETRSEL_2   = ( 0x4UL << TIM1_OR2_ETRSEL_Pos );

  /*******************  Bit definition for TIM1_OR3 register  *******************/
  static constexpr uint32_t TIM1_OR3_BK2INE_Pos   = ( 0U );
  static constexpr uint32_t TIM1_OR3_BK2INE_Msk   = ( 0x1UL << TIM1_OR3_BK2INE_Pos );
  static constexpr uint32_t TIM1_OR3_BK2INE       = TIM1_OR3_BK2INE_Msk;
  static constexpr uint32_t TIM1_OR3_BK2CMP1E_Pos = ( 1U );
  static constexpr uint32_t TIM1_OR3_BK2CMP1E_Msk = ( 0x1UL << TIM1_OR3_BK2CMP1E_Pos );
  static constexpr uint32_t TIM1_OR3_BK2CMP1E     = TIM1_OR3_BK2CMP1E_Msk;
  static constexpr uint32_t TIM1_OR3_BK2CMP2E_Pos = ( 2U );
  static constexpr uint32_t TIM1_OR3_BK2CMP2E_Msk = ( 0x1UL << TIM1_OR3_BK2CMP2E_Pos );
  static constexpr uint32_t TIM1_OR3_BK2CMP2E     = TIM1_OR3_BK2CMP2E_Msk;
  static constexpr uint32_t TIM1_OR3_BK2INP_Pos   = ( 9U );
  static constexpr uint32_t TIM1_OR3_BK2INP_Msk   = ( 0x1UL << TIM1_OR3_BK2INP_Pos );
  static constexpr uint32_t TIM1_OR3_BK2INP       = TIM1_OR3_BK2INP_Msk;
  static constexpr uint32_t TIM1_OR3_BK2CMP1P_Pos = ( 10U );
  static constexpr uint32_t TIM1_OR3_BK2CMP1P_Msk = ( 0x1UL << TIM1_OR3_BK2CMP1P_Pos );
  static constexpr uint32_t TIM1_OR3_BK2CMP1P     = TIM1_OR3_BK2CMP1P_Msk;
  static constexpr uint32_t TIM1_OR3_BK2CMP2P_Pos = ( 11U );
  static constexpr uint32_t TIM1_OR3_BK2CMP2P_Msk = ( 0x1UL << TIM1_OR3_BK2CMP2P_Pos );
  static constexpr uint32_t TIM1_OR3_BK2CMP2P     = TIM1_OR3_BK2CMP2P_Msk;


  /*******************  Bit definition for TIM2_OR1 register  *******************/
  static constexpr uint32_t TIM2_OR1_ITR1_RMP_Pos = ( 0U );
  static constexpr uint32_t TIM2_OR1_ITR1_RMP_Msk = ( 0x1UL << TIM2_OR1_ITR1_RMP_Pos );
  static constexpr uint32_t TIM2_OR1_ITR1_RMP     = TIM2_OR1_ITR1_RMP_Msk;
  static constexpr uint32_t TIM2_OR1_ETR1_RMP_Pos = ( 1U );
  static constexpr uint32_t TIM2_OR1_ETR1_RMP_Msk = ( 0x1UL << TIM2_OR1_ETR1_RMP_Pos );
  static constexpr uint32_t TIM2_OR1_ETR1_RMP     = TIM2_OR1_ETR1_RMP_Msk;

  static constexpr uint32_t TIM2_OR1_TI4_RMP_Pos = ( 2U );
  static constexpr uint32_t TIM2_OR1_TI4_RMP_Msk = ( 0x3UL << TIM2_OR1_TI4_RMP_Pos );
  static constexpr uint32_t TIM2_OR1_TI4_RMP     = TIM2_OR1_TI4_RMP_Msk;
  static constexpr uint32_t TIM2_OR1_TI4_RMP_0   = ( 0x1UL << TIM2_OR1_TI4_RMP_Pos );
  static constexpr uint32_t TIM2_OR1_TI4_RMP_1   = ( 0x2UL << TIM2_OR1_TI4_RMP_Pos );

  /*******************  Bit definition for TIM2_OR2 register  *******************/
  static constexpr uint32_t TIM2_OR2_ETRSEL_Pos = ( 14U );
  static constexpr uint32_t TIM2_OR2_ETRSEL_Msk = ( 0x7UL << TIM2_OR2_ETRSEL_Pos );
  static constexpr uint32_t TIM2_OR2_ETRSEL     = TIM2_OR2_ETRSEL_Msk;
  static constexpr uint32_t TIM2_OR2_ETRSEL_0   = ( 0x1UL << TIM2_OR2_ETRSEL_Pos );
  static constexpr uint32_t TIM2_OR2_ETRSEL_1   = ( 0x2UL << TIM2_OR2_ETRSEL_Pos );
  static constexpr uint32_t TIM2_OR2_ETRSEL_2   = ( 0x4UL << TIM2_OR2_ETRSEL_Pos );


  /*******************  Bit definition for TIM15_OR1 register  ******************/
  static constexpr uint32_t TIM15_OR1_TI1_RMP_Pos = ( 0U );
  static constexpr uint32_t TIM15_OR1_TI1_RMP_Msk = ( 0x1UL << TIM15_OR1_TI1_RMP_Pos );
  static constexpr uint32_t TIM15_OR1_TI1_RMP     = TIM15_OR1_TI1_RMP_Msk;

  static constexpr uint32_t TIM15_OR1_ENCODER_MODE_Pos = ( 1U );
  static constexpr uint32_t TIM15_OR1_ENCODER_MODE_Msk = ( 0x3UL << TIM15_OR1_ENCODER_MODE_Pos );
  static constexpr uint32_t TIM15_OR1_ENCODER_MODE     = TIM15_OR1_ENCODER_MODE_Msk;
  static constexpr uint32_t TIM15_OR1_ENCODER_MODE_0   = ( 0x1UL << TIM15_OR1_ENCODER_MODE_Pos );
  static constexpr uint32_t TIM15_OR1_ENCODER_MODE_1   = ( 0x2UL << TIM15_OR1_ENCODER_MODE_Pos );

  /*******************  Bit definition for TIM15_OR2 register  ******************/
  static constexpr uint32_t TIM15_OR2_BKINE_Pos   = ( 0U );
  static constexpr uint32_t TIM15_OR2_BKINE_Msk   = ( 0x1UL << TIM15_OR2_BKINE_Pos );
  static constexpr uint32_t TIM15_OR2_BKINE       = TIM15_OR2_BKINE_Msk;
  static constexpr uint32_t TIM15_OR2_BKCMP1E_Pos = ( 1U );
  static constexpr uint32_t TIM15_OR2_BKCMP1E_Msk = ( 0x1UL << TIM15_OR2_BKCMP1E_Pos );
  static constexpr uint32_t TIM15_OR2_BKCMP1E     = TIM15_OR2_BKCMP1E_Msk;
  static constexpr uint32_t TIM15_OR2_BKCMP2E_Pos = ( 2U );
  static constexpr uint32_t TIM15_OR2_BKCMP2E_Msk = ( 0x1UL << TIM15_OR2_BKCMP2E_Pos );
  static constexpr uint32_t TIM15_OR2_BKCMP2E     = TIM15_OR2_BKCMP2E_Msk;
  static constexpr uint32_t TIM15_OR2_BKINP_Pos   = ( 9U );
  static constexpr uint32_t TIM15_OR2_BKINP_Msk   = ( 0x1UL << TIM15_OR2_BKINP_Pos );
  static constexpr uint32_t TIM15_OR2_BKINP       = TIM15_OR2_BKINP_Msk;
  static constexpr uint32_t TIM15_OR2_BKCMP1P_Pos = ( 10U );
  static constexpr uint32_t TIM15_OR2_BKCMP1P_Msk = ( 0x1UL << TIM15_OR2_BKCMP1P_Pos );
  static constexpr uint32_t TIM15_OR2_BKCMP1P     = TIM15_OR2_BKCMP1P_Msk;
  static constexpr uint32_t TIM15_OR2_BKCMP2P_Pos = ( 11U );
  static constexpr uint32_t TIM15_OR2_BKCMP2P_Msk = ( 0x1UL << TIM15_OR2_BKCMP2P_Pos );
  static constexpr uint32_t TIM15_OR2_BKCMP2P     = TIM15_OR2_BKCMP2P_Msk;

  /*******************  Bit definition for TIM16_OR1 register  ******************/
  static constexpr uint32_t TIM16_OR1_TI1_RMP_Pos = ( 0U );
  static constexpr uint32_t TIM16_OR1_TI1_RMP_Msk = ( 0x7UL << TIM16_OR1_TI1_RMP_Pos );
  static constexpr uint32_t TIM16_OR1_TI1_RMP     = TIM16_OR1_TI1_RMP_Msk;
  static constexpr uint32_t TIM16_OR1_TI1_RMP_0   = ( 0x1UL << TIM16_OR1_TI1_RMP_Pos );
  static constexpr uint32_t TIM16_OR1_TI1_RMP_1   = ( 0x2UL << TIM16_OR1_TI1_RMP_Pos );
  static constexpr uint32_t TIM16_OR1_TI1_RMP_2   = ( 0x4UL << TIM16_OR1_TI1_RMP_Pos );

  /*******************  Bit definition for TIM16_OR2 register  ******************/
  static constexpr uint32_t TIM16_OR2_BKINE_Pos   = ( 0U );
  static constexpr uint32_t TIM16_OR2_BKINE_Msk   = ( 0x1UL << TIM16_OR2_BKINE_Pos );
  static constexpr uint32_t TIM16_OR2_BKINE       = TIM16_OR2_BKINE_Msk;
  static constexpr uint32_t TIM16_OR2_BKCMP1E_Pos = ( 1U );
  static constexpr uint32_t TIM16_OR2_BKCMP1E_Msk = ( 0x1UL << TIM16_OR2_BKCMP1E_Pos );
  static constexpr uint32_t TIM16_OR2_BKCMP1E     = TIM16_OR2_BKCMP1E_Msk;
  static constexpr uint32_t TIM16_OR2_BKCMP2E_Pos = ( 2U );
  static constexpr uint32_t TIM16_OR2_BKCMP2E_Msk = ( 0x1UL << TIM16_OR2_BKCMP2E_Pos );
  static constexpr uint32_t TIM16_OR2_BKCMP2E     = TIM16_OR2_BKCMP2E_Msk;
  static constexpr uint32_t TIM16_OR2_BKINP_Pos   = ( 9U );
  static constexpr uint32_t TIM16_OR2_BKINP_Msk   = ( 0x1UL << TIM16_OR2_BKINP_Pos );
  static constexpr uint32_t TIM16_OR2_BKINP       = TIM16_OR2_BKINP_Msk;
  static constexpr uint32_t TIM16_OR2_BKCMP1P_Pos = ( 10U );
  static constexpr uint32_t TIM16_OR2_BKCMP1P_Msk = ( 0x1UL << TIM16_OR2_BKCMP1P_Pos );
  static constexpr uint32_t TIM16_OR2_BKCMP1P     = TIM16_OR2_BKCMP1P_Msk;
  static constexpr uint32_t TIM16_OR2_BKCMP2P_Pos = ( 11U );
  static constexpr uint32_t TIM16_OR2_BKCMP2P_Msk = ( 0x1UL << TIM16_OR2_BKCMP2P_Pos );
  static constexpr uint32_t TIM16_OR2_BKCMP2P     = TIM16_OR2_BKCMP2P_Msk;

  /******************  Bit definition for LPTIM_ISR register  *******************/
  static constexpr uint32_t LPTIM_ISR_CMPM_Pos    = ( 0U );
  static constexpr uint32_t LPTIM_ISR_CMPM_Msk    = ( 0x1UL << LPTIM_ISR_CMPM_Pos );
  static constexpr uint32_t LPTIM_ISR_CMPM        = LPTIM_ISR_CMPM_Msk;
  static constexpr uint32_t LPTIM_ISR_ARRM_Pos    = ( 1U );
  static constexpr uint32_t LPTIM_ISR_ARRM_Msk    = ( 0x1UL << LPTIM_ISR_ARRM_Pos );
  static constexpr uint32_t LPTIM_ISR_ARRM        = LPTIM_ISR_ARRM_Msk;
  static constexpr uint32_t LPTIM_ISR_EXTTRIG_Pos = ( 2U );
  static constexpr uint32_t LPTIM_ISR_EXTTRIG_Msk = ( 0x1UL << LPTIM_ISR_EXTTRIG_Pos );
  static constexpr uint32_t LPTIM_ISR_EXTTRIG     = LPTIM_ISR_EXTTRIG_Msk;
  static constexpr uint32_t LPTIM_ISR_CMPOK_Pos   = ( 3U );
  static constexpr uint32_t LPTIM_ISR_CMPOK_Msk   = ( 0x1UL << LPTIM_ISR_CMPOK_Pos );
  static constexpr uint32_t LPTIM_ISR_CMPOK       = LPTIM_ISR_CMPOK_Msk;
  static constexpr uint32_t LPTIM_ISR_ARROK_Pos   = ( 4U );
  static constexpr uint32_t LPTIM_ISR_ARROK_Msk   = ( 0x1UL << LPTIM_ISR_ARROK_Pos );
  static constexpr uint32_t LPTIM_ISR_ARROK       = LPTIM_ISR_ARROK_Msk;
  static constexpr uint32_t LPTIM_ISR_UP_Pos      = ( 5U );
  static constexpr uint32_t LPTIM_ISR_UP_Msk      = ( 0x1UL << LPTIM_ISR_UP_Pos );
  static constexpr uint32_t LPTIM_ISR_UP          = LPTIM_ISR_UP_Msk;
  static constexpr uint32_t LPTIM_ISR_DOWN_Pos    = ( 6U );
  static constexpr uint32_t LPTIM_ISR_DOWN_Msk    = ( 0x1UL << LPTIM_ISR_DOWN_Pos );
  static constexpr uint32_t LPTIM_ISR_DOWN        = LPTIM_ISR_DOWN_Msk;

  /******************  Bit definition for LPTIM_ICR register  *******************/
  static constexpr uint32_t LPTIM_ICR_CMPMCF_Pos    = ( 0U );
  static constexpr uint32_t LPTIM_ICR_CMPMCF_Msk    = ( 0x1UL << LPTIM_ICR_CMPMCF_Pos );
  static constexpr uint32_t LPTIM_ICR_CMPMCF        = LPTIM_ICR_CMPMCF_Msk;
  static constexpr uint32_t LPTIM_ICR_ARRMCF_Pos    = ( 1U );
  static constexpr uint32_t LPTIM_ICR_ARRMCF_Msk    = ( 0x1UL << LPTIM_ICR_ARRMCF_Pos );
  static constexpr uint32_t LPTIM_ICR_ARRMCF        = LPTIM_ICR_ARRMCF_Msk;
  static constexpr uint32_t LPTIM_ICR_EXTTRIGCF_Pos = ( 2U );
  static constexpr uint32_t LPTIM_ICR_EXTTRIGCF_Msk = ( 0x1UL << LPTIM_ICR_EXTTRIGCF_Pos );
  static constexpr uint32_t LPTIM_ICR_EXTTRIGCF     = LPTIM_ICR_EXTTRIGCF_Msk;
  static constexpr uint32_t LPTIM_ICR_CMPOKCF_Pos   = ( 3U );
  static constexpr uint32_t LPTIM_ICR_CMPOKCF_Msk   = ( 0x1UL << LPTIM_ICR_CMPOKCF_Pos );
  static constexpr uint32_t LPTIM_ICR_CMPOKCF       = LPTIM_ICR_CMPOKCF_Msk;
  static constexpr uint32_t LPTIM_ICR_ARROKCF_Pos   = ( 4U );
  static constexpr uint32_t LPTIM_ICR_ARROKCF_Msk   = ( 0x1UL << LPTIM_ICR_ARROKCF_Pos );
  static constexpr uint32_t LPTIM_ICR_ARROKCF       = LPTIM_ICR_ARROKCF_Msk;
  static constexpr uint32_t LPTIM_ICR_UPCF_Pos      = ( 5U );
  static constexpr uint32_t LPTIM_ICR_UPCF_Msk      = ( 0x1UL << LPTIM_ICR_UPCF_Pos );
  static constexpr uint32_t LPTIM_ICR_UPCF          = LPTIM_ICR_UPCF_Msk;
  static constexpr uint32_t LPTIM_ICR_DOWNCF_Pos    = ( 6U );
  static constexpr uint32_t LPTIM_ICR_DOWNCF_Msk    = ( 0x1UL << LPTIM_ICR_DOWNCF_Pos );
  static constexpr uint32_t LPTIM_ICR_DOWNCF        = LPTIM_ICR_DOWNCF_Msk;

  /******************  Bit definition for LPTIM_IER register ********************/
  static constexpr uint32_t LPTIM_IER_CMPMIE_Pos    = ( 0U );
  static constexpr uint32_t LPTIM_IER_CMPMIE_Msk    = ( 0x1UL << LPTIM_IER_CMPMIE_Pos );
  static constexpr uint32_t LPTIM_IER_CMPMIE        = LPTIM_IER_CMPMIE_Msk;
  static constexpr uint32_t LPTIM_IER_ARRMIE_Pos    = ( 1U );
  static constexpr uint32_t LPTIM_IER_ARRMIE_Msk    = ( 0x1UL << LPTIM_IER_ARRMIE_Pos );
  static constexpr uint32_t LPTIM_IER_ARRMIE        = LPTIM_IER_ARRMIE_Msk;
  static constexpr uint32_t LPTIM_IER_EXTTRIGIE_Pos = ( 2U );
  static constexpr uint32_t LPTIM_IER_EXTTRIGIE_Msk = ( 0x1UL << LPTIM_IER_EXTTRIGIE_Pos );
  static constexpr uint32_t LPTIM_IER_EXTTRIGIE     = LPTIM_IER_EXTTRIGIE_Msk;
  static constexpr uint32_t LPTIM_IER_CMPOKIE_Pos   = ( 3U );
  static constexpr uint32_t LPTIM_IER_CMPOKIE_Msk   = ( 0x1UL << LPTIM_IER_CMPOKIE_Pos );
  static constexpr uint32_t LPTIM_IER_CMPOKIE       = LPTIM_IER_CMPOKIE_Msk;
  static constexpr uint32_t LPTIM_IER_ARROKIE_Pos   = ( 4U );
  static constexpr uint32_t LPTIM_IER_ARROKIE_Msk   = ( 0x1UL << LPTIM_IER_ARROKIE_Pos );
  static constexpr uint32_t LPTIM_IER_ARROKIE       = LPTIM_IER_ARROKIE_Msk;
  static constexpr uint32_t LPTIM_IER_UPIE_Pos      = ( 5U );
  static constexpr uint32_t LPTIM_IER_UPIE_Msk      = ( 0x1UL << LPTIM_IER_UPIE_Pos );
  static constexpr uint32_t LPTIM_IER_UPIE          = LPTIM_IER_UPIE_Msk;
  static constexpr uint32_t LPTIM_IER_DOWNIE_Pos    = ( 6U );
  static constexpr uint32_t LPTIM_IER_DOWNIE_Msk    = ( 0x1UL << LPTIM_IER_DOWNIE_Pos );
  static constexpr uint32_t LPTIM_IER_DOWNIE        = LPTIM_IER_DOWNIE_Msk;

  /******************  Bit definition for LPTIM_CFGR register *******************/
  static constexpr uint32_t LPTIM_CFGR_CKSEL_Pos = ( 0U );
  static constexpr uint32_t LPTIM_CFGR_CKSEL_Msk = ( 0x1UL << LPTIM_CFGR_CKSEL_Pos );
  static constexpr uint32_t LPTIM_CFGR_CKSEL     = LPTIM_CFGR_CKSEL_Msk;

  static constexpr uint32_t LPTIM_CFGR_CKPOL_Pos = ( 1U );
  static constexpr uint32_t LPTIM_CFGR_CKPOL_Msk = ( 0x3UL << LPTIM_CFGR_CKPOL_Pos );
  static constexpr uint32_t LPTIM_CFGR_CKPOL     = LPTIM_CFGR_CKPOL_Msk;
  static constexpr uint32_t LPTIM_CFGR_CKPOL_0   = ( 0x1UL << LPTIM_CFGR_CKPOL_Pos );
  static constexpr uint32_t LPTIM_CFGR_CKPOL_1   = ( 0x2UL << LPTIM_CFGR_CKPOL_Pos );

  static constexpr uint32_t LPTIM_CFGR_CKFLT_Pos = ( 3U );
  static constexpr uint32_t LPTIM_CFGR_CKFLT_Msk = ( 0x3UL << LPTIM_CFGR_CKFLT_Pos );
  static constexpr uint32_t LPTIM_CFGR_CKFLT     = LPTIM_CFGR_CKFLT_Msk;
  static constexpr uint32_t LPTIM_CFGR_CKFLT_0   = ( 0x1UL << LPTIM_CFGR_CKFLT_Pos );
  static constexpr uint32_t LPTIM_CFGR_CKFLT_1   = ( 0x2UL << LPTIM_CFGR_CKFLT_Pos );

  static constexpr uint32_t LPTIM_CFGR_TRGFLT_Pos = ( 6U );
  static constexpr uint32_t LPTIM_CFGR_TRGFLT_Msk = ( 0x3UL << LPTIM_CFGR_TRGFLT_Pos );
  static constexpr uint32_t LPTIM_CFGR_TRGFLT     = LPTIM_CFGR_TRGFLT_Msk;
  static constexpr uint32_t LPTIM_CFGR_TRGFLT_0   = ( 0x1UL << LPTIM_CFGR_TRGFLT_Pos );
  static constexpr uint32_t LPTIM_CFGR_TRGFLT_1   = ( 0x2UL << LPTIM_CFGR_TRGFLT_Pos );

  static constexpr uint32_t LPTIM_CFGR_PRESC_Pos = ( 9U );
  static constexpr uint32_t LPTIM_CFGR_PRESC_Msk = ( 0x7UL << LPTIM_CFGR_PRESC_Pos );
  static constexpr uint32_t LPTIM_CFGR_PRESC     = LPTIM_CFGR_PRESC_Msk;
  static constexpr uint32_t LPTIM_CFGR_PRESC_0   = ( 0x1UL << LPTIM_CFGR_PRESC_Pos );
  static constexpr uint32_t LPTIM_CFGR_PRESC_1   = ( 0x2UL << LPTIM_CFGR_PRESC_Pos );
  static constexpr uint32_t LPTIM_CFGR_PRESC_2   = ( 0x4UL << LPTIM_CFGR_PRESC_Pos );

  static constexpr uint32_t LPTIM_CFGR_TRIGSEL_Pos = ( 13U );
  static constexpr uint32_t LPTIM_CFGR_TRIGSEL_Msk = ( 0x7UL << LPTIM_CFGR_TRIGSEL_Pos );
  static constexpr uint32_t LPTIM_CFGR_TRIGSEL     = LPTIM_CFGR_TRIGSEL_Msk;
  static constexpr uint32_t LPTIM_CFGR_TRIGSEL_0   = ( 0x1UL << LPTIM_CFGR_TRIGSEL_Pos );
  static constexpr uint32_t LPTIM_CFGR_TRIGSEL_1   = ( 0x2UL << LPTIM_CFGR_TRIGSEL_Pos );
  static constexpr uint32_t LPTIM_CFGR_TRIGSEL_2   = ( 0x4UL << LPTIM_CFGR_TRIGSEL_Pos );

  static constexpr uint32_t LPTIM_CFGR_TRIGEN_Pos = ( 17U );
  static constexpr uint32_t LPTIM_CFGR_TRIGEN_Msk = ( 0x3UL << LPTIM_CFGR_TRIGEN_Pos );
  static constexpr uint32_t LPTIM_CFGR_TRIGEN     = LPTIM_CFGR_TRIGEN_Msk;
  static constexpr uint32_t LPTIM_CFGR_TRIGEN_0   = ( 0x1UL << LPTIM_CFGR_TRIGEN_Pos );
  static constexpr uint32_t LPTIM_CFGR_TRIGEN_1   = ( 0x2UL << LPTIM_CFGR_TRIGEN_Pos );

  static constexpr uint32_t LPTIM_CFGR_TIMOUT_Pos    = ( 19U );
  static constexpr uint32_t LPTIM_CFGR_TIMOUT_Msk    = ( 0x1UL << LPTIM_CFGR_TIMOUT_Pos );
  static constexpr uint32_t LPTIM_CFGR_TIMOUT        = LPTIM_CFGR_TIMOUT_Msk;
  static constexpr uint32_t LPTIM_CFGR_WAVE_Pos      = ( 20U );
  static constexpr uint32_t LPTIM_CFGR_WAVE_Msk      = ( 0x1UL << LPTIM_CFGR_WAVE_Pos );
  static constexpr uint32_t LPTIM_CFGR_WAVE          = LPTIM_CFGR_WAVE_Msk;
  static constexpr uint32_t LPTIM_CFGR_WAVPOL_Pos    = ( 21U );
  static constexpr uint32_t LPTIM_CFGR_WAVPOL_Msk    = ( 0x1UL << LPTIM_CFGR_WAVPOL_Pos );
  static constexpr uint32_t LPTIM_CFGR_WAVPOL        = LPTIM_CFGR_WAVPOL_Msk;
  static constexpr uint32_t LPTIM_CFGR_PRELOAD_Pos   = ( 22U );
  static constexpr uint32_t LPTIM_CFGR_PRELOAD_Msk   = ( 0x1UL << LPTIM_CFGR_PRELOAD_Pos );
  static constexpr uint32_t LPTIM_CFGR_PRELOAD       = LPTIM_CFGR_PRELOAD_Msk;
  static constexpr uint32_t LPTIM_CFGR_COUNTMODE_Pos = ( 23U );
  static constexpr uint32_t LPTIM_CFGR_COUNTMODE_Msk = ( 0x1UL << LPTIM_CFGR_COUNTMODE_Pos );
  static constexpr uint32_t LPTIM_CFGR_COUNTMODE     = LPTIM_CFGR_COUNTMODE_Msk;
  static constexpr uint32_t LPTIM_CFGR_ENC_Pos       = ( 24U );
  static constexpr uint32_t LPTIM_CFGR_ENC_Msk       = ( 0x1UL << LPTIM_CFGR_ENC_Pos );
  static constexpr uint32_t LPTIM_CFGR_ENC           = LPTIM_CFGR_ENC_Msk;

  /******************  Bit definition for LPTIM_CR register  ********************/
  static constexpr uint32_t LPTIM_CR_ENABLE_Pos  = ( 0U );
  static constexpr uint32_t LPTIM_CR_ENABLE_Msk  = ( 0x1UL << LPTIM_CR_ENABLE_Pos );
  static constexpr uint32_t LPTIM_CR_ENABLE      = LPTIM_CR_ENABLE_Msk;
  static constexpr uint32_t LPTIM_CR_SNGSTRT_Pos = ( 1U );
  static constexpr uint32_t LPTIM_CR_SNGSTRT_Msk = ( 0x1UL << LPTIM_CR_SNGSTRT_Pos );
  static constexpr uint32_t LPTIM_CR_SNGSTRT     = LPTIM_CR_SNGSTRT_Msk;
  static constexpr uint32_t LPTIM_CR_CNTSTRT_Pos = ( 2U );
  static constexpr uint32_t LPTIM_CR_CNTSTRT_Msk = ( 0x1UL << LPTIM_CR_CNTSTRT_Pos );
  static constexpr uint32_t LPTIM_CR_CNTSTRT     = LPTIM_CR_CNTSTRT_Msk;

  /******************  Bit definition for LPTIM_CMP register  *******************/
  static constexpr uint32_t LPTIM_CMP_CMP_Pos = ( 0U );
  static constexpr uint32_t LPTIM_CMP_CMP_Msk = ( 0xFFFFUL << LPTIM_CMP_CMP_Pos );
  static constexpr uint32_t LPTIM_CMP_CMP     = LPTIM_CMP_CMP_Msk;

  /******************  Bit definition for LPTIM_ARR register  *******************/
  static constexpr uint32_t LPTIM_ARR_ARR_Pos = ( 0U );
  static constexpr uint32_t LPTIM_ARR_ARR_Msk = ( 0xFFFFUL << LPTIM_ARR_ARR_Pos );
  static constexpr uint32_t LPTIM_ARR_ARR     = LPTIM_ARR_ARR_Msk;

  /******************  Bit definition for LPTIM_CNT register  *******************/
  static constexpr uint32_t LPTIM_CNT_CNT_Pos = ( 0U );
  static constexpr uint32_t LPTIM_CNT_CNT_Msk = ( 0xFFFFUL << LPTIM_CNT_CNT_Pos );
  static constexpr uint32_t LPTIM_CNT_CNT     = LPTIM_CNT_CNT_Msk;

  /******************  Bit definition for LPTIM_OR register  ********************/
  static constexpr uint32_t LPTIM_OR_OR_Pos = ( 0U );
  static constexpr uint32_t LPTIM_OR_OR_Msk = ( 0x3UL << LPTIM_OR_OR_Pos );
  static constexpr uint32_t LPTIM_OR_OR     = LPTIM_OR_OR_Msk;
  static constexpr uint32_t LPTIM_OR_OR_0   = ( 0x1UL << LPTIM_OR_OR_Pos );
  static constexpr uint32_t LPTIM_OR_OR_1   = ( 0x2UL << LPTIM_OR_OR_Pos );

}    // namespace Thor::LLD::TIMER

#endif /* !THOR_HW_TIMER_REGISTER_STM32L432KC_HPP */