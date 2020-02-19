/********************************************************************************
 *   File Name:
 *    hw_rcc_register_stm32f446xx.hpp
 *
 *   Description:
 *    RCC register definitions for the STM32F446xx series chips.
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_POWER_REGISTER_HPP
#define THOR_HW_POWER_REGISTER_HPP

/* C++ Includes */
#include <cstdint>

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/lld/stm32f4x/system/variant/sys_memory_map_stm32f446xx.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_PWR == 1 )

#define STM32_PWR1_PERIPH_AVAILABLE

namespace Thor::Driver::PWR
{
  static constexpr uint32_t PWR_BASE_ADDR = Thor::System::MemoryMap::APB1PERIPH_BASE_ADDR + 0x7000U;

  /*------------------------------------------------
  CR Register
  ------------------------------------------------*/
  static constexpr uint32_t CR_LPDS_Pos   = ( 0U );
  static constexpr uint32_t CR_LPDS_Msk   = ( 0x1U << CR_LPDS_Pos ); /**< 0x00000001 */
  static constexpr uint32_t CR_LPDS       = CR_LPDS_Msk;             /**< Low-Power Deepsleep                 */
  static constexpr uint32_t CR_PDDS_Pos   = ( 1U );
  static constexpr uint32_t CR_PDDS_Msk   = ( 0x1U << CR_PDDS_Pos ); /**< 0x00000002 */
  static constexpr uint32_t CR_PDDS       = CR_PDDS_Msk;             /**< Power Down Deepsleep                */
  static constexpr uint32_t CR_CWUF_Pos   = ( 2U );
  static constexpr uint32_t CR_CWUF_Msk   = ( 0x1U << CR_CWUF_Pos ); /**< 0x00000004 */
  static constexpr uint32_t CR_CWUF       = CR_CWUF_Msk;             /**< Clear Wakeup Flag                   */
  static constexpr uint32_t CR_CSBF_Pos   = ( 3U );
  static constexpr uint32_t CR_CSBF_Msk   = ( 0x1U << CR_CSBF_Pos ); /**< 0x00000008 */
  static constexpr uint32_t CR_CSBF       = CR_CSBF_Msk;             /**< Clear Standby Flag                  */
  static constexpr uint32_t CR_PVDE_Pos   = ( 4U );
  static constexpr uint32_t CR_PVDE_Msk   = ( 0x1U << CR_PVDE_Pos ); /**< 0x00000010 */
  static constexpr uint32_t CR_PVDE       = CR_PVDE_Msk;             /**< Power Voltage Detector Enable       */
  static constexpr uint32_t CR_PLS_Pos    = ( 5U );
  static constexpr uint32_t CR_PLS_Msk    = ( 0x7U << CR_PLS_Pos ); /**< 0x000000E0 */
  static constexpr uint32_t CR_PLS        = CR_PLS_Msk;             /**< PLS[2:0] bits (PVD Level Selection) */
  static constexpr uint32_t CR_PLS_0      = ( 0x1U << CR_PLS_Pos ); /**< 0x00000020 */
  static constexpr uint32_t CR_PLS_1      = ( 0x2U << CR_PLS_Pos ); /**< 0x00000040 */
  static constexpr uint32_t CR_PLS_2      = ( 0x4U << CR_PLS_Pos ); /**< 0x00000080 */
  static constexpr uint32_t CR_PLS_LEV0   = 0x00000000U;            /**< PVD level 0 */
  static constexpr uint32_t CR_PLS_LEV1   = 0x00000020U;            /**< PVD level 1 */
  static constexpr uint32_t CR_PLS_LEV2   = 0x00000040U;            /**< PVD level 2 */
  static constexpr uint32_t CR_PLS_LEV3   = 0x00000060U;            /**< PVD level 3 */
  static constexpr uint32_t CR_PLS_LEV4   = 0x00000080U;            /**< PVD level 4 */
  static constexpr uint32_t CR_PLS_LEV5   = 0x000000A0U;            /**< PVD level 5 */
  static constexpr uint32_t CR_PLS_LEV6   = 0x000000C0U;            /**< PVD level 6 */
  static constexpr uint32_t CR_PLS_LEV7   = 0x000000E0U;            /**< PVD level 7 */
  static constexpr uint32_t CR_DBP_Pos    = ( 8U );
  static constexpr uint32_t CR_DBP_Msk    = ( 0x1U << CR_DBP_Pos ); /**< 0x00000100 */
  static constexpr uint32_t CR_DBP        = CR_DBP_Msk; /**< Disable Backup Domain write protection                     */
  static constexpr uint32_t CR_FPDS_Pos   = ( 9U );
  static constexpr uint32_t CR_FPDS_Msk   = ( 0x1U << CR_FPDS_Pos ); /**< 0x00000200 */
  static constexpr uint32_t CR_FPDS       = CR_FPDS_Msk; /**< Flash power down in Stop mode                              */
  static constexpr uint32_t CR_LPLVDS_Pos = ( 10U );
  static constexpr uint32_t CR_LPLVDS_Msk = ( 0x1U << CR_LPLVDS_Pos ); /**< 0x00000400 */
  static constexpr uint32_t CR_LPLVDS     = CR_LPLVDS_Msk; /**< Low-Power Regulator Low Voltage Scaling in Stop mode */
  static constexpr uint32_t CR_MRLVDS_Pos = ( 11U );
  static constexpr uint32_t CR_MRLVDS_Msk = ( 0x1U << CR_MRLVDS_Pos ); /**< 0x00000800 */
  static constexpr uint32_t CR_MRLVDS     = CR_MRLVDS_Msk;             /**< Main regulator Low Voltage Scaling in Stop mode */
  static constexpr uint32_t CR_ADCDC1_Pos = ( 13U );
  static constexpr uint32_t CR_ADCDC1_Msk = ( 0x1U << CR_ADCDC1_Pos ); /**< 0x00002000 */
  static constexpr uint32_t CR_ADCDC1     = CR_ADCDC1_Msk; /**< Refer to AN4073 on how to use this bit             */
  static constexpr uint32_t CR_VOS_Pos    = ( 14U );
  static constexpr uint32_t CR_VOS_Msk    = ( 0x3U << CR_VOS_Pos ); /**< 0x0000C000 */
  static constexpr uint32_t CR_VOS        = CR_VOS_Msk;  /**< VOS[1:0] bits (Regulator voltage scaling output selection) */
  static constexpr uint32_t CR_VOS_0      = 0x00004000U; /**< Bit 0 */
  static constexpr uint32_t CR_VOS_1      = 0x00008000U; /**< Bit 1 */
  static constexpr uint32_t CR_ODEN_Pos   = ( 16U );
  static constexpr uint32_t CR_ODEN_Msk   = ( 0x1U << CR_ODEN_Pos ); /**< 0x00010000 */
  static constexpr uint32_t CR_ODEN       = CR_ODEN_Msk;             /**< Over Drive enable                   */
  static constexpr uint32_t CR_ODSWEN_Pos = ( 17U );
  static constexpr uint32_t CR_ODSWEN_Msk = ( 0x1U << CR_ODSWEN_Pos ); /**< 0x00020000 */
  static constexpr uint32_t CR_ODSWEN     = CR_ODSWEN_Msk;             /**< Over Drive switch enabled           */
  static constexpr uint32_t CR_UDEN_Pos   = ( 18U );
  static constexpr uint32_t CR_UDEN_Msk   = ( 0x3U << CR_UDEN_Pos ); /**< 0x000C0000 */
  static constexpr uint32_t CR_UDEN       = CR_UDEN_Msk;             /**< Under Drive enable in stop mode     */
  static constexpr uint32_t CR_UDEN_0     = ( 0x1U << CR_UDEN_Pos ); /**< 0x00040000 */
  static constexpr uint32_t CR_UDEN_1     = ( 0x2U << CR_UDEN_Pos ); /**< 0x00080000 */
  static constexpr uint32_t CR_FMSSR_Pos  = ( 20U );
  static constexpr uint32_t CR_FMSSR_Msk  = ( 0x1U << CR_FMSSR_Pos ); /**< 0x00100000 */
  static constexpr uint32_t CR_FMSSR      = CR_FMSSR_Msk;             /**< Flash Memory Sleep System Run        */
  static constexpr uint32_t CR_FISSR_Pos  = ( 21U );
  static constexpr uint32_t CR_FISSR_Msk  = ( 0x1U << CR_FISSR_Pos ); /**< 0x00200000 */
  static constexpr uint32_t CR_FISSR      = CR_FISSR_Msk;             /**< Flash Interface Stop while System Run */

  /*------------------------------------------------
  CSR Register
  ------------------------------------------------*/
  static constexpr uint32_t CSR_WUF_Pos     = ( 0U );
  static constexpr uint32_t CSR_WUF_Msk     = ( 0x1U << CSR_WUF_Pos ); /**< 0x00000001 */
  static constexpr uint32_t CSR_WUF         = CSR_WUF_Msk;             /**< Wakeup Flag                                      */
  static constexpr uint32_t CSR_SBF_Pos     = ( 1U );
  static constexpr uint32_t CSR_SBF_Msk     = ( 0x1U << CSR_SBF_Pos ); /**< 0x00000002 */
  static constexpr uint32_t CSR_SBF         = CSR_SBF_Msk;             /**< Standby Flag                                     */
  static constexpr uint32_t CSR_PVDO_Pos    = ( 2U );
  static constexpr uint32_t CSR_PVDO_Msk    = ( 0x1U << CSR_PVDO_Pos ); /**< 0x00000004 */
  static constexpr uint32_t CSR_PVDO        = CSR_PVDO_Msk;             /**< PVD Output                                       */
  static constexpr uint32_t CSR_BRR_Pos     = ( 3U );
  static constexpr uint32_t CSR_BRR_Msk     = ( 0x1U << CSR_BRR_Pos ); /**< 0x00000008 */
  static constexpr uint32_t CSR_BRR         = CSR_BRR_Msk;             /**< Backup regulator ready                           */
  static constexpr uint32_t CSR_EWUP2_Pos   = ( 7U );
  static constexpr uint32_t CSR_EWUP2_Msk   = ( 0x1U << CSR_EWUP2_Pos ); /**< 0x00000080 */
  static constexpr uint32_t CSR_EWUP2       = CSR_EWUP2_Msk; /**< Enable WKUP pin 2                                */
  static constexpr uint32_t CSR_EWUP1_Pos   = ( 8U );
  static constexpr uint32_t CSR_EWUP1_Msk   = ( 0x1U << CSR_EWUP1_Pos ); /**< 0x00000100 */
  static constexpr uint32_t CSR_EWUP1       = CSR_EWUP1_Msk; /**< Enable WKUP pin 1                                */
  static constexpr uint32_t CSR_BRE_Pos     = ( 9U );
  static constexpr uint32_t CSR_BRE_Msk     = ( 0x1U << CSR_BRE_Pos ); /**< 0x00000200 */
  static constexpr uint32_t CSR_BRE         = CSR_BRE_Msk;             /**< Backup regulator enable                          */
  static constexpr uint32_t CSR_VOSRDY_Pos  = ( 14U );
  static constexpr uint32_t CSR_VOSRDY_Msk  = ( 0x1U << CSR_VOSRDY_Pos ); /**< 0x00004000 */
  static constexpr uint32_t CSR_VOSRDY      = CSR_VOSRDY_Msk; /**< Regulator voltage scaling output selection ready */
  static constexpr uint32_t CSR_ODRDY_Pos   = ( 16U );
  static constexpr uint32_t CSR_ODRDY_Msk   = ( 0x1U << CSR_ODRDY_Pos ); /**< 0x00010000 */
  static constexpr uint32_t CSR_ODRDY       = CSR_ODRDY_Msk; /**< Over Drive generator ready                       */
  static constexpr uint32_t CSR_ODSWRDY_Pos = ( 17U );
  static constexpr uint32_t CSR_ODSWRDY_Msk = ( 0x1U << CSR_ODSWRDY_Pos ); /**< 0x00020000 */
  static constexpr uint32_t CSR_ODSWRDY     = CSR_ODSWRDY_Msk; /**< Over Drive Switch ready                          */
  static constexpr uint32_t CSR_UDRDY_Pos   = ( 18U );
  static constexpr uint32_t CSR_UDRDY_Msk   = ( 0x3U << CSR_UDRDY_Pos ); /**< 0x000C0000 */
  static constexpr uint32_t CSR_UDRDY       = CSR_UDRDY_Msk; /**< Under Drive ready                                */
}    // namespace Thor::Driver::PWR

#endif /* TARGET_STM32F4 && THOR_DRIVER_PWR */
#endif /* !THOR_HW_POWER_REGISTER_HPP */