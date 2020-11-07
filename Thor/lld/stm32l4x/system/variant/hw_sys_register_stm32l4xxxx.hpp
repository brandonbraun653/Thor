/********************************************************************************
 *  File Name:
 *    hw_sys_register_stm32l4xxxx.hpp
 *
 *  Description:
 *    SYSCFG register definitions for the STM32L432KC series chips
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_HW_SYSCFG_REGISTER_STM32L4XXXX_HPP
#define THOR_HW_SYSCFG_REGISTER_STM32L4XXXX_HPP

/* Chimera Includes */
#include <Chimera/common>

/* Driver Includes */
#include <Thor/lld/stm32l4x/system/variant/sys_memory_map_stm32l432kc.hpp>

namespace Thor::LLD::SYS
{
  /*-------------------------------------------------------------------------------
  Peripheral Instance Memory Map Base
  -------------------------------------------------------------------------------*/
  static constexpr uint32_t SYSCFG_BASE_ADDR = Thor::System::MemoryMap::SYSCFG_PERIPH_START_ADDRESS;

  /*-------------------------------------------------------------------------------
  Peripheral Register Definitions
  -------------------------------------------------------------------------------*/
  /******************  Bit definition for MEMRMP register  ***************/
  static constexpr Reg32_t MEMRMP_MEM_MODE_Pos = ( 0U );
  static constexpr Reg32_t MEMRMP_MEM_MODE_Msk = ( 0x7UL << MEMRMP_MEM_MODE_Pos );
  static constexpr Reg32_t MEMRMP_MEM_MODE     = MEMRMP_MEM_MODE_Msk;
  static constexpr Reg32_t MEMRMP_MEM_MODE_0   = ( 0x1UL << MEMRMP_MEM_MODE_Pos );
  static constexpr Reg32_t MEMRMP_MEM_MODE_1   = ( 0x2UL << MEMRMP_MEM_MODE_Pos );
  static constexpr Reg32_t MEMRMP_MEM_MODE_2   = ( 0x4UL << MEMRMP_MEM_MODE_Pos );

  /******************  Bit definition for CFGR1 register  ******************/
  static constexpr Reg32_t CFGR1_FWDIS_Pos       = ( 0U );
  static constexpr Reg32_t CFGR1_FWDIS_Msk       = ( 0x1UL << CFGR1_FWDIS_Pos );
  static constexpr Reg32_t CFGR1_FWDIS           = CFGR1_FWDIS_Msk;
  static constexpr Reg32_t CFGR1_BOOSTEN_Pos     = ( 8U );
  static constexpr Reg32_t CFGR1_BOOSTEN_Msk     = ( 0x1UL << CFGR1_BOOSTEN_Pos );
  static constexpr Reg32_t CFGR1_BOOSTEN         = CFGR1_BOOSTEN_Msk;
  static constexpr Reg32_t CFGR1_I2C_PB6_FMP_Pos = ( 16U );
  static constexpr Reg32_t CFGR1_I2C_PB6_FMP_Msk = ( 0x1UL << CFGR1_I2C_PB6_FMP_Pos );
  static constexpr Reg32_t CFGR1_I2C_PB6_FMP     = CFGR1_I2C_PB6_FMP_Msk;
  static constexpr Reg32_t CFGR1_I2C_PB7_FMP_Pos = ( 17U );
  static constexpr Reg32_t CFGR1_I2C_PB7_FMP_Msk = ( 0x1UL << CFGR1_I2C_PB7_FMP_Pos );
  static constexpr Reg32_t CFGR1_I2C_PB7_FMP     = CFGR1_I2C_PB7_FMP_Msk;
  static constexpr Reg32_t CFGR1_I2C1_FMP_Pos    = ( 20U );
  static constexpr Reg32_t CFGR1_I2C1_FMP_Msk    = ( 0x1UL << CFGR1_I2C1_FMP_Pos );
  static constexpr Reg32_t CFGR1_I2C1_FMP        = CFGR1_I2C1_FMP_Msk;
  static constexpr Reg32_t CFGR1_I2C3_FMP_Pos    = ( 22U );
  static constexpr Reg32_t CFGR1_I2C3_FMP_Msk    = ( 0x1UL << CFGR1_I2C3_FMP_Pos );
  static constexpr Reg32_t CFGR1_I2C3_FMP        = CFGR1_I2C3_FMP_Msk;
  static constexpr Reg32_t CFGR1_FPU_IE_0        = ( 0x04000000UL );
  static constexpr Reg32_t CFGR1_FPU_IE_1        = ( 0x08000000UL );
  static constexpr Reg32_t CFGR1_FPU_IE_2        = ( 0x10000000UL );
  static constexpr Reg32_t CFGR1_FPU_IE_3        = ( 0x20000000UL );
  static constexpr Reg32_t CFGR1_FPU_IE_4        = ( 0x40000000UL );
  static constexpr Reg32_t CFGR1_FPU_IE_5        = ( 0x80000000UL );

  /*****************  Bit definition for EXTICR1 register  ***************/
  static constexpr Reg32_t EXTICR1_EXTI0_Pos = ( 0U );
  static constexpr Reg32_t EXTICR1_EXTI0_Msk = ( 0x7UL << EXTICR1_EXTI0_Pos );
  static constexpr Reg32_t EXTICR1_EXTI0     = EXTICR1_EXTI0_Msk;
  static constexpr Reg32_t EXTICR1_EXTI1_Pos = ( 4U );
  static constexpr Reg32_t EXTICR1_EXTI1_Msk = ( 0x7UL << EXTICR1_EXTI1_Pos );
  static constexpr Reg32_t EXTICR1_EXTI1     = EXTICR1_EXTI1_Msk;
  static constexpr Reg32_t EXTICR1_EXTI2_Pos = ( 8U );
  static constexpr Reg32_t EXTICR1_EXTI2_Msk = ( 0x7UL << EXTICR1_EXTI2_Pos );
  static constexpr Reg32_t EXTICR1_EXTI2     = EXTICR1_EXTI2_Msk;
  static constexpr Reg32_t EXTICR1_EXTI3_Pos = ( 12U );
  static constexpr Reg32_t EXTICR1_EXTI3_Msk = ( 0x7UL << EXTICR1_EXTI3_Pos );
  static constexpr Reg32_t EXTICR1_EXTI3     = EXTICR1_EXTI3_Msk;

  /**
   * @brief   EXTI0 configuration
   */
  static constexpr Reg32_t EXTICR1_EXTI0_PA = ( 0x00000000UL );
  static constexpr Reg32_t EXTICR1_EXTI0_PB = ( 0x00000001UL );
  static constexpr Reg32_t EXTICR1_EXTI0_PH = ( 0x00000007UL );

  /**
   * @brief   EXTI1 configuration
   */
  static constexpr Reg32_t EXTICR1_EXTI1_PA = ( 0x00000000UL );
  static constexpr Reg32_t EXTICR1_EXTI1_PB = ( 0x00000010UL );
  static constexpr Reg32_t EXTICR1_EXTI1_PH = ( 0x00000070UL );

  /**
   * @brief   EXTI2 configuration
   */
  static constexpr Reg32_t EXTICR1_EXTI2_PA = ( 0x00000000UL );

  /**
   * @brief   EXTI3 configuration
   */
  static constexpr Reg32_t EXTICR1_EXTI3_PA = ( 0x00000000UL );
  static constexpr Reg32_t EXTICR1_EXTI3_PB = ( 0x00001000UL );
  static constexpr Reg32_t EXTICR1_EXTI3_PG = ( 0x00006000UL );

  /*****************  Bit definition for EXTICR2 register  ***************/
  static constexpr Reg32_t EXTICR2_EXTI4_Pos = ( 0U );
  static constexpr Reg32_t EXTICR2_EXTI4_Msk = ( 0x7UL << EXTICR2_EXTI4_Pos );
  static constexpr Reg32_t EXTICR2_EXTI4     = EXTICR2_EXTI4_Msk;
  static constexpr Reg32_t EXTICR2_EXTI5_Pos = ( 4U );
  static constexpr Reg32_t EXTICR2_EXTI5_Msk = ( 0x7UL << EXTICR2_EXTI5_Pos );
  static constexpr Reg32_t EXTICR2_EXTI5     = EXTICR2_EXTI5_Msk;
  static constexpr Reg32_t EXTICR2_EXTI6_Pos = ( 8U );
  static constexpr Reg32_t EXTICR2_EXTI6_Msk = ( 0x7UL << EXTICR2_EXTI6_Pos );
  static constexpr Reg32_t EXTICR2_EXTI6     = EXTICR2_EXTI6_Msk;
  static constexpr Reg32_t EXTICR2_EXTI7_Pos = ( 12U );
  static constexpr Reg32_t EXTICR2_EXTI7_Msk = ( 0x7UL << EXTICR2_EXTI7_Pos );
  static constexpr Reg32_t EXTICR2_EXTI7     = EXTICR2_EXTI7_Msk;
  /**
   * @brief   EXTI4 configuration
   */
  static constexpr Reg32_t EXTICR2_EXTI4_PA = ( 0x00000000UL );
  static constexpr Reg32_t EXTICR2_EXTI4_PB = ( 0x00000001UL );

  /**
   * @brief   EXTI5 configuration
   */
  static constexpr Reg32_t EXTICR2_EXTI5_PA = ( 0x00000000UL );
  static constexpr Reg32_t EXTICR2_EXTI5_PB = ( 0x00000010UL );

  /**
   * @brief   EXTI6 configuration
   */
  static constexpr Reg32_t EXTICR2_EXTI6_PA = ( 0x00000000UL );
  static constexpr Reg32_t EXTICR2_EXTI6_PB = ( 0x00000100UL );

  /**
   * @brief   EXTI7 configuration
   */
  static constexpr Reg32_t EXTICR2_EXTI7_PA = ( 0x00000000UL );
  static constexpr Reg32_t EXTICR2_EXTI7_PB = ( 0x00001000UL );

  /*****************  Bit definition for EXTICR3 register  ***************/
  static constexpr Reg32_t EXTICR3_EXTI8_Pos  = ( 0U );
  static constexpr Reg32_t EXTICR3_EXTI8_Msk  = ( 0x7UL << EXTICR3_EXTI8_Pos );
  static constexpr Reg32_t EXTICR3_EXTI8      = EXTICR3_EXTI8_Msk;
  static constexpr Reg32_t EXTICR3_EXTI9_Pos  = ( 4U );
  static constexpr Reg32_t EXTICR3_EXTI9_Msk  = ( 0x7UL << EXTICR3_EXTI9_Pos );
  static constexpr Reg32_t EXTICR3_EXTI9      = EXTICR3_EXTI9_Msk;
  static constexpr Reg32_t EXTICR3_EXTI10_Pos = ( 8U );
  static constexpr Reg32_t EXTICR3_EXTI10_Msk = ( 0x7UL << EXTICR3_EXTI10_Pos );
  static constexpr Reg32_t EXTICR3_EXTI10     = EXTICR3_EXTI10_Msk;
  static constexpr Reg32_t EXTICR3_EXTI11_Pos = ( 12U );
  static constexpr Reg32_t EXTICR3_EXTI11_Msk = ( 0x7UL << EXTICR3_EXTI11_Pos );
  static constexpr Reg32_t EXTICR3_EXTI11     = EXTICR3_EXTI11_Msk;

  /**
   * @brief   EXTI8 configuration
   */
  static constexpr Reg32_t EXTICR3_EXTI8_PA = ( 0x00000000UL );

  /**
   * @brief   EXTI9 configuration
   */
  static constexpr Reg32_t EXTICR3_EXTI9_PA = ( 0x00000000UL );

  /**
   * @brief   EXTI10 configuration
   */
  static constexpr Reg32_t EXTICR3_EXTI10_PA = ( 0x00000000UL );

  /**
   * @brief   EXTI11 configuration
   */
  static constexpr Reg32_t EXTICR3_EXTI11_PA = ( 0x00000000UL );

  /*****************  Bit definition for EXTICR4 register  ***************/
  static constexpr Reg32_t EXTICR4_EXTI12_Pos = ( 0U );
  static constexpr Reg32_t EXTICR4_EXTI12_Msk = ( 0x7UL << EXTICR4_EXTI12_Pos );
  static constexpr Reg32_t EXTICR4_EXTI12     = EXTICR4_EXTI12_Msk;
  static constexpr Reg32_t EXTICR4_EXTI13_Pos = ( 4U );
  static constexpr Reg32_t EXTICR4_EXTI13_Msk = ( 0x7UL << EXTICR4_EXTI13_Pos );
  static constexpr Reg32_t EXTICR4_EXTI13     = EXTICR4_EXTI13_Msk;
  static constexpr Reg32_t EXTICR4_EXTI14_Pos = ( 8U );
  static constexpr Reg32_t EXTICR4_EXTI14_Msk = ( 0x7UL << EXTICR4_EXTI14_Pos );
  static constexpr Reg32_t EXTICR4_EXTI14     = EXTICR4_EXTI14_Msk;
  static constexpr Reg32_t EXTICR4_EXTI15_Pos = ( 12U );
  static constexpr Reg32_t EXTICR4_EXTI15_Msk = ( 0x7UL << EXTICR4_EXTI15_Pos );
  static constexpr Reg32_t EXTICR4_EXTI15     = EXTICR4_EXTI15_Msk;

  /**
   * @brief   EXTI12 configuration
   */
  static constexpr Reg32_t EXTICR4_EXTI12_PA = ( 0x00000000UL );

  /**
   * @brief   EXTI13 configuration
   */
  static constexpr Reg32_t EXTICR4_EXTI13_PA = ( 0x00000000UL );

  /**
   * @brief   EXTI14 configuration
   */
  static constexpr Reg32_t EXTICR4_EXTI14_PA = ( 0x00000000UL );
  static constexpr Reg32_t EXTICR4_EXTI14_PC = ( 0x00000200UL );

  /**
   * @brief   EXTI15 configuration
   */
  static constexpr Reg32_t EXTICR4_EXTI15_PA = ( 0x00000000UL );
  static constexpr Reg32_t EXTICR4_EXTI15_PC = ( 0x00002000UL );

  /******************  Bit definition for SCSR register  ****************/
  static constexpr Reg32_t SCSR_SRAM2ER_Pos  = ( 0U );
  static constexpr Reg32_t SCSR_SRAM2ER_Msk  = ( 0x1UL << SCSR_SRAM2ER_Pos );
  static constexpr Reg32_t SCSR_SRAM2ER      = SCSR_SRAM2ER_Msk;
  static constexpr Reg32_t SCSR_SRAM2BSY_Pos = ( 1U );
  static constexpr Reg32_t SCSR_SRAM2BSY_Msk = ( 0x1UL << SCSR_SRAM2BSY_Pos );
  static constexpr Reg32_t SCSR_SRAM2BSY     = SCSR_SRAM2BSY_Msk;

  /******************  Bit definition for CFGR2 register  ****************/
  static constexpr Reg32_t CFGR2_CLL_Pos  = ( 0U );
  static constexpr Reg32_t CFGR2_CLL_Msk  = ( 0x1UL << CFGR2_CLL_Pos );
  static constexpr Reg32_t CFGR2_CLL      = CFGR2_CLL_Msk;
  static constexpr Reg32_t CFGR2_SPL_Pos  = ( 1U );
  static constexpr Reg32_t CFGR2_SPL_Msk  = ( 0x1UL << CFGR2_SPL_Pos );
  static constexpr Reg32_t CFGR2_SPL      = CFGR2_SPL_Msk;
  static constexpr Reg32_t CFGR2_PVDL_Pos = ( 2U );
  static constexpr Reg32_t CFGR2_PVDL_Msk = ( 0x1UL << CFGR2_PVDL_Pos );
  static constexpr Reg32_t CFGR2_PVDL     = CFGR2_PVDL_Msk;
  static constexpr Reg32_t CFGR2_ECCL_Pos = ( 3U );
  static constexpr Reg32_t CFGR2_ECCL_Msk = ( 0x1UL << CFGR2_ECCL_Pos );
  static constexpr Reg32_t CFGR2_ECCL     = CFGR2_ECCL_Msk;
  static constexpr Reg32_t CFGR2_SPF_Pos  = ( 8U );
  static constexpr Reg32_t CFGR2_SPF_Msk  = ( 0x1UL << CFGR2_SPF_Pos );
  static constexpr Reg32_t CFGR2_SPF      = CFGR2_SPF_Msk;

  /******************  Bit definition for SWPR register  ****************/
  static constexpr Reg32_t SWPR_PAGE0_Pos  = ( 0U );
  static constexpr Reg32_t SWPR_PAGE0_Msk  = ( 0x1UL << SWPR_PAGE0_Pos );
  static constexpr Reg32_t SWPR_PAGE0      = SWPR_PAGE0_Msk;
  static constexpr Reg32_t SWPR_PAGE1_Pos  = ( 1U );
  static constexpr Reg32_t SWPR_PAGE1_Msk  = ( 0x1UL << SWPR_PAGE1_Pos );
  static constexpr Reg32_t SWPR_PAGE1      = SWPR_PAGE1_Msk;
  static constexpr Reg32_t SWPR_PAGE2_Pos  = ( 2U );
  static constexpr Reg32_t SWPR_PAGE2_Msk  = ( 0x1UL << SWPR_PAGE2_Pos );
  static constexpr Reg32_t SWPR_PAGE2      = SWPR_PAGE2_Msk;
  static constexpr Reg32_t SWPR_PAGE3_Pos  = ( 3U );
  static constexpr Reg32_t SWPR_PAGE3_Msk  = ( 0x1UL << SWPR_PAGE3_Pos );
  static constexpr Reg32_t SWPR_PAGE3      = SWPR_PAGE3_Msk;
  static constexpr Reg32_t SWPR_PAGE4_Pos  = ( 4U );
  static constexpr Reg32_t SWPR_PAGE4_Msk  = ( 0x1UL << SWPR_PAGE4_Pos );
  static constexpr Reg32_t SWPR_PAGE4      = SWPR_PAGE4_Msk;
  static constexpr Reg32_t SWPR_PAGE5_Pos  = ( 5U );
  static constexpr Reg32_t SWPR_PAGE5_Msk  = ( 0x1UL << SWPR_PAGE5_Pos );
  static constexpr Reg32_t SWPR_PAGE5      = SWPR_PAGE5_Msk;
  static constexpr Reg32_t SWPR_PAGE6_Pos  = ( 6U );
  static constexpr Reg32_t SWPR_PAGE6_Msk  = ( 0x1UL << SWPR_PAGE6_Pos );
  static constexpr Reg32_t SWPR_PAGE6      = SWPR_PAGE6_Msk;
  static constexpr Reg32_t SWPR_PAGE7_Pos  = ( 7U );
  static constexpr Reg32_t SWPR_PAGE7_Msk  = ( 0x1UL << SWPR_PAGE7_Pos );
  static constexpr Reg32_t SWPR_PAGE7      = SWPR_PAGE7_Msk;
  static constexpr Reg32_t SWPR_PAGE8_Pos  = ( 8U );
  static constexpr Reg32_t SWPR_PAGE8_Msk  = ( 0x1UL << SWPR_PAGE8_Pos );
  static constexpr Reg32_t SWPR_PAGE8      = SWPR_PAGE8_Msk;
  static constexpr Reg32_t SWPR_PAGE9_Pos  = ( 9U );
  static constexpr Reg32_t SWPR_PAGE9_Msk  = ( 0x1UL << SWPR_PAGE9_Pos );
  static constexpr Reg32_t SWPR_PAGE9      = SWPR_PAGE9_Msk;
  static constexpr Reg32_t SWPR_PAGE10_Pos = ( 10U );
  static constexpr Reg32_t SWPR_PAGE10_Msk = ( 0x1UL << SWPR_PAGE10_Pos );
  static constexpr Reg32_t SWPR_PAGE10     = SWPR_PAGE10_Msk;
  static constexpr Reg32_t SWPR_PAGE11_Pos = ( 11U );
  static constexpr Reg32_t SWPR_PAGE11_Msk = ( 0x1UL << SWPR_PAGE11_Pos );
  static constexpr Reg32_t SWPR_PAGE11     = SWPR_PAGE11_Msk;
  static constexpr Reg32_t SWPR_PAGE12_Pos = ( 12U );
  static constexpr Reg32_t SWPR_PAGE12_Msk = ( 0x1UL << SWPR_PAGE12_Pos );
  static constexpr Reg32_t SWPR_PAGE12     = SWPR_PAGE12_Msk;
  static constexpr Reg32_t SWPR_PAGE13_Pos = ( 13U );
  static constexpr Reg32_t SWPR_PAGE13_Msk = ( 0x1UL << SWPR_PAGE13_Pos );
  static constexpr Reg32_t SWPR_PAGE13     = SWPR_PAGE13_Msk;
  static constexpr Reg32_t SWPR_PAGE14_Pos = ( 14U );
  static constexpr Reg32_t SWPR_PAGE14_Msk = ( 0x1UL << SWPR_PAGE14_Pos );
  static constexpr Reg32_t SWPR_PAGE14     = SWPR_PAGE14_Msk;
  static constexpr Reg32_t SWPR_PAGE15_Pos = ( 15U );
  static constexpr Reg32_t SWPR_PAGE15_Msk = ( 0x1UL << SWPR_PAGE15_Pos );
  static constexpr Reg32_t SWPR_PAGE15     = SWPR_PAGE15_Msk;

  /******************  Bit definition for SKR register  ****************/
  static constexpr Reg32_t SKR_KEY_Pos = ( 0U );
  static constexpr Reg32_t SKR_KEY_Msk = ( 0xFFUL << SKR_KEY_Pos );
  static constexpr Reg32_t SKR_KEY     = SKR_KEY_Msk;

}    // namespace Thor::LLD::SYS

#endif /* !THOR_HW_SYSCFG_REGISTER_STM32L4XXXX_HPP */
