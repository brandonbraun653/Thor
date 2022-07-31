/******************************************************************************
 *  File Name:
 *    lld_timer_intf_register_bits.hpp
 *
 *  Description:
 *    Register bit descriptors for the common timer modules
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_TIMER_INTF_REGISTER_BITS_HPP
#define THOR_LLD_TIMER_INTF_REGISTER_BITS_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>

namespace Thor::LLD::TIMER
{
  /*******************  Bit definition for CR1 register  ********************/
  static constexpr uint32_t CR1_CEN_Pos  = ( 0U );
  static constexpr uint32_t CR1_CEN_Msk  = ( 0x1UL << CR1_CEN_Pos );
  static constexpr uint32_t CR1_CEN      = CR1_CEN_Msk;
  static constexpr uint32_t CR1_UDIS_Pos = ( 1U );
  static constexpr uint32_t CR1_UDIS_Msk = ( 0x1UL << CR1_UDIS_Pos );
  static constexpr uint32_t CR1_UDIS     = CR1_UDIS_Msk;
  static constexpr uint32_t CR1_URS_Pos  = ( 2U );
  static constexpr uint32_t CR1_URS_Msk  = ( 0x1UL << CR1_URS_Pos );
  static constexpr uint32_t CR1_URS      = CR1_URS_Msk;
  static constexpr uint32_t CR1_OPM_Pos  = ( 3U );
  static constexpr uint32_t CR1_OPM_Msk  = ( 0x1UL << CR1_OPM_Pos );
  static constexpr uint32_t CR1_OPM      = CR1_OPM_Msk;
  static constexpr uint32_t CR1_DIR_Pos  = ( 4U );
  static constexpr uint32_t CR1_DIR_Msk  = ( 0x1UL << CR1_DIR_Pos );
  static constexpr uint32_t CR1_DIR      = CR1_DIR_Msk;

  static constexpr uint32_t CR1_CMS_Pos = ( 5U );
  static constexpr uint32_t CR1_CMS_Msk = ( 0x3UL << CR1_CMS_Pos );
  static constexpr uint32_t CR1_CMS     = CR1_CMS_Msk;
  static constexpr uint32_t CR1_CMS_0   = ( 0x1UL << CR1_CMS_Pos );
  static constexpr uint32_t CR1_CMS_1   = ( 0x2UL << CR1_CMS_Pos );

  static constexpr uint32_t CR1_ARPE_Pos = ( 7U );
  static constexpr uint32_t CR1_ARPE_Msk = ( 0x1UL << CR1_ARPE_Pos );
  static constexpr uint32_t CR1_ARPE     = CR1_ARPE_Msk;

  static constexpr uint32_t CR1_CKD_Pos = ( 8U );
  static constexpr uint32_t CR1_CKD_Msk = ( 0x3UL << CR1_CKD_Pos );
  static constexpr uint32_t CR1_CKD     = CR1_CKD_Msk;
  static constexpr uint32_t CR1_CKD_0   = ( 0x1UL << CR1_CKD_Pos );
  static constexpr uint32_t CR1_CKD_1   = ( 0x2UL << CR1_CKD_Pos );

  static constexpr uint32_t CR1_UIFREMAP_Pos = ( 11U );
  static constexpr uint32_t CR1_UIFREMAP_Msk = ( 0x1UL << CR1_UIFREMAP_Pos );
  static constexpr uint32_t CR1_UIFREMAP     = CR1_UIFREMAP_Msk;

  /*******************  Bit definition for CR2 register  ********************/
  static constexpr uint32_t CR2_ALL      = ( 0x00F57FFD );
  static constexpr uint32_t CR2_CCPC_Pos = ( 0U );
  static constexpr uint32_t CR2_CCPC_Msk = ( 0x1UL << CR2_CCPC_Pos );
  static constexpr uint32_t CR2_CCPC     = CR2_CCPC_Msk;
  static constexpr uint32_t CR2_CCUS_Pos = ( 2U );
  static constexpr uint32_t CR2_CCUS_Msk = ( 0x1UL << CR2_CCUS_Pos );
  static constexpr uint32_t CR2_CCUS     = CR2_CCUS_Msk;
  static constexpr uint32_t CR2_CCDS_Pos = ( 3U );
  static constexpr uint32_t CR2_CCDS_Msk = ( 0x1UL << CR2_CCDS_Pos );
  static constexpr uint32_t CR2_CCDS     = CR2_CCDS_Msk;

  static constexpr uint32_t CR2_MMS_Pos = ( 4U );
  static constexpr uint32_t CR2_MMS_Msk = ( 0x7UL << CR2_MMS_Pos );
  static constexpr uint32_t CR2_MMS     = CR2_MMS_Msk;
  static constexpr uint32_t CR2_MMS_0   = ( 0x1UL << CR2_MMS_Pos );
  static constexpr uint32_t CR2_MMS_1   = ( 0x2UL << CR2_MMS_Pos );
  static constexpr uint32_t CR2_MMS_2   = ( 0x4UL << CR2_MMS_Pos );

  static constexpr uint32_t CR2_TI1S_Pos  = ( 7U );
  static constexpr uint32_t CR2_TI1S_Msk  = ( 0x1UL << CR2_TI1S_Pos );
  static constexpr uint32_t CR2_TI1S      = CR2_TI1S_Msk;
  static constexpr uint32_t CR2_OIS1_Pos  = ( 8U );
  static constexpr uint32_t CR2_OIS1_Msk  = ( 0x1UL << CR2_OIS1_Pos );
  static constexpr uint32_t CR2_OIS1      = CR2_OIS1_Msk;
  static constexpr uint32_t CR2_OIS1N_Pos = ( 9U );
  static constexpr uint32_t CR2_OIS1N_Msk = ( 0x1UL << CR2_OIS1N_Pos );
  static constexpr uint32_t CR2_OIS1N     = CR2_OIS1N_Msk;
  static constexpr uint32_t CR2_OIS2_Pos  = ( 10U );
  static constexpr uint32_t CR2_OIS2_Msk  = ( 0x1UL << CR2_OIS2_Pos );
  static constexpr uint32_t CR2_OIS2      = CR2_OIS2_Msk;
  static constexpr uint32_t CR2_OIS2N_Pos = ( 11U );
  static constexpr uint32_t CR2_OIS2N_Msk = ( 0x1UL << CR2_OIS2N_Pos );
  static constexpr uint32_t CR2_OIS2N     = CR2_OIS2N_Msk;
  static constexpr uint32_t CR2_OIS3_Pos  = ( 12U );
  static constexpr uint32_t CR2_OIS3_Msk  = ( 0x1UL << CR2_OIS3_Pos );
  static constexpr uint32_t CR2_OIS3      = CR2_OIS3_Msk;
  static constexpr uint32_t CR2_OIS3N_Pos = ( 13U );
  static constexpr uint32_t CR2_OIS3N_Msk = ( 0x1UL << CR2_OIS3N_Pos );
  static constexpr uint32_t CR2_OIS3N     = CR2_OIS3N_Msk;
  static constexpr uint32_t CR2_OIS4_Pos  = ( 14U );
  static constexpr uint32_t CR2_OIS4_Msk  = ( 0x1UL << CR2_OIS4_Pos );
  static constexpr uint32_t CR2_OIS4      = CR2_OIS4_Msk;
  static constexpr uint32_t CR2_OIS5_Pos  = ( 16U );
  static constexpr uint32_t CR2_OIS5_Msk  = ( 0x1UL << CR2_OIS5_Pos );
  static constexpr uint32_t CR2_OIS5      = CR2_OIS5_Msk;
  static constexpr uint32_t CR2_OIS6_Pos  = ( 18U );
  static constexpr uint32_t CR2_OIS6_Msk  = ( 0x1UL << CR2_OIS6_Pos );
  static constexpr uint32_t CR2_OIS6      = CR2_OIS6_Msk;

  static constexpr uint32_t CR2_MMS2_Pos = ( 20U );
  static constexpr uint32_t CR2_MMS2_Msk = ( 0xFUL << CR2_MMS2_Pos );
  static constexpr uint32_t CR2_MMS2     = CR2_MMS2_Msk;
  static constexpr uint32_t CR2_MMS2_0   = ( 0x1UL << CR2_MMS2_Pos );
  static constexpr uint32_t CR2_MMS2_1   = ( 0x2UL << CR2_MMS2_Pos );
  static constexpr uint32_t CR2_MMS2_2   = ( 0x4UL << CR2_MMS2_Pos );
  static constexpr uint32_t CR2_MMS2_3   = ( 0x8UL << CR2_MMS2_Pos );

  /*******************  Bit definition for SMCR register  *******************/
  static constexpr uint32_t SMCR_SMS_Pos = ( 0U );
  static constexpr uint32_t SMCR_SMS_Msk = ( 0x10007UL << SMCR_SMS_Pos );
  static constexpr uint32_t SMCR_SMS     = SMCR_SMS_Msk;
  static constexpr uint32_t SMCR_SMS_0   = ( 0x00001UL << SMCR_SMS_Pos );
  static constexpr uint32_t SMCR_SMS_1   = ( 0x00002UL << SMCR_SMS_Pos );
  static constexpr uint32_t SMCR_SMS_2   = ( 0x00004UL << SMCR_SMS_Pos );
  static constexpr uint32_t SMCR_SMS_3   = ( 0x10000UL << SMCR_SMS_Pos );

  static constexpr uint32_t SMCR_OCCS_Pos = ( 3U );
  static constexpr uint32_t SMCR_OCCS_Msk = ( 0x1UL << SMCR_OCCS_Pos );
  static constexpr uint32_t SMCR_OCCS     = SMCR_OCCS_Msk;

  static constexpr uint32_t SMCR_TS_Pos = ( 4U );
  static constexpr uint32_t SMCR_TS_Msk = ( 0x7UL << SMCR_TS_Pos );
  static constexpr uint32_t SMCR_TS     = SMCR_TS_Msk;
  static constexpr uint32_t SMCR_TS_0   = ( 0x1UL << SMCR_TS_Pos );
  static constexpr uint32_t SMCR_TS_1   = ( 0x2UL << SMCR_TS_Pos );
  static constexpr uint32_t SMCR_TS_2   = ( 0x4UL << SMCR_TS_Pos );

  static constexpr uint32_t SMCR_MSM_Pos = ( 7U );
  static constexpr uint32_t SMCR_MSM_Msk = ( 0x1UL << SMCR_MSM_Pos );
  static constexpr uint32_t SMCR_MSM     = SMCR_MSM_Msk;

  static constexpr uint32_t SMCR_ETF_Pos = ( 8U );
  static constexpr uint32_t SMCR_ETF_Msk = ( 0xFUL << SMCR_ETF_Pos );
  static constexpr uint32_t SMCR_ETF     = SMCR_ETF_Msk;
  static constexpr uint32_t SMCR_ETF_0   = ( 0x1UL << SMCR_ETF_Pos );
  static constexpr uint32_t SMCR_ETF_1   = ( 0x2UL << SMCR_ETF_Pos );
  static constexpr uint32_t SMCR_ETF_2   = ( 0x4UL << SMCR_ETF_Pos );
  static constexpr uint32_t SMCR_ETF_3   = ( 0x8UL << SMCR_ETF_Pos );

  static constexpr uint32_t SMCR_ETPS_Pos = ( 12U );
  static constexpr uint32_t SMCR_ETPS_Msk = ( 0x3UL << SMCR_ETPS_Pos );
  static constexpr uint32_t SMCR_ETPS     = SMCR_ETPS_Msk;
  static constexpr uint32_t SMCR_ETPS_0   = ( 0x1UL << SMCR_ETPS_Pos );
  static constexpr uint32_t SMCR_ETPS_1   = ( 0x2UL << SMCR_ETPS_Pos );

  static constexpr uint32_t SMCR_ECE_Pos = ( 14U );
  static constexpr uint32_t SMCR_ECE_Msk = ( 0x1UL << SMCR_ECE_Pos );
  static constexpr uint32_t SMCR_ECE     = SMCR_ECE_Msk;
  static constexpr uint32_t SMCR_ETP_Pos = ( 15U );
  static constexpr uint32_t SMCR_ETP_Msk = ( 0x1UL << SMCR_ETP_Pos );
  static constexpr uint32_t SMCR_ETP     = SMCR_ETP_Msk;

  /*******************  Bit definition for DIER register  *******************/
  static constexpr uint32_t DIER_ALL_Msk = ( 0x7FFF );

  static constexpr uint32_t DIER_UIE_Pos   = ( 0U );
  static constexpr uint32_t DIER_UIE_Msk   = ( 0x1UL << DIER_UIE_Pos );
  static constexpr uint32_t DIER_UIE       = DIER_UIE_Msk;
  static constexpr uint32_t DIER_CC1IE_Pos = ( 1U );
  static constexpr uint32_t DIER_CC1IE_Msk = ( 0x1UL << DIER_CC1IE_Pos );
  static constexpr uint32_t DIER_CC1IE     = DIER_CC1IE_Msk;
  static constexpr uint32_t DIER_CC2IE_Pos = ( 2U );
  static constexpr uint32_t DIER_CC2IE_Msk = ( 0x1UL << DIER_CC2IE_Pos );
  static constexpr uint32_t DIER_CC2IE     = DIER_CC2IE_Msk;
  static constexpr uint32_t DIER_CC3IE_Pos = ( 3U );
  static constexpr uint32_t DIER_CC3IE_Msk = ( 0x1UL << DIER_CC3IE_Pos );
  static constexpr uint32_t DIER_CC3IE     = DIER_CC3IE_Msk;
  static constexpr uint32_t DIER_CC4IE_Pos = ( 4U );
  static constexpr uint32_t DIER_CC4IE_Msk = ( 0x1UL << DIER_CC4IE_Pos );
  static constexpr uint32_t DIER_CC4IE     = DIER_CC4IE_Msk;
  static constexpr uint32_t DIER_COMIE_Pos = ( 5U );
  static constexpr uint32_t DIER_COMIE_Msk = ( 0x1UL << DIER_COMIE_Pos );
  static constexpr uint32_t DIER_COMIE     = DIER_COMIE_Msk;
  static constexpr uint32_t DIER_TIE_Pos   = ( 6U );
  static constexpr uint32_t DIER_TIE_Msk   = ( 0x1UL << DIER_TIE_Pos );
  static constexpr uint32_t DIER_TIE       = DIER_TIE_Msk;
  static constexpr uint32_t DIER_BIE_Pos   = ( 7U );
  static constexpr uint32_t DIER_BIE_Msk   = ( 0x1UL << DIER_BIE_Pos );
  static constexpr uint32_t DIER_BIE       = DIER_BIE_Msk;
  static constexpr uint32_t DIER_UDE_Pos   = ( 8U );
  static constexpr uint32_t DIER_UDE_Msk   = ( 0x1UL << DIER_UDE_Pos );
  static constexpr uint32_t DIER_UDE       = DIER_UDE_Msk;
  static constexpr uint32_t DIER_CC1DE_Pos = ( 9U );
  static constexpr uint32_t DIER_CC1DE_Msk = ( 0x1UL << DIER_CC1DE_Pos );
  static constexpr uint32_t DIER_CC1DE     = DIER_CC1DE_Msk;
  static constexpr uint32_t DIER_CC2DE_Pos = ( 10U );
  static constexpr uint32_t DIER_CC2DE_Msk = ( 0x1UL << DIER_CC2DE_Pos );
  static constexpr uint32_t DIER_CC2DE     = DIER_CC2DE_Msk;
  static constexpr uint32_t DIER_CC3DE_Pos = ( 11U );
  static constexpr uint32_t DIER_CC3DE_Msk = ( 0x1UL << DIER_CC3DE_Pos );
  static constexpr uint32_t DIER_CC3DE     = DIER_CC3DE_Msk;
  static constexpr uint32_t DIER_CC4DE_Pos = ( 12U );
  static constexpr uint32_t DIER_CC4DE_Msk = ( 0x1UL << DIER_CC4DE_Pos );
  static constexpr uint32_t DIER_CC4DE     = DIER_CC4DE_Msk;
  static constexpr uint32_t DIER_COMDE_Pos = ( 13U );
  static constexpr uint32_t DIER_COMDE_Msk = ( 0x1UL << DIER_COMDE_Pos );
  static constexpr uint32_t DIER_COMDE     = DIER_COMDE_Msk;
  static constexpr uint32_t DIER_TDE_Pos   = ( 14U );
  static constexpr uint32_t DIER_TDE_Msk   = ( 0x1UL << DIER_TDE_Pos );
  static constexpr uint32_t DIER_TDE       = DIER_TDE_Msk;

  /********************  Bit definition for SR register  ********************/
  static constexpr uint32_t SR_UIF_Pos   = ( 0U );
  static constexpr uint32_t SR_UIF_Msk   = ( 0x1UL << SR_UIF_Pos );
  static constexpr uint32_t SR_UIF       = SR_UIF_Msk;
  static constexpr uint32_t SR_CC1IF_Pos = ( 1U );
  static constexpr uint32_t SR_CC1IF_Msk = ( 0x1UL << SR_CC1IF_Pos );
  static constexpr uint32_t SR_CC1IF     = SR_CC1IF_Msk;
  static constexpr uint32_t SR_CC2IF_Pos = ( 2U );
  static constexpr uint32_t SR_CC2IF_Msk = ( 0x1UL << SR_CC2IF_Pos );
  static constexpr uint32_t SR_CC2IF     = SR_CC2IF_Msk;
  static constexpr uint32_t SR_CC3IF_Pos = ( 3U );
  static constexpr uint32_t SR_CC3IF_Msk = ( 0x1UL << SR_CC3IF_Pos );
  static constexpr uint32_t SR_CC3IF     = SR_CC3IF_Msk;
  static constexpr uint32_t SR_CC4IF_Pos = ( 4U );
  static constexpr uint32_t SR_CC4IF_Msk = ( 0x1UL << SR_CC4IF_Pos );
  static constexpr uint32_t SR_CC4IF     = SR_CC4IF_Msk;
  static constexpr uint32_t SR_COMIF_Pos = ( 5U );
  static constexpr uint32_t SR_COMIF_Msk = ( 0x1UL << SR_COMIF_Pos );
  static constexpr uint32_t SR_COMIF     = SR_COMIF_Msk;
  static constexpr uint32_t SR_TIF_Pos   = ( 6U );
  static constexpr uint32_t SR_TIF_Msk   = ( 0x1UL << SR_TIF_Pos );
  static constexpr uint32_t SR_TIF       = SR_TIF_Msk;
  static constexpr uint32_t SR_BIF_Pos   = ( 7U );
  static constexpr uint32_t SR_BIF_Msk   = ( 0x1UL << SR_BIF_Pos );
  static constexpr uint32_t SR_BIF       = SR_BIF_Msk;
  static constexpr uint32_t SR_B2IF_Pos  = ( 8U );
  static constexpr uint32_t SR_B2IF_Msk  = ( 0x1UL << SR_B2IF_Pos );
  static constexpr uint32_t SR_B2IF      = SR_B2IF_Msk;
  static constexpr uint32_t SR_CC1OF_Pos = ( 9U );
  static constexpr uint32_t SR_CC1OF_Msk = ( 0x1UL << SR_CC1OF_Pos );
  static constexpr uint32_t SR_CC1OF     = SR_CC1OF_Msk;
  static constexpr uint32_t SR_CC2OF_Pos = ( 10U );
  static constexpr uint32_t SR_CC2OF_Msk = ( 0x1UL << SR_CC2OF_Pos );
  static constexpr uint32_t SR_CC2OF     = SR_CC2OF_Msk;
  static constexpr uint32_t SR_CC3OF_Pos = ( 11U );
  static constexpr uint32_t SR_CC3OF_Msk = ( 0x1UL << SR_CC3OF_Pos );
  static constexpr uint32_t SR_CC3OF     = SR_CC3OF_Msk;
  static constexpr uint32_t SR_CC4OF_Pos = ( 12U );
  static constexpr uint32_t SR_CC4OF_Msk = ( 0x1UL << SR_CC4OF_Pos );
  static constexpr uint32_t SR_CC4OF     = SR_CC4OF_Msk;
  static constexpr uint32_t SR_SBIF_Pos  = ( 13U );
  static constexpr uint32_t SR_SBIF_Msk  = ( 0x1UL << SR_SBIF_Pos );
  static constexpr uint32_t SR_SBIF      = SR_SBIF_Msk;
  static constexpr uint32_t SR_CC5IF_Pos = ( 16U );
  static constexpr uint32_t SR_CC5IF_Msk = ( 0x1UL << SR_CC5IF_Pos );
  static constexpr uint32_t SR_CC5IF     = SR_CC5IF_Msk;
  static constexpr uint32_t SR_CC6IF_Pos = ( 17U );
  static constexpr uint32_t SR_CC6IF_Msk = ( 0x1UL << SR_CC6IF_Pos );
  static constexpr uint32_t SR_CC6IF     = SR_CC6IF_Msk;

  /*******************  Bit definition for EGR register  ********************/
  static constexpr uint32_t EGR_UG_Pos   = ( 0U );
  static constexpr uint32_t EGR_UG_Msk   = ( 0x1UL << EGR_UG_Pos );
  static constexpr uint32_t EGR_UG       = EGR_UG_Msk;
  static constexpr uint32_t EGR_CC1G_Pos = ( 1U );
  static constexpr uint32_t EGR_CC1G_Msk = ( 0x1UL << EGR_CC1G_Pos );
  static constexpr uint32_t EGR_CC1G     = EGR_CC1G_Msk;
  static constexpr uint32_t EGR_CC2G_Pos = ( 2U );
  static constexpr uint32_t EGR_CC2G_Msk = ( 0x1UL << EGR_CC2G_Pos );
  static constexpr uint32_t EGR_CC2G     = EGR_CC2G_Msk;
  static constexpr uint32_t EGR_CC3G_Pos = ( 3U );
  static constexpr uint32_t EGR_CC3G_Msk = ( 0x1UL << EGR_CC3G_Pos );
  static constexpr uint32_t EGR_CC3G     = EGR_CC3G_Msk;
  static constexpr uint32_t EGR_CC4G_Pos = ( 4U );
  static constexpr uint32_t EGR_CC4G_Msk = ( 0x1UL << EGR_CC4G_Pos );
  static constexpr uint32_t EGR_CC4G     = EGR_CC4G_Msk;
  static constexpr uint32_t EGR_COMG_Pos = ( 5U );
  static constexpr uint32_t EGR_COMG_Msk = ( 0x1UL << EGR_COMG_Pos );
  static constexpr uint32_t EGR_COMG     = EGR_COMG_Msk;
  static constexpr uint32_t EGR_TG_Pos   = ( 6U );
  static constexpr uint32_t EGR_TG_Msk   = ( 0x1UL << EGR_TG_Pos );
  static constexpr uint32_t EGR_TG       = EGR_TG_Msk;
  static constexpr uint32_t EGR_BG_Pos   = ( 7U );
  static constexpr uint32_t EGR_BG_Msk   = ( 0x1UL << EGR_BG_Pos );
  static constexpr uint32_t EGR_BG       = EGR_BG_Msk;
  static constexpr uint32_t EGR_B2G_Pos  = ( 8U );
  static constexpr uint32_t EGR_B2G_Msk  = ( 0x1UL << EGR_B2G_Pos );
  static constexpr uint32_t EGR_B2G      = EGR_B2G_Msk;

  /******************  Bit definition for CCMR1 register  *******************/
  static constexpr uint32_t CCMR1_CC1S_Pos = ( 0U );
  static constexpr uint32_t CCMR1_CC1S_Msk = ( 0x3UL << CCMR1_CC1S_Pos );
  static constexpr uint32_t CCMR1_CC1S     = CCMR1_CC1S_Msk;
  static constexpr uint32_t CCMR1_CC1S_0   = ( 0x1UL << CCMR1_CC1S_Pos );
  static constexpr uint32_t CCMR1_CC1S_1   = ( 0x2UL << CCMR1_CC1S_Pos );

  static constexpr uint32_t CCMR1_OC1FE_Pos = ( 2U );
  static constexpr uint32_t CCMR1_OC1FE_Msk = ( 0x1UL << CCMR1_OC1FE_Pos );
  static constexpr uint32_t CCMR1_OC1FE     = CCMR1_OC1FE_Msk;
  static constexpr uint32_t CCMR1_OC1PE_Pos = ( 3U );
  static constexpr uint32_t CCMR1_OC1PE_Msk = ( 0x1UL << CCMR1_OC1PE_Pos );
  static constexpr uint32_t CCMR1_OC1PE     = CCMR1_OC1PE_Msk;

  static constexpr uint32_t CCMR1_OC1M_Pos = ( 4U );
  static constexpr uint32_t CCMR1_OC1M_Msk = ( 0x1007UL << CCMR1_OC1M_Pos );
  static constexpr uint32_t CCMR1_OC1M     = CCMR1_OC1M_Msk;
  static constexpr uint32_t CCMR1_OC1M_0   = ( 0x0001UL << CCMR1_OC1M_Pos );
  static constexpr uint32_t CCMR1_OC1M_1   = ( 0x0002UL << CCMR1_OC1M_Pos );
  static constexpr uint32_t CCMR1_OC1M_2   = ( 0x0004UL << CCMR1_OC1M_Pos );
  static constexpr uint32_t CCMR1_OC1M_3   = ( 0x1000UL << CCMR1_OC1M_Pos );

  static constexpr uint32_t CCMR1_OC1CE_Pos = ( 7U );
  static constexpr uint32_t CCMR1_OC1CE_Msk = ( 0x1UL << CCMR1_OC1CE_Pos );
  static constexpr uint32_t CCMR1_OC1CE     = CCMR1_OC1CE_Msk;

  static constexpr uint32_t CCMR1_CC2S_Pos = ( 8U );
  static constexpr uint32_t CCMR1_CC2S_Msk = ( 0x3UL << CCMR1_CC2S_Pos );
  static constexpr uint32_t CCMR1_CC2S     = CCMR1_CC2S_Msk;
  static constexpr uint32_t CCMR1_CC2S_0   = ( 0x1UL << CCMR1_CC2S_Pos );
  static constexpr uint32_t CCMR1_CC2S_1   = ( 0x2UL << CCMR1_CC2S_Pos );

  static constexpr uint32_t CCMR1_OC2FE_Pos = ( 10U );
  static constexpr uint32_t CCMR1_OC2FE_Msk = ( 0x1UL << CCMR1_OC2FE_Pos );
  static constexpr uint32_t CCMR1_OC2FE     = CCMR1_OC2FE_Msk;
  static constexpr uint32_t CCMR1_OC2PE_Pos = ( 11U );
  static constexpr uint32_t CCMR1_OC2PE_Msk = ( 0x1UL << CCMR1_OC2PE_Pos );
  static constexpr uint32_t CCMR1_OC2PE     = CCMR1_OC2PE_Msk;

  static constexpr uint32_t CCMR1_OC2M_Pos = ( 12U );
  static constexpr uint32_t CCMR1_OC2M_Msk = ( 0x1007UL << CCMR1_OC2M_Pos );
  static constexpr uint32_t CCMR1_OC2M     = CCMR1_OC2M_Msk;
  static constexpr uint32_t CCMR1_OC2M_0   = ( 0x0001UL << CCMR1_OC2M_Pos );
  static constexpr uint32_t CCMR1_OC2M_1   = ( 0x0002UL << CCMR1_OC2M_Pos );
  static constexpr uint32_t CCMR1_OC2M_2   = ( 0x0004UL << CCMR1_OC2M_Pos );
  static constexpr uint32_t CCMR1_OC2M_3   = ( 0x1000UL << CCMR1_OC2M_Pos );

  static constexpr uint32_t CCMR1_OC2CE_Pos = ( 15U );
  static constexpr uint32_t CCMR1_OC2CE_Msk = ( 0x1UL << CCMR1_OC2CE_Pos );
  static constexpr uint32_t CCMR1_OC2CE     = CCMR1_OC2CE_Msk;

  static constexpr uint32_t CCMR1_IC1PSC_Pos = ( 2U );
  static constexpr uint32_t CCMR1_IC1PSC_Msk = ( 0x3UL << CCMR1_IC1PSC_Pos );
  static constexpr uint32_t CCMR1_IC1PSC     = CCMR1_IC1PSC_Msk;
  static constexpr uint32_t CCMR1_IC1PSC_0   = ( 0x1UL << CCMR1_IC1PSC_Pos );
  static constexpr uint32_t CCMR1_IC1PSC_1   = ( 0x2UL << CCMR1_IC1PSC_Pos );

  static constexpr uint32_t CCMR1_IC1F_Pos = ( 4U );
  static constexpr uint32_t CCMR1_IC1F_Msk = ( 0xFUL << CCMR1_IC1F_Pos );
  static constexpr uint32_t CCMR1_IC1F     = CCMR1_IC1F_Msk;
  static constexpr uint32_t CCMR1_IC1F_0   = ( 0x1UL << CCMR1_IC1F_Pos );
  static constexpr uint32_t CCMR1_IC1F_1   = ( 0x2UL << CCMR1_IC1F_Pos );
  static constexpr uint32_t CCMR1_IC1F_2   = ( 0x4UL << CCMR1_IC1F_Pos );
  static constexpr uint32_t CCMR1_IC1F_3   = ( 0x8UL << CCMR1_IC1F_Pos );

  static constexpr uint32_t CCMR1_IC2PSC_Pos = ( 10U );
  static constexpr uint32_t CCMR1_IC2PSC_Msk = ( 0x3UL << CCMR1_IC2PSC_Pos );
  static constexpr uint32_t CCMR1_IC2PSC     = CCMR1_IC2PSC_Msk;
  static constexpr uint32_t CCMR1_IC2PSC_0   = ( 0x1UL << CCMR1_IC2PSC_Pos );
  static constexpr uint32_t CCMR1_IC2PSC_1   = ( 0x2UL << CCMR1_IC2PSC_Pos );

  static constexpr uint32_t CCMR1_IC2F_Pos = ( 12U );
  static constexpr uint32_t CCMR1_IC2F_Msk = ( 0xFUL << CCMR1_IC2F_Pos );
  static constexpr uint32_t CCMR1_IC2F     = CCMR1_IC2F_Msk;
  static constexpr uint32_t CCMR1_IC2F_0   = ( 0x1UL << CCMR1_IC2F_Pos );
  static constexpr uint32_t CCMR1_IC2F_1   = ( 0x2UL << CCMR1_IC2F_Pos );
  static constexpr uint32_t CCMR1_IC2F_2   = ( 0x4UL << CCMR1_IC2F_Pos );
  static constexpr uint32_t CCMR1_IC2F_3   = ( 0x8UL << CCMR1_IC2F_Pos );

  /******************  Bit definition for CCMR2 register  *******************/
  static constexpr uint32_t CCMR2_CC3S_Pos = ( 0U );
  static constexpr uint32_t CCMR2_CC3S_Msk = ( 0x3UL << CCMR2_CC3S_Pos );
  static constexpr uint32_t CCMR2_CC3S     = CCMR2_CC3S_Msk;
  static constexpr uint32_t CCMR2_CC3S_0   = ( 0x1UL << CCMR2_CC3S_Pos );
  static constexpr uint32_t CCMR2_CC3S_1   = ( 0x2UL << CCMR2_CC3S_Pos );

  static constexpr uint32_t CCMR2_OC3FE_Pos = ( 2U );
  static constexpr uint32_t CCMR2_OC3FE_Msk = ( 0x1UL << CCMR2_OC3FE_Pos );
  static constexpr uint32_t CCMR2_OC3FE     = CCMR2_OC3FE_Msk;
  static constexpr uint32_t CCMR2_OC3PE_Pos = ( 3U );
  static constexpr uint32_t CCMR2_OC3PE_Msk = ( 0x1UL << CCMR2_OC3PE_Pos );
  static constexpr uint32_t CCMR2_OC3PE     = CCMR2_OC3PE_Msk;

  static constexpr uint32_t CCMR2_OC3M_Pos = ( 4U );
  static constexpr uint32_t CCMR2_OC3M_Msk = ( 0x1007UL << CCMR2_OC3M_Pos );
  static constexpr uint32_t CCMR2_OC3M     = CCMR2_OC3M_Msk;
  static constexpr uint32_t CCMR2_OC3M_0   = ( 0x0001UL << CCMR2_OC3M_Pos );
  static constexpr uint32_t CCMR2_OC3M_1   = ( 0x0002UL << CCMR2_OC3M_Pos );
  static constexpr uint32_t CCMR2_OC3M_2   = ( 0x0004UL << CCMR2_OC3M_Pos );
  static constexpr uint32_t CCMR2_OC3M_3   = ( 0x1000UL << CCMR2_OC3M_Pos );

  static constexpr uint32_t CCMR2_OC3CE_Pos = ( 7U );
  static constexpr uint32_t CCMR2_OC3CE_Msk = ( 0x1UL << CCMR2_OC3CE_Pos );
  static constexpr uint32_t CCMR2_OC3CE     = CCMR2_OC3CE_Msk;

  static constexpr uint32_t CCMR2_CC4S_Pos = ( 8U );
  static constexpr uint32_t CCMR2_CC4S_Msk = ( 0x3UL << CCMR2_CC4S_Pos );
  static constexpr uint32_t CCMR2_CC4S     = CCMR2_CC4S_Msk;
  static constexpr uint32_t CCMR2_CC4S_0   = ( 0x1UL << CCMR2_CC4S_Pos );
  static constexpr uint32_t CCMR2_CC4S_1   = ( 0x2UL << CCMR2_CC4S_Pos );

  static constexpr uint32_t CCMR2_OC4FE_Pos = ( 10U );
  static constexpr uint32_t CCMR2_OC4FE_Msk = ( 0x1UL << CCMR2_OC4FE_Pos );
  static constexpr uint32_t CCMR2_OC4FE     = CCMR2_OC4FE_Msk;
  static constexpr uint32_t CCMR2_OC4PE_Pos = ( 11U );
  static constexpr uint32_t CCMR2_OC4PE_Msk = ( 0x1UL << CCMR2_OC4PE_Pos );
  static constexpr uint32_t CCMR2_OC4PE     = CCMR2_OC4PE_Msk;

  static constexpr uint32_t CCMR2_OC4M_Pos = ( 12U );
  static constexpr uint32_t CCMR2_OC4M_Msk = ( 0x1007UL << CCMR2_OC4M_Pos );
  static constexpr uint32_t CCMR2_OC4M     = CCMR2_OC4M_Msk;
  static constexpr uint32_t CCMR2_OC4M_0   = ( 0x0001UL << CCMR2_OC4M_Pos );
  static constexpr uint32_t CCMR2_OC4M_1   = ( 0x0002UL << CCMR2_OC4M_Pos );
  static constexpr uint32_t CCMR2_OC4M_2   = ( 0x0004UL << CCMR2_OC4M_Pos );
  static constexpr uint32_t CCMR2_OC4M_3   = ( 0x1000UL << CCMR2_OC4M_Pos );

  static constexpr uint32_t CCMR2_OC4CE_Pos = ( 15U );
  static constexpr uint32_t CCMR2_OC4CE_Msk = ( 0x1UL << CCMR2_OC4CE_Pos );
  static constexpr uint32_t CCMR2_OC4CE     = CCMR2_OC4CE_Msk;

  static constexpr uint32_t CCMR2_IC3PSC_Pos = ( 2U );
  static constexpr uint32_t CCMR2_IC3PSC_Msk = ( 0x3UL << CCMR2_IC3PSC_Pos );
  static constexpr uint32_t CCMR2_IC3PSC     = CCMR2_IC3PSC_Msk;
  static constexpr uint32_t CCMR2_IC3PSC_0   = ( 0x1UL << CCMR2_IC3PSC_Pos );
  static constexpr uint32_t CCMR2_IC3PSC_1   = ( 0x2UL << CCMR2_IC3PSC_Pos );

  static constexpr uint32_t CCMR2_IC3F_Pos = ( 4U );
  static constexpr uint32_t CCMR2_IC3F_Msk = ( 0xFUL << CCMR2_IC3F_Pos );
  static constexpr uint32_t CCMR2_IC3F     = CCMR2_IC3F_Msk;
  static constexpr uint32_t CCMR2_IC3F_0   = ( 0x1UL << CCMR2_IC3F_Pos );
  static constexpr uint32_t CCMR2_IC3F_1   = ( 0x2UL << CCMR2_IC3F_Pos );
  static constexpr uint32_t CCMR2_IC3F_2   = ( 0x4UL << CCMR2_IC3F_Pos );
  static constexpr uint32_t CCMR2_IC3F_3   = ( 0x8UL << CCMR2_IC3F_Pos );

  static constexpr uint32_t CCMR2_IC4PSC_Pos = ( 10U );
  static constexpr uint32_t CCMR2_IC4PSC_Msk = ( 0x3UL << CCMR2_IC4PSC_Pos );
  static constexpr uint32_t CCMR2_IC4PSC     = CCMR2_IC4PSC_Msk;
  static constexpr uint32_t CCMR2_IC4PSC_0   = ( 0x1UL << CCMR2_IC4PSC_Pos );
  static constexpr uint32_t CCMR2_IC4PSC_1   = ( 0x2UL << CCMR2_IC4PSC_Pos );

  static constexpr uint32_t CCMR2_IC4F_Pos = ( 12U );
  static constexpr uint32_t CCMR2_IC4F_Msk = ( 0xFUL << CCMR2_IC4F_Pos );
  static constexpr uint32_t CCMR2_IC4F     = CCMR2_IC4F_Msk;
  static constexpr uint32_t CCMR2_IC4F_0   = ( 0x1UL << CCMR2_IC4F_Pos );
  static constexpr uint32_t CCMR2_IC4F_1   = ( 0x2UL << CCMR2_IC4F_Pos );
  static constexpr uint32_t CCMR2_IC4F_2   = ( 0x4UL << CCMR2_IC4F_Pos );
  static constexpr uint32_t CCMR2_IC4F_3   = ( 0x8UL << CCMR2_IC4F_Pos );

  /******************  Bit definition for CCMR3 register  *******************/
  static constexpr uint32_t CCMR3_OC5FE_Pos = ( 2U );
  static constexpr uint32_t CCMR3_OC5FE_Msk = ( 0x1UL << CCMR3_OC5FE_Pos );
  static constexpr uint32_t CCMR3_OC5FE     = CCMR3_OC5FE_Msk;
  static constexpr uint32_t CCMR3_OC5PE_Pos = ( 3U );
  static constexpr uint32_t CCMR3_OC5PE_Msk = ( 0x1UL << CCMR3_OC5PE_Pos );
  static constexpr uint32_t CCMR3_OC5PE     = CCMR3_OC5PE_Msk;

  static constexpr uint32_t CCMR3_OC5M_Pos = ( 4U );
  static constexpr uint32_t CCMR3_OC5M_Msk = ( 0x1007UL << CCMR3_OC5M_Pos );
  static constexpr uint32_t CCMR3_OC5M     = CCMR3_OC5M_Msk;
  static constexpr uint32_t CCMR3_OC5M_0   = ( 0x0001UL << CCMR3_OC5M_Pos );
  static constexpr uint32_t CCMR3_OC5M_1   = ( 0x0002UL << CCMR3_OC5M_Pos );
  static constexpr uint32_t CCMR3_OC5M_2   = ( 0x0004UL << CCMR3_OC5M_Pos );
  static constexpr uint32_t CCMR3_OC5M_3   = ( 0x1000UL << CCMR3_OC5M_Pos );

  static constexpr uint32_t CCMR3_OC5CE_Pos = ( 7U );
  static constexpr uint32_t CCMR3_OC5CE_Msk = ( 0x1UL << CCMR3_OC5CE_Pos );
  static constexpr uint32_t CCMR3_OC5CE     = CCMR3_OC5CE_Msk;

  static constexpr uint32_t CCMR3_OC6FE_Pos = ( 10U );
  static constexpr uint32_t CCMR3_OC6FE_Msk = ( 0x1UL << CCMR3_OC6FE_Pos );
  static constexpr uint32_t CCMR3_OC6FE     = CCMR3_OC6FE_Msk;
  static constexpr uint32_t CCMR3_OC6PE_Pos = ( 11U );
  static constexpr uint32_t CCMR3_OC6PE_Msk = ( 0x1UL << CCMR3_OC6PE_Pos );
  static constexpr uint32_t CCMR3_OC6PE     = CCMR3_OC6PE_Msk;

  static constexpr uint32_t CCMR3_OC6M_Pos = ( 12U );
  static constexpr uint32_t CCMR3_OC6M_Msk = ( 0x1007UL << CCMR3_OC6M_Pos );
  static constexpr uint32_t CCMR3_OC6M     = CCMR3_OC6M_Msk;
  static constexpr uint32_t CCMR3_OC6M_0   = ( 0x0001UL << CCMR3_OC6M_Pos );
  static constexpr uint32_t CCMR3_OC6M_1   = ( 0x0002UL << CCMR3_OC6M_Pos );
  static constexpr uint32_t CCMR3_OC6M_2   = ( 0x0004UL << CCMR3_OC6M_Pos );
  static constexpr uint32_t CCMR3_OC6M_3   = ( 0x1000UL << CCMR3_OC6M_Pos );

  static constexpr uint32_t CCMR3_OC6CE_Pos = ( 15U );
  static constexpr uint32_t CCMR3_OC6CE_Msk = ( 0x1UL << CCMR3_OC6CE_Pos );
  static constexpr uint32_t CCMR3_OC6CE     = CCMR3_OC6CE_Msk;

  /*******************  Bit definition for CCER register  *******************/
  static constexpr uint32_t CCER_ALL = ( 0x0033BFFF );

  static constexpr uint32_t CCER_CC1E_Pos  = ( 0U );
  static constexpr uint32_t CCER_CC1E_Msk  = ( 0x1UL << CCER_CC1E_Pos );
  static constexpr uint32_t CCER_CC1E      = CCER_CC1E_Msk;
  static constexpr uint32_t CCER_CC1P_Pos  = ( 1U );
  static constexpr uint32_t CCER_CC1P_Msk  = ( 0x1UL << CCER_CC1P_Pos );
  static constexpr uint32_t CCER_CC1P      = CCER_CC1P_Msk;
  static constexpr uint32_t CCER_CC1NE_Pos = ( 2U );
  static constexpr uint32_t CCER_CC1NE_Msk = ( 0x1UL << CCER_CC1NE_Pos );
  static constexpr uint32_t CCER_CC1NE     = CCER_CC1NE_Msk;
  static constexpr uint32_t CCER_CC1NP_Pos = ( 3U );
  static constexpr uint32_t CCER_CC1NP_Msk = ( 0x1UL << CCER_CC1NP_Pos );
  static constexpr uint32_t CCER_CC1NP     = CCER_CC1NP_Msk;
  static constexpr uint32_t CCER_CC2E_Pos  = ( 4U );
  static constexpr uint32_t CCER_CC2E_Msk  = ( 0x1UL << CCER_CC2E_Pos );
  static constexpr uint32_t CCER_CC2E      = CCER_CC2E_Msk;
  static constexpr uint32_t CCER_CC2P_Pos  = ( 5U );
  static constexpr uint32_t CCER_CC2P_Msk  = ( 0x1UL << CCER_CC2P_Pos );
  static constexpr uint32_t CCER_CC2P      = CCER_CC2P_Msk;
  static constexpr uint32_t CCER_CC2NE_Pos = ( 6U );
  static constexpr uint32_t CCER_CC2NE_Msk = ( 0x1UL << CCER_CC2NE_Pos );
  static constexpr uint32_t CCER_CC2NE     = CCER_CC2NE_Msk;
  static constexpr uint32_t CCER_CC2NP_Pos = ( 7U );
  static constexpr uint32_t CCER_CC2NP_Msk = ( 0x1UL << CCER_CC2NP_Pos );
  static constexpr uint32_t CCER_CC2NP     = CCER_CC2NP_Msk;
  static constexpr uint32_t CCER_CC3E_Pos  = ( 8U );
  static constexpr uint32_t CCER_CC3E_Msk  = ( 0x1UL << CCER_CC3E_Pos );
  static constexpr uint32_t CCER_CC3E      = CCER_CC3E_Msk;
  static constexpr uint32_t CCER_CC3P_Pos  = ( 9U );
  static constexpr uint32_t CCER_CC3P_Msk  = ( 0x1UL << CCER_CC3P_Pos );
  static constexpr uint32_t CCER_CC3P      = CCER_CC3P_Msk;
  static constexpr uint32_t CCER_CC3NE_Pos = ( 10U );
  static constexpr uint32_t CCER_CC3NE_Msk = ( 0x1UL << CCER_CC3NE_Pos );
  static constexpr uint32_t CCER_CC3NE     = CCER_CC3NE_Msk;
  static constexpr uint32_t CCER_CC3NP_Pos = ( 11U );
  static constexpr uint32_t CCER_CC3NP_Msk = ( 0x1UL << CCER_CC3NP_Pos );
  static constexpr uint32_t CCER_CC3NP     = CCER_CC3NP_Msk;
  static constexpr uint32_t CCER_CC4E_Pos  = ( 12U );
  static constexpr uint32_t CCER_CC4E_Msk  = ( 0x1UL << CCER_CC4E_Pos );
  static constexpr uint32_t CCER_CC4E      = CCER_CC4E_Msk;
  static constexpr uint32_t CCER_CC4P_Pos  = ( 13U );
  static constexpr uint32_t CCER_CC4P_Msk  = ( 0x1UL << CCER_CC4P_Pos );
  static constexpr uint32_t CCER_CC4P      = CCER_CC4P_Msk;
  static constexpr uint32_t CCER_CC4NP_Pos = ( 15U );
  static constexpr uint32_t CCER_CC4NP_Msk = ( 0x1UL << CCER_CC4NP_Pos );
  static constexpr uint32_t CCER_CC4NP     = CCER_CC4NP_Msk;
  static constexpr uint32_t CCER_CC5E_Pos  = ( 16U );
  static constexpr uint32_t CCER_CC5E_Msk  = ( 0x1UL << CCER_CC5E_Pos );
  static constexpr uint32_t CCER_CC5E      = CCER_CC5E_Msk;
  static constexpr uint32_t CCER_CC5P_Pos  = ( 17U );
  static constexpr uint32_t CCER_CC5P_Msk  = ( 0x1UL << CCER_CC5P_Pos );
  static constexpr uint32_t CCER_CC5P      = CCER_CC5P_Msk;
  static constexpr uint32_t CCER_CC6E_Pos  = ( 20U );
  static constexpr uint32_t CCER_CC6E_Msk  = ( 0x1UL << CCER_CC6E_Pos );
  static constexpr uint32_t CCER_CC6E      = CCER_CC6E_Msk;
  static constexpr uint32_t CCER_CC6P_Pos  = ( 21U );
  static constexpr uint32_t CCER_CC6P_Msk  = ( 0x1UL << CCER_CC6P_Pos );
  static constexpr uint32_t CCER_CC6P      = CCER_CC6P_Msk;

  /*******************  Bit definition for CNT register  ********************/
  static constexpr uint32_t CNT_CNT_Pos    = ( 0U );
  static constexpr uint32_t CNT_CNT_Msk    = ( 0xFFFFFFFFUL << CNT_CNT_Pos );
  static constexpr uint32_t CNT_CNT        = CNT_CNT_Msk;
  static constexpr uint32_t CNT_UIFCPY_Pos = ( 31U );
  static constexpr uint32_t CNT_UIFCPY_Msk = ( 0x1UL << CNT_UIFCPY_Pos );
  static constexpr uint32_t CNT_UIFCPY     = CNT_UIFCPY_Msk;

  /*******************  Bit definition for PSC register  ********************/
  static constexpr uint32_t PSC_PSC_Pos = ( 0U );
  static constexpr uint32_t PSC_PSC_Msk = ( 0xFFFFUL << PSC_PSC_Pos );
  static constexpr uint32_t PSC_PSC     = PSC_PSC_Msk;

  /*******************  Bit definition for ARR register  ********************/
  static constexpr uint32_t ARR_ARR_Pos = ( 0U );
  static constexpr uint32_t ARR_ARR_Msk = ( 0xFFFFFFFFUL << ARR_ARR_Pos );
  static constexpr uint32_t ARR_ARR     = ARR_ARR_Msk;

  /*******************  Bit definition for RCR register  ********************/
  static constexpr uint32_t RCR_REP_Pos = ( 0U );
  static constexpr uint32_t RCR_REP_Msk = ( 0xFFFFUL << RCR_REP_Pos );
  static constexpr uint32_t RCR_REP     = RCR_REP_Msk;

  /*******************  Bit definition for CCR1 register  *******************/
  static constexpr uint32_t CCR1_CCR1_Pos = ( 0U );
  static constexpr uint32_t CCR1_CCR1_Msk = ( 0xFFFFUL << CCR1_CCR1_Pos );
  static constexpr uint32_t CCR1_CCR1     = CCR1_CCR1_Msk;

  /*******************  Bit definition for CCR2 register  *******************/
  static constexpr uint32_t CCR2_CCR2_Pos = ( 0U );
  static constexpr uint32_t CCR2_CCR2_Msk = ( 0xFFFFUL << CCR2_CCR2_Pos );
  static constexpr uint32_t CCR2_CCR2     = CCR2_CCR2_Msk;

  /*******************  Bit definition for CCR3 register  *******************/
  static constexpr uint32_t CCR3_CCR3_Pos = ( 0U );
  static constexpr uint32_t CCR3_CCR3_Msk = ( 0xFFFFUL << CCR3_CCR3_Pos );
  static constexpr uint32_t CCR3_CCR3     = CCR3_CCR3_Msk;

  /*******************  Bit definition for CCR4 register  *******************/
  static constexpr uint32_t CCR4_CCR4_Pos = ( 0U );
  static constexpr uint32_t CCR4_CCR4_Msk = ( 0xFFFFUL << CCR4_CCR4_Pos );
  static constexpr uint32_t CCR4_CCR4     = CCR4_CCR4_Msk;

  /*******************  Bit definition for CCR5 register  *******************/
  static constexpr uint32_t CCR5_CCR5_Pos  = ( 0U );
  static constexpr uint32_t CCR5_CCR5_Msk  = ( 0xFFFFFFFFUL << CCR5_CCR5_Pos );
  static constexpr uint32_t CCR5_CCR5      = CCR5_CCR5_Msk;
  static constexpr uint32_t CCR5_GC5C1_Pos = ( 29U );
  static constexpr uint32_t CCR5_GC5C1_Msk = ( 0x1UL << CCR5_GC5C1_Pos );
  static constexpr uint32_t CCR5_GC5C1     = CCR5_GC5C1_Msk;
  static constexpr uint32_t CCR5_GC5C2_Pos = ( 30U );
  static constexpr uint32_t CCR5_GC5C2_Msk = ( 0x1UL << CCR5_GC5C2_Pos );
  static constexpr uint32_t CCR5_GC5C2     = CCR5_GC5C2_Msk;
  static constexpr uint32_t CCR5_GC5C3_Pos = ( 31U );
  static constexpr uint32_t CCR5_GC5C3_Msk = ( 0x1UL << CCR5_GC5C3_Pos );
  static constexpr uint32_t CCR5_GC5C3     = CCR5_GC5C3_Msk;

  /*******************  Bit definition for CCR6 register  *******************/
  static constexpr uint32_t CCR6_CCR6_Pos = ( 0U );
  static constexpr uint32_t CCR6_CCR6_Msk = ( 0xFFFFUL << CCR6_CCR6_Pos );
  static constexpr uint32_t CCR6_CCR6     = CCR6_CCR6_Msk;

  /*******************  Bit definition for BDTR register  *******************/
  static constexpr uint32_t BDTR_DTG_Pos = ( 0U );
  static constexpr uint32_t BDTR_DTG_Msk = ( 0xFFUL << BDTR_DTG_Pos );
  static constexpr uint32_t BDTR_DTG     = BDTR_DTG_Msk;
  static constexpr uint32_t BDTR_DTG_0   = ( 0x01UL << BDTR_DTG_Pos );
  static constexpr uint32_t BDTR_DTG_1   = ( 0x02UL << BDTR_DTG_Pos );
  static constexpr uint32_t BDTR_DTG_2   = ( 0x04UL << BDTR_DTG_Pos );
  static constexpr uint32_t BDTR_DTG_3   = ( 0x08UL << BDTR_DTG_Pos );
  static constexpr uint32_t BDTR_DTG_4   = ( 0x10UL << BDTR_DTG_Pos );
  static constexpr uint32_t BDTR_DTG_5   = ( 0x20UL << BDTR_DTG_Pos );
  static constexpr uint32_t BDTR_DTG_6   = ( 0x40UL << BDTR_DTG_Pos );
  static constexpr uint32_t BDTR_DTG_7   = ( 0x80UL << BDTR_DTG_Pos );

  static constexpr uint32_t BDTR_LOCK_Pos = ( 8U );
  static constexpr uint32_t BDTR_LOCK_Msk = ( 0x3UL << BDTR_LOCK_Pos );
  static constexpr uint32_t BDTR_LOCK     = BDTR_LOCK_Msk;
  static constexpr uint32_t BDTR_LOCK_0   = ( 0x1UL << BDTR_LOCK_Pos );
  static constexpr uint32_t BDTR_LOCK_1   = ( 0x2UL << BDTR_LOCK_Pos );

  static constexpr uint32_t BDTR_OSSI_Pos = ( 10U );
  static constexpr uint32_t BDTR_OSSI_Msk = ( 0x1UL << BDTR_OSSI_Pos );
  static constexpr uint32_t BDTR_OSSI     = BDTR_OSSI_Msk;
  static constexpr uint32_t BDTR_OSSR_Pos = ( 11U );
  static constexpr uint32_t BDTR_OSSR_Msk = ( 0x1UL << BDTR_OSSR_Pos );
  static constexpr uint32_t BDTR_OSSR     = BDTR_OSSR_Msk;
  static constexpr uint32_t BDTR_BKE_Pos  = ( 12U );
  static constexpr uint32_t BDTR_BKE_Msk  = ( 0x1UL << BDTR_BKE_Pos );
  static constexpr uint32_t BDTR_BKE      = BDTR_BKE_Msk;
  static constexpr uint32_t BDTR_BKP_Pos  = ( 13U );
  static constexpr uint32_t BDTR_BKP_Msk  = ( 0x1UL << BDTR_BKP_Pos );
  static constexpr uint32_t BDTR_BKP      = BDTR_BKP_Msk;
  static constexpr uint32_t BDTR_AOE_Pos  = ( 14U );
  static constexpr uint32_t BDTR_AOE_Msk  = ( 0x1UL << BDTR_AOE_Pos );
  static constexpr uint32_t BDTR_AOE      = BDTR_AOE_Msk;
  static constexpr uint32_t BDTR_MOE_Pos  = ( 15U );
  static constexpr uint32_t BDTR_MOE_Msk  = ( 0x1UL << BDTR_MOE_Pos );
  static constexpr uint32_t BDTR_MOE      = BDTR_MOE_Msk;

  static constexpr uint32_t BDTR_BKF_Pos  = ( 16U );
  static constexpr uint32_t BDTR_BKF_Msk  = ( 0xFUL << BDTR_BKF_Pos );
  static constexpr uint32_t BDTR_BKF      = BDTR_BKF_Msk;
  static constexpr uint32_t BDTR_BK2F_Pos = ( 20U );
  static constexpr uint32_t BDTR_BK2F_Msk = ( 0xFUL << BDTR_BK2F_Pos );
  static constexpr uint32_t BDTR_BK2F     = BDTR_BK2F_Msk;

  static constexpr uint32_t BDTR_BK2E_Pos = ( 24U );
  static constexpr uint32_t BDTR_BK2E_Msk = ( 0x1UL << BDTR_BK2E_Pos );
  static constexpr uint32_t BDTR_BK2E     = BDTR_BK2E_Msk;
  static constexpr uint32_t BDTR_BK2P_Pos = ( 25U );
  static constexpr uint32_t BDTR_BK2P_Msk = ( 0x1UL << BDTR_BK2P_Pos );
  static constexpr uint32_t BDTR_BK2P     = BDTR_BK2P_Msk;

  /*******************  Bit definition for DCR register  ********************/
  static constexpr uint32_t DCR_DBA_Pos = ( 0U );
  static constexpr uint32_t DCR_DBA_Msk = ( 0x1FUL << DCR_DBA_Pos );
  static constexpr uint32_t DCR_DBA     = DCR_DBA_Msk;
  static constexpr uint32_t DCR_DBA_0   = ( 0x01UL << DCR_DBA_Pos );
  static constexpr uint32_t DCR_DBA_1   = ( 0x02UL << DCR_DBA_Pos );
  static constexpr uint32_t DCR_DBA_2   = ( 0x04UL << DCR_DBA_Pos );
  static constexpr uint32_t DCR_DBA_3   = ( 0x08UL << DCR_DBA_Pos );
  static constexpr uint32_t DCR_DBA_4   = ( 0x10UL << DCR_DBA_Pos );

  static constexpr uint32_t DCR_DBL_Pos = ( 8U );
  static constexpr uint32_t DCR_DBL_Msk = ( 0x1FUL << DCR_DBL_Pos );
  static constexpr uint32_t DCR_DBL     = DCR_DBL_Msk;
  static constexpr uint32_t DCR_DBL_0   = ( 0x01UL << DCR_DBL_Pos );
  static constexpr uint32_t DCR_DBL_1   = ( 0x02UL << DCR_DBL_Pos );
  static constexpr uint32_t DCR_DBL_2   = ( 0x04UL << DCR_DBL_Pos );
  static constexpr uint32_t DCR_DBL_3   = ( 0x08UL << DCR_DBL_Pos );
  static constexpr uint32_t DCR_DBL_4   = ( 0x10UL << DCR_DBL_Pos );

  /*******************  Bit definition for DMAR register  *******************/
  static constexpr uint32_t DMAR_DMAB_Pos = ( 0U );
  static constexpr uint32_t DMAR_DMAB_Msk = ( 0xFFFFUL << DMAR_DMAB_Pos );
  static constexpr uint32_t DMAR_DMAB     = DMAR_DMAB_Msk;

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

  /******************  Bit definition for LPISR register  *******************/
  static constexpr uint32_t LPISR_CMPM_Pos    = ( 0U );
  static constexpr uint32_t LPISR_CMPM_Msk    = ( 0x1UL << LPISR_CMPM_Pos );
  static constexpr uint32_t LPISR_CMPM        = LPISR_CMPM_Msk;
  static constexpr uint32_t LPISR_ARRM_Pos    = ( 1U );
  static constexpr uint32_t LPISR_ARRM_Msk    = ( 0x1UL << LPISR_ARRM_Pos );
  static constexpr uint32_t LPISR_ARRM        = LPISR_ARRM_Msk;
  static constexpr uint32_t LPISR_EXTTRIG_Pos = ( 2U );
  static constexpr uint32_t LPISR_EXTTRIG_Msk = ( 0x1UL << LPISR_EXTTRIG_Pos );
  static constexpr uint32_t LPISR_EXTTRIG     = LPISR_EXTTRIG_Msk;
  static constexpr uint32_t LPISR_CMPOK_Pos   = ( 3U );
  static constexpr uint32_t LPISR_CMPOK_Msk   = ( 0x1UL << LPISR_CMPOK_Pos );
  static constexpr uint32_t LPISR_CMPOK       = LPISR_CMPOK_Msk;
  static constexpr uint32_t LPISR_ARROK_Pos   = ( 4U );
  static constexpr uint32_t LPISR_ARROK_Msk   = ( 0x1UL << LPISR_ARROK_Pos );
  static constexpr uint32_t LPISR_ARROK       = LPISR_ARROK_Msk;
  static constexpr uint32_t LPISR_UP_Pos      = ( 5U );
  static constexpr uint32_t LPISR_UP_Msk      = ( 0x1UL << LPISR_UP_Pos );
  static constexpr uint32_t LPISR_UP          = LPISR_UP_Msk;
  static constexpr uint32_t LPISR_DOWN_Pos    = ( 6U );
  static constexpr uint32_t LPISR_DOWN_Msk    = ( 0x1UL << LPISR_DOWN_Pos );
  static constexpr uint32_t LPISR_DOWN        = LPISR_DOWN_Msk;

  /******************  Bit definition for LPICR register  *******************/
  static constexpr uint32_t LPICR_CMPMCF_Pos    = ( 0U );
  static constexpr uint32_t LPICR_CMPMCF_Msk    = ( 0x1UL << LPICR_CMPMCF_Pos );
  static constexpr uint32_t LPICR_CMPMCF        = LPICR_CMPMCF_Msk;
  static constexpr uint32_t LPICR_ARRMCF_Pos    = ( 1U );
  static constexpr uint32_t LPICR_ARRMCF_Msk    = ( 0x1UL << LPICR_ARRMCF_Pos );
  static constexpr uint32_t LPICR_ARRMCF        = LPICR_ARRMCF_Msk;
  static constexpr uint32_t LPICR_EXTTRIGCF_Pos = ( 2U );
  static constexpr uint32_t LPICR_EXTTRIGCF_Msk = ( 0x1UL << LPICR_EXTTRIGCF_Pos );
  static constexpr uint32_t LPICR_EXTTRIGCF     = LPICR_EXTTRIGCF_Msk;
  static constexpr uint32_t LPICR_CMPOKCF_Pos   = ( 3U );
  static constexpr uint32_t LPICR_CMPOKCF_Msk   = ( 0x1UL << LPICR_CMPOKCF_Pos );
  static constexpr uint32_t LPICR_CMPOKCF       = LPICR_CMPOKCF_Msk;
  static constexpr uint32_t LPICR_ARROKCF_Pos   = ( 4U );
  static constexpr uint32_t LPICR_ARROKCF_Msk   = ( 0x1UL << LPICR_ARROKCF_Pos );
  static constexpr uint32_t LPICR_ARROKCF       = LPICR_ARROKCF_Msk;
  static constexpr uint32_t LPICR_UPCF_Pos      = ( 5U );
  static constexpr uint32_t LPICR_UPCF_Msk      = ( 0x1UL << LPICR_UPCF_Pos );
  static constexpr uint32_t LPICR_UPCF          = LPICR_UPCF_Msk;
  static constexpr uint32_t LPICR_DOWNCF_Pos    = ( 6U );
  static constexpr uint32_t LPICR_DOWNCF_Msk    = ( 0x1UL << LPICR_DOWNCF_Pos );
  static constexpr uint32_t LPICR_DOWNCF        = LPICR_DOWNCF_Msk;

  /******************  Bit definition for LPIER register ********************/
  static constexpr uint32_t LPIER_CMPMIE_Pos    = ( 0U );
  static constexpr uint32_t LPIER_CMPMIE_Msk    = ( 0x1UL << LPIER_CMPMIE_Pos );
  static constexpr uint32_t LPIER_CMPMIE        = LPIER_CMPMIE_Msk;
  static constexpr uint32_t LPIER_ARRMIE_Pos    = ( 1U );
  static constexpr uint32_t LPIER_ARRMIE_Msk    = ( 0x1UL << LPIER_ARRMIE_Pos );
  static constexpr uint32_t LPIER_ARRMIE        = LPIER_ARRMIE_Msk;
  static constexpr uint32_t LPIER_EXTTRIGIE_Pos = ( 2U );
  static constexpr uint32_t LPIER_EXTTRIGIE_Msk = ( 0x1UL << LPIER_EXTTRIGIE_Pos );
  static constexpr uint32_t LPIER_EXTTRIGIE     = LPIER_EXTTRIGIE_Msk;
  static constexpr uint32_t LPIER_CMPOKIE_Pos   = ( 3U );
  static constexpr uint32_t LPIER_CMPOKIE_Msk   = ( 0x1UL << LPIER_CMPOKIE_Pos );
  static constexpr uint32_t LPIER_CMPOKIE       = LPIER_CMPOKIE_Msk;
  static constexpr uint32_t LPIER_ARROKIE_Pos   = ( 4U );
  static constexpr uint32_t LPIER_ARROKIE_Msk   = ( 0x1UL << LPIER_ARROKIE_Pos );
  static constexpr uint32_t LPIER_ARROKIE       = LPIER_ARROKIE_Msk;
  static constexpr uint32_t LPIER_UPIE_Pos      = ( 5U );
  static constexpr uint32_t LPIER_UPIE_Msk      = ( 0x1UL << LPIER_UPIE_Pos );
  static constexpr uint32_t LPIER_UPIE          = LPIER_UPIE_Msk;
  static constexpr uint32_t LPIER_DOWNIE_Pos    = ( 6U );
  static constexpr uint32_t LPIER_DOWNIE_Msk    = ( 0x1UL << LPIER_DOWNIE_Pos );
  static constexpr uint32_t LPIER_DOWNIE        = LPIER_DOWNIE_Msk;

  /******************  Bit definition for LPCFGR register *******************/
  static constexpr uint32_t LPCFGR_CKSEL_Pos = ( 0U );
  static constexpr uint32_t LPCFGR_CKSEL_Msk = ( 0x1UL << LPCFGR_CKSEL_Pos );
  static constexpr uint32_t LPCFGR_CKSEL     = LPCFGR_CKSEL_Msk;

  static constexpr uint32_t LPCFGR_CKPOL_Pos = ( 1U );
  static constexpr uint32_t LPCFGR_CKPOL_Msk = ( 0x3UL << LPCFGR_CKPOL_Pos );
  static constexpr uint32_t LPCFGR_CKPOL     = LPCFGR_CKPOL_Msk;
  static constexpr uint32_t LPCFGR_CKPOL_0   = ( 0x1UL << LPCFGR_CKPOL_Pos );
  static constexpr uint32_t LPCFGR_CKPOL_1   = ( 0x2UL << LPCFGR_CKPOL_Pos );

  static constexpr uint32_t LPCFGR_CKFLT_Pos = ( 3U );
  static constexpr uint32_t LPCFGR_CKFLT_Msk = ( 0x3UL << LPCFGR_CKFLT_Pos );
  static constexpr uint32_t LPCFGR_CKFLT     = LPCFGR_CKFLT_Msk;
  static constexpr uint32_t LPCFGR_CKFLT_0   = ( 0x1UL << LPCFGR_CKFLT_Pos );
  static constexpr uint32_t LPCFGR_CKFLT_1   = ( 0x2UL << LPCFGR_CKFLT_Pos );

  static constexpr uint32_t LPCFGR_TRGFLT_Pos = ( 6U );
  static constexpr uint32_t LPCFGR_TRGFLT_Msk = ( 0x3UL << LPCFGR_TRGFLT_Pos );
  static constexpr uint32_t LPCFGR_TRGFLT     = LPCFGR_TRGFLT_Msk;
  static constexpr uint32_t LPCFGR_TRGFLT_0   = ( 0x1UL << LPCFGR_TRGFLT_Pos );
  static constexpr uint32_t LPCFGR_TRGFLT_1   = ( 0x2UL << LPCFGR_TRGFLT_Pos );

  static constexpr uint32_t LPCFGR_PRESC_Pos = ( 9U );
  static constexpr uint32_t LPCFGR_PRESC_Msk = ( 0x7UL << LPCFGR_PRESC_Pos );
  static constexpr uint32_t LPCFGR_PRESC     = LPCFGR_PRESC_Msk;
  static constexpr uint32_t LPCFGR_PRESC_0   = ( 0x1UL << LPCFGR_PRESC_Pos );
  static constexpr uint32_t LPCFGR_PRESC_1   = ( 0x2UL << LPCFGR_PRESC_Pos );
  static constexpr uint32_t LPCFGR_PRESC_2   = ( 0x4UL << LPCFGR_PRESC_Pos );

  static constexpr uint32_t LPCFGR_TRIGSEL_Pos = ( 13U );
  static constexpr uint32_t LPCFGR_TRIGSEL_Msk = ( 0x7UL << LPCFGR_TRIGSEL_Pos );
  static constexpr uint32_t LPCFGR_TRIGSEL     = LPCFGR_TRIGSEL_Msk;
  static constexpr uint32_t LPCFGR_TRIGSEL_0   = ( 0x1UL << LPCFGR_TRIGSEL_Pos );
  static constexpr uint32_t LPCFGR_TRIGSEL_1   = ( 0x2UL << LPCFGR_TRIGSEL_Pos );
  static constexpr uint32_t LPCFGR_TRIGSEL_2   = ( 0x4UL << LPCFGR_TRIGSEL_Pos );

  static constexpr uint32_t LPCFGR_TRIGEN_Pos = ( 17U );
  static constexpr uint32_t LPCFGR_TRIGEN_Msk = ( 0x3UL << LPCFGR_TRIGEN_Pos );
  static constexpr uint32_t LPCFGR_TRIGEN     = LPCFGR_TRIGEN_Msk;
  static constexpr uint32_t LPCFGR_TRIGEN_0   = ( 0x1UL << LPCFGR_TRIGEN_Pos );
  static constexpr uint32_t LPCFGR_TRIGEN_1   = ( 0x2UL << LPCFGR_TRIGEN_Pos );

  static constexpr uint32_t LPCFGR_TIMOUT_Pos    = ( 19U );
  static constexpr uint32_t LPCFGR_TIMOUT_Msk    = ( 0x1UL << LPCFGR_TIMOUT_Pos );
  static constexpr uint32_t LPCFGR_TIMOUT        = LPCFGR_TIMOUT_Msk;
  static constexpr uint32_t LPCFGR_WAVE_Pos      = ( 20U );
  static constexpr uint32_t LPCFGR_WAVE_Msk      = ( 0x1UL << LPCFGR_WAVE_Pos );
  static constexpr uint32_t LPCFGR_WAVE          = LPCFGR_WAVE_Msk;
  static constexpr uint32_t LPCFGR_WAVPOL_Pos    = ( 21U );
  static constexpr uint32_t LPCFGR_WAVPOL_Msk    = ( 0x1UL << LPCFGR_WAVPOL_Pos );
  static constexpr uint32_t LPCFGR_WAVPOL        = LPCFGR_WAVPOL_Msk;
  static constexpr uint32_t LPCFGR_PRELOAD_Pos   = ( 22U );
  static constexpr uint32_t LPCFGR_PRELOAD_Msk   = ( 0x1UL << LPCFGR_PRELOAD_Pos );
  static constexpr uint32_t LPCFGR_PRELOAD       = LPCFGR_PRELOAD_Msk;
  static constexpr uint32_t LPCFGR_COUNTMODE_Pos = ( 23U );
  static constexpr uint32_t LPCFGR_COUNTMODE_Msk = ( 0x1UL << LPCFGR_COUNTMODE_Pos );
  static constexpr uint32_t LPCFGR_COUNTMODE     = LPCFGR_COUNTMODE_Msk;
  static constexpr uint32_t LPCFGR_ENC_Pos       = ( 24U );
  static constexpr uint32_t LPCFGR_ENC_Msk       = ( 0x1UL << LPCFGR_ENC_Pos );
  static constexpr uint32_t LPCFGR_ENC           = LPCFGR_ENC_Msk;

  /******************  Bit definition for LPCR register  ********************/
  static constexpr uint32_t LPCR_ENABLE_Pos  = ( 0U );
  static constexpr uint32_t LPCR_ENABLE_Msk  = ( 0x1UL << LPCR_ENABLE_Pos );
  static constexpr uint32_t LPCR_ENABLE      = LPCR_ENABLE_Msk;
  static constexpr uint32_t LPCR_SNGSTRT_Pos = ( 1U );
  static constexpr uint32_t LPCR_SNGSTRT_Msk = ( 0x1UL << LPCR_SNGSTRT_Pos );
  static constexpr uint32_t LPCR_SNGSTRT     = LPCR_SNGSTRT_Msk;
  static constexpr uint32_t LPCR_CNTSTRT_Pos = ( 2U );
  static constexpr uint32_t LPCR_CNTSTRT_Msk = ( 0x1UL << LPCR_CNTSTRT_Pos );
  static constexpr uint32_t LPCR_CNTSTRT     = LPCR_CNTSTRT_Msk;

  /******************  Bit definition for LPCMP register  *******************/
  static constexpr uint32_t LPCMP_CMP_Pos = ( 0U );
  static constexpr uint32_t LPCMP_CMP_Msk = ( 0xFFFFUL << LPCMP_CMP_Pos );
  static constexpr uint32_t LPCMP_CMP     = LPCMP_CMP_Msk;

  /******************  Bit definition for LPARR register  *******************/
  static constexpr uint32_t LPARR_ARR_Pos = ( 0U );
  static constexpr uint32_t LPARR_ARR_Msk = ( 0xFFFFUL << LPARR_ARR_Pos );
  static constexpr uint32_t LPARR_ARR     = LPARR_ARR_Msk;

  /******************  Bit definition for LPCNT register  *******************/
  static constexpr uint32_t LPCNT_CNT_Pos = ( 0U );
  static constexpr uint32_t LPCNT_CNT_Msk = ( 0xFFFFUL << LPCNT_CNT_Pos );
  static constexpr uint32_t LPCNT_CNT     = LPCNT_CNT_Msk;

  /******************  Bit definition for LPOR register  ********************/
  static constexpr uint32_t LPOR_OR_Pos = ( 0U );
  static constexpr uint32_t LPOR_OR_Msk = ( 0x3UL << LPOR_OR_Pos );
  static constexpr uint32_t LPOR_OR     = LPOR_OR_Msk;
  static constexpr uint32_t LPOR_OR_0   = ( 0x1UL << LPOR_OR_Pos );
  static constexpr uint32_t LPOR_OR_1   = ( 0x2UL << LPOR_OR_Pos );

}    // namespace Thor::LLD::TIMER

#endif /* !THOR_LLD_TIMER_INTF_REGISTER_BITS_HPP */
