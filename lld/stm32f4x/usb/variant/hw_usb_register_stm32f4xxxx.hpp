/********************************************************************************
 *  File Name:
 *    hw_usb_register_stm32l4xxxx.hpp
 *
 *  Description:
 *    USB register definitions for the STM32F4xxxx series chips.
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_HW_USB_REGISTER_STM32F4XXXX_HPP
#define THOR_HW_USB_REGISTER_STM32F4XXXX_HPP

/* C++ Includes */
#include <cstdint>

/* Chimera Includes */
#include <Chimera/common>

/* Driver Includes */
#include <Thor/lld/stm32f4x/system/variant/sys_memory_map_stm32f446xx.hpp>


namespace Thor::LLD::USB
{
  /*-------------------------------------------------
  Peripheral Instance Memory Map Base
  -------------------------------------------------*/
  static constexpr uint32_t USB1_OTG_HS_BASE_ADDR = Thor::System::MemoryMap::USB_OTG_HS_START_ADDRESS;
  static constexpr uint32_t USB1_OTG_FS_BASE_ADDR = Thor::System::MemoryMap::USB_OTG_FS_START_ADDRESS;


  /*-------------------------------------------------
  Peripheral Register Definitions
  -------------------------------------------------*/
  /********************  Bit definition for OTG_GOTGCTL register  ***********/
  static constexpr uint32_t OTG_GOTGCTL_SRQSCS_Pos    = ( 0U );
  static constexpr uint32_t OTG_GOTGCTL_SRQSCS_Msk    = ( 0x1UL << OTG_GOTGCTL_SRQSCS_Pos );
  static constexpr uint32_t OTG_GOTGCTL_SRQSCS        = OTG_GOTGCTL_SRQSCS_Msk;
  static constexpr uint32_t OTG_GOTGCTL_SRQ_Pos       = ( 1U );
  static constexpr uint32_t OTG_GOTGCTL_SRQ_Msk       = ( 0x1UL << OTG_GOTGCTL_SRQ_Pos );
  static constexpr uint32_t OTG_GOTGCTL_SRQ           = OTG_GOTGCTL_SRQ_Msk;
  static constexpr uint32_t OTG_GOTGCTL_VBVALOEN_Pos  = ( 2U );
  static constexpr uint32_t OTG_GOTGCTL_VBVALOEN_Msk  = ( 0x1UL << OTG_GOTGCTL_VBVALOEN_Pos );
  static constexpr uint32_t OTG_GOTGCTL_VBVALOEN      = OTG_GOTGCTL_VBVALOEN_Msk;
  static constexpr uint32_t OTG_GOTGCTL_VBVALOVAL_Pos = ( 3U );
  static constexpr uint32_t OTG_GOTGCTL_VBVALOVAL_Msk = ( 0x1UL << OTG_GOTGCTL_VBVALOVAL_Pos );
  static constexpr uint32_t OTG_GOTGCTL_VBVALOVAL     = OTG_GOTGCTL_VBVALOVAL_Msk;
  static constexpr uint32_t OTG_GOTGCTL_AVALOEN_Pos   = ( 4U );
  static constexpr uint32_t OTG_GOTGCTL_AVALOEN_Msk   = ( 0x1UL << OTG_GOTGCTL_AVALOEN_Pos );
  static constexpr uint32_t OTG_GOTGCTL_AVALOEN       = OTG_GOTGCTL_AVALOEN_Msk;
  static constexpr uint32_t OTG_GOTGCTL_AVALOVAL_Pos  = ( 5U );
  static constexpr uint32_t OTG_GOTGCTL_AVALOVAL_Msk  = ( 0x1UL << OTG_GOTGCTL_AVALOVAL_Pos );
  static constexpr uint32_t OTG_GOTGCTL_AVALOVAL      = OTG_GOTGCTL_AVALOVAL_Msk;
  static constexpr uint32_t OTG_GOTGCTL_BVALOEN_Pos   = ( 6U );
  static constexpr uint32_t OTG_GOTGCTL_BVALOEN_Msk   = ( 0x1UL << OTG_GOTGCTL_BVALOEN_Pos );
  static constexpr uint32_t OTG_GOTGCTL_BVALOEN       = OTG_GOTGCTL_BVALOEN_Msk;
  static constexpr uint32_t OTG_GOTGCTL_BVALOVAL_Pos  = ( 7U );
  static constexpr uint32_t OTG_GOTGCTL_BVALOVAL_Msk  = ( 0x1UL << OTG_GOTGCTL_BVALOVAL_Pos );
  static constexpr uint32_t OTG_GOTGCTL_BVALOVAL      = OTG_GOTGCTL_BVALOVAL_Msk;
  static constexpr uint32_t OTG_GOTGCTL_HNGSCS_Pos    = ( 8U );
  static constexpr uint32_t OTG_GOTGCTL_HNGSCS_Msk    = ( 0x1UL << OTG_GOTGCTL_HNGSCS_Pos );
  static constexpr uint32_t OTG_GOTGCTL_HNGSCS        = OTG_GOTGCTL_HNGSCS_Msk;
  static constexpr uint32_t OTG_GOTGCTL_HNPRQ_Pos     = ( 9U );
  static constexpr uint32_t OTG_GOTGCTL_HNPRQ_Msk     = ( 0x1UL << OTG_GOTGCTL_HNPRQ_Pos );
  static constexpr uint32_t OTG_GOTGCTL_HNPRQ         = OTG_GOTGCTL_HNPRQ_Msk;
  static constexpr uint32_t OTG_GOTGCTL_HSHNPEN_Pos   = ( 10U );
  static constexpr uint32_t OTG_GOTGCTL_HSHNPEN_Msk   = ( 0x1UL << OTG_GOTGCTL_HSHNPEN_Pos );
  static constexpr uint32_t OTG_GOTGCTL_HSHNPEN       = OTG_GOTGCTL_HSHNPEN_Msk;
  static constexpr uint32_t OTG_GOTGCTL_DHNPEN_Pos    = ( 11U );
  static constexpr uint32_t OTG_GOTGCTL_DHNPEN_Msk    = ( 0x1UL << OTG_GOTGCTL_DHNPEN_Pos );
  static constexpr uint32_t OTG_GOTGCTL_DHNPEN        = OTG_GOTGCTL_DHNPEN_Msk;
  static constexpr uint32_t OTG_GOTGCTL_EHEN_Pos      = ( 12U );
  static constexpr uint32_t OTG_GOTGCTL_EHEN_Msk      = ( 0x1UL << OTG_GOTGCTL_EHEN_Pos );
  static constexpr uint32_t OTG_GOTGCTL_EHEN          = OTG_GOTGCTL_EHEN_Msk;
  static constexpr uint32_t OTG_GOTGCTL_CIDSTS_Pos    = ( 16U );
  static constexpr uint32_t OTG_GOTGCTL_CIDSTS_Msk    = ( 0x1UL << OTG_GOTGCTL_CIDSTS_Pos );
  static constexpr uint32_t OTG_GOTGCTL_CIDSTS        = OTG_GOTGCTL_CIDSTS_Msk;
  static constexpr uint32_t OTG_GOTGCTL_DBCT_Pos      = ( 17U );
  static constexpr uint32_t OTG_GOTGCTL_DBCT_Msk      = ( 0x1UL << OTG_GOTGCTL_DBCT_Pos );
  static constexpr uint32_t OTG_GOTGCTL_DBCT          = OTG_GOTGCTL_DBCT_Msk;
  static constexpr uint32_t OTG_GOTGCTL_ASVLD_Pos     = ( 18U );
  static constexpr uint32_t OTG_GOTGCTL_ASVLD_Msk     = ( 0x1UL << OTG_GOTGCTL_ASVLD_Pos );
  static constexpr uint32_t OTG_GOTGCTL_ASVLD         = OTG_GOTGCTL_ASVLD_Msk;
  static constexpr uint32_t OTG_GOTGCTL_BSESVLD_Pos   = ( 19U );
  static constexpr uint32_t OTG_GOTGCTL_BSESVLD_Msk   = ( 0x1UL << OTG_GOTGCTL_BSESVLD_Pos );
  static constexpr uint32_t OTG_GOTGCTL_BSESVLD       = OTG_GOTGCTL_BSESVLD_Msk;
  static constexpr uint32_t OTG_GOTGCTL_OTGVER_Pos    = ( 20U );
  static constexpr uint32_t OTG_GOTGCTL_OTGVER_Msk    = ( 0x1UL << OTG_GOTGCTL_OTGVER_Pos );
  static constexpr uint32_t OTG_GOTGCTL_OTGVER        = OTG_GOTGCTL_OTGVER_Msk;

  /********************  Bit definition forOTG_HCFG register  ********************/

  static constexpr uint32_t OTG_HCFG_FSLSPCS_Pos = ( 0U );
  static constexpr uint32_t OTG_HCFG_FSLSPCS_Msk = ( 0x3UL << OTG_HCFG_FSLSPCS_Pos );
  static constexpr uint32_t OTG_HCFG_FSLSPCS     = OTG_HCFG_FSLSPCS_Msk;
  static constexpr uint32_t OTG_HCFG_FSLSPCS_0   = ( 0x1UL << OTG_HCFG_FSLSPCS_Pos );
  static constexpr uint32_t OTG_HCFG_FSLSPCS_1   = ( 0x2UL << OTG_HCFG_FSLSPCS_Pos );
  static constexpr uint32_t OTG_HCFG_FSLSS_Pos   = ( 2U );
  static constexpr uint32_t OTG_HCFG_FSLSS_Msk   = ( 0x1UL << OTG_HCFG_FSLSS_Pos );
  static constexpr uint32_t OTG_HCFG_FSLSS       = OTG_HCFG_FSLSS_Msk;

  /********************  Bit definition for OTG_DCFG register  ********************/

  static constexpr uint32_t OTG_DCFG_DSPD_Pos     = ( 0U );
  static constexpr uint32_t OTG_DCFG_DSPD_Msk     = ( 0x3UL << OTG_DCFG_DSPD_Pos );
  static constexpr uint32_t OTG_DCFG_DSPD         = OTG_DCFG_DSPD_Msk;
  static constexpr uint32_t OTG_DCFG_DSPD_0       = ( 0x1UL << OTG_DCFG_DSPD_Pos );
  static constexpr uint32_t OTG_DCFG_DSPD_1       = ( 0x2UL << OTG_DCFG_DSPD_Pos );
  static constexpr uint32_t OTG_DCFG_NZLSOHSK_Pos = ( 2U );
  static constexpr uint32_t OTG_DCFG_NZLSOHSK_Msk = ( 0x1UL << OTG_DCFG_NZLSOHSK_Pos );
  static constexpr uint32_t OTG_DCFG_NZLSOHSK     = OTG_DCFG_NZLSOHSK_Msk;

  static constexpr uint32_t OTG_DCFG_DAD_Pos = ( 4U );
  static constexpr uint32_t OTG_DCFG_DAD_Msk = ( 0x7FUL << OTG_DCFG_DAD_Pos );
  static constexpr uint32_t OTG_DCFG_DAD     = OTG_DCFG_DAD_Msk;
  static constexpr uint32_t OTG_DCFG_DAD_0   = ( 0x01UL << OTG_DCFG_DAD_Pos );
  static constexpr uint32_t OTG_DCFG_DAD_1   = ( 0x02UL << OTG_DCFG_DAD_Pos );
  static constexpr uint32_t OTG_DCFG_DAD_2   = ( 0x04UL << OTG_DCFG_DAD_Pos );
  static constexpr uint32_t OTG_DCFG_DAD_3   = ( 0x08UL << OTG_DCFG_DAD_Pos );
  static constexpr uint32_t OTG_DCFG_DAD_4   = ( 0x10UL << OTG_DCFG_DAD_Pos );
  static constexpr uint32_t OTG_DCFG_DAD_5   = ( 0x20UL << OTG_DCFG_DAD_Pos );
  static constexpr uint32_t OTG_DCFG_DAD_6   = ( 0x40UL << OTG_DCFG_DAD_Pos );

  static constexpr uint32_t OTG_DCFG_PFIVL_Pos = ( 11U );
  static constexpr uint32_t OTG_DCFG_PFIVL_Msk = ( 0x3UL << OTG_DCFG_PFIVL_Pos );
  static constexpr uint32_t OTG_DCFG_PFIVL     = OTG_DCFG_PFIVL_Msk;
  static constexpr uint32_t OTG_DCFG_PFIVL_0   = ( 0x1UL << OTG_DCFG_PFIVL_Pos );
  static constexpr uint32_t OTG_DCFG_PFIVL_1   = ( 0x2UL << OTG_DCFG_PFIVL_Pos );

  static constexpr uint32_t OTG_DCFG_XCVRDLY_Pos = ( 14U );
  static constexpr uint32_t OTG_DCFG_XCVRDLY_Msk = ( 0x1UL << OTG_DCFG_XCVRDLY_Pos );
  static constexpr uint32_t OTG_DCFG_XCVRDLY     = OTG_DCFG_XCVRDLY_Msk;

  static constexpr uint32_t OTG_DCFG_ERRATIM_Pos = ( 15U );
  static constexpr uint32_t OTG_DCFG_ERRATIM_Msk = ( 0x1UL << OTG_DCFG_ERRATIM_Pos );
  static constexpr uint32_t OTG_DCFG_ERRATIM     = OTG_DCFG_ERRATIM_Msk;

  static constexpr uint32_t OTG_DCFG_PERSCHIVL_Pos = ( 24U );
  static constexpr uint32_t OTG_DCFG_PERSCHIVL_Msk = ( 0x3UL << OTG_DCFG_PERSCHIVL_Pos );
  static constexpr uint32_t OTG_DCFG_PERSCHIVL     = OTG_DCFG_PERSCHIVL_Msk;
  static constexpr uint32_t OTG_DCFG_PERSCHIVL_0   = ( 0x1UL << OTG_DCFG_PERSCHIVL_Pos );
  static constexpr uint32_t OTG_DCFG_PERSCHIVL_1   = ( 0x2UL << OTG_DCFG_PERSCHIVL_Pos );

  /********************  Bit definition for OTG_PCGCR register  ********************/
  static constexpr uint32_t OTG_PCGCR_STPPCLK_Pos  = ( 0U );
  static constexpr uint32_t OTG_PCGCR_STPPCLK_Msk  = ( 0x1UL << OTG_PCGCR_STPPCLK_Pos );
  static constexpr uint32_t OTG_PCGCR_STPPCLK      = OTG_PCGCR_STPPCLK_Msk;
  static constexpr uint32_t OTG_PCGCR_GATEHCLK_Pos = ( 1U );
  static constexpr uint32_t OTG_PCGCR_GATEHCLK_Msk = ( 0x1UL << OTG_PCGCR_GATEHCLK_Pos );
  static constexpr uint32_t OTG_PCGCR_GATEHCLK     = OTG_PCGCR_GATEHCLK_Msk;
  static constexpr uint32_t OTG_PCGCR_PHYSUSP_Pos  = ( 4U );
  static constexpr uint32_t OTG_PCGCR_PHYSUSP_Msk  = ( 0x1UL << OTG_PCGCR_PHYSUSP_Pos );
  static constexpr uint32_t OTG_PCGCR_PHYSUSP      = OTG_PCGCR_PHYSUSP_Msk;

  /********************  Bit definition for OTG_GOTGINT register  ********************/
  static constexpr uint32_t OTG_GOTGINT_SEDET_Pos   = ( 2U );
  static constexpr uint32_t OTG_GOTGINT_SEDET_Msk   = ( 0x1UL << OTG_GOTGINT_SEDET_Pos );
  static constexpr uint32_t OTG_GOTGINT_SEDET       = OTG_GOTGINT_SEDET_Msk;
  static constexpr uint32_t OTG_GOTGINT_SRSSCHG_Pos = ( 8U );
  static constexpr uint32_t OTG_GOTGINT_SRSSCHG_Msk = ( 0x1UL << OTG_GOTGINT_SRSSCHG_Pos );
  static constexpr uint32_t OTG_GOTGINT_SRSSCHG     = OTG_GOTGINT_SRSSCHG_Msk;
  static constexpr uint32_t OTG_GOTGINT_HNSSCHG_Pos = ( 9U );
  static constexpr uint32_t OTG_GOTGINT_HNSSCHG_Msk = ( 0x1UL << OTG_GOTGINT_HNSSCHG_Pos );
  static constexpr uint32_t OTG_GOTGINT_HNSSCHG     = OTG_GOTGINT_HNSSCHG_Msk;
  static constexpr uint32_t OTG_GOTGINT_HNGDET_Pos  = ( 17U );
  static constexpr uint32_t OTG_GOTGINT_HNGDET_Msk  = ( 0x1UL << OTG_GOTGINT_HNGDET_Pos );
  static constexpr uint32_t OTG_GOTGINT_HNGDET      = OTG_GOTGINT_HNGDET_Msk;
  static constexpr uint32_t OTG_GOTGINT_ADTOCHG_Pos = ( 18U );
  static constexpr uint32_t OTG_GOTGINT_ADTOCHG_Msk = ( 0x1UL << OTG_GOTGINT_ADTOCHG_Pos );
  static constexpr uint32_t OTG_GOTGINT_ADTOCHG     = OTG_GOTGINT_ADTOCHG_Msk;
  static constexpr uint32_t OTG_GOTGINT_DBCDNE_Pos  = ( 19U );
  static constexpr uint32_t OTG_GOTGINT_DBCDNE_Msk  = ( 0x1UL << OTG_GOTGINT_DBCDNE_Pos );
  static constexpr uint32_t OTG_GOTGINT_DBCDNE      = OTG_GOTGINT_DBCDNE_Msk;
  static constexpr uint32_t OTG_GOTGINT_IDCHNG_Pos  = ( 20U );
  static constexpr uint32_t OTG_GOTGINT_IDCHNG_Msk  = ( 0x1UL << OTG_GOTGINT_IDCHNG_Pos );
  static constexpr uint32_t OTG_GOTGINT_IDCHNG      = OTG_GOTGINT_IDCHNG_Msk;

  /********************  Bit definition for OTG_DCTL register  ********************/
  static constexpr uint32_t OTG_DCTL_RWUSIG_Pos = ( 0U );
  static constexpr uint32_t OTG_DCTL_RWUSIG_Msk = ( 0x1UL << OTG_DCTL_RWUSIG_Pos );
  static constexpr uint32_t OTG_DCTL_RWUSIG     = OTG_DCTL_RWUSIG_Msk;
  static constexpr uint32_t OTG_DCTL_SDIS_Pos   = ( 1U );
  static constexpr uint32_t OTG_DCTL_SDIS_Msk   = ( 0x1UL << OTG_DCTL_SDIS_Pos );
  static constexpr uint32_t OTG_DCTL_SDIS       = OTG_DCTL_SDIS_Msk;
  static constexpr uint32_t OTG_DCTL_GINSTS_Pos = ( 2U );
  static constexpr uint32_t OTG_DCTL_GINSTS_Msk = ( 0x1UL << OTG_DCTL_GINSTS_Pos );
  static constexpr uint32_t OTG_DCTL_GINSTS     = OTG_DCTL_GINSTS_Msk;
  static constexpr uint32_t OTG_DCTL_GONSTS_Pos = ( 3U );
  static constexpr uint32_t OTG_DCTL_GONSTS_Msk = ( 0x1UL << OTG_DCTL_GONSTS_Pos );
  static constexpr uint32_t OTG_DCTL_GONSTS     = OTG_DCTL_GONSTS_Msk;

  static constexpr uint32_t OTG_DCTL_TCTL_Pos     = ( 4U );
  static constexpr uint32_t OTG_DCTL_TCTL_Msk     = ( 0x7UL << OTG_DCTL_TCTL_Pos );
  static constexpr uint32_t OTG_DCTL_TCTL         = OTG_DCTL_TCTL_Msk;
  static constexpr uint32_t OTG_DCTL_TCTL_0       = ( 0x1UL << OTG_DCTL_TCTL_Pos );
  static constexpr uint32_t OTG_DCTL_TCTL_1       = ( 0x2UL << OTG_DCTL_TCTL_Pos );
  static constexpr uint32_t OTG_DCTL_TCTL_2       = ( 0x4UL << OTG_DCTL_TCTL_Pos );
  static constexpr uint32_t OTG_DCTL_SGINAK_Pos   = ( 7U );
  static constexpr uint32_t OTG_DCTL_SGINAK_Msk   = ( 0x1UL << OTG_DCTL_SGINAK_Pos );
  static constexpr uint32_t OTG_DCTL_SGINAK       = OTG_DCTL_SGINAK_Msk;
  static constexpr uint32_t OTG_DCTL_CGINAK_Pos   = ( 8U );
  static constexpr uint32_t OTG_DCTL_CGINAK_Msk   = ( 0x1UL << OTG_DCTL_CGINAK_Pos );
  static constexpr uint32_t OTG_DCTL_CGINAK       = OTG_DCTL_CGINAK_Msk;
  static constexpr uint32_t OTG_DCTL_SGONAK_Pos   = ( 9U );
  static constexpr uint32_t OTG_DCTL_SGONAK_Msk   = ( 0x1UL << OTG_DCTL_SGONAK_Pos );
  static constexpr uint32_t OTG_DCTL_SGONAK       = OTG_DCTL_SGONAK_Msk;
  static constexpr uint32_t OTG_DCTL_CGONAK_Pos   = ( 10U );
  static constexpr uint32_t OTG_DCTL_CGONAK_Msk   = ( 0x1UL << OTG_DCTL_CGONAK_Pos );
  static constexpr uint32_t OTG_DCTL_CGONAK       = OTG_DCTL_CGONAK_Msk;
  static constexpr uint32_t OTG_DCTL_POPRGDNE_Pos = ( 11U );
  static constexpr uint32_t OTG_DCTL_POPRGDNE_Msk = ( 0x1UL << OTG_DCTL_POPRGDNE_Pos );
  static constexpr uint32_t OTG_DCTL_POPRGDNE     = OTG_DCTL_POPRGDNE_Msk;

  /********************  Bit definition for OTG_HFIR register  ********************/
  static constexpr uint32_t OTG_HFIR_FRIVL_Pos = ( 0U );
  static constexpr uint32_t OTG_HFIR_FRIVL_Msk = ( 0xFFFFUL << OTG_HFIR_FRIVL_Pos );
  static constexpr uint32_t OTG_HFIR_FRIVL     = OTG_HFIR_FRIVL_Msk;

  /********************  Bit definition for OTG_HFNUM register  ********************/
  static constexpr uint32_t OTG_HFNUM_FRNUM_Pos = ( 0U );
  static constexpr uint32_t OTG_HFNUM_FRNUM_Msk = ( 0xFFFFUL << OTG_HFNUM_FRNUM_Pos );
  static constexpr uint32_t OTG_HFNUM_FRNUM     = OTG_HFNUM_FRNUM_Msk;
  static constexpr uint32_t OTG_HFNUM_FTREM_Pos = ( 16U );
  static constexpr uint32_t OTG_HFNUM_FTREM_Msk = ( 0xFFFFUL << OTG_HFNUM_FTREM_Pos );
  static constexpr uint32_t OTG_HFNUM_FTREM     = OTG_HFNUM_FTREM_Msk;

  /********************  Bit definition for OTG_DSTS register  ********************/
  static constexpr uint32_t OTG_DSTS_SUSPSTS_Pos = ( 0U );
  static constexpr uint32_t OTG_DSTS_SUSPSTS_Msk = ( 0x1UL << OTG_DSTS_SUSPSTS_Pos );
  static constexpr uint32_t OTG_DSTS_SUSPSTS     = OTG_DSTS_SUSPSTS_Msk;

  static constexpr uint32_t OTG_DSTS_ENUMSPD_Pos = ( 1U );
  static constexpr uint32_t OTG_DSTS_ENUMSPD_Msk = ( 0x3UL << OTG_DSTS_ENUMSPD_Pos );
  static constexpr uint32_t OTG_DSTS_ENUMSPD     = OTG_DSTS_ENUMSPD_Msk;
  static constexpr uint32_t OTG_DSTS_ENUMSPD_0   = ( 0x1UL << OTG_DSTS_ENUMSPD_Pos );
  static constexpr uint32_t OTG_DSTS_ENUMSPD_1   = ( 0x2UL << OTG_DSTS_ENUMSPD_Pos );
  static constexpr uint32_t OTG_DSTS_EERR_Pos    = ( 3U );
  static constexpr uint32_t OTG_DSTS_EERR_Msk    = ( 0x1UL << OTG_DSTS_EERR_Pos );
  static constexpr uint32_t OTG_DSTS_EERR        = OTG_DSTS_EERR_Msk;
  static constexpr uint32_t OTG_DSTS_FNSOF_Pos   = ( 8U );
  static constexpr uint32_t OTG_DSTS_FNSOF_Msk   = ( 0x3FFFUL << OTG_DSTS_FNSOF_Pos );
  static constexpr uint32_t OTG_DSTS_FNSOF       = OTG_DSTS_FNSOF_Msk;

  /********************  Bit definition for OTG_GAHBCFG register  ********************/
  static constexpr uint32_t OTG_GAHBCFG_GINT_Pos     = ( 0U );
  static constexpr uint32_t OTG_GAHBCFG_GINT_Msk     = ( 0x1UL << OTG_GAHBCFG_GINT_Pos );
  static constexpr uint32_t OTG_GAHBCFG_GINT         = OTG_GAHBCFG_GINT_Msk;
  static constexpr uint32_t OTG_GAHBCFG_HBSTLEN_Pos  = ( 1U );
  static constexpr uint32_t OTG_GAHBCFG_HBSTLEN_Msk  = ( 0xFUL << OTG_GAHBCFG_HBSTLEN_Pos );
  static constexpr uint32_t OTG_GAHBCFG_HBSTLEN      = OTG_GAHBCFG_HBSTLEN_Msk;
  static constexpr uint32_t OTG_GAHBCFG_HBSTLEN_0    = ( 0x0UL << OTG_GAHBCFG_HBSTLEN_Pos );
  static constexpr uint32_t OTG_GAHBCFG_HBSTLEN_1    = ( 0x1UL << OTG_GAHBCFG_HBSTLEN_Pos );
  static constexpr uint32_t OTG_GAHBCFG_HBSTLEN_2    = ( 0x3UL << OTG_GAHBCFG_HBSTLEN_Pos );
  static constexpr uint32_t OTG_GAHBCFG_HBSTLEN_3    = ( 0x5UL << OTG_GAHBCFG_HBSTLEN_Pos );
  static constexpr uint32_t OTG_GAHBCFG_HBSTLEN_4    = ( 0x7UL << OTG_GAHBCFG_HBSTLEN_Pos );
  static constexpr uint32_t OTG_GAHBCFG_DMAEN_Pos    = ( 5U );
  static constexpr uint32_t OTG_GAHBCFG_DMAEN_Msk    = ( 0x1UL << OTG_GAHBCFG_DMAEN_Pos );
  static constexpr uint32_t OTG_GAHBCFG_DMAEN        = OTG_GAHBCFG_DMAEN_Msk;
  static constexpr uint32_t OTG_GAHBCFG_TXFELVL_Pos  = ( 7U );
  static constexpr uint32_t OTG_GAHBCFG_TXFELVL_Msk  = ( 0x1UL << OTG_GAHBCFG_TXFELVL_Pos );
  static constexpr uint32_t OTG_GAHBCFG_TXFELVL      = OTG_GAHBCFG_TXFELVL_Msk;
  static constexpr uint32_t OTG_GAHBCFG_PTXFELVL_Pos = ( 8U );
  static constexpr uint32_t OTG_GAHBCFG_PTXFELVL_Msk = ( 0x1UL << OTG_GAHBCFG_PTXFELVL_Pos );
  static constexpr uint32_t OTG_GAHBCFG_PTXFELVL     = OTG_GAHBCFG_PTXFELVL_Msk;

  /********************  Bit definition for OTG_GUSBCFG register  ********************/

  static constexpr uint32_t OTG_GUSBCFG_TOCAL_Pos      = ( 0U );
  static constexpr uint32_t OTG_GUSBCFG_TOCAL_Msk      = ( 0x7UL << OTG_GUSBCFG_TOCAL_Pos );
  static constexpr uint32_t OTG_GUSBCFG_TOCAL          = OTG_GUSBCFG_TOCAL_Msk;
  static constexpr uint32_t OTG_GUSBCFG_TOCAL_0        = ( 0x1UL << OTG_GUSBCFG_TOCAL_Pos );
  static constexpr uint32_t OTG_GUSBCFG_TOCAL_1        = ( 0x2UL << OTG_GUSBCFG_TOCAL_Pos );
  static constexpr uint32_t OTG_GUSBCFG_TOCAL_2        = ( 0x4UL << OTG_GUSBCFG_TOCAL_Pos );
  static constexpr uint32_t OTG_GUSBCFG_PHYSEL_Pos     = ( 6U );
  static constexpr uint32_t OTG_GUSBCFG_PHYSEL_Msk     = ( 0x1UL << OTG_GUSBCFG_PHYSEL_Pos );
  static constexpr uint32_t OTG_GUSBCFG_PHYSEL         = OTG_GUSBCFG_PHYSEL_Msk;
  static constexpr uint32_t OTG_GUSBCFG_SRPCAP_Pos     = ( 8U );
  static constexpr uint32_t OTG_GUSBCFG_SRPCAP_Msk     = ( 0x1UL << OTG_GUSBCFG_SRPCAP_Pos );
  static constexpr uint32_t OTG_GUSBCFG_SRPCAP         = OTG_GUSBCFG_SRPCAP_Msk;
  static constexpr uint32_t OTG_GUSBCFG_HNPCAP_Pos     = ( 9U );
  static constexpr uint32_t OTG_GUSBCFG_HNPCAP_Msk     = ( 0x1UL << OTG_GUSBCFG_HNPCAP_Pos );
  static constexpr uint32_t OTG_GUSBCFG_HNPCAP         = OTG_GUSBCFG_HNPCAP_Msk;
  static constexpr uint32_t OTG_GUSBCFG_TRDT_Pos       = ( 10U );
  static constexpr uint32_t OTG_GUSBCFG_TRDT_Msk       = ( 0xFUL << OTG_GUSBCFG_TRDT_Pos );
  static constexpr uint32_t OTG_GUSBCFG_TRDT           = OTG_GUSBCFG_TRDT_Msk;
  static constexpr uint32_t OTG_GUSBCFG_TRDT_0         = ( 0x1UL << OTG_GUSBCFG_TRDT_Pos );
  static constexpr uint32_t OTG_GUSBCFG_TRDT_1         = ( 0x2UL << OTG_GUSBCFG_TRDT_Pos );
  static constexpr uint32_t OTG_GUSBCFG_TRDT_2         = ( 0x4UL << OTG_GUSBCFG_TRDT_Pos );
  static constexpr uint32_t OTG_GUSBCFG_TRDT_3         = ( 0x8UL << OTG_GUSBCFG_TRDT_Pos );
  static constexpr uint32_t OTG_GUSBCFG_PHYLPCS_Pos    = ( 15U );
  static constexpr uint32_t OTG_GUSBCFG_PHYLPCS_Msk    = ( 0x1UL << OTG_GUSBCFG_PHYLPCS_Pos );
  static constexpr uint32_t OTG_GUSBCFG_PHYLPCS        = OTG_GUSBCFG_PHYLPCS_Msk;
  static constexpr uint32_t OTG_GUSBCFG_ULPIFSLS_Pos   = ( 17U );
  static constexpr uint32_t OTG_GUSBCFG_ULPIFSLS_Msk   = ( 0x1UL << OTG_GUSBCFG_ULPIFSLS_Pos );
  static constexpr uint32_t OTG_GUSBCFG_ULPIFSLS       = OTG_GUSBCFG_ULPIFSLS_Msk;
  static constexpr uint32_t OTG_GUSBCFG_ULPIAR_Pos     = ( 18U );
  static constexpr uint32_t OTG_GUSBCFG_ULPIAR_Msk     = ( 0x1UL << OTG_GUSBCFG_ULPIAR_Pos );
  static constexpr uint32_t OTG_GUSBCFG_ULPIAR         = OTG_GUSBCFG_ULPIAR_Msk;
  static constexpr uint32_t OTG_GUSBCFG_ULPICSM_Pos    = ( 19U );
  static constexpr uint32_t OTG_GUSBCFG_ULPICSM_Msk    = ( 0x1UL << OTG_GUSBCFG_ULPICSM_Pos );
  static constexpr uint32_t OTG_GUSBCFG_ULPICSM        = OTG_GUSBCFG_ULPICSM_Msk;
  static constexpr uint32_t OTG_GUSBCFG_ULPIEVBUSD_Pos = ( 20U );
  static constexpr uint32_t OTG_GUSBCFG_ULPIEVBUSD_Msk = ( 0x1UL << OTG_GUSBCFG_ULPIEVBUSD_Pos );
  static constexpr uint32_t OTG_GUSBCFG_ULPIEVBUSD     = OTG_GUSBCFG_ULPIEVBUSD_Msk;
  static constexpr uint32_t OTG_GUSBCFG_ULPIEVBUSI_Pos = ( 21U );
  static constexpr uint32_t OTG_GUSBCFG_ULPIEVBUSI_Msk = ( 0x1UL << OTG_GUSBCFG_ULPIEVBUSI_Pos );
  static constexpr uint32_t OTG_GUSBCFG_ULPIEVBUSI     = OTG_GUSBCFG_ULPIEVBUSI_Msk;
  static constexpr uint32_t OTG_GUSBCFG_TSDPS_Pos      = ( 22U );
  static constexpr uint32_t OTG_GUSBCFG_TSDPS_Msk      = ( 0x1UL << OTG_GUSBCFG_TSDPS_Pos );
  static constexpr uint32_t OTG_GUSBCFG_TSDPS          = OTG_GUSBCFG_TSDPS_Msk;
  static constexpr uint32_t OTG_GUSBCFG_PCCI_Pos       = ( 23U );
  static constexpr uint32_t OTG_GUSBCFG_PCCI_Msk       = ( 0x1UL << OTG_GUSBCFG_PCCI_Pos );
  static constexpr uint32_t OTG_GUSBCFG_PCCI           = OTG_GUSBCFG_PCCI_Msk;
  static constexpr uint32_t OTG_GUSBCFG_PTCI_Pos       = ( 24U );
  static constexpr uint32_t OTG_GUSBCFG_PTCI_Msk       = ( 0x1UL << OTG_GUSBCFG_PTCI_Pos );
  static constexpr uint32_t OTG_GUSBCFG_PTCI           = OTG_GUSBCFG_PTCI_Msk;
  static constexpr uint32_t OTG_GUSBCFG_ULPIIPD_Pos    = ( 25U );
  static constexpr uint32_t OTG_GUSBCFG_ULPIIPD_Msk    = ( 0x1UL << OTG_GUSBCFG_ULPIIPD_Pos );
  static constexpr uint32_t OTG_GUSBCFG_ULPIIPD        = OTG_GUSBCFG_ULPIIPD_Msk;
  static constexpr uint32_t OTG_GUSBCFG_FHMOD_Pos      = ( 29U );
  static constexpr uint32_t OTG_GUSBCFG_FHMOD_Msk      = ( 0x1UL << OTG_GUSBCFG_FHMOD_Pos );
  static constexpr uint32_t OTG_GUSBCFG_FHMOD          = OTG_GUSBCFG_FHMOD_Msk;
  static constexpr uint32_t OTG_GUSBCFG_FDMOD_Pos      = ( 30U );
  static constexpr uint32_t OTG_GUSBCFG_FDMOD_Msk      = ( 0x1UL << OTG_GUSBCFG_FDMOD_Pos );
  static constexpr uint32_t OTG_GUSBCFG_FDMOD          = OTG_GUSBCFG_FDMOD_Msk;
  static constexpr uint32_t OTG_GUSBCFG_CTXPKT_Pos     = ( 31U );
  static constexpr uint32_t OTG_GUSBCFG_CTXPKT_Msk     = ( 0x1UL << OTG_GUSBCFG_CTXPKT_Pos );
  static constexpr uint32_t OTG_GUSBCFG_CTXPKT         = OTG_GUSBCFG_CTXPKT_Msk;

  /********************  Bit definition for OTG_GRSTCTL register  ********************/
  static constexpr uint32_t OTG_GRSTCTL_CSRST_Pos   = ( 0U );
  static constexpr uint32_t OTG_GRSTCTL_CSRST_Msk   = ( 0x1UL << OTG_GRSTCTL_CSRST_Pos );
  static constexpr uint32_t OTG_GRSTCTL_CSRST       = OTG_GRSTCTL_CSRST_Msk;
  static constexpr uint32_t OTG_GRSTCTL_HSRST_Pos   = ( 1U );
  static constexpr uint32_t OTG_GRSTCTL_HSRST_Msk   = ( 0x1UL << OTG_GRSTCTL_HSRST_Pos );
  static constexpr uint32_t OTG_GRSTCTL_HSRST       = OTG_GRSTCTL_HSRST_Msk;
  static constexpr uint32_t OTG_GRSTCTL_FCRST_Pos   = ( 2U );
  static constexpr uint32_t OTG_GRSTCTL_FCRST_Msk   = ( 0x1UL << OTG_GRSTCTL_FCRST_Pos );
  static constexpr uint32_t OTG_GRSTCTL_FCRST       = OTG_GRSTCTL_FCRST_Msk;
  static constexpr uint32_t OTG_GRSTCTL_RXFFLSH_Pos = ( 4U );
  static constexpr uint32_t OTG_GRSTCTL_RXFFLSH_Msk = ( 0x1UL << OTG_GRSTCTL_RXFFLSH_Pos );
  static constexpr uint32_t OTG_GRSTCTL_RXFFLSH     = OTG_GRSTCTL_RXFFLSH_Msk;
  static constexpr uint32_t OTG_GRSTCTL_TXFFLSH_Pos = ( 5U );
  static constexpr uint32_t OTG_GRSTCTL_TXFFLSH_Msk = ( 0x1UL << OTG_GRSTCTL_TXFFLSH_Pos );
  static constexpr uint32_t OTG_GRSTCTL_TXFFLSH     = OTG_GRSTCTL_TXFFLSH_Msk;


  static constexpr uint32_t OTG_GRSTCTL_TXFNUM_Pos = ( 6U );
  static constexpr uint32_t OTG_GRSTCTL_TXFNUM_Msk = ( 0x1FUL << OTG_GRSTCTL_TXFNUM_Pos );
  static constexpr uint32_t OTG_GRSTCTL_TXFNUM     = OTG_GRSTCTL_TXFNUM_Msk;
  static constexpr uint32_t OTG_GRSTCTL_TXFNUM_0   = ( 0x01UL << OTG_GRSTCTL_TXFNUM_Pos );
  static constexpr uint32_t OTG_GRSTCTL_TXFNUM_1   = ( 0x02UL << OTG_GRSTCTL_TXFNUM_Pos );
  static constexpr uint32_t OTG_GRSTCTL_TXFNUM_2   = ( 0x04UL << OTG_GRSTCTL_TXFNUM_Pos );
  static constexpr uint32_t OTG_GRSTCTL_TXFNUM_3   = ( 0x08UL << OTG_GRSTCTL_TXFNUM_Pos );
  static constexpr uint32_t OTG_GRSTCTL_TXFNUM_4   = ( 0x10UL << OTG_GRSTCTL_TXFNUM_Pos );
  static constexpr uint32_t OTG_GRSTCTL_DMAREQ_Pos = ( 30U );
  static constexpr uint32_t OTG_GRSTCTL_DMAREQ_Msk = ( 0x1UL << OTG_GRSTCTL_DMAREQ_Pos );
  static constexpr uint32_t OTG_GRSTCTL_DMAREQ     = OTG_GRSTCTL_DMAREQ_Msk;
  static constexpr uint32_t OTG_GRSTCTL_AHBIDL_Pos = ( 31U );
  static constexpr uint32_t OTG_GRSTCTL_AHBIDL_Msk = ( 0x1UL << OTG_GRSTCTL_AHBIDL_Pos );
  static constexpr uint32_t OTG_GRSTCTL_AHBIDL     = OTG_GRSTCTL_AHBIDL_Msk;

  /********************  Bit definition for OTG_DIEPMSK register  ********************/
  static constexpr uint32_t OTG_DIEPMSK_XFRCM_Pos     = ( 0U );
  static constexpr uint32_t OTG_DIEPMSK_XFRCM_Msk     = ( 0x1UL << OTG_DIEPMSK_XFRCM_Pos );
  static constexpr uint32_t OTG_DIEPMSK_XFRCM         = OTG_DIEPMSK_XFRCM_Msk;
  static constexpr uint32_t OTG_DIEPMSK_EPDM_Pos      = ( 1U );
  static constexpr uint32_t OTG_DIEPMSK_EPDM_Msk      = ( 0x1UL << OTG_DIEPMSK_EPDM_Pos );
  static constexpr uint32_t OTG_DIEPMSK_EPDM          = OTG_DIEPMSK_EPDM_Msk;
  static constexpr uint32_t OTG_DIEPMSK_TOM_Pos       = ( 3U );
  static constexpr uint32_t OTG_DIEPMSK_TOM_Msk       = ( 0x1UL << OTG_DIEPMSK_TOM_Pos );
  static constexpr uint32_t OTG_DIEPMSK_TOM           = OTG_DIEPMSK_TOM_Msk;
  static constexpr uint32_t OTG_DIEPMSK_ITTXFEMSK_Pos = ( 4U );
  static constexpr uint32_t OTG_DIEPMSK_ITTXFEMSK_Msk = ( 0x1UL << OTG_DIEPMSK_ITTXFEMSK_Pos );
  static constexpr uint32_t OTG_DIEPMSK_ITTXFEMSK     = OTG_DIEPMSK_ITTXFEMSK_Msk;
  static constexpr uint32_t OTG_DIEPMSK_INEPNMM_Pos   = ( 5U );
  static constexpr uint32_t OTG_DIEPMSK_INEPNMM_Msk   = ( 0x1UL << OTG_DIEPMSK_INEPNMM_Pos );
  static constexpr uint32_t OTG_DIEPMSK_INEPNMM       = OTG_DIEPMSK_INEPNMM_Msk;
  static constexpr uint32_t OTG_DIEPMSK_INEPNEM_Pos   = ( 6U );
  static constexpr uint32_t OTG_DIEPMSK_INEPNEM_Msk   = ( 0x1UL << OTG_DIEPMSK_INEPNEM_Pos );
  static constexpr uint32_t OTG_DIEPMSK_INEPNEM       = OTG_DIEPMSK_INEPNEM_Msk;
  static constexpr uint32_t OTG_DIEPMSK_TXFURM_Pos    = ( 8U );
  static constexpr uint32_t OTG_DIEPMSK_TXFURM_Msk    = ( 0x1UL << OTG_DIEPMSK_TXFURM_Pos );
  static constexpr uint32_t OTG_DIEPMSK_TXFURM        = OTG_DIEPMSK_TXFURM_Msk;
  static constexpr uint32_t OTG_DIEPMSK_BIM_Pos       = ( 9U );
  static constexpr uint32_t OTG_DIEPMSK_BIM_Msk       = ( 0x1UL << OTG_DIEPMSK_BIM_Pos );
  static constexpr uint32_t OTG_DIEPMSK_BIM           = OTG_DIEPMSK_BIM_Msk;

  /********************  Bit definition for OTG_HPTXSTS register  ********************/
  static constexpr uint32_t OTG_HPTXSTS_PTXFSAVL_Pos = ( 0U );
  static constexpr uint32_t OTG_HPTXSTS_PTXFSAVL_Msk = ( 0xFFFFUL << OTG_HPTXSTS_PTXFSAVL_Pos );
  static constexpr uint32_t OTG_HPTXSTS_PTXFSAVL     = OTG_HPTXSTS_PTXFSAVL_Msk;
  static constexpr uint32_t OTG_HPTXSTS_PTXQSAV_Pos  = ( 16U );
  static constexpr uint32_t OTG_HPTXSTS_PTXQSAV_Msk  = ( 0xFFUL << OTG_HPTXSTS_PTXQSAV_Pos );
  static constexpr uint32_t OTG_HPTXSTS_PTXQSAV      = OTG_HPTXSTS_PTXQSAV_Msk;
  static constexpr uint32_t OTG_HPTXSTS_PTXQSAV_0    = ( 0x01UL << OTG_HPTXSTS_PTXQSAV_Pos );
  static constexpr uint32_t OTG_HPTXSTS_PTXQSAV_1    = ( 0x02UL << OTG_HPTXSTS_PTXQSAV_Pos );
  static constexpr uint32_t OTG_HPTXSTS_PTXQSAV_2    = ( 0x04UL << OTG_HPTXSTS_PTXQSAV_Pos );
  static constexpr uint32_t OTG_HPTXSTS_PTXQSAV_3    = ( 0x08UL << OTG_HPTXSTS_PTXQSAV_Pos );
  static constexpr uint32_t OTG_HPTXSTS_PTXQSAV_4    = ( 0x10UL << OTG_HPTXSTS_PTXQSAV_Pos );
  static constexpr uint32_t OTG_HPTXSTS_PTXQSAV_5    = ( 0x20UL << OTG_HPTXSTS_PTXQSAV_Pos );
  static constexpr uint32_t OTG_HPTXSTS_PTXQSAV_6    = ( 0x40UL << OTG_HPTXSTS_PTXQSAV_Pos );
  static constexpr uint32_t OTG_HPTXSTS_PTXQSAV_7    = ( 0x80UL << OTG_HPTXSTS_PTXQSAV_Pos );

  static constexpr uint32_t OTG_HPTXSTS_PTXQTOP_Pos = ( 24U );
  static constexpr uint32_t OTG_HPTXSTS_PTXQTOP_Msk = ( 0xFFUL << OTG_HPTXSTS_PTXQTOP_Pos );
  static constexpr uint32_t OTG_HPTXSTS_PTXQTOP     = OTG_HPTXSTS_PTXQTOP_Msk;
  static constexpr uint32_t OTG_HPTXSTS_PTXQTOP_0   = ( 0x01UL << OTG_HPTXSTS_PTXQTOP_Pos );
  static constexpr uint32_t OTG_HPTXSTS_PTXQTOP_1   = ( 0x02UL << OTG_HPTXSTS_PTXQTOP_Pos );
  static constexpr uint32_t OTG_HPTXSTS_PTXQTOP_2   = ( 0x04UL << OTG_HPTXSTS_PTXQTOP_Pos );
  static constexpr uint32_t OTG_HPTXSTS_PTXQTOP_3   = ( 0x08UL << OTG_HPTXSTS_PTXQTOP_Pos );
  static constexpr uint32_t OTG_HPTXSTS_PTXQTOP_4   = ( 0x10UL << OTG_HPTXSTS_PTXQTOP_Pos );
  static constexpr uint32_t OTG_HPTXSTS_PTXQTOP_5   = ( 0x20UL << OTG_HPTXSTS_PTXQTOP_Pos );
  static constexpr uint32_t OTG_HPTXSTS_PTXQTOP_6   = ( 0x40UL << OTG_HPTXSTS_PTXQTOP_Pos );
  static constexpr uint32_t OTG_HPTXSTS_PTXQTOP_7   = ( 0x80UL << OTG_HPTXSTS_PTXQTOP_Pos );

  /********************  Bit definition for OTG_HAINT register  ********************/
  static constexpr uint32_t OTG_HAINT_HAINT_Pos = ( 0U );
  static constexpr uint32_t OTG_HAINT_HAINT_Msk = ( 0xFFFFUL << OTG_HAINT_HAINT_Pos );
  static constexpr uint32_t OTG_HAINT_HAINT     = OTG_HAINT_HAINT_Msk;

  /********************  Bit definition for OTG_DOEPMSK register  ********************/
  static constexpr uint32_t OTG_DOEPMSK_XFRCM_Pos    = ( 0U );
  static constexpr uint32_t OTG_DOEPMSK_XFRCM_Msk    = ( 0x1UL << OTG_DOEPMSK_XFRCM_Pos );
  static constexpr uint32_t OTG_DOEPMSK_XFRCM        = OTG_DOEPMSK_XFRCM_Msk;
  static constexpr uint32_t OTG_DOEPMSK_EPDM_Pos     = ( 1U );
  static constexpr uint32_t OTG_DOEPMSK_EPDM_Msk     = ( 0x1UL << OTG_DOEPMSK_EPDM_Pos );
  static constexpr uint32_t OTG_DOEPMSK_EPDM         = OTG_DOEPMSK_EPDM_Msk;
  static constexpr uint32_t OTG_DOEPMSK_AHBERRM_Pos  = ( 2U );
  static constexpr uint32_t OTG_DOEPMSK_AHBERRM_Msk  = ( 0x1UL << OTG_DOEPMSK_AHBERRM_Pos );
  static constexpr uint32_t OTG_DOEPMSK_AHBERRM      = OTG_DOEPMSK_AHBERRM_Msk;
  static constexpr uint32_t OTG_DOEPMSK_STUPM_Pos    = ( 3U );
  static constexpr uint32_t OTG_DOEPMSK_STUPM_Msk    = ( 0x1UL << OTG_DOEPMSK_STUPM_Pos );
  static constexpr uint32_t OTG_DOEPMSK_STUPM        = OTG_DOEPMSK_STUPM_Msk;
  static constexpr uint32_t OTG_DOEPMSK_OTEPDM_Pos   = ( 4U );
  static constexpr uint32_t OTG_DOEPMSK_OTEPDM_Msk   = ( 0x1UL << OTG_DOEPMSK_OTEPDM_Pos );
  static constexpr uint32_t OTG_DOEPMSK_OTEPDM       = OTG_DOEPMSK_OTEPDM_Msk;
  static constexpr uint32_t OTG_DOEPMSK_OTEPSPRM_Pos = ( 5U );
  static constexpr uint32_t OTG_DOEPMSK_OTEPSPRM_Msk = ( 0x1UL << OTG_DOEPMSK_OTEPSPRM_Pos );
  static constexpr uint32_t OTG_DOEPMSK_OTEPSPRM     = OTG_DOEPMSK_OTEPSPRM_Msk;
  static constexpr uint32_t OTG_DOEPMSK_B2BSTUP_Pos  = ( 6U );
  static constexpr uint32_t OTG_DOEPMSK_B2BSTUP_Msk  = ( 0x1UL << OTG_DOEPMSK_B2BSTUP_Pos );
  static constexpr uint32_t OTG_DOEPMSK_B2BSTUP      = OTG_DOEPMSK_B2BSTUP_Msk;
  static constexpr uint32_t OTG_DOEPMSK_OPEM_Pos     = ( 8U );
  static constexpr uint32_t OTG_DOEPMSK_OPEM_Msk     = ( 0x1UL << OTG_DOEPMSK_OPEM_Pos );
  static constexpr uint32_t OTG_DOEPMSK_OPEM         = OTG_DOEPMSK_OPEM_Msk;
  static constexpr uint32_t OTG_DOEPMSK_BOIM_Pos     = ( 9U );
  static constexpr uint32_t OTG_DOEPMSK_BOIM_Msk     = ( 0x1UL << OTG_DOEPMSK_BOIM_Pos );
  static constexpr uint32_t OTG_DOEPMSK_BOIM         = OTG_DOEPMSK_BOIM_Msk;
  static constexpr uint32_t OTG_DOEPMSK_BERRM_Pos    = ( 12U );
  static constexpr uint32_t OTG_DOEPMSK_BERRM_Msk    = ( 0x1UL << OTG_DOEPMSK_BERRM_Pos );
  static constexpr uint32_t OTG_DOEPMSK_BERRM        = OTG_DOEPMSK_BERRM_Msk;
  static constexpr uint32_t OTG_DOEPMSK_NAKM_Pos     = ( 13U );
  static constexpr uint32_t OTG_DOEPMSK_NAKM_Msk     = ( 0x1UL << OTG_DOEPMSK_NAKM_Pos );
  static constexpr uint32_t OTG_DOEPMSK_NAKM         = OTG_DOEPMSK_NAKM_Msk;
  static constexpr uint32_t OTG_DOEPMSK_NYETM_Pos    = ( 14U );
  static constexpr uint32_t OTG_DOEPMSK_NYETM_Msk    = ( 0x1UL << OTG_DOEPMSK_NYETM_Pos );
  static constexpr uint32_t OTG_DOEPMSK_NYETM        = OTG_DOEPMSK_NYETM_Msk;
  /********************  Bit definition for OTG_GINTSTS register  ********************/
  static constexpr uint32_t OTG_GINTSTS_CMOD_Pos              = ( 0U );
  static constexpr uint32_t OTG_GINTSTS_CMOD_Msk              = ( 0x1UL << OTG_GINTSTS_CMOD_Pos );
  static constexpr uint32_t OTG_GINTSTS_CMOD                  = OTG_GINTSTS_CMOD_Msk;
  static constexpr uint32_t OTG_GINTSTS_MMIS_Pos              = ( 1U );
  static constexpr uint32_t OTG_GINTSTS_MMIS_Msk              = ( 0x1UL << OTG_GINTSTS_MMIS_Pos );
  static constexpr uint32_t OTG_GINTSTS_MMIS                  = OTG_GINTSTS_MMIS_Msk;
  static constexpr uint32_t OTG_GINTSTS_OTGINT_Pos            = ( 2U );
  static constexpr uint32_t OTG_GINTSTS_OTGINT_Msk            = ( 0x1UL << OTG_GINTSTS_OTGINT_Pos );
  static constexpr uint32_t OTG_GINTSTS_OTGINT                = OTG_GINTSTS_OTGINT_Msk;
  static constexpr uint32_t OTG_GINTSTS_SOF_Pos               = ( 3U );
  static constexpr uint32_t OTG_GINTSTS_SOF_Msk               = ( 0x1UL << OTG_GINTSTS_SOF_Pos );
  static constexpr uint32_t OTG_GINTSTS_SOF                   = OTG_GINTSTS_SOF_Msk;
  static constexpr uint32_t OTG_GINTSTS_RXFLVL_Pos            = ( 4U );
  static constexpr uint32_t OTG_GINTSTS_RXFLVL_Msk            = ( 0x1UL << OTG_GINTSTS_RXFLVL_Pos );
  static constexpr uint32_t OTG_GINTSTS_RXFLVL                = OTG_GINTSTS_RXFLVL_Msk;
  static constexpr uint32_t OTG_GINTSTS_NPTXFE_Pos            = ( 5U );
  static constexpr uint32_t OTG_GINTSTS_NPTXFE_Msk            = ( 0x1UL << OTG_GINTSTS_NPTXFE_Pos );
  static constexpr uint32_t OTG_GINTSTS_NPTXFE                = OTG_GINTSTS_NPTXFE_Msk;
  static constexpr uint32_t OTG_GINTSTS_GINAKEFF_Pos          = ( 6U );
  static constexpr uint32_t OTG_GINTSTS_GINAKEFF_Msk          = ( 0x1UL << OTG_GINTSTS_GINAKEFF_Pos );
  static constexpr uint32_t OTG_GINTSTS_GINAKEFF              = OTG_GINTSTS_GINAKEFF_Msk;
  static constexpr uint32_t OTG_GINTSTS_BOUTNAKEFF_Pos        = ( 7U );
  static constexpr uint32_t OTG_GINTSTS_BOUTNAKEFF_Msk        = ( 0x1UL << OTG_GINTSTS_BOUTNAKEFF_Pos );
  static constexpr uint32_t OTG_GINTSTS_BOUTNAKEFF            = OTG_GINTSTS_BOUTNAKEFF_Msk;
  static constexpr uint32_t OTG_GINTSTS_ESUSP_Pos             = ( 10U );
  static constexpr uint32_t OTG_GINTSTS_ESUSP_Msk             = ( 0x1UL << OTG_GINTSTS_ESUSP_Pos );
  static constexpr uint32_t OTG_GINTSTS_ESUSP                 = OTG_GINTSTS_ESUSP_Msk;
  static constexpr uint32_t OTG_GINTSTS_USBSUSP_Pos           = ( 11U );
  static constexpr uint32_t OTG_GINTSTS_USBSUSP_Msk           = ( 0x1UL << OTG_GINTSTS_USBSUSP_Pos );
  static constexpr uint32_t OTG_GINTSTS_USBSUSP               = OTG_GINTSTS_USBSUSP_Msk;
  static constexpr uint32_t OTG_GINTSTS_USBRST_Pos            = ( 12U );
  static constexpr uint32_t OTG_GINTSTS_USBRST_Msk            = ( 0x1UL << OTG_GINTSTS_USBRST_Pos );
  static constexpr uint32_t OTG_GINTSTS_USBRST                = OTG_GINTSTS_USBRST_Msk;
  static constexpr uint32_t OTG_GINTSTS_ENUMDNE_Pos           = ( 13U );
  static constexpr uint32_t OTG_GINTSTS_ENUMDNE_Msk           = ( 0x1UL << OTG_GINTSTS_ENUMDNE_Pos );
  static constexpr uint32_t OTG_GINTSTS_ENUMDNE               = OTG_GINTSTS_ENUMDNE_Msk;
  static constexpr uint32_t OTG_GINTSTS_ISOODRP_Pos           = ( 14U );
  static constexpr uint32_t OTG_GINTSTS_ISOODRP_Msk           = ( 0x1UL << OTG_GINTSTS_ISOODRP_Pos );
  static constexpr uint32_t OTG_GINTSTS_ISOODRP               = OTG_GINTSTS_ISOODRP_Msk;
  static constexpr uint32_t OTG_GINTSTS_EOPF_Pos              = ( 15U );
  static constexpr uint32_t OTG_GINTSTS_EOPF_Msk              = ( 0x1UL << OTG_GINTSTS_EOPF_Pos );
  static constexpr uint32_t OTG_GINTSTS_EOPF                  = OTG_GINTSTS_EOPF_Msk;
  static constexpr uint32_t OTG_GINTSTS_IEPINT_Pos            = ( 18U );
  static constexpr uint32_t OTG_GINTSTS_IEPINT_Msk            = ( 0x1UL << OTG_GINTSTS_IEPINT_Pos );
  static constexpr uint32_t OTG_GINTSTS_IEPINT                = OTG_GINTSTS_IEPINT_Msk;
  static constexpr uint32_t OTG_GINTSTS_OEPINT_Pos            = ( 19U );
  static constexpr uint32_t OTG_GINTSTS_OEPINT_Msk            = ( 0x1UL << OTG_GINTSTS_OEPINT_Pos );
  static constexpr uint32_t OTG_GINTSTS_OEPINT                = OTG_GINTSTS_OEPINT_Msk;
  static constexpr uint32_t OTG_GINTSTS_IISOIXFR_Pos          = ( 20U );
  static constexpr uint32_t OTG_GINTSTS_IISOIXFR_Msk          = ( 0x1UL << OTG_GINTSTS_IISOIXFR_Pos );
  static constexpr uint32_t OTG_GINTSTS_IISOIXFR              = OTG_GINTSTS_IISOIXFR_Msk;
  static constexpr uint32_t OTG_GINTSTS_PXFR_INCOMPISOOUT_Pos = ( 21U );
  static constexpr uint32_t OTG_GINTSTS_PXFR_INCOMPISOOUT_Msk = ( 0x1UL << OTG_GINTSTS_PXFR_INCOMPISOOUT_Pos );
  static constexpr uint32_t OTG_GINTSTS_PXFR_INCOMPISOOUT     = OTG_GINTSTS_PXFR_INCOMPISOOUT_Msk;
  static constexpr uint32_t OTG_GINTSTS_DATAFSUSP_Pos         = ( 22U );
  static constexpr uint32_t OTG_GINTSTS_DATAFSUSP_Msk         = ( 0x1UL << OTG_GINTSTS_DATAFSUSP_Pos );
  static constexpr uint32_t OTG_GINTSTS_DATAFSUSP             = OTG_GINTSTS_DATAFSUSP_Msk;
  static constexpr uint32_t OTG_GINTSTS_RSTDET_Pos            = ( 23U );
  static constexpr uint32_t OTG_GINTSTS_RSTDET_Msk            = ( 0x1UL << OTG_GINTSTS_RSTDET_Pos );
  static constexpr uint32_t OTG_GINTSTS_RSTDET                = OTG_GINTSTS_RSTDET_Msk;
  static constexpr uint32_t OTG_GINTSTS_HPRTINT_Pos           = ( 24U );
  static constexpr uint32_t OTG_GINTSTS_HPRTINT_Msk           = ( 0x1UL << OTG_GINTSTS_HPRTINT_Pos );
  static constexpr uint32_t OTG_GINTSTS_HPRTINT               = OTG_GINTSTS_HPRTINT_Msk;
  static constexpr uint32_t OTG_GINTSTS_HCINT_Pos             = ( 25U );
  static constexpr uint32_t OTG_GINTSTS_HCINT_Msk             = ( 0x1UL << OTG_GINTSTS_HCINT_Pos );
  static constexpr uint32_t OTG_GINTSTS_HCINT                 = OTG_GINTSTS_HCINT_Msk;
  static constexpr uint32_t OTG_GINTSTS_PTXFE_Pos             = ( 26U );
  static constexpr uint32_t OTG_GINTSTS_PTXFE_Msk             = ( 0x1UL << OTG_GINTSTS_PTXFE_Pos );
  static constexpr uint32_t OTG_GINTSTS_PTXFE                 = OTG_GINTSTS_PTXFE_Msk;
  static constexpr uint32_t OTG_GINTSTS_LPMINT_Pos            = ( 27U );
  static constexpr uint32_t OTG_GINTSTS_LPMINT_Msk            = ( 0x1UL << OTG_GINTSTS_LPMINT_Pos );
  static constexpr uint32_t OTG_GINTSTS_LPMINT                = OTG_GINTSTS_LPMINT_Msk;
  static constexpr uint32_t OTG_GINTSTS_CIDSCHG_Pos           = ( 28U );
  static constexpr uint32_t OTG_GINTSTS_CIDSCHG_Msk           = ( 0x1UL << OTG_GINTSTS_CIDSCHG_Pos );
  static constexpr uint32_t OTG_GINTSTS_CIDSCHG               = OTG_GINTSTS_CIDSCHG_Msk;
  static constexpr uint32_t OTG_GINTSTS_DISCINT_Pos           = ( 29U );
  static constexpr uint32_t OTG_GINTSTS_DISCINT_Msk           = ( 0x1UL << OTG_GINTSTS_DISCINT_Pos );
  static constexpr uint32_t OTG_GINTSTS_DISCINT               = OTG_GINTSTS_DISCINT_Msk;
  static constexpr uint32_t OTG_GINTSTS_SRQINT_Pos            = ( 30U );
  static constexpr uint32_t OTG_GINTSTS_SRQINT_Msk            = ( 0x1UL << OTG_GINTSTS_SRQINT_Pos );
  static constexpr uint32_t OTG_GINTSTS_SRQINT                = OTG_GINTSTS_SRQINT_Msk;
  static constexpr uint32_t OTG_GINTSTS_WKUINT_Pos            = ( 31U );
  static constexpr uint32_t OTG_GINTSTS_WKUINT_Msk            = ( 0x1UL << OTG_GINTSTS_WKUINT_Pos );
  static constexpr uint32_t OTG_GINTSTS_WKUINT                = OTG_GINTSTS_WKUINT_Msk;

  /********************  Bit definition for OTG_GINTMSK register  ********************/
  static constexpr uint32_t OTG_GINTMSK_MMISM_Pos           = ( 1U );
  static constexpr uint32_t OTG_GINTMSK_MMISM_Msk           = ( 0x1UL << OTG_GINTMSK_MMISM_Pos );
  static constexpr uint32_t OTG_GINTMSK_MMISM               = OTG_GINTMSK_MMISM_Msk;
  static constexpr uint32_t OTG_GINTMSK_OTGINT_Pos          = ( 2U );
  static constexpr uint32_t OTG_GINTMSK_OTGINT_Msk          = ( 0x1UL << OTG_GINTMSK_OTGINT_Pos );
  static constexpr uint32_t OTG_GINTMSK_OTGINT              = OTG_GINTMSK_OTGINT_Msk;
  static constexpr uint32_t OTG_GINTMSK_SOFM_Pos            = ( 3U );
  static constexpr uint32_t OTG_GINTMSK_SOFM_Msk            = ( 0x1UL << OTG_GINTMSK_SOFM_Pos );
  static constexpr uint32_t OTG_GINTMSK_SOFM                = OTG_GINTMSK_SOFM_Msk;
  static constexpr uint32_t OTG_GINTMSK_RXFLVLM_Pos         = ( 4U );
  static constexpr uint32_t OTG_GINTMSK_RXFLVLM_Msk         = ( 0x1UL << OTG_GINTMSK_RXFLVLM_Pos );
  static constexpr uint32_t OTG_GINTMSK_RXFLVLM             = OTG_GINTMSK_RXFLVLM_Msk;
  static constexpr uint32_t OTG_GINTMSK_NPTXFEM_Pos         = ( 5U );
  static constexpr uint32_t OTG_GINTMSK_NPTXFEM_Msk         = ( 0x1UL << OTG_GINTMSK_NPTXFEM_Pos );
  static constexpr uint32_t OTG_GINTMSK_NPTXFEM             = OTG_GINTMSK_NPTXFEM_Msk;
  static constexpr uint32_t OTG_GINTMSK_GINAKEFFM_Pos       = ( 6U );
  static constexpr uint32_t OTG_GINTMSK_GINAKEFFM_Msk       = ( 0x1UL << OTG_GINTMSK_GINAKEFFM_Pos );
  static constexpr uint32_t OTG_GINTMSK_GINAKEFFM           = OTG_GINTMSK_GINAKEFFM_Msk;
  static constexpr uint32_t OTG_GINTMSK_GONAKEFFM_Pos       = ( 7U );
  static constexpr uint32_t OTG_GINTMSK_GONAKEFFM_Msk       = ( 0x1UL << OTG_GINTMSK_GONAKEFFM_Pos );
  static constexpr uint32_t OTG_GINTMSK_GONAKEFFM           = OTG_GINTMSK_GONAKEFFM_Msk;
  static constexpr uint32_t OTG_GINTMSK_ESUSPM_Pos          = ( 10U );
  static constexpr uint32_t OTG_GINTMSK_ESUSPM_Msk          = ( 0x1UL << OTG_GINTMSK_ESUSPM_Pos );
  static constexpr uint32_t OTG_GINTMSK_ESUSPM              = OTG_GINTMSK_ESUSPM_Msk;
  static constexpr uint32_t OTG_GINTMSK_USBSUSPM_Pos        = ( 11U );
  static constexpr uint32_t OTG_GINTMSK_USBSUSPM_Msk        = ( 0x1UL << OTG_GINTMSK_USBSUSPM_Pos );
  static constexpr uint32_t OTG_GINTMSK_USBSUSPM            = OTG_GINTMSK_USBSUSPM_Msk;
  static constexpr uint32_t OTG_GINTMSK_USBRST_Pos          = ( 12U );
  static constexpr uint32_t OTG_GINTMSK_USBRST_Msk          = ( 0x1UL << OTG_GINTMSK_USBRST_Pos );
  static constexpr uint32_t OTG_GINTMSK_USBRST              = OTG_GINTMSK_USBRST_Msk;
  static constexpr uint32_t OTG_GINTMSK_ENUMDNEM_Pos        = ( 13U );
  static constexpr uint32_t OTG_GINTMSK_ENUMDNEM_Msk        = ( 0x1UL << OTG_GINTMSK_ENUMDNEM_Pos );
  static constexpr uint32_t OTG_GINTMSK_ENUMDNEM            = OTG_GINTMSK_ENUMDNEM_Msk;
  static constexpr uint32_t OTG_GINTMSK_ISOODRPM_Pos        = ( 14U );
  static constexpr uint32_t OTG_GINTMSK_ISOODRPM_Msk        = ( 0x1UL << OTG_GINTMSK_ISOODRPM_Pos );
  static constexpr uint32_t OTG_GINTMSK_ISOODRPM            = OTG_GINTMSK_ISOODRPM_Msk;
  static constexpr uint32_t OTG_GINTMSK_EOPFM_Pos           = ( 15U );
  static constexpr uint32_t OTG_GINTMSK_EOPFM_Msk           = ( 0x1UL << OTG_GINTMSK_EOPFM_Pos );
  static constexpr uint32_t OTG_GINTMSK_EOPFM               = OTG_GINTMSK_EOPFM_Msk;
  static constexpr uint32_t OTG_GINTMSK_EPMISM_Pos          = ( 17U );
  static constexpr uint32_t OTG_GINTMSK_EPMISM_Msk          = ( 0x1UL << OTG_GINTMSK_EPMISM_Pos );
  static constexpr uint32_t OTG_GINTMSK_EPMISM              = OTG_GINTMSK_EPMISM_Msk;
  static constexpr uint32_t OTG_GINTMSK_IEPINT_Pos          = ( 18U );
  static constexpr uint32_t OTG_GINTMSK_IEPINT_Msk          = ( 0x1UL << OTG_GINTMSK_IEPINT_Pos );
  static constexpr uint32_t OTG_GINTMSK_IEPINT              = OTG_GINTMSK_IEPINT_Msk;
  static constexpr uint32_t OTG_GINTMSK_OEPINT_Pos          = ( 19U );
  static constexpr uint32_t OTG_GINTMSK_OEPINT_Msk          = ( 0x1UL << OTG_GINTMSK_OEPINT_Pos );
  static constexpr uint32_t OTG_GINTMSK_OEPINT              = OTG_GINTMSK_OEPINT_Msk;
  static constexpr uint32_t OTG_GINTMSK_IISOIXFRM_Pos       = ( 20U );
  static constexpr uint32_t OTG_GINTMSK_IISOIXFRM_Msk       = ( 0x1UL << OTG_GINTMSK_IISOIXFRM_Pos );
  static constexpr uint32_t OTG_GINTMSK_IISOIXFRM           = OTG_GINTMSK_IISOIXFRM_Msk;
  static constexpr uint32_t OTG_GINTMSK_PXFRM_IISOOXFRM_Pos = ( 21U );
  static constexpr uint32_t OTG_GINTMSK_PXFRM_IISOOXFRM_Msk = ( 0x1UL << OTG_GINTMSK_PXFRM_IISOOXFRM_Pos );
  static constexpr uint32_t OTG_GINTMSK_PXFRM_IISOOXFRM     = OTG_GINTMSK_PXFRM_IISOOXFRM_Msk;
  static constexpr uint32_t OTG_GINTMSK_FSUSPM_Pos          = ( 22U );
  static constexpr uint32_t OTG_GINTMSK_FSUSPM_Msk          = ( 0x1UL << OTG_GINTMSK_FSUSPM_Pos );
  static constexpr uint32_t OTG_GINTMSK_FSUSPM              = OTG_GINTMSK_FSUSPM_Msk;
  static constexpr uint32_t OTG_GINTMSK_RSTDEM_Pos          = ( 23U );
  static constexpr uint32_t OTG_GINTMSK_RSTDEM_Msk          = ( 0x1UL << OTG_GINTMSK_RSTDEM_Pos );
  static constexpr uint32_t OTG_GINTMSK_RSTDEM              = OTG_GINTMSK_RSTDEM_Msk;
  static constexpr uint32_t OTG_GINTMSK_PRTIM_Pos           = ( 24U );
  static constexpr uint32_t OTG_GINTMSK_PRTIM_Msk           = ( 0x1UL << OTG_GINTMSK_PRTIM_Pos );
  static constexpr uint32_t OTG_GINTMSK_PRTIM               = OTG_GINTMSK_PRTIM_Msk;
  static constexpr uint32_t OTG_GINTMSK_HCIM_Pos            = ( 25U );
  static constexpr uint32_t OTG_GINTMSK_HCIM_Msk            = ( 0x1UL << OTG_GINTMSK_HCIM_Pos );
  static constexpr uint32_t OTG_GINTMSK_HCIM                = OTG_GINTMSK_HCIM_Msk;
  static constexpr uint32_t OTG_GINTMSK_PTXFEM_Pos          = ( 26U );
  static constexpr uint32_t OTG_GINTMSK_PTXFEM_Msk          = ( 0x1UL << OTG_GINTMSK_PTXFEM_Pos );
  static constexpr uint32_t OTG_GINTMSK_PTXFEM              = OTG_GINTMSK_PTXFEM_Msk;
  static constexpr uint32_t OTG_GINTMSK_LPMINTM_Pos         = ( 27U );
  static constexpr uint32_t OTG_GINTMSK_LPMINTM_Msk         = ( 0x1UL << OTG_GINTMSK_LPMINTM_Pos );
  static constexpr uint32_t OTG_GINTMSK_LPMINTM             = OTG_GINTMSK_LPMINTM_Msk;
  static constexpr uint32_t OTG_GINTMSK_CIDSCHGM_Pos        = ( 28U );
  static constexpr uint32_t OTG_GINTMSK_CIDSCHGM_Msk        = ( 0x1UL << OTG_GINTMSK_CIDSCHGM_Pos );
  static constexpr uint32_t OTG_GINTMSK_CIDSCHGM            = OTG_GINTMSK_CIDSCHGM_Msk;
  static constexpr uint32_t OTG_GINTMSK_DISCINT_Pos         = ( 29U );
  static constexpr uint32_t OTG_GINTMSK_DISCINT_Msk         = ( 0x1UL << OTG_GINTMSK_DISCINT_Pos );
  static constexpr uint32_t OTG_GINTMSK_DISCINT             = OTG_GINTMSK_DISCINT_Msk;
  static constexpr uint32_t OTG_GINTMSK_SRQIM_Pos           = ( 30U );
  static constexpr uint32_t OTG_GINTMSK_SRQIM_Msk           = ( 0x1UL << OTG_GINTMSK_SRQIM_Pos );
  static constexpr uint32_t OTG_GINTMSK_SRQIM               = OTG_GINTMSK_SRQIM_Msk;
  static constexpr uint32_t OTG_GINTMSK_WUIM_Pos            = ( 31U );
  static constexpr uint32_t OTG_GINTMSK_WUIM_Msk            = ( 0x1UL << OTG_GINTMSK_WUIM_Pos );
  static constexpr uint32_t OTG_GINTMSK_WUIM                = OTG_GINTMSK_WUIM_Msk;

  /********************  Bit definition for OTG_DAINT register  ********************/
  static constexpr uint32_t OTG_DAINT_IEPINT_Pos = ( 0U );
  static constexpr uint32_t OTG_DAINT_IEPINT_Msk = ( 0xFFFFUL << OTG_DAINT_IEPINT_Pos );
  static constexpr uint32_t OTG_DAINT_IEPINT     = OTG_DAINT_IEPINT_Msk;
  static constexpr uint32_t OTG_DAINT_OEPINT_Pos = ( 16U );
  static constexpr uint32_t OTG_DAINT_OEPINT_Msk = ( 0xFFFFUL << OTG_DAINT_OEPINT_Pos );
  static constexpr uint32_t OTG_DAINT_OEPINT     = OTG_DAINT_OEPINT_Msk;

  /********************  Bit definition for OTG_HAINTMSK register  ********************/
  static constexpr uint32_t OTG_HAINTMSK_HAINTM_Pos = ( 0U );
  static constexpr uint32_t OTG_HAINTMSK_HAINTM_Msk = ( 0xFFFFUL << OTG_HAINTMSK_HAINTM_Pos );
  static constexpr uint32_t OTG_HAINTMSK_HAINTM     = OTG_HAINTMSK_HAINTM_Msk;

  /********************  Bit definition for OTG_GRXSTSP register  ********************/
  static constexpr uint32_t OTG_GRXSTSP_EPNUM_Pos  = ( 0U );
  static constexpr uint32_t OTG_GRXSTSP_EPNUM_Msk  = ( 0xFUL << OTG_GRXSTSP_EPNUM_Pos );
  static constexpr uint32_t OTG_GRXSTSP_EPNUM      = OTG_GRXSTSP_EPNUM_Msk;
  static constexpr uint32_t OTG_GRXSTSP_BCNT_Pos   = ( 4U );
  static constexpr uint32_t OTG_GRXSTSP_BCNT_Msk   = ( 0x7FFUL << OTG_GRXSTSP_BCNT_Pos );
  static constexpr uint32_t OTG_GRXSTSP_BCNT       = OTG_GRXSTSP_BCNT_Msk;
  static constexpr uint32_t OTG_GRXSTSP_DPID_Pos   = ( 15U );
  static constexpr uint32_t OTG_GRXSTSP_DPID_Msk   = ( 0x3UL << OTG_GRXSTSP_DPID_Pos );
  static constexpr uint32_t OTG_GRXSTSP_DPID       = OTG_GRXSTSP_DPID_Msk;
  static constexpr uint32_t OTG_GRXSTSP_PKTSTS_Pos = ( 17U );
  static constexpr uint32_t OTG_GRXSTSP_PKTSTS_Msk = ( 0xFUL << OTG_GRXSTSP_PKTSTS_Pos );
  static constexpr uint32_t OTG_GRXSTSP_PKTSTS     = OTG_GRXSTSP_PKTSTS_Msk;

  /********************  Bit definition for OTG_DAINTMSK register  ********************/
  static constexpr uint32_t OTG_DAINTMSK_IEPM_Pos = ( 0U );
  static constexpr uint32_t OTG_DAINTMSK_IEPM_Msk = ( 0xFFFFUL << OTG_DAINTMSK_IEPM_Pos );
  static constexpr uint32_t OTG_DAINTMSK_IEPM     = OTG_DAINTMSK_IEPM_Msk;
  static constexpr uint32_t OTG_DAINTMSK_OEPM_Pos = ( 16U );
  static constexpr uint32_t OTG_DAINTMSK_OEPM_Msk = ( 0xFFFFUL << OTG_DAINTMSK_OEPM_Pos );
  static constexpr uint32_t OTG_DAINTMSK_OEPM     = OTG_DAINTMSK_OEPM_Msk;

  /********************  Bit definition for OTG_GRXFSIZ register  ********************/
  static constexpr uint32_t OTG_GRXFSIZ_RXFD_Pos = ( 0U );
  static constexpr uint32_t OTG_GRXFSIZ_RXFD_Msk = ( 0xFFFFUL << OTG_GRXFSIZ_RXFD_Pos );
  static constexpr uint32_t OTG_GRXFSIZ_RXFD     = OTG_GRXFSIZ_RXFD_Msk;

  /********************  Bit definition for OTG_DVBUSDIS register  ********************/
  static constexpr uint32_t OTG_DVBUSDIS_VBUSDT_Pos = ( 0U );
  static constexpr uint32_t OTG_DVBUSDIS_VBUSDT_Msk = ( 0xFFFFUL << OTG_DVBUSDIS_VBUSDT_Pos );
  static constexpr uint32_t OTG_DVBUSDIS_VBUSDT     = OTG_DVBUSDIS_VBUSDT_Msk;

  /********************  Bit definition for OTG register  ********************/
  static constexpr uint32_t OTG_NPTXFSA_Pos = ( 0U );
  static constexpr uint32_t OTG_NPTXFSA_Msk = ( 0xFFFFUL << OTG_NPTXFSA_Pos );
  static constexpr uint32_t OTG_NPTXFSA     = OTG_NPTXFSA_Msk;
  static constexpr uint32_t OTG_NPTXFD_Pos  = ( 16U );
  static constexpr uint32_t OTG_NPTXFD_Msk  = ( 0xFFFFUL << OTG_NPTXFD_Pos );
  static constexpr uint32_t OTG_NPTXFD      = OTG_NPTXFD_Msk;
  static constexpr uint32_t OTG_TX0FSA_Pos  = ( 0U );
  static constexpr uint32_t OTG_TX0FSA_Msk  = ( 0xFFFFUL << OTG_TX0FSA_Pos );
  static constexpr uint32_t OTG_TX0FSA      = OTG_TX0FSA_Msk;
  static constexpr uint32_t OTG_TX0FD_Pos   = ( 16U );
  static constexpr uint32_t OTG_TX0FD_Msk   = ( 0xFFFFUL << OTG_TX0FD_Pos );
  static constexpr uint32_t OTG_TX0FD       = OTG_TX0FD_Msk;

  /********************  Bit definition forOTG_DVBUSPULSE register  ********************/
  static constexpr uint32_t OTG_DVBUSPULSE_DVBUSP_Pos = ( 0U );
  static constexpr uint32_t OTG_DVBUSPULSE_DVBUSP_Msk = ( 0xFFFUL << OTG_DVBUSPULSE_DVBUSP_Pos );
  static constexpr uint32_t OTG_DVBUSPULSE_DVBUSP     = OTG_DVBUSPULSE_DVBUSP_Msk;

  /********************  Bit definition for OTG_GNPTXSTS register  ********************/
  static constexpr uint32_t OTG_GNPTXSTS_NPTXFSAV_Pos = ( 0U );
  static constexpr uint32_t OTG_GNPTXSTS_NPTXFSAV_Msk = ( 0xFFFFUL << OTG_GNPTXSTS_NPTXFSAV_Pos );
  static constexpr uint32_t OTG_GNPTXSTS_NPTXFSAV     = OTG_GNPTXSTS_NPTXFSAV_Msk;

  static constexpr uint32_t OTG_GNPTXSTS_NPTQXSAV_Pos = ( 16U );
  static constexpr uint32_t OTG_GNPTXSTS_NPTQXSAV_Msk = ( 0xFFUL << OTG_GNPTXSTS_NPTQXSAV_Pos );
  static constexpr uint32_t OTG_GNPTXSTS_NPTQXSAV     = OTG_GNPTXSTS_NPTQXSAV_Msk;
  static constexpr uint32_t OTG_GNPTXSTS_NPTQXSAV_0   = ( 0x01UL << OTG_GNPTXSTS_NPTQXSAV_Pos );
  static constexpr uint32_t OTG_GNPTXSTS_NPTQXSAV_1   = ( 0x02UL << OTG_GNPTXSTS_NPTQXSAV_Pos );
  static constexpr uint32_t OTG_GNPTXSTS_NPTQXSAV_2   = ( 0x04UL << OTG_GNPTXSTS_NPTQXSAV_Pos );
  static constexpr uint32_t OTG_GNPTXSTS_NPTQXSAV_3   = ( 0x08UL << OTG_GNPTXSTS_NPTQXSAV_Pos );
  static constexpr uint32_t OTG_GNPTXSTS_NPTQXSAV_4   = ( 0x10UL << OTG_GNPTXSTS_NPTQXSAV_Pos );
  static constexpr uint32_t OTG_GNPTXSTS_NPTQXSAV_5   = ( 0x20UL << OTG_GNPTXSTS_NPTQXSAV_Pos );
  static constexpr uint32_t OTG_GNPTXSTS_NPTQXSAV_6   = ( 0x40UL << OTG_GNPTXSTS_NPTQXSAV_Pos );
  static constexpr uint32_t OTG_GNPTXSTS_NPTQXSAV_7   = ( 0x80UL << OTG_GNPTXSTS_NPTQXSAV_Pos );

  static constexpr uint32_t OTG_GNPTXSTS_NPTXQTOP_Pos = ( 24U );
  static constexpr uint32_t OTG_GNPTXSTS_NPTXQTOP_Msk = ( 0x7FUL << OTG_GNPTXSTS_NPTXQTOP_Pos );
  static constexpr uint32_t OTG_GNPTXSTS_NPTXQTOP     = OTG_GNPTXSTS_NPTXQTOP_Msk;
  static constexpr uint32_t OTG_GNPTXSTS_NPTXQTOP_0   = ( 0x01UL << OTG_GNPTXSTS_NPTXQTOP_Pos );
  static constexpr uint32_t OTG_GNPTXSTS_NPTXQTOP_1   = ( 0x02UL << OTG_GNPTXSTS_NPTXQTOP_Pos );
  static constexpr uint32_t OTG_GNPTXSTS_NPTXQTOP_2   = ( 0x04UL << OTG_GNPTXSTS_NPTXQTOP_Pos );
  static constexpr uint32_t OTG_GNPTXSTS_NPTXQTOP_3   = ( 0x08UL << OTG_GNPTXSTS_NPTXQTOP_Pos );
  static constexpr uint32_t OTG_GNPTXSTS_NPTXQTOP_4   = ( 0x10UL << OTG_GNPTXSTS_NPTXQTOP_Pos );
  static constexpr uint32_t OTG_GNPTXSTS_NPTXQTOP_5   = ( 0x20UL << OTG_GNPTXSTS_NPTXQTOP_Pos );
  static constexpr uint32_t OTG_GNPTXSTS_NPTXQTOP_6   = ( 0x40UL << OTG_GNPTXSTS_NPTXQTOP_Pos );

  /********************  Bit definition for OTG_DTHRCTL register  ********************/
  static constexpr uint32_t OTG_DTHRCTL_NONISOTHREN_Pos = ( 0U );
  static constexpr uint32_t OTG_DTHRCTL_NONISOTHREN_Msk = ( 0x1UL << OTG_DTHRCTL_NONISOTHREN_Pos );
  static constexpr uint32_t OTG_DTHRCTL_NONISOTHREN     = OTG_DTHRCTL_NONISOTHREN_Msk;
  static constexpr uint32_t OTG_DTHRCTL_ISOTHREN_Pos    = ( 1U );
  static constexpr uint32_t OTG_DTHRCTL_ISOTHREN_Msk    = ( 0x1UL << OTG_DTHRCTL_ISOTHREN_Pos );
  static constexpr uint32_t OTG_DTHRCTL_ISOTHREN        = OTG_DTHRCTL_ISOTHREN_Msk;

  static constexpr uint32_t OTG_DTHRCTL_TXTHRLEN_Pos = ( 2U );
  static constexpr uint32_t OTG_DTHRCTL_TXTHRLEN_Msk = ( 0x1FFUL << OTG_DTHRCTL_TXTHRLEN_Pos );
  static constexpr uint32_t OTG_DTHRCTL_TXTHRLEN     = OTG_DTHRCTL_TXTHRLEN_Msk;
  static constexpr uint32_t OTG_DTHRCTL_TXTHRLEN_0   = ( 0x001UL << OTG_DTHRCTL_TXTHRLEN_Pos );
  static constexpr uint32_t OTG_DTHRCTL_TXTHRLEN_1   = ( 0x002UL << OTG_DTHRCTL_TXTHRLEN_Pos );
  static constexpr uint32_t OTG_DTHRCTL_TXTHRLEN_2   = ( 0x004UL << OTG_DTHRCTL_TXTHRLEN_Pos );
  static constexpr uint32_t OTG_DTHRCTL_TXTHRLEN_3   = ( 0x008UL << OTG_DTHRCTL_TXTHRLEN_Pos );
  static constexpr uint32_t OTG_DTHRCTL_TXTHRLEN_4   = ( 0x010UL << OTG_DTHRCTL_TXTHRLEN_Pos );
  static constexpr uint32_t OTG_DTHRCTL_TXTHRLEN_5   = ( 0x020UL << OTG_DTHRCTL_TXTHRLEN_Pos );
  static constexpr uint32_t OTG_DTHRCTL_TXTHRLEN_6   = ( 0x040UL << OTG_DTHRCTL_TXTHRLEN_Pos );
  static constexpr uint32_t OTG_DTHRCTL_TXTHRLEN_7   = ( 0x080UL << OTG_DTHRCTL_TXTHRLEN_Pos );
  static constexpr uint32_t OTG_DTHRCTL_TXTHRLEN_8   = ( 0x100UL << OTG_DTHRCTL_TXTHRLEN_Pos );
  static constexpr uint32_t OTG_DTHRCTL_RXTHREN_Pos  = ( 16U );
  static constexpr uint32_t OTG_DTHRCTL_RXTHREN_Msk  = ( 0x1UL << OTG_DTHRCTL_RXTHREN_Pos );
  static constexpr uint32_t OTG_DTHRCTL_RXTHREN      = OTG_DTHRCTL_RXTHREN_Msk;

  static constexpr uint32_t OTG_DTHRCTL_RXTHRLEN_Pos = ( 17U );
  static constexpr uint32_t OTG_DTHRCTL_RXTHRLEN_Msk = ( 0x1FFUL << OTG_DTHRCTL_RXTHRLEN_Pos );
  static constexpr uint32_t OTG_DTHRCTL_RXTHRLEN     = OTG_DTHRCTL_RXTHRLEN_Msk;
  static constexpr uint32_t OTG_DTHRCTL_RXTHRLEN_0   = ( 0x001UL << OTG_DTHRCTL_RXTHRLEN_Pos );
  static constexpr uint32_t OTG_DTHRCTL_RXTHRLEN_1   = ( 0x002UL << OTG_DTHRCTL_RXTHRLEN_Pos );
  static constexpr uint32_t OTG_DTHRCTL_RXTHRLEN_2   = ( 0x004UL << OTG_DTHRCTL_RXTHRLEN_Pos );
  static constexpr uint32_t OTG_DTHRCTL_RXTHRLEN_3   = ( 0x008UL << OTG_DTHRCTL_RXTHRLEN_Pos );
  static constexpr uint32_t OTG_DTHRCTL_RXTHRLEN_4   = ( 0x010UL << OTG_DTHRCTL_RXTHRLEN_Pos );
  static constexpr uint32_t OTG_DTHRCTL_RXTHRLEN_5   = ( 0x020UL << OTG_DTHRCTL_RXTHRLEN_Pos );
  static constexpr uint32_t OTG_DTHRCTL_RXTHRLEN_6   = ( 0x040UL << OTG_DTHRCTL_RXTHRLEN_Pos );
  static constexpr uint32_t OTG_DTHRCTL_RXTHRLEN_7   = ( 0x080UL << OTG_DTHRCTL_RXTHRLEN_Pos );
  static constexpr uint32_t OTG_DTHRCTL_RXTHRLEN_8   = ( 0x100UL << OTG_DTHRCTL_RXTHRLEN_Pos );
  static constexpr uint32_t OTG_DTHRCTL_ARPEN_Pos    = ( 27U );
  static constexpr uint32_t OTG_DTHRCTL_ARPEN_Msk    = ( 0x1UL << OTG_DTHRCTL_ARPEN_Pos );
  static constexpr uint32_t OTG_DTHRCTL_ARPEN        = OTG_DTHRCTL_ARPEN_Msk;

  /********************  Bit definition for OTG_DIEPEMPMSK register  ********************/
  static constexpr uint32_t OTG_DIEPEMPMSK_INEPTXFEM_Pos = ( 0U );
  static constexpr uint32_t OTG_DIEPEMPMSK_INEPTXFEM_Msk = ( 0xFFFFUL << OTG_DIEPEMPMSK_INEPTXFEM_Pos );
  static constexpr uint32_t OTG_DIEPEMPMSK_INEPTXFEM     = OTG_DIEPEMPMSK_INEPTXFEM_Msk;

  /********************  Bit definition for OTG_DEACHINT register  ********************/
  static constexpr uint32_t OTG_DEACHINT_IEP1INT_Pos = ( 1U );
  static constexpr uint32_t OTG_DEACHINT_IEP1INT_Msk = ( 0x1UL << OTG_DEACHINT_IEP1INT_Pos );
  static constexpr uint32_t OTG_DEACHINT_IEP1INT     = OTG_DEACHINT_IEP1INT_Msk;
  static constexpr uint32_t OTG_DEACHINT_OEP1INT_Pos = ( 17U );
  static constexpr uint32_t OTG_DEACHINT_OEP1INT_Msk = ( 0x1UL << OTG_DEACHINT_OEP1INT_Pos );
  static constexpr uint32_t OTG_DEACHINT_OEP1INT     = OTG_DEACHINT_OEP1INT_Msk;

  /********************  Bit definition for OTG_GCCFG register  ********************/
  static constexpr uint32_t OTG_GCCFG_PWRDWN_Pos = ( 16U );
  static constexpr uint32_t OTG_GCCFG_PWRDWN_Msk = ( 0x1UL << OTG_GCCFG_PWRDWN_Pos );
  static constexpr uint32_t OTG_GCCFG_PWRDWN     = OTG_GCCFG_PWRDWN_Msk;
  static constexpr uint32_t OTG_GCCFG_VBDEN_Pos  = ( 21U );
  static constexpr uint32_t OTG_GCCFG_VBDEN_Msk  = ( 0x1UL << OTG_GCCFG_VBDEN_Pos );
  static constexpr uint32_t OTG_GCCFG_VBDEN      = OTG_GCCFG_VBDEN_Msk;

  /********************  Bit definition forOTG_DEACHINTMSK register  ********************/
  static constexpr uint32_t OTG_DEACHINTMSK_IEP1INTM_Pos = ( 1U );
  static constexpr uint32_t OTG_DEACHINTMSK_IEP1INTM_Msk = ( 0x1UL << OTG_DEACHINTMSK_IEP1INTM_Pos );
  static constexpr uint32_t OTG_DEACHINTMSK_IEP1INTM     = OTG_DEACHINTMSK_IEP1INTM_Msk;
  static constexpr uint32_t OTG_DEACHINTMSK_OEP1INTM_Pos = ( 17U );
  static constexpr uint32_t OTG_DEACHINTMSK_OEP1INTM_Msk = ( 0x1UL << OTG_DEACHINTMSK_OEP1INTM_Pos );
  static constexpr uint32_t OTG_DEACHINTMSK_OEP1INTM     = OTG_DEACHINTMSK_OEP1INTM_Msk;

  /********************  Bit definition for OTG_CID register  ********************/
  static constexpr uint32_t OTG_CID_PRODUCT_ID_Pos = ( 0U );
  static constexpr uint32_t OTG_CID_PRODUCT_ID_Msk = ( 0xFFFFFFFFUL << OTG_CID_PRODUCT_ID_Pos );
  static constexpr uint32_t OTG_CID_PRODUCT_ID     = OTG_CID_PRODUCT_ID_Msk;

  /********************  Bit definition for OTG_GLPMCFG register  ********************/
  static constexpr uint32_t OTG_GLPMCFG_LPMEN_Pos      = ( 0U );
  static constexpr uint32_t OTG_GLPMCFG_LPMEN_Msk      = ( 0x1UL << OTG_GLPMCFG_LPMEN_Pos );
  static constexpr uint32_t OTG_GLPMCFG_LPMEN          = OTG_GLPMCFG_LPMEN_Msk;
  static constexpr uint32_t OTG_GLPMCFG_LPMACK_Pos     = ( 1U );
  static constexpr uint32_t OTG_GLPMCFG_LPMACK_Msk     = ( 0x1UL << OTG_GLPMCFG_LPMACK_Pos );
  static constexpr uint32_t OTG_GLPMCFG_LPMACK         = OTG_GLPMCFG_LPMACK_Msk;
  static constexpr uint32_t OTG_GLPMCFG_BESL_Pos       = ( 2U );
  static constexpr uint32_t OTG_GLPMCFG_BESL_Msk       = ( 0xFUL << OTG_GLPMCFG_BESL_Pos );
  static constexpr uint32_t OTG_GLPMCFG_BESL           = OTG_GLPMCFG_BESL_Msk;
  static constexpr uint32_t OTG_GLPMCFG_REMWAKE_Pos    = ( 6U );
  static constexpr uint32_t OTG_GLPMCFG_REMWAKE_Msk    = ( 0x1UL << OTG_GLPMCFG_REMWAKE_Pos );
  static constexpr uint32_t OTG_GLPMCFG_REMWAKE        = OTG_GLPMCFG_REMWAKE_Msk;
  static constexpr uint32_t OTG_GLPMCFG_L1SSEN_Pos     = ( 7U );
  static constexpr uint32_t OTG_GLPMCFG_L1SSEN_Msk     = ( 0x1UL << OTG_GLPMCFG_L1SSEN_Pos );
  static constexpr uint32_t OTG_GLPMCFG_L1SSEN         = OTG_GLPMCFG_L1SSEN_Msk;
  static constexpr uint32_t OTG_GLPMCFG_BESLTHRS_Pos   = ( 8U );
  static constexpr uint32_t OTG_GLPMCFG_BESLTHRS_Msk   = ( 0xFUL << OTG_GLPMCFG_BESLTHRS_Pos );
  static constexpr uint32_t OTG_GLPMCFG_BESLTHRS       = OTG_GLPMCFG_BESLTHRS_Msk;
  static constexpr uint32_t OTG_GLPMCFG_L1DSEN_Pos     = ( 12U );
  static constexpr uint32_t OTG_GLPMCFG_L1DSEN_Msk     = ( 0x1UL << OTG_GLPMCFG_L1DSEN_Pos );
  static constexpr uint32_t OTG_GLPMCFG_L1DSEN         = OTG_GLPMCFG_L1DSEN_Msk;
  static constexpr uint32_t OTG_GLPMCFG_LPMRSP_Pos     = ( 13U );
  static constexpr uint32_t OTG_GLPMCFG_LPMRSP_Msk     = ( 0x3UL << OTG_GLPMCFG_LPMRSP_Pos );
  static constexpr uint32_t OTG_GLPMCFG_LPMRSP         = OTG_GLPMCFG_LPMRSP_Msk;
  static constexpr uint32_t OTG_GLPMCFG_SLPSTS_Pos     = ( 15U );
  static constexpr uint32_t OTG_GLPMCFG_SLPSTS_Msk     = ( 0x1UL << OTG_GLPMCFG_SLPSTS_Pos );
  static constexpr uint32_t OTG_GLPMCFG_SLPSTS         = OTG_GLPMCFG_SLPSTS_Msk;
  static constexpr uint32_t OTG_GLPMCFG_L1RSMOK_Pos    = ( 16U );
  static constexpr uint32_t OTG_GLPMCFG_L1RSMOK_Msk    = ( 0x1UL << OTG_GLPMCFG_L1RSMOK_Pos );
  static constexpr uint32_t OTG_GLPMCFG_L1RSMOK        = OTG_GLPMCFG_L1RSMOK_Msk;
  static constexpr uint32_t OTG_GLPMCFG_LPMCHIDX_Pos   = ( 17U );
  static constexpr uint32_t OTG_GLPMCFG_LPMCHIDX_Msk   = ( 0xFUL << OTG_GLPMCFG_LPMCHIDX_Pos );
  static constexpr uint32_t OTG_GLPMCFG_LPMCHIDX       = OTG_GLPMCFG_LPMCHIDX_Msk;
  static constexpr uint32_t OTG_GLPMCFG_LPMRCNT_Pos    = ( 21U );
  static constexpr uint32_t OTG_GLPMCFG_LPMRCNT_Msk    = ( 0x7UL << OTG_GLPMCFG_LPMRCNT_Pos );
  static constexpr uint32_t OTG_GLPMCFG_LPMRCNT        = OTG_GLPMCFG_LPMRCNT_Msk;
  static constexpr uint32_t OTG_GLPMCFG_SNDLPM_Pos     = ( 24U );
  static constexpr uint32_t OTG_GLPMCFG_SNDLPM_Msk     = ( 0x1UL << OTG_GLPMCFG_SNDLPM_Pos );
  static constexpr uint32_t OTG_GLPMCFG_SNDLPM         = OTG_GLPMCFG_SNDLPM_Msk;
  static constexpr uint32_t OTG_GLPMCFG_LPMRCNTSTS_Pos = ( 25U );
  static constexpr uint32_t OTG_GLPMCFG_LPMRCNTSTS_Msk = ( 0x7UL << OTG_GLPMCFG_LPMRCNTSTS_Pos );
  static constexpr uint32_t OTG_GLPMCFG_LPMRCNTSTS     = OTG_GLPMCFG_LPMRCNTSTS_Msk;
  static constexpr uint32_t OTG_GLPMCFG_ENBESL_Pos     = ( 28U );
  static constexpr uint32_t OTG_GLPMCFG_ENBESL_Msk     = ( 0x1UL << OTG_GLPMCFG_ENBESL_Pos );
  static constexpr uint32_t OTG_GLPMCFG_ENBESL         = OTG_GLPMCFG_ENBESL_Msk;

  /********************  Bit definition for OTG_DIEPEACHMSK1 register  ********************/
  static constexpr uint32_t OTG_DIEPEACHMSK1_XFRCM_Pos     = ( 0U );
  static constexpr uint32_t OTG_DIEPEACHMSK1_XFRCM_Msk     = ( 0x1UL << OTG_DIEPEACHMSK1_XFRCM_Pos );
  static constexpr uint32_t OTG_DIEPEACHMSK1_XFRCM         = OTG_DIEPEACHMSK1_XFRCM_Msk;
  static constexpr uint32_t OTG_DIEPEACHMSK1_EPDM_Pos      = ( 1U );
  static constexpr uint32_t OTG_DIEPEACHMSK1_EPDM_Msk      = ( 0x1UL << OTG_DIEPEACHMSK1_EPDM_Pos );
  static constexpr uint32_t OTG_DIEPEACHMSK1_EPDM          = OTG_DIEPEACHMSK1_EPDM_Msk;
  static constexpr uint32_t OTG_DIEPEACHMSK1_TOM_Pos       = ( 3U );
  static constexpr uint32_t OTG_DIEPEACHMSK1_TOM_Msk       = ( 0x1UL << OTG_DIEPEACHMSK1_TOM_Pos );
  static constexpr uint32_t OTG_DIEPEACHMSK1_TOM           = OTG_DIEPEACHMSK1_TOM_Msk;
  static constexpr uint32_t OTG_DIEPEACHMSK1_ITTXFEMSK_Pos = ( 4U );
  static constexpr uint32_t OTG_DIEPEACHMSK1_ITTXFEMSK_Msk = ( 0x1UL << OTG_DIEPEACHMSK1_ITTXFEMSK_Pos );
  static constexpr uint32_t OTG_DIEPEACHMSK1_ITTXFEMSK     = OTG_DIEPEACHMSK1_ITTXFEMSK_Msk;
  static constexpr uint32_t OTG_DIEPEACHMSK1_INEPNMM_Pos   = ( 5U );
  static constexpr uint32_t OTG_DIEPEACHMSK1_INEPNMM_Msk   = ( 0x1UL << OTG_DIEPEACHMSK1_INEPNMM_Pos );
  static constexpr uint32_t OTG_DIEPEACHMSK1_INEPNMM       = OTG_DIEPEACHMSK1_INEPNMM_Msk;
  static constexpr uint32_t OTG_DIEPEACHMSK1_INEPNEM_Pos   = ( 6U );
  static constexpr uint32_t OTG_DIEPEACHMSK1_INEPNEM_Msk   = ( 0x1UL << OTG_DIEPEACHMSK1_INEPNEM_Pos );
  static constexpr uint32_t OTG_DIEPEACHMSK1_INEPNEM       = OTG_DIEPEACHMSK1_INEPNEM_Msk;
  static constexpr uint32_t OTG_DIEPEACHMSK1_TXFURM_Pos    = ( 8U );
  static constexpr uint32_t OTG_DIEPEACHMSK1_TXFURM_Msk    = ( 0x1UL << OTG_DIEPEACHMSK1_TXFURM_Pos );
  static constexpr uint32_t OTG_DIEPEACHMSK1_TXFURM        = OTG_DIEPEACHMSK1_TXFURM_Msk;
  static constexpr uint32_t OTG_DIEPEACHMSK1_BIM_Pos       = ( 9U );
  static constexpr uint32_t OTG_DIEPEACHMSK1_BIM_Msk       = ( 0x1UL << OTG_DIEPEACHMSK1_BIM_Pos );
  static constexpr uint32_t OTG_DIEPEACHMSK1_BIM           = OTG_DIEPEACHMSK1_BIM_Msk;
  static constexpr uint32_t OTG_DIEPEACHMSK1_NAKM_Pos      = ( 13U );
  static constexpr uint32_t OTG_DIEPEACHMSK1_NAKM_Msk      = ( 0x1UL << OTG_DIEPEACHMSK1_NAKM_Pos );
  static constexpr uint32_t OTG_DIEPEACHMSK1_NAKM          = OTG_DIEPEACHMSK1_NAKM_Msk;

  /********************  Bit definition for OTG_HPRT register  ********************/
  static constexpr uint32_t OTG_HPRT_PCSTS_Pos   = ( 0U );
  static constexpr uint32_t OTG_HPRT_PCSTS_Msk   = ( 0x1UL << OTG_HPRT_PCSTS_Pos );
  static constexpr uint32_t OTG_HPRT_PCSTS       = OTG_HPRT_PCSTS_Msk;
  static constexpr uint32_t OTG_HPRT_PCDET_Pos   = ( 1U );
  static constexpr uint32_t OTG_HPRT_PCDET_Msk   = ( 0x1UL << OTG_HPRT_PCDET_Pos );
  static constexpr uint32_t OTG_HPRT_PCDET       = OTG_HPRT_PCDET_Msk;
  static constexpr uint32_t OTG_HPRT_PENA_Pos    = ( 2U );
  static constexpr uint32_t OTG_HPRT_PENA_Msk    = ( 0x1UL << OTG_HPRT_PENA_Pos );
  static constexpr uint32_t OTG_HPRT_PENA        = OTG_HPRT_PENA_Msk;
  static constexpr uint32_t OTG_HPRT_PENCHNG_Pos = ( 3U );
  static constexpr uint32_t OTG_HPRT_PENCHNG_Msk = ( 0x1UL << OTG_HPRT_PENCHNG_Pos );
  static constexpr uint32_t OTG_HPRT_PENCHNG     = OTG_HPRT_PENCHNG_Msk;
  static constexpr uint32_t OTG_HPRT_POCA_Pos    = ( 4U );
  static constexpr uint32_t OTG_HPRT_POCA_Msk    = ( 0x1UL << OTG_HPRT_POCA_Pos );
  static constexpr uint32_t OTG_HPRT_POCA        = OTG_HPRT_POCA_Msk;
  static constexpr uint32_t OTG_HPRT_POCCHNG_Pos = ( 5U );
  static constexpr uint32_t OTG_HPRT_POCCHNG_Msk = ( 0x1UL << OTG_HPRT_POCCHNG_Pos );
  static constexpr uint32_t OTG_HPRT_POCCHNG     = OTG_HPRT_POCCHNG_Msk;
  static constexpr uint32_t OTG_HPRT_PRES_Pos    = ( 6U );
  static constexpr uint32_t OTG_HPRT_PRES_Msk    = ( 0x1UL << OTG_HPRT_PRES_Pos );
  static constexpr uint32_t OTG_HPRT_PRES        = OTG_HPRT_PRES_Msk;
  static constexpr uint32_t OTG_HPRT_PSUSP_Pos   = ( 7U );
  static constexpr uint32_t OTG_HPRT_PSUSP_Msk   = ( 0x1UL << OTG_HPRT_PSUSP_Pos );
  static constexpr uint32_t OTG_HPRT_PSUSP       = OTG_HPRT_PSUSP_Msk;
  static constexpr uint32_t OTG_HPRT_PRST_Pos    = ( 8U );
  static constexpr uint32_t OTG_HPRT_PRST_Msk    = ( 0x1UL << OTG_HPRT_PRST_Pos );
  static constexpr uint32_t OTG_HPRT_PRST        = OTG_HPRT_PRST_Msk;

  static constexpr uint32_t OTG_HPRT_PLSTS_Pos = ( 10U );
  static constexpr uint32_t OTG_HPRT_PLSTS_Msk = ( 0x3UL << OTG_HPRT_PLSTS_Pos );
  static constexpr uint32_t OTG_HPRT_PLSTS     = OTG_HPRT_PLSTS_Msk;
  static constexpr uint32_t OTG_HPRT_PLSTS_0   = ( 0x1UL << OTG_HPRT_PLSTS_Pos );
  static constexpr uint32_t OTG_HPRT_PLSTS_1   = ( 0x2UL << OTG_HPRT_PLSTS_Pos );
  static constexpr uint32_t OTG_HPRT_PPWR_Pos  = ( 12U );
  static constexpr uint32_t OTG_HPRT_PPWR_Msk  = ( 0x1UL << OTG_HPRT_PPWR_Pos );
  static constexpr uint32_t OTG_HPRT_PPWR      = OTG_HPRT_PPWR_Msk;

  static constexpr uint32_t OTG_HPRT_PTCTL_Pos = ( 13U );
  static constexpr uint32_t OTG_HPRT_PTCTL_Msk = ( 0xFUL << OTG_HPRT_PTCTL_Pos );
  static constexpr uint32_t OTG_HPRT_PTCTL     = OTG_HPRT_PTCTL_Msk;
  static constexpr uint32_t OTG_HPRT_PTCTL_0   = ( 0x1UL << OTG_HPRT_PTCTL_Pos );
  static constexpr uint32_t OTG_HPRT_PTCTL_1   = ( 0x2UL << OTG_HPRT_PTCTL_Pos );
  static constexpr uint32_t OTG_HPRT_PTCTL_2   = ( 0x4UL << OTG_HPRT_PTCTL_Pos );
  static constexpr uint32_t OTG_HPRT_PTCTL_3   = ( 0x8UL << OTG_HPRT_PTCTL_Pos );

  static constexpr uint32_t OTG_HPRT_PSPD_Pos = ( 17U );
  static constexpr uint32_t OTG_HPRT_PSPD_Msk = ( 0x3UL << OTG_HPRT_PSPD_Pos );
  static constexpr uint32_t OTG_HPRT_PSPD     = OTG_HPRT_PSPD_Msk;
  static constexpr uint32_t OTG_HPRT_PSPD_0   = ( 0x1UL << OTG_HPRT_PSPD_Pos );
  static constexpr uint32_t OTG_HPRT_PSPD_1   = ( 0x2UL << OTG_HPRT_PSPD_Pos );

  /********************  Bit definition for OTG_DOEPEACHMSK1 register  ********************/
  static constexpr uint32_t OTG_DOEPEACHMSK1_XFRCM_Pos     = ( 0U );
  static constexpr uint32_t OTG_DOEPEACHMSK1_XFRCM_Msk     = ( 0x1UL << OTG_DOEPEACHMSK1_XFRCM_Pos );
  static constexpr uint32_t OTG_DOEPEACHMSK1_XFRCM         = OTG_DOEPEACHMSK1_XFRCM_Msk;
  static constexpr uint32_t OTG_DOEPEACHMSK1_EPDM_Pos      = ( 1U );
  static constexpr uint32_t OTG_DOEPEACHMSK1_EPDM_Msk      = ( 0x1UL << OTG_DOEPEACHMSK1_EPDM_Pos );
  static constexpr uint32_t OTG_DOEPEACHMSK1_EPDM          = OTG_DOEPEACHMSK1_EPDM_Msk;
  static constexpr uint32_t OTG_DOEPEACHMSK1_TOM_Pos       = ( 3U );
  static constexpr uint32_t OTG_DOEPEACHMSK1_TOM_Msk       = ( 0x1UL << OTG_DOEPEACHMSK1_TOM_Pos );
  static constexpr uint32_t OTG_DOEPEACHMSK1_TOM           = OTG_DOEPEACHMSK1_TOM_Msk;
  static constexpr uint32_t OTG_DOEPEACHMSK1_ITTXFEMSK_Pos = ( 4U );
  static constexpr uint32_t OTG_DOEPEACHMSK1_ITTXFEMSK_Msk = ( 0x1UL << OTG_DOEPEACHMSK1_ITTXFEMSK_Pos );
  static constexpr uint32_t OTG_DOEPEACHMSK1_ITTXFEMSK     = OTG_DOEPEACHMSK1_ITTXFEMSK_Msk;
  static constexpr uint32_t OTG_DOEPEACHMSK1_INEPNMM_Pos   = ( 5U );
  static constexpr uint32_t OTG_DOEPEACHMSK1_INEPNMM_Msk   = ( 0x1UL << OTG_DOEPEACHMSK1_INEPNMM_Pos );
  static constexpr uint32_t OTG_DOEPEACHMSK1_INEPNMM       = OTG_DOEPEACHMSK1_INEPNMM_Msk;
  static constexpr uint32_t OTG_DOEPEACHMSK1_INEPNEM_Pos   = ( 6U );
  static constexpr uint32_t OTG_DOEPEACHMSK1_INEPNEM_Msk   = ( 0x1UL << OTG_DOEPEACHMSK1_INEPNEM_Pos );
  static constexpr uint32_t OTG_DOEPEACHMSK1_INEPNEM       = OTG_DOEPEACHMSK1_INEPNEM_Msk;
  static constexpr uint32_t OTG_DOEPEACHMSK1_TXFURM_Pos    = ( 8U );
  static constexpr uint32_t OTG_DOEPEACHMSK1_TXFURM_Msk    = ( 0x1UL << OTG_DOEPEACHMSK1_TXFURM_Pos );
  static constexpr uint32_t OTG_DOEPEACHMSK1_TXFURM        = OTG_DOEPEACHMSK1_TXFURM_Msk;
  static constexpr uint32_t OTG_DOEPEACHMSK1_BIM_Pos       = ( 9U );
  static constexpr uint32_t OTG_DOEPEACHMSK1_BIM_Msk       = ( 0x1UL << OTG_DOEPEACHMSK1_BIM_Pos );
  static constexpr uint32_t OTG_DOEPEACHMSK1_BIM           = OTG_DOEPEACHMSK1_BIM_Msk;
  static constexpr uint32_t OTG_DOEPEACHMSK1_BERRM_Pos     = ( 12U );
  static constexpr uint32_t OTG_DOEPEACHMSK1_BERRM_Msk     = ( 0x1UL << OTG_DOEPEACHMSK1_BERRM_Pos );
  static constexpr uint32_t OTG_DOEPEACHMSK1_BERRM         = OTG_DOEPEACHMSK1_BERRM_Msk;
  static constexpr uint32_t OTG_DOEPEACHMSK1_NAKM_Pos      = ( 13U );
  static constexpr uint32_t OTG_DOEPEACHMSK1_NAKM_Msk      = ( 0x1UL << OTG_DOEPEACHMSK1_NAKM_Pos );
  static constexpr uint32_t OTG_DOEPEACHMSK1_NAKM          = OTG_DOEPEACHMSK1_NAKM_Msk;
  static constexpr uint32_t OTG_DOEPEACHMSK1_NYETM_Pos     = ( 14U );
  static constexpr uint32_t OTG_DOEPEACHMSK1_NYETM_Msk     = ( 0x1UL << OTG_DOEPEACHMSK1_NYETM_Pos );
  static constexpr uint32_t OTG_DOEPEACHMSK1_NYETM         = OTG_DOEPEACHMSK1_NYETM_Msk;

  /********************  Bit definition for OTG_HPTXFSIZ register  ********************/
  static constexpr uint32_t OTG_HPTXFSIZ_PTXSA_Pos = ( 0U );
  static constexpr uint32_t OTG_HPTXFSIZ_PTXSA_Msk = ( 0xFFFFUL << OTG_HPTXFSIZ_PTXSA_Pos );
  static constexpr uint32_t OTG_HPTXFSIZ_PTXSA     = OTG_HPTXFSIZ_PTXSA_Msk;
  static constexpr uint32_t OTG_HPTXFSIZ_PTXFD_Pos = ( 16U );
  static constexpr uint32_t OTG_HPTXFSIZ_PTXFD_Msk = ( 0xFFFFUL << OTG_HPTXFSIZ_PTXFD_Pos );
  static constexpr uint32_t OTG_HPTXFSIZ_PTXFD     = OTG_HPTXFSIZ_PTXFD_Msk;

  /********************  Bit definition for OTG_DIEPCTL register  ********************/
  static constexpr uint32_t OTG_DIEPCTL_MPSIZ_Pos      = ( 0U );
  static constexpr uint32_t OTG_DIEPCTL_MPSIZ_Msk      = ( 0x7FFUL << OTG_DIEPCTL_MPSIZ_Pos );
  static constexpr uint32_t OTG_DIEPCTL_MPSIZ          = OTG_DIEPCTL_MPSIZ_Msk;
  static constexpr uint32_t OTG_DIEPCTL_USBAEP_Pos     = ( 15U );
  static constexpr uint32_t OTG_DIEPCTL_USBAEP_Msk     = ( 0x1UL << OTG_DIEPCTL_USBAEP_Pos );
  static constexpr uint32_t OTG_DIEPCTL_USBAEP         = OTG_DIEPCTL_USBAEP_Msk;
  static constexpr uint32_t OTG_DIEPCTL_EONUM_DPID_Pos = ( 16U );
  static constexpr uint32_t OTG_DIEPCTL_EONUM_DPID_Msk = ( 0x1UL << OTG_DIEPCTL_EONUM_DPID_Pos );
  static constexpr uint32_t OTG_DIEPCTL_EONUM_DPID     = OTG_DIEPCTL_EONUM_DPID_Msk;
  static constexpr uint32_t OTG_DIEPCTL_NAKSTS_Pos     = ( 17U );
  static constexpr uint32_t OTG_DIEPCTL_NAKSTS_Msk     = ( 0x1UL << OTG_DIEPCTL_NAKSTS_Pos );
  static constexpr uint32_t OTG_DIEPCTL_NAKSTS         = OTG_DIEPCTL_NAKSTS_Msk;

  static constexpr uint32_t OTG_DIEPCTL_EPTYP_Pos = ( 18U );
  static constexpr uint32_t OTG_DIEPCTL_EPTYP_Msk = ( 0x3UL << OTG_DIEPCTL_EPTYP_Pos );
  static constexpr uint32_t OTG_DIEPCTL_EPTYP     = OTG_DIEPCTL_EPTYP_Msk;
  static constexpr uint32_t OTG_DIEPCTL_EPTYP_0   = ( 0x1UL << OTG_DIEPCTL_EPTYP_Pos );
  static constexpr uint32_t OTG_DIEPCTL_EPTYP_1   = ( 0x2UL << OTG_DIEPCTL_EPTYP_Pos );
  static constexpr uint32_t OTG_DIEPCTL_STALL_Pos = ( 21U );
  static constexpr uint32_t OTG_DIEPCTL_STALL_Msk = ( 0x1UL << OTG_DIEPCTL_STALL_Pos );
  static constexpr uint32_t OTG_DIEPCTL_STALL     = OTG_DIEPCTL_STALL_Msk;

  static constexpr uint32_t OTG_DIEPCTL_TXFNUM_Pos         = ( 22U );
  static constexpr uint32_t OTG_DIEPCTL_TXFNUM_Msk         = ( 0xFUL << OTG_DIEPCTL_TXFNUM_Pos );
  static constexpr uint32_t OTG_DIEPCTL_TXFNUM             = OTG_DIEPCTL_TXFNUM_Msk;
  static constexpr uint32_t OTG_DIEPCTL_TXFNUM_0           = ( 0x1UL << OTG_DIEPCTL_TXFNUM_Pos );
  static constexpr uint32_t OTG_DIEPCTL_TXFNUM_1           = ( 0x2UL << OTG_DIEPCTL_TXFNUM_Pos );
  static constexpr uint32_t OTG_DIEPCTL_TXFNUM_2           = ( 0x4UL << OTG_DIEPCTL_TXFNUM_Pos );
  static constexpr uint32_t OTG_DIEPCTL_TXFNUM_3           = ( 0x8UL << OTG_DIEPCTL_TXFNUM_Pos );
  static constexpr uint32_t OTG_DIEPCTL_CNAK_Pos           = ( 26U );
  static constexpr uint32_t OTG_DIEPCTL_CNAK_Msk           = ( 0x1UL << OTG_DIEPCTL_CNAK_Pos );
  static constexpr uint32_t OTG_DIEPCTL_CNAK               = OTG_DIEPCTL_CNAK_Msk;
  static constexpr uint32_t OTG_DIEPCTL_SNAK_Pos           = ( 27U );
  static constexpr uint32_t OTG_DIEPCTL_SNAK_Msk           = ( 0x1UL << OTG_DIEPCTL_SNAK_Pos );
  static constexpr uint32_t OTG_DIEPCTL_SNAK               = OTG_DIEPCTL_SNAK_Msk;
  static constexpr uint32_t OTG_DIEPCTL_SD0PID_SEVNFRM_Pos = ( 28U );
  static constexpr uint32_t OTG_DIEPCTL_SD0PID_SEVNFRM_Msk = ( 0x1UL << OTG_DIEPCTL_SD0PID_SEVNFRM_Pos );
  static constexpr uint32_t OTG_DIEPCTL_SD0PID_SEVNFRM     = OTG_DIEPCTL_SD0PID_SEVNFRM_Msk;
  static constexpr uint32_t OTG_DIEPCTL_SODDFRM_Pos        = ( 29U );
  static constexpr uint32_t OTG_DIEPCTL_SODDFRM_Msk        = ( 0x1UL << OTG_DIEPCTL_SODDFRM_Pos );
  static constexpr uint32_t OTG_DIEPCTL_SODDFRM            = OTG_DIEPCTL_SODDFRM_Msk;
  static constexpr uint32_t OTG_DIEPCTL_EPDIS_Pos          = ( 30U );
  static constexpr uint32_t OTG_DIEPCTL_EPDIS_Msk          = ( 0x1UL << OTG_DIEPCTL_EPDIS_Pos );
  static constexpr uint32_t OTG_DIEPCTL_EPDIS              = OTG_DIEPCTL_EPDIS_Msk;
  static constexpr uint32_t OTG_DIEPCTL_EPENA_Pos          = ( 31U );
  static constexpr uint32_t OTG_DIEPCTL_EPENA_Msk          = ( 0x1UL << OTG_DIEPCTL_EPENA_Pos );
  static constexpr uint32_t OTG_DIEPCTL_EPENA              = OTG_DIEPCTL_EPENA_Msk;

  /********************  Bit definition for OTG_HCCHAR register  ********************/
  static constexpr uint32_t OTG_HCCHAR_MPSIZ_Pos = ( 0U );
  static constexpr uint32_t OTG_HCCHAR_MPSIZ_Msk = ( 0x7FFUL << OTG_HCCHAR_MPSIZ_Pos );
  static constexpr uint32_t OTG_HCCHAR_MPSIZ     = OTG_HCCHAR_MPSIZ_Msk;

  static constexpr uint32_t OTG_HCCHAR_EPNUM_Pos = ( 11U );
  static constexpr uint32_t OTG_HCCHAR_EPNUM_Msk = ( 0xFUL << OTG_HCCHAR_EPNUM_Pos );
  static constexpr uint32_t OTG_HCCHAR_EPNUM     = OTG_HCCHAR_EPNUM_Msk;
  static constexpr uint32_t OTG_HCCHAR_EPNUM_0   = ( 0x1UL << OTG_HCCHAR_EPNUM_Pos );
  static constexpr uint32_t OTG_HCCHAR_EPNUM_1   = ( 0x2UL << OTG_HCCHAR_EPNUM_Pos );
  static constexpr uint32_t OTG_HCCHAR_EPNUM_2   = ( 0x4UL << OTG_HCCHAR_EPNUM_Pos );
  static constexpr uint32_t OTG_HCCHAR_EPNUM_3   = ( 0x8UL << OTG_HCCHAR_EPNUM_Pos );
  static constexpr uint32_t OTG_HCCHAR_EPDIR_Pos = ( 15U );
  static constexpr uint32_t OTG_HCCHAR_EPDIR_Msk = ( 0x1UL << OTG_HCCHAR_EPDIR_Pos );
  static constexpr uint32_t OTG_HCCHAR_EPDIR     = OTG_HCCHAR_EPDIR_Msk;
  static constexpr uint32_t OTG_HCCHAR_LSDEV_Pos = ( 17U );
  static constexpr uint32_t OTG_HCCHAR_LSDEV_Msk = ( 0x1UL << OTG_HCCHAR_LSDEV_Pos );
  static constexpr uint32_t OTG_HCCHAR_LSDEV     = OTG_HCCHAR_LSDEV_Msk;

  static constexpr uint32_t OTG_HCCHAR_EPTYP_Pos = ( 18U );
  static constexpr uint32_t OTG_HCCHAR_EPTYP_Msk = ( 0x3UL << OTG_HCCHAR_EPTYP_Pos );
  static constexpr uint32_t OTG_HCCHAR_EPTYP     = OTG_HCCHAR_EPTYP_Msk;
  static constexpr uint32_t OTG_HCCHAR_EPTYP_0   = ( 0x1UL << OTG_HCCHAR_EPTYP_Pos );
  static constexpr uint32_t OTG_HCCHAR_EPTYP_1   = ( 0x2UL << OTG_HCCHAR_EPTYP_Pos );

  static constexpr uint32_t OTG_HCCHAR_MC_Pos = ( 20U );
  static constexpr uint32_t OTG_HCCHAR_MC_Msk = ( 0x3UL << OTG_HCCHAR_MC_Pos );
  static constexpr uint32_t OTG_HCCHAR_MC     = OTG_HCCHAR_MC_Msk;
  static constexpr uint32_t OTG_HCCHAR_MC_0   = ( 0x1UL << OTG_HCCHAR_MC_Pos );
  static constexpr uint32_t OTG_HCCHAR_MC_1   = ( 0x2UL << OTG_HCCHAR_MC_Pos );

  static constexpr uint32_t OTG_HCCHAR_DAD_Pos    = ( 22U );
  static constexpr uint32_t OTG_HCCHAR_DAD_Msk    = ( 0x7FUL << OTG_HCCHAR_DAD_Pos );
  static constexpr uint32_t OTG_HCCHAR_DAD        = OTG_HCCHAR_DAD_Msk;
  static constexpr uint32_t OTG_HCCHAR_DAD_0      = ( 0x01UL << OTG_HCCHAR_DAD_Pos );
  static constexpr uint32_t OTG_HCCHAR_DAD_1      = ( 0x02UL << OTG_HCCHAR_DAD_Pos );
  static constexpr uint32_t OTG_HCCHAR_DAD_2      = ( 0x04UL << OTG_HCCHAR_DAD_Pos );
  static constexpr uint32_t OTG_HCCHAR_DAD_3      = ( 0x08UL << OTG_HCCHAR_DAD_Pos );
  static constexpr uint32_t OTG_HCCHAR_DAD_4      = ( 0x10UL << OTG_HCCHAR_DAD_Pos );
  static constexpr uint32_t OTG_HCCHAR_DAD_5      = ( 0x20UL << OTG_HCCHAR_DAD_Pos );
  static constexpr uint32_t OTG_HCCHAR_DAD_6      = ( 0x40UL << OTG_HCCHAR_DAD_Pos );
  static constexpr uint32_t OTG_HCCHAR_ODDFRM_Pos = ( 29U );
  static constexpr uint32_t OTG_HCCHAR_ODDFRM_Msk = ( 0x1UL << OTG_HCCHAR_ODDFRM_Pos );
  static constexpr uint32_t OTG_HCCHAR_ODDFRM     = OTG_HCCHAR_ODDFRM_Msk;
  static constexpr uint32_t OTG_HCCHAR_CHDIS_Pos  = ( 30U );
  static constexpr uint32_t OTG_HCCHAR_CHDIS_Msk  = ( 0x1UL << OTG_HCCHAR_CHDIS_Pos );
  static constexpr uint32_t OTG_HCCHAR_CHDIS      = OTG_HCCHAR_CHDIS_Msk;
  static constexpr uint32_t OTG_HCCHAR_CHENA_Pos  = ( 31U );
  static constexpr uint32_t OTG_HCCHAR_CHENA_Msk  = ( 0x1UL << OTG_HCCHAR_CHENA_Pos );
  static constexpr uint32_t OTG_HCCHAR_CHENA      = OTG_HCCHAR_CHENA_Msk;

  /********************  Bit definition for OTG_HCSPLT register  ********************/

  static constexpr uint32_t OTG_HCSPLT_PRTADDR_Pos = ( 0U );
  static constexpr uint32_t OTG_HCSPLT_PRTADDR_Msk = ( 0x7FUL << OTG_HCSPLT_PRTADDR_Pos );
  static constexpr uint32_t OTG_HCSPLT_PRTADDR     = OTG_HCSPLT_PRTADDR_Msk;
  static constexpr uint32_t OTG_HCSPLT_PRTADDR_0   = ( 0x01UL << OTG_HCSPLT_PRTADDR_Pos );
  static constexpr uint32_t OTG_HCSPLT_PRTADDR_1   = ( 0x02UL << OTG_HCSPLT_PRTADDR_Pos );
  static constexpr uint32_t OTG_HCSPLT_PRTADDR_2   = ( 0x04UL << OTG_HCSPLT_PRTADDR_Pos );
  static constexpr uint32_t OTG_HCSPLT_PRTADDR_3   = ( 0x08UL << OTG_HCSPLT_PRTADDR_Pos );
  static constexpr uint32_t OTG_HCSPLT_PRTADDR_4   = ( 0x10UL << OTG_HCSPLT_PRTADDR_Pos );
  static constexpr uint32_t OTG_HCSPLT_PRTADDR_5   = ( 0x20UL << OTG_HCSPLT_PRTADDR_Pos );
  static constexpr uint32_t OTG_HCSPLT_PRTADDR_6   = ( 0x40UL << OTG_HCSPLT_PRTADDR_Pos );

  static constexpr uint32_t OTG_HCSPLT_HUBADDR_Pos = ( 7U );
  static constexpr uint32_t OTG_HCSPLT_HUBADDR_Msk = ( 0x7FUL << OTG_HCSPLT_HUBADDR_Pos );
  static constexpr uint32_t OTG_HCSPLT_HUBADDR     = OTG_HCSPLT_HUBADDR_Msk;
  static constexpr uint32_t OTG_HCSPLT_HUBADDR_0   = ( 0x01UL << OTG_HCSPLT_HUBADDR_Pos );
  static constexpr uint32_t OTG_HCSPLT_HUBADDR_1   = ( 0x02UL << OTG_HCSPLT_HUBADDR_Pos );
  static constexpr uint32_t OTG_HCSPLT_HUBADDR_2   = ( 0x04UL << OTG_HCSPLT_HUBADDR_Pos );
  static constexpr uint32_t OTG_HCSPLT_HUBADDR_3   = ( 0x08UL << OTG_HCSPLT_HUBADDR_Pos );
  static constexpr uint32_t OTG_HCSPLT_HUBADDR_4   = ( 0x10UL << OTG_HCSPLT_HUBADDR_Pos );
  static constexpr uint32_t OTG_HCSPLT_HUBADDR_5   = ( 0x20UL << OTG_HCSPLT_HUBADDR_Pos );
  static constexpr uint32_t OTG_HCSPLT_HUBADDR_6   = ( 0x40UL << OTG_HCSPLT_HUBADDR_Pos );

  static constexpr uint32_t OTG_HCSPLT_XACTPOS_Pos   = ( 14U );
  static constexpr uint32_t OTG_HCSPLT_XACTPOS_Msk   = ( 0x3UL << OTG_HCSPLT_XACTPOS_Pos );
  static constexpr uint32_t OTG_HCSPLT_XACTPOS       = OTG_HCSPLT_XACTPOS_Msk;
  static constexpr uint32_t OTG_HCSPLT_XACTPOS_0     = ( 0x1UL << OTG_HCSPLT_XACTPOS_Pos );
  static constexpr uint32_t OTG_HCSPLT_XACTPOS_1     = ( 0x2UL << OTG_HCSPLT_XACTPOS_Pos );
  static constexpr uint32_t OTG_HCSPLT_COMPLSPLT_Pos = ( 16U );
  static constexpr uint32_t OTG_HCSPLT_COMPLSPLT_Msk = ( 0x1UL << OTG_HCSPLT_COMPLSPLT_Pos );
  static constexpr uint32_t OTG_HCSPLT_COMPLSPLT     = OTG_HCSPLT_COMPLSPLT_Msk;
  static constexpr uint32_t OTG_HCSPLT_SPLITEN_Pos   = ( 31U );
  static constexpr uint32_t OTG_HCSPLT_SPLITEN_Msk   = ( 0x1UL << OTG_HCSPLT_SPLITEN_Pos );
  static constexpr uint32_t OTG_HCSPLT_SPLITEN       = OTG_HCSPLT_SPLITEN_Msk;

  /********************  Bit definition for OTG_HCINT register  ********************/
  static constexpr uint32_t OTG_HCINT_XFRC_Pos   = ( 0U );
  static constexpr uint32_t OTG_HCINT_XFRC_Msk   = ( 0x1UL << OTG_HCINT_XFRC_Pos );
  static constexpr uint32_t OTG_HCINT_XFRC       = OTG_HCINT_XFRC_Msk;
  static constexpr uint32_t OTG_HCINT_CHH_Pos    = ( 1U );
  static constexpr uint32_t OTG_HCINT_CHH_Msk    = ( 0x1UL << OTG_HCINT_CHH_Pos );
  static constexpr uint32_t OTG_HCINT_CHH        = OTG_HCINT_CHH_Msk;
  static constexpr uint32_t OTG_HCINT_AHBERR_Pos = ( 2U );
  static constexpr uint32_t OTG_HCINT_AHBERR_Msk = ( 0x1UL << OTG_HCINT_AHBERR_Pos );
  static constexpr uint32_t OTG_HCINT_AHBERR     = OTG_HCINT_AHBERR_Msk;
  static constexpr uint32_t OTG_HCINT_STALL_Pos  = ( 3U );
  static constexpr uint32_t OTG_HCINT_STALL_Msk  = ( 0x1UL << OTG_HCINT_STALL_Pos );
  static constexpr uint32_t OTG_HCINT_STALL      = OTG_HCINT_STALL_Msk;
  static constexpr uint32_t OTG_HCINT_NAK_Pos    = ( 4U );
  static constexpr uint32_t OTG_HCINT_NAK_Msk    = ( 0x1UL << OTG_HCINT_NAK_Pos );
  static constexpr uint32_t OTG_HCINT_NAK        = OTG_HCINT_NAK_Msk;
  static constexpr uint32_t OTG_HCINT_ACK_Pos    = ( 5U );
  static constexpr uint32_t OTG_HCINT_ACK_Msk    = ( 0x1UL << OTG_HCINT_ACK_Pos );
  static constexpr uint32_t OTG_HCINT_ACK        = OTG_HCINT_ACK_Msk;
  static constexpr uint32_t OTG_HCINT_NYET_Pos   = ( 6U );
  static constexpr uint32_t OTG_HCINT_NYET_Msk   = ( 0x1UL << OTG_HCINT_NYET_Pos );
  static constexpr uint32_t OTG_HCINT_NYET       = OTG_HCINT_NYET_Msk;
  static constexpr uint32_t OTG_HCINT_TXERR_Pos  = ( 7U );
  static constexpr uint32_t OTG_HCINT_TXERR_Msk  = ( 0x1UL << OTG_HCINT_TXERR_Pos );
  static constexpr uint32_t OTG_HCINT_TXERR      = OTG_HCINT_TXERR_Msk;
  static constexpr uint32_t OTG_HCINT_BBERR_Pos  = ( 8U );
  static constexpr uint32_t OTG_HCINT_BBERR_Msk  = ( 0x1UL << OTG_HCINT_BBERR_Pos );
  static constexpr uint32_t OTG_HCINT_BBERR      = OTG_HCINT_BBERR_Msk;
  static constexpr uint32_t OTG_HCINT_FRMOR_Pos  = ( 9U );
  static constexpr uint32_t OTG_HCINT_FRMOR_Msk  = ( 0x1UL << OTG_HCINT_FRMOR_Pos );
  static constexpr uint32_t OTG_HCINT_FRMOR      = OTG_HCINT_FRMOR_Msk;
  static constexpr uint32_t OTG_HCINT_DTERR_Pos  = ( 10U );
  static constexpr uint32_t OTG_HCINT_DTERR_Msk  = ( 0x1UL << OTG_HCINT_DTERR_Pos );
  static constexpr uint32_t OTG_HCINT_DTERR      = OTG_HCINT_DTERR_Msk;

  /********************  Bit definition for OTG_DIEPINT register  ********************/
  static constexpr uint32_t OTG_DIEPINT_XFRC_Pos       = ( 0U );
  static constexpr uint32_t OTG_DIEPINT_XFRC_Msk       = ( 0x1UL << OTG_DIEPINT_XFRC_Pos );
  static constexpr uint32_t OTG_DIEPINT_XFRC           = OTG_DIEPINT_XFRC_Msk;
  static constexpr uint32_t OTG_DIEPINT_EPDISD_Pos     = ( 1U );
  static constexpr uint32_t OTG_DIEPINT_EPDISD_Msk     = ( 0x1UL << OTG_DIEPINT_EPDISD_Pos );
  static constexpr uint32_t OTG_DIEPINT_EPDISD         = OTG_DIEPINT_EPDISD_Msk;
  static constexpr uint32_t OTG_DIEPINT_AHBERR_Pos     = ( 2U );
  static constexpr uint32_t OTG_DIEPINT_AHBERR_Msk     = ( 0x1UL << OTG_DIEPINT_AHBERR_Pos );
  static constexpr uint32_t OTG_DIEPINT_AHBERR         = OTG_DIEPINT_AHBERR_Msk;
  static constexpr uint32_t OTG_DIEPINT_TOC_Pos        = ( 3U );
  static constexpr uint32_t OTG_DIEPINT_TOC_Msk        = ( 0x1UL << OTG_DIEPINT_TOC_Pos );
  static constexpr uint32_t OTG_DIEPINT_TOC            = OTG_DIEPINT_TOC_Msk;
  static constexpr uint32_t OTG_DIEPINT_ITTXFE_Pos     = ( 4U );
  static constexpr uint32_t OTG_DIEPINT_ITTXFE_Msk     = ( 0x1UL << OTG_DIEPINT_ITTXFE_Pos );
  static constexpr uint32_t OTG_DIEPINT_ITTXFE         = OTG_DIEPINT_ITTXFE_Msk;
  static constexpr uint32_t OTG_DIEPINT_INEPNM_Pos     = ( 5U );
  static constexpr uint32_t OTG_DIEPINT_INEPNM_Msk     = ( 0x1UL << OTG_DIEPINT_INEPNM_Pos );
  static constexpr uint32_t OTG_DIEPINT_INEPNM         = OTG_DIEPINT_INEPNM_Msk;
  static constexpr uint32_t OTG_DIEPINT_INEPNE_Pos     = ( 6U );
  static constexpr uint32_t OTG_DIEPINT_INEPNE_Msk     = ( 0x1UL << OTG_DIEPINT_INEPNE_Pos );
  static constexpr uint32_t OTG_DIEPINT_INEPNE         = OTG_DIEPINT_INEPNE_Msk;
  static constexpr uint32_t OTG_DIEPINT_TXFE_Pos       = ( 7U );
  static constexpr uint32_t OTG_DIEPINT_TXFE_Msk       = ( 0x1UL << OTG_DIEPINT_TXFE_Pos );
  static constexpr uint32_t OTG_DIEPINT_TXFE           = OTG_DIEPINT_TXFE_Msk;
  static constexpr uint32_t OTG_DIEPINT_TXFIFOUDRN_Pos = ( 8U );
  static constexpr uint32_t OTG_DIEPINT_TXFIFOUDRN_Msk = ( 0x1UL << OTG_DIEPINT_TXFIFOUDRN_Pos );
  static constexpr uint32_t OTG_DIEPINT_TXFIFOUDRN     = OTG_DIEPINT_TXFIFOUDRN_Msk;
  static constexpr uint32_t OTG_DIEPINT_BNA_Pos        = ( 9U );
  static constexpr uint32_t OTG_DIEPINT_BNA_Msk        = ( 0x1UL << OTG_DIEPINT_BNA_Pos );
  static constexpr uint32_t OTG_DIEPINT_BNA            = OTG_DIEPINT_BNA_Msk;
  static constexpr uint32_t OTG_DIEPINT_PKTDRPSTS_Pos  = ( 11U );
  static constexpr uint32_t OTG_DIEPINT_PKTDRPSTS_Msk  = ( 0x1UL << OTG_DIEPINT_PKTDRPSTS_Pos );
  static constexpr uint32_t OTG_DIEPINT_PKTDRPSTS      = OTG_DIEPINT_PKTDRPSTS_Msk;
  static constexpr uint32_t OTG_DIEPINT_BERR_Pos       = ( 12U );
  static constexpr uint32_t OTG_DIEPINT_BERR_Msk       = ( 0x1UL << OTG_DIEPINT_BERR_Pos );
  static constexpr uint32_t OTG_DIEPINT_BERR           = OTG_DIEPINT_BERR_Msk;
  static constexpr uint32_t OTG_DIEPINT_NAK_Pos        = ( 13U );
  static constexpr uint32_t OTG_DIEPINT_NAK_Msk        = ( 0x1UL << OTG_DIEPINT_NAK_Pos );
  static constexpr uint32_t OTG_DIEPINT_NAK            = OTG_DIEPINT_NAK_Msk;

  /********************  Bit definition forOTG_HCINTMSK register  ********************/
  static constexpr uint32_t OTG_HCINTMSK_XFRCM_Pos  = ( 0U );
  static constexpr uint32_t OTG_HCINTMSK_XFRCM_Msk  = ( 0x1UL << OTG_HCINTMSK_XFRCM_Pos );
  static constexpr uint32_t OTG_HCINTMSK_XFRCM      = OTG_HCINTMSK_XFRCM_Msk;
  static constexpr uint32_t OTG_HCINTMSK_CHHM_Pos   = ( 1U );
  static constexpr uint32_t OTG_HCINTMSK_CHHM_Msk   = ( 0x1UL << OTG_HCINTMSK_CHHM_Pos );
  static constexpr uint32_t OTG_HCINTMSK_CHHM       = OTG_HCINTMSK_CHHM_Msk;
  static constexpr uint32_t OTG_HCINTMSK_AHBERR_Pos = ( 2U );
  static constexpr uint32_t OTG_HCINTMSK_AHBERR_Msk = ( 0x1UL << OTG_HCINTMSK_AHBERR_Pos );
  static constexpr uint32_t OTG_HCINTMSK_AHBERR     = OTG_HCINTMSK_AHBERR_Msk;
  static constexpr uint32_t OTG_HCINTMSK_STALLM_Pos = ( 3U );
  static constexpr uint32_t OTG_HCINTMSK_STALLM_Msk = ( 0x1UL << OTG_HCINTMSK_STALLM_Pos );
  static constexpr uint32_t OTG_HCINTMSK_STALLM     = OTG_HCINTMSK_STALLM_Msk;
  static constexpr uint32_t OTG_HCINTMSK_NAKM_Pos   = ( 4U );
  static constexpr uint32_t OTG_HCINTMSK_NAKM_Msk   = ( 0x1UL << OTG_HCINTMSK_NAKM_Pos );
  static constexpr uint32_t OTG_HCINTMSK_NAKM       = OTG_HCINTMSK_NAKM_Msk;
  static constexpr uint32_t OTG_HCINTMSK_ACKM_Pos   = ( 5U );
  static constexpr uint32_t OTG_HCINTMSK_ACKM_Msk   = ( 0x1UL << OTG_HCINTMSK_ACKM_Pos );
  static constexpr uint32_t OTG_HCINTMSK_ACKM       = OTG_HCINTMSK_ACKM_Msk;
  static constexpr uint32_t OTG_HCINTMSK_NYET_Pos   = ( 6U );
  static constexpr uint32_t OTG_HCINTMSK_NYET_Msk   = ( 0x1UL << OTG_HCINTMSK_NYET_Pos );
  static constexpr uint32_t OTG_HCINTMSK_NYET       = OTG_HCINTMSK_NYET_Msk;
  static constexpr uint32_t OTG_HCINTMSK_TXERRM_Pos = ( 7U );
  static constexpr uint32_t OTG_HCINTMSK_TXERRM_Msk = ( 0x1UL << OTG_HCINTMSK_TXERRM_Pos );
  static constexpr uint32_t OTG_HCINTMSK_TXERRM     = OTG_HCINTMSK_TXERRM_Msk;
  static constexpr uint32_t OTG_HCINTMSK_BBERRM_Pos = ( 8U );
  static constexpr uint32_t OTG_HCINTMSK_BBERRM_Msk = ( 0x1UL << OTG_HCINTMSK_BBERRM_Pos );
  static constexpr uint32_t OTG_HCINTMSK_BBERRM     = OTG_HCINTMSK_BBERRM_Msk;
  static constexpr uint32_t OTG_HCINTMSK_FRMORM_Pos = ( 9U );
  static constexpr uint32_t OTG_HCINTMSK_FRMORM_Msk = ( 0x1UL << OTG_HCINTMSK_FRMORM_Pos );
  static constexpr uint32_t OTG_HCINTMSK_FRMORM     = OTG_HCINTMSK_FRMORM_Msk;
  static constexpr uint32_t OTG_HCINTMSK_DTERRM_Pos = ( 10U );
  static constexpr uint32_t OTG_HCINTMSK_DTERRM_Msk = ( 0x1UL << OTG_HCINTMSK_DTERRM_Pos );
  static constexpr uint32_t OTG_HCINTMSK_DTERRM     = OTG_HCINTMSK_DTERRM_Msk;

  /********************  Bit definition for OTG_DIEPTSIZ register  ********************/

  static constexpr uint32_t OTG_DIEPTSIZ_XFRSIZ_Pos = ( 0U );
  static constexpr uint32_t OTG_DIEPTSIZ_XFRSIZ_Msk = ( 0x7FFFFUL << OTG_DIEPTSIZ_XFRSIZ_Pos );
  static constexpr uint32_t OTG_DIEPTSIZ_XFRSIZ     = OTG_DIEPTSIZ_XFRSIZ_Msk;
  static constexpr uint32_t OTG_DIEPTSIZ_PKTCNT_Pos = ( 19U );
  static constexpr uint32_t OTG_DIEPTSIZ_PKTCNT_Msk = ( 0x3FFUL << OTG_DIEPTSIZ_PKTCNT_Pos );
  static constexpr uint32_t OTG_DIEPTSIZ_PKTCNT     = OTG_DIEPTSIZ_PKTCNT_Msk;
  static constexpr uint32_t OTG_DIEPTSIZ_MULCNT_Pos = ( 29U );
  static constexpr uint32_t OTG_DIEPTSIZ_MULCNT_Msk = ( 0x3UL << OTG_DIEPTSIZ_MULCNT_Pos );
  static constexpr uint32_t OTG_DIEPTSIZ_MULCNT     = OTG_DIEPTSIZ_MULCNT_Msk;
  /********************  Bit definition for OTG_HCTSIZ register  ********************/
  static constexpr uint32_t OTG_HCTSIZ_XFRSIZ_Pos = ( 0U );
  static constexpr uint32_t OTG_HCTSIZ_XFRSIZ_Msk = ( 0x7FFFFUL << OTG_HCTSIZ_XFRSIZ_Pos );
  static constexpr uint32_t OTG_HCTSIZ_XFRSIZ     = OTG_HCTSIZ_XFRSIZ_Msk;
  static constexpr uint32_t OTG_HCTSIZ_PKTCNT_Pos = ( 19U );
  static constexpr uint32_t OTG_HCTSIZ_PKTCNT_Msk = ( 0x3FFUL << OTG_HCTSIZ_PKTCNT_Pos );
  static constexpr uint32_t OTG_HCTSIZ_PKTCNT     = OTG_HCTSIZ_PKTCNT_Msk;
  static constexpr uint32_t OTG_HCTSIZ_DOPING_Pos = ( 31U );
  static constexpr uint32_t OTG_HCTSIZ_DOPING_Msk = ( 0x1UL << OTG_HCTSIZ_DOPING_Pos );
  static constexpr uint32_t OTG_HCTSIZ_DOPING     = OTG_HCTSIZ_DOPING_Msk;
  static constexpr uint32_t OTG_HCTSIZ_DPID_Pos   = ( 29U );
  static constexpr uint32_t OTG_HCTSIZ_DPID_Msk   = ( 0x3UL << OTG_HCTSIZ_DPID_Pos );
  static constexpr uint32_t OTG_HCTSIZ_DPID       = OTG_HCTSIZ_DPID_Msk;
  static constexpr uint32_t OTG_HCTSIZ_DPID_0     = ( 0x1UL << OTG_HCTSIZ_DPID_Pos );
  static constexpr uint32_t OTG_HCTSIZ_DPID_1     = ( 0x2UL << OTG_HCTSIZ_DPID_Pos );

  /********************  Bit definition for OTG_DIEPDMA register  ********************/
  static constexpr uint32_t OTG_DIEPDMA_DMAADDR_Pos = ( 0U );
  static constexpr uint32_t OTG_DIEPDMA_DMAADDR_Msk = ( 0xFFFFFFFFUL << OTG_DIEPDMA_DMAADDR_Pos );
  static constexpr uint32_t OTG_DIEPDMA_DMAADDR     = OTG_DIEPDMA_DMAADDR_Msk;

  /********************  Bit definition for OTG_HCDMA register  ********************/
  static constexpr uint32_t OTG_HCDMA_DMAADDR_Pos = ( 0U );
  static constexpr uint32_t OTG_HCDMA_DMAADDR_Msk = ( 0xFFFFFFFFUL << OTG_HCDMA_DMAADDR_Pos );
  static constexpr uint32_t OTG_HCDMA_DMAADDR     = OTG_HCDMA_DMAADDR_Msk;

  /********************  Bit definition for OTG_DTXFSTS register  ********************/
  static constexpr uint32_t OTG_DTXFSTS_INEPTFSAV_Pos = ( 0U );
  static constexpr uint32_t OTG_DTXFSTS_INEPTFSAV_Msk = ( 0xFFFFUL << OTG_DTXFSTS_INEPTFSAV_Pos );
  static constexpr uint32_t OTG_DTXFSTS_INEPTFSAV     = OTG_DTXFSTS_INEPTFSAV_Msk;

  /********************  Bit definition for OTG_DIEPTXF register  ********************/
  static constexpr uint32_t OTG_DIEPTXF_INEPTXSA_Pos = ( 0U );
  static constexpr uint32_t OTG_DIEPTXF_INEPTXSA_Msk = ( 0xFFFFUL << OTG_DIEPTXF_INEPTXSA_Pos );
  static constexpr uint32_t OTG_DIEPTXF_INEPTXSA     = OTG_DIEPTXF_INEPTXSA_Msk;
  static constexpr uint32_t OTG_DIEPTXF_INEPTXFD_Pos = ( 16U );
  static constexpr uint32_t OTG_DIEPTXF_INEPTXFD_Msk = ( 0xFFFFUL << OTG_DIEPTXF_INEPTXFD_Pos );
  static constexpr uint32_t OTG_DIEPTXF_INEPTXFD     = OTG_DIEPTXF_INEPTXFD_Msk;

  /********************  Bit definition for OTG_DOEPCTL register  ********************/

  static constexpr uint32_t OTG_DOEPCTL_MPSIZ_Pos          = ( 0U );
  static constexpr uint32_t OTG_DOEPCTL_MPSIZ_Msk          = ( 0x7FFUL << OTG_DOEPCTL_MPSIZ_Pos );
  static constexpr uint32_t OTG_DOEPCTL_MPSIZ              = OTG_DOEPCTL_MPSIZ_Msk;
  static constexpr uint32_t OTG_DOEPCTL_USBAEP_Pos         = ( 15U );
  static constexpr uint32_t OTG_DOEPCTL_USBAEP_Msk         = ( 0x1UL << OTG_DOEPCTL_USBAEP_Pos );
  static constexpr uint32_t OTG_DOEPCTL_USBAEP             = OTG_DOEPCTL_USBAEP_Msk;
  static constexpr uint32_t OTG_DOEPCTL_NAKSTS_Pos         = ( 17U );
  static constexpr uint32_t OTG_DOEPCTL_NAKSTS_Msk         = ( 0x1UL << OTG_DOEPCTL_NAKSTS_Pos );
  static constexpr uint32_t OTG_DOEPCTL_NAKSTS             = OTG_DOEPCTL_NAKSTS_Msk;
  static constexpr uint32_t OTG_DOEPCTL_SD0PID_SEVNFRM_Pos = ( 28U );
  static constexpr uint32_t OTG_DOEPCTL_SD0PID_SEVNFRM_Msk = ( 0x1UL << OTG_DOEPCTL_SD0PID_SEVNFRM_Pos );
  static constexpr uint32_t OTG_DOEPCTL_SD0PID_SEVNFRM     = OTG_DOEPCTL_SD0PID_SEVNFRM_Msk;
  static constexpr uint32_t OTG_DOEPCTL_SODDFRM_Pos        = ( 29U );
  static constexpr uint32_t OTG_DOEPCTL_SODDFRM_Msk        = ( 0x1UL << OTG_DOEPCTL_SODDFRM_Pos );
  static constexpr uint32_t OTG_DOEPCTL_SODDFRM            = OTG_DOEPCTL_SODDFRM_Msk;
  static constexpr uint32_t OTG_DOEPCTL_EPTYP_Pos          = ( 18U );
  static constexpr uint32_t OTG_DOEPCTL_EPTYP_Msk          = ( 0x3UL << OTG_DOEPCTL_EPTYP_Pos );
  static constexpr uint32_t OTG_DOEPCTL_EPTYP              = OTG_DOEPCTL_EPTYP_Msk;
  static constexpr uint32_t OTG_DOEPCTL_EPTYP_0            = ( 0x1UL << OTG_DOEPCTL_EPTYP_Pos );
  static constexpr uint32_t OTG_DOEPCTL_EPTYP_1            = ( 0x2UL << OTG_DOEPCTL_EPTYP_Pos );
  static constexpr uint32_t OTG_DOEPCTL_SNPM_Pos           = ( 20U );
  static constexpr uint32_t OTG_DOEPCTL_SNPM_Msk           = ( 0x1UL << OTG_DOEPCTL_SNPM_Pos );
  static constexpr uint32_t OTG_DOEPCTL_SNPM               = OTG_DOEPCTL_SNPM_Msk;
  static constexpr uint32_t OTG_DOEPCTL_STALL_Pos          = ( 21U );
  static constexpr uint32_t OTG_DOEPCTL_STALL_Msk          = ( 0x1UL << OTG_DOEPCTL_STALL_Pos );
  static constexpr uint32_t OTG_DOEPCTL_STALL              = OTG_DOEPCTL_STALL_Msk;
  static constexpr uint32_t OTG_DOEPCTL_CNAK_Pos           = ( 26U );
  static constexpr uint32_t OTG_DOEPCTL_CNAK_Msk           = ( 0x1UL << OTG_DOEPCTL_CNAK_Pos );
  static constexpr uint32_t OTG_DOEPCTL_CNAK               = OTG_DOEPCTL_CNAK_Msk;
  static constexpr uint32_t OTG_DOEPCTL_SNAK_Pos           = ( 27U );
  static constexpr uint32_t OTG_DOEPCTL_SNAK_Msk           = ( 0x1UL << OTG_DOEPCTL_SNAK_Pos );
  static constexpr uint32_t OTG_DOEPCTL_SNAK               = OTG_DOEPCTL_SNAK_Msk;
  static constexpr uint32_t OTG_DOEPCTL_EPDIS_Pos          = ( 30U );
  static constexpr uint32_t OTG_DOEPCTL_EPDIS_Msk          = ( 0x1UL << OTG_DOEPCTL_EPDIS_Pos );
  static constexpr uint32_t OTG_DOEPCTL_EPDIS              = OTG_DOEPCTL_EPDIS_Msk;
  static constexpr uint32_t OTG_DOEPCTL_EPENA_Pos          = ( 31U );
  static constexpr uint32_t OTG_DOEPCTL_EPENA_Msk          = ( 0x1UL << OTG_DOEPCTL_EPENA_Pos );
  static constexpr uint32_t OTG_DOEPCTL_EPENA              = OTG_DOEPCTL_EPENA_Msk;

  /********************  Bit definition for OTG_DOEPINT register  ********************/
  static constexpr uint32_t OTG_DOEPINT_XFRC_Pos      = ( 0U );
  static constexpr uint32_t OTG_DOEPINT_XFRC_Msk      = ( 0x1UL << OTG_DOEPINT_XFRC_Pos );
  static constexpr uint32_t OTG_DOEPINT_XFRC          = OTG_DOEPINT_XFRC_Msk;
  static constexpr uint32_t OTG_DOEPINT_EPDISD_Pos    = ( 1U );
  static constexpr uint32_t OTG_DOEPINT_EPDISD_Msk    = ( 0x1UL << OTG_DOEPINT_EPDISD_Pos );
  static constexpr uint32_t OTG_DOEPINT_EPDISD        = OTG_DOEPINT_EPDISD_Msk;
  static constexpr uint32_t OTG_DOEPINT_AHBERR_Pos    = ( 2U );
  static constexpr uint32_t OTG_DOEPINT_AHBERR_Msk    = ( 0x1UL << OTG_DOEPINT_AHBERR_Pos );
  static constexpr uint32_t OTG_DOEPINT_AHBERR        = OTG_DOEPINT_AHBERR_Msk;
  static constexpr uint32_t OTG_DOEPINT_STUP_Pos      = ( 3U );
  static constexpr uint32_t OTG_DOEPINT_STUP_Msk      = ( 0x1UL << OTG_DOEPINT_STUP_Pos );
  static constexpr uint32_t OTG_DOEPINT_STUP          = OTG_DOEPINT_STUP_Msk;
  static constexpr uint32_t OTG_DOEPINT_OTEPDIS_Pos   = ( 4U );
  static constexpr uint32_t OTG_DOEPINT_OTEPDIS_Msk   = ( 0x1UL << OTG_DOEPINT_OTEPDIS_Pos );
  static constexpr uint32_t OTG_DOEPINT_OTEPDIS       = OTG_DOEPINT_OTEPDIS_Msk;
  static constexpr uint32_t OTG_DOEPINT_OTEPSPR_Pos   = ( 5U );
  static constexpr uint32_t OTG_DOEPINT_OTEPSPR_Msk   = ( 0x1UL << OTG_DOEPINT_OTEPSPR_Pos );
  static constexpr uint32_t OTG_DOEPINT_OTEPSPR       = OTG_DOEPINT_OTEPSPR_Msk;
  static constexpr uint32_t OTG_DOEPINT_B2BSTUP_Pos   = ( 6U );
  static constexpr uint32_t OTG_DOEPINT_B2BSTUP_Msk   = ( 0x1UL << OTG_DOEPINT_B2BSTUP_Pos );
  static constexpr uint32_t OTG_DOEPINT_B2BSTUP       = OTG_DOEPINT_B2BSTUP_Msk;
  static constexpr uint32_t OTG_DOEPINT_OUTPKTERR_Pos = ( 8U );
  static constexpr uint32_t OTG_DOEPINT_OUTPKTERR_Msk = ( 0x1UL << OTG_DOEPINT_OUTPKTERR_Pos );
  static constexpr uint32_t OTG_DOEPINT_OUTPKTERR     = OTG_DOEPINT_OUTPKTERR_Msk;
  static constexpr uint32_t OTG_DOEPINT_NAK_Pos       = ( 13U );
  static constexpr uint32_t OTG_DOEPINT_NAK_Msk       = ( 0x1UL << OTG_DOEPINT_NAK_Pos );
  static constexpr uint32_t OTG_DOEPINT_NAK           = OTG_DOEPINT_NAK_Msk;
  static constexpr uint32_t OTG_DOEPINT_NYET_Pos      = ( 14U );
  static constexpr uint32_t OTG_DOEPINT_NYET_Msk      = ( 0x1UL << OTG_DOEPINT_NYET_Pos );
  static constexpr uint32_t OTG_DOEPINT_NYET          = OTG_DOEPINT_NYET_Msk;
  static constexpr uint32_t OTG_DOEPINT_STPKTRX_Pos   = ( 15U );
  static constexpr uint32_t OTG_DOEPINT_STPKTRX_Msk   = ( 0x1UL << OTG_DOEPINT_STPKTRX_Pos );
  static constexpr uint32_t OTG_DOEPINT_STPKTRX       = OTG_DOEPINT_STPKTRX_Msk;
  /********************  Bit definition for OTG_DOEPTSIZ register  ********************/

  static constexpr uint32_t OTG_DOEPTSIZ_XFRSIZ_Pos = ( 0U );
  static constexpr uint32_t OTG_DOEPTSIZ_XFRSIZ_Msk = ( 0x7FFFFUL << OTG_DOEPTSIZ_XFRSIZ_Pos );
  static constexpr uint32_t OTG_DOEPTSIZ_XFRSIZ     = OTG_DOEPTSIZ_XFRSIZ_Msk;
  static constexpr uint32_t OTG_DOEPTSIZ_PKTCNT_Pos = ( 19U );
  static constexpr uint32_t OTG_DOEPTSIZ_PKTCNT_Msk = ( 0x3FFUL << OTG_DOEPTSIZ_PKTCNT_Pos );
  static constexpr uint32_t OTG_DOEPTSIZ_PKTCNT     = OTG_DOEPTSIZ_PKTCNT_Msk;

  static constexpr uint32_t OTG_DOEPTSIZ_STUPCNT_Pos = ( 29U );
  static constexpr uint32_t OTG_DOEPTSIZ_STUPCNT_Msk = ( 0x3UL << OTG_DOEPTSIZ_STUPCNT_Pos );
  static constexpr uint32_t OTG_DOEPTSIZ_STUPCNT     = OTG_DOEPTSIZ_STUPCNT_Msk;
  static constexpr uint32_t OTG_DOEPTSIZ_STUPCNT_0   = ( 0x1UL << OTG_DOEPTSIZ_STUPCNT_Pos );
  static constexpr uint32_t OTG_DOEPTSIZ_STUPCNT_1   = ( 0x2UL << OTG_DOEPTSIZ_STUPCNT_Pos );

  /********************  Bit definition for PCGCCTL register  ********************/
  static constexpr uint32_t OTG_PCGCCTL_STOPCLK_Pos = ( 0U );
  static constexpr uint32_t OTG_PCGCCTL_STOPCLK_Msk = ( 0x1UL << OTG_PCGCCTL_STOPCLK_Pos );
  static constexpr uint32_t OTG_PCGCCTL_STOPCLK     = OTG_PCGCCTL_STOPCLK_Msk;
  static constexpr uint32_t OTG_PCGCCTL_GATECLK_Pos = ( 1U );
  static constexpr uint32_t OTG_PCGCCTL_GATECLK_Msk = ( 0x1UL << OTG_PCGCCTL_GATECLK_Pos );
  static constexpr uint32_t OTG_PCGCCTL_GATECLK     = OTG_PCGCCTL_GATECLK_Msk;
  static constexpr uint32_t OTG_PCGCCTL_PHYSUSP_Pos = ( 4U );
  static constexpr uint32_t OTG_PCGCCTL_PHYSUSP_Msk = ( 0x1UL << OTG_PCGCCTL_PHYSUSP_Pos );
  static constexpr uint32_t OTG_PCGCCTL_PHYSUSP     = OTG_PCGCCTL_PHYSUSP_Msk;

  /* Legacy define */
  /********************  Bit definition for OTG register  ********************/
  static constexpr uint32_t OTG_CHNUM_Pos = ( 0U );
  static constexpr uint32_t OTG_CHNUM_Msk = ( 0xFUL << OTG_CHNUM_Pos );
  static constexpr uint32_t OTG_CHNUM     = OTG_CHNUM_Msk;
  static constexpr uint32_t OTG_CHNUM_0   = ( 0x1UL << OTG_CHNUM_Pos );
  static constexpr uint32_t OTG_CHNUM_1   = ( 0x2UL << OTG_CHNUM_Pos );
  static constexpr uint32_t OTG_CHNUM_2   = ( 0x4UL << OTG_CHNUM_Pos );
  static constexpr uint32_t OTG_CHNUM_3   = ( 0x8UL << OTG_CHNUM_Pos );
  static constexpr uint32_t OTG_BCNT_Pos  = ( 4U );
  static constexpr uint32_t OTG_BCNT_Msk  = ( 0x7FFUL << OTG_BCNT_Pos );
  static constexpr uint32_t OTG_BCNT      = OTG_BCNT_Msk;

  static constexpr uint32_t OTG_DPID_Pos = ( 15U );
  static constexpr uint32_t OTG_DPID_Msk = ( 0x3UL << OTG_DPID_Pos );
  static constexpr uint32_t OTG_DPID     = OTG_DPID_Msk;
  static constexpr uint32_t OTG_DPID_0   = ( 0x1UL << OTG_DPID_Pos );
  static constexpr uint32_t OTG_DPID_1   = ( 0x2UL << OTG_DPID_Pos );

  static constexpr uint32_t OTG_PKTSTS_Pos = ( 17U );
  static constexpr uint32_t OTG_PKTSTS_Msk = ( 0xFUL << OTG_PKTSTS_Pos );
  static constexpr uint32_t OTG_PKTSTS     = OTG_PKTSTS_Msk;
  static constexpr uint32_t OTG_PKTSTS_0   = ( 0x1UL << OTG_PKTSTS_Pos );
  static constexpr uint32_t OTG_PKTSTS_1   = ( 0x2UL << OTG_PKTSTS_Pos );
  static constexpr uint32_t OTG_PKTSTS_2   = ( 0x4UL << OTG_PKTSTS_Pos );
  static constexpr uint32_t OTG_PKTSTS_3   = ( 0x8UL << OTG_PKTSTS_Pos );

  static constexpr uint32_t OTG_EPNUM_Pos = ( 0U );
  static constexpr uint32_t OTG_EPNUM_Msk = ( 0xFUL << OTG_EPNUM_Pos );
  static constexpr uint32_t OTG_EPNUM     = OTG_EPNUM_Msk;
  static constexpr uint32_t OTG_EPNUM_0   = ( 0x1UL << OTG_EPNUM_Pos );
  static constexpr uint32_t OTG_EPNUM_1   = ( 0x2UL << OTG_EPNUM_Pos );
  static constexpr uint32_t OTG_EPNUM_2   = ( 0x4UL << OTG_EPNUM_Pos );
  static constexpr uint32_t OTG_EPNUM_3   = ( 0x8UL << OTG_EPNUM_Pos );

  static constexpr uint32_t OTG_FRMNUM_Pos = ( 21U );
  static constexpr uint32_t OTG_FRMNUM_Msk = ( 0xFUL << OTG_FRMNUM_Pos );
  static constexpr uint32_t OTG_FRMNUM     = OTG_FRMNUM_Msk;
  static constexpr uint32_t OTG_FRMNUM_0   = ( 0x1UL << OTG_FRMNUM_Pos );
  static constexpr uint32_t OTG_FRMNUM_1   = ( 0x2UL << OTG_FRMNUM_Pos );
  static constexpr uint32_t OTG_FRMNUM_2   = ( 0x4UL << OTG_FRMNUM_Pos );
  static constexpr uint32_t OTG_FRMNUM_3   = ( 0x8UL << OTG_FRMNUM_Pos );

}    // namespace Thor::LLD::USB

#endif /* !THOR_HW_USB_REGISTER_STM32F4XXXX_HPP */
