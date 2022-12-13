/******************************************************************************
 *  File Name:
 *    hw_crs_data.cpp
 *
 *  Description:
 *    Provides structures for conversion and mapping between data types for fast
 *    runtime performance of driver code.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/stm32l4x/crs/hw_crs_prj.hpp>
#include <Thor/lld/stm32l4x/crs/hw_crs_types.hpp>
#include <Thor/lld/stm32l4x/crs/hw_crs_prv_data.hpp>

#if defined( TARGET_STM32L4 ) && defined( THOR_CRS )

namespace Thor::LLD::CRS
{
  /*---------------------------------------------------------------------------
  Peripheral Memory Maps
  ---------------------------------------------------------------------------*/
#if defined( STM32_CRS1_PERIPH_AVAILABLE )
  RegisterMap *CRS1_PERIPH = reinterpret_cast<RegisterMap *>( CRS1_BASE_ADDR );
#endif

  /*---------------------------------------------------------------------------
  Configuration Maps
  ---------------------------------------------------------------------------*/
  namespace ConfigMap
  { /* clang-format off */
    LLD_CONST Reg32_t SyncSourceMap[ static_cast<size_t>( SyncSource::NUM_OPTIONS ) ] = {
      ( 0x00 << CFGR_SYNCSRC_Pos ) & CFGR_SYNCSRC_Msk,  // GPIO
      ( 0x01 << CFGR_SYNCSRC_Pos ) & CFGR_SYNCSRC_Msk,  // LSE
      ( 0x02 << CFGR_SYNCSRC_Pos ) & CFGR_SYNCSRC_Msk   // USB_SOF
    };

    LLD_CONST Reg32_t SyncDivMap[ static_cast<size_t>( SyncDiv::NUM_OPTIONS ) ] = {
      ( 0x00 << CFGR_SYNCDIV_Pos ) & CFGR_SYNCDIV_Msk,  // DIV1
      ( 0x01 << CFGR_SYNCDIV_Pos ) & CFGR_SYNCDIV_Msk,  // DIV2
      ( 0x02 << CFGR_SYNCDIV_Pos ) & CFGR_SYNCDIV_Msk,  // DIV4
      ( 0x03 << CFGR_SYNCDIV_Pos ) & CFGR_SYNCDIV_Msk,  // DIV8
      ( 0x04 << CFGR_SYNCDIV_Pos ) & CFGR_SYNCDIV_Msk,  // DIV16
      ( 0x05 << CFGR_SYNCDIV_Pos ) & CFGR_SYNCDIV_Msk,  // DIV32
      ( 0x06 << CFGR_SYNCDIV_Pos ) & CFGR_SYNCDIV_Msk,  // DIV64
      ( 0x07 << CFGR_SYNCDIV_Pos ) & CFGR_SYNCDIV_Msk   // DIV128
    };

  } /* clang-format on */

}    // namespace Thor::LLD::CRS

#endif /* TARGET_STM32L4 & THOR_LLD_CRS */
