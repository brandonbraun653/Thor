/********************************************************************************
 *  File Name:
 *    hw_crs_driver.cpp
 *
 *  Description:
 *    CRS driver definition for STM32L4xxxx
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* STL Includes */
#include <cmath>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/hld/interrupt/hld_interrupt_definitions.hpp>
#include <Thor/lld/interface/interrupt/interrupt_detail.hpp>
#include <Thor/lld/stm32l4x/crs/hw_crs_driver.hpp>
#include <Thor/lld/stm32l4x/crs/hw_crs_prj.hpp>
#include <Thor/lld/stm32l4x/crs/hw_crs_types.hpp>
#include <Thor/lld/stm32l4x/crs/hw_crs_prv_data.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_driver.hpp>

#if defined( TARGET_STM32L4 ) && defined( THOR_LLD_CRS )

namespace Thor::LLD::CRS
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  void enableClock()
  {
    auto rcc = Thor::LLD::RCC::getPeripheralClock();
    rcc->enableClock( Chimera::Peripheral::Type::PERIPH_CRS, CRS1_RESOURCE_INDEX );
  }


  void clockDisable()
  {
    auto rcc = Thor::LLD::RCC::getPeripheralClock();
    rcc->disableClock( Chimera::Peripheral::Type::PERIPH_CRS, CRS1_RESOURCE_INDEX );
  }


  Chimera::Status_t initialize()
  {
    /*-------------------------------------------------
    Turn on the clock
    -------------------------------------------------*/
    enableClock();

    /*-------------------------------------------------
    Disable interrupts on a peripheral level
    -------------------------------------------------*/
    Thor::LLD::IT::disableIRQ( CRS_IRQn );
    Thor::LLD::IT::clearPendingIRQ( CRS_IRQn );

    /*-------------------------------------------------
    Disable individual interrupts
    -------------------------------------------------*/
    // Disable individual interrupts
    ESYNCIE::clear( CRS1_PERIPH, CR_ESYNCIE );
    ERRIE::clear( CRS1_PERIPH, CR_ERRIE );
    SYNCWARNIE::clear( CRS1_PERIPH, CR_SYNCWARNIE );
    SYNCOKIE::clear( CRS1_PERIPH, CR_SYNCOKIE );

    // Ack any pending ISRs
    ESYNCC::set( CRS1_PERIPH, ICR_ESYNCC );
    ERRC::set( CRS1_PERIPH, ICR_ERRC );
    SYNCWARNC::set( CRS1_PERIPH, ICR_SYNCWARNC );
    SYNCOKC::set( CRS1_PERIPH, ICR_SYNCOKC );

    /*-------------------------------------------------
    Configure the CR register back to reset value
    -------------------------------------------------*/
    CR_ALL::set( CRS1_PERIPH, CR_Rst );

    /*-------------------------------------------------
    Configure the CFGR register back to reset value
    -------------------------------------------------*/
    CFGR_ALL::set( CRS1_PERIPH, CFGR_Rst );

    return Chimera::Status::OK;
  }


  Chimera::Status_t selectSyncSource( const SyncSource src )
  {
    if ( src < SyncSource::NUM_OPTIONS )
    {
      SYNCSRC::set( CRS1_PERIPH, ConfigMap::SyncSourceMap[ static_cast<size_t>( src ) ] );
      return Chimera::Status::OK;
    }

    return Chimera::Status::INVAL_FUNC_PARAM;
  }


  Chimera::Status_t configureDiv( const SyncDiv div )
  {
    if ( div < SyncDiv::NUM_OPTIONS )
    {
      SYNCDIV::set( CRS1_PERIPH, ConfigMap::SyncDivMap[ static_cast<size_t>( div ) ] );
      return Chimera::Status::OK;
    }

    return Chimera::Status::INVAL_FUNC_PARAM;
  }


  Chimera::Status_t configureHWAlgo( const size_t targetFreq, const size_t syncFreq, const size_t stepSize )
  {
    /*-------------------------------------------------
    Local constants
    -------------------------------------------------*/
    constexpr size_t MAX_RELOAD_VAL = 0xFFFF;
    constexpr size_t MAX_FELIM_VAL  = 0xFF;

    /*-------------------------------------------------
    Input protection. Make sure the math below doesn't
    end up becoming invalid or dividing by zero.
    -------------------------------------------------*/
    if ( ( targetFreq == 0 ) || ( syncFreq == 0 ) || ( stepSize == 0 ) || ( syncFreq > targetFreq ) || ( stepSize > 100 ) )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------
    Disable the frequency error counter so config vals
    can even be set
    -------------------------------------------------*/
    toggleFEQ( false );

    /*-------------------------------------------------
    RELOAD value: RM 7.3.5
    -------------------------------------------------*/
    Reg32_t reloadVal = ( targetFreq / syncFreq ) - 1u;

    if ( reloadVal <= MAX_RELOAD_VAL )
    {
      reloadVal = ( reloadVal << CFGR_RELOAD_Pos ) & CFGR_RELOAD_Msk;
      RELOAD::set( CRS1_PERIPH, reloadVal );
    }
    else
    {
      return Chimera::Status::FAIL;
    }

    /*-------------------------------------------------
    FELIM value: RM 7.3.5
    -------------------------------------------------*/
    float percentStep = static_cast<float>( stepSize ) / 100.0f;
    float tF          = static_cast<float>( targetFreq );
    float sF          = static_cast<float>( syncFreq );
    float rawFELIM    = ( ( tF / sF ) * ( percentStep / 100.0f ) ) / 2.0f;

    // Highest accuracy achieved by rounding up to nearest integer
    Reg32_t felimVal = static_cast<Reg32_t>( ceil( rawFELIM ) );

    if ( felimVal <= MAX_FELIM_VAL )
    {
      felimVal = ( felimVal << CFGR_FELIM_Pos ) & CFGR_FELIM_Msk;
      FELIM::set( CRS1_PERIPH, felimVal );
    }
    else
    {
      return Chimera::Status::FAIL;
    }

    return Chimera::Status::OK;
  }


  void toggleAutoTrim( const bool state )
  {
    if ( state )
    {
      AUTOTRIMEN::set( CRS1_PERIPH, CR_AUTOTRIMEN );
    }
    else
    {
      AUTOTRIMEN::clear( CRS1_PERIPH, CR_AUTOTRIMEN );
    }
  }


  void toggleFEQ( const bool state )
  {
    if ( state )
    {
      CEN::set( CRS1_PERIPH, CR_CEN );
    }
    else
    {
      CEN::clear( CRS1_PERIPH, CR_CEN );
    }
  }
}    // namespace Thor::LLD::CRS

#endif /* TARGET_STM32L4 & THOR_LLD_CRS */
