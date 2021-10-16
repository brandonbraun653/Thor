/********************************************************************************
 *  File Name:
 *    hw_rcc_periph_ctrl.cpp
 *
 *  Description:
 *    Peripheral clock controller implementation
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/clock>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/common/cortex-m4/system_time.hpp>
#include <Thor/lld/interface/inc/rcc>
#include <Thor/lld/interface/inc/power>


namespace Thor::LLD::RCC
{
  /*-------------------------------------------------------------------------------
  Static Data
  -------------------------------------------------------------------------------*/
  static PeripheralController s_periph_clock;

  /*-------------------------------------------------------------------------------
  PeripheralController Class Implementation
  -------------------------------------------------------------------------------*/
  PeripheralController *getPeriphClockCtrl()
  {
    return &s_periph_clock;
  }


  PeripheralController::PeripheralController()
  {
    initialize();
  }


  PeripheralController::~PeripheralController()
  {
  }


  Chimera::Status_t PeripheralController::reset( const Chimera::Peripheral::Type type, const size_t index )
  {
    /*-------------------------------------------------
    Input protection
    -------------------------------------------------*/
    auto registry = getPCCRegistry( type );
    if ( !registry || !registry->reset || ( index >= registry->elements ) || !registry->reset[ index ].reg )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }

    /*------------------------------------------------
    Begin the reset operation
    ------------------------------------------------*/
    uint32_t tmp = *( registry->reset[ index ].reg );
    tmp |= registry->reset[ index ].mask;
    *registry->reset[ index ].reg = tmp;

    /*-------------------------------------------------
    Burn a few cycles to avoid accessing HW too early
    -------------------------------------------------*/
    for ( auto x = 0; x < 5; x++ )
    {
      asm( "nop" );
    }

    /*------------------------------------------------
    Remove the reset flag as it is not cleared automatically by hardware
    ------------------------------------------------*/
    tmp &= ~registry->reset[ index ].mask;
    *( registry->reset[ index ].reg ) = tmp;

    return Chimera::Status::OK;
  }


  Chimera::Status_t PeripheralController::enableClock( const Chimera::Peripheral::Type type, const size_t index )
  {
    /*-------------------------------------------------
    Input protection
    -------------------------------------------------*/
    auto registry = getPCCRegistry( type );
    if ( !registry || !registry->clock || ( index >= registry->elements ) || !registry->clock[ index ].reg )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }

    /*-------------------------------------------------
    Enable the clock
    -------------------------------------------------*/
    *( registry->clock[ index ].reg ) |= registry->clock[ index ].mask;
    return Chimera::Status::OK;
  }


  Chimera::Status_t PeripheralController::disableClock( const Chimera::Peripheral::Type type, const size_t index )
  {
    /*-------------------------------------------------
    Input protection
    -------------------------------------------------*/
    auto registry = getPCCRegistry( type );
    if ( !registry || !registry->clock || ( index >= registry->elements ) || !registry->clock[ index ].reg )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }

    /*-------------------------------------------------
    Enable the clock
    -------------------------------------------------*/
    *( registry->clock[ index ].reg ) &= ~registry->clock[ index ].mask;
    return Chimera::Status::OK;
  }


  Chimera::Status_t PeripheralController::enableClockLowPower( const Chimera::Peripheral::Type type, const size_t index )
  {
    /*-------------------------------------------------
    Input protection
    -------------------------------------------------*/
    auto registry = getPCCRegistry( type );
    if ( !registry || !registry->clockLP || ( index >= registry->elements ) || !registry->clockLP[ index ].reg )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }

    /*-------------------------------------------------
    Enable the clock
    -------------------------------------------------*/
    *( registry->clockLP[ index ].reg ) |= registry->clockLP[ index ].mask;
    return Chimera::Status::OK;
  }


  Chimera::Status_t PeripheralController::disableClockLowPower( const Chimera::Peripheral::Type type, const size_t index )
  {
    /*-------------------------------------------------
    Input protection
    -------------------------------------------------*/
    auto registry = getPCCRegistry( type );
    if ( !registry || !registry->clockLP || ( index >= registry->elements ) || !registry->clockLP[ index ].reg )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }

    /*-------------------------------------------------
    Enable the clock
    -------------------------------------------------*/
    *( registry->clockLP[ index ].reg ) &= ~registry->clockLP[ index ].mask;
    return Chimera::Status::OK;
  }

}  // namespace Thor::LLD::RCC
