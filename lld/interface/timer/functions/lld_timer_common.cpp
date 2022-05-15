/******************************************************************************
 *  File Name:
 *    lld_timer_common.cpp
 *
 *  Description:
 *    Timer functionality common to all variants
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/timer>
#include <Thor/lld/interface/inc/rcc>
#include <Thor/lld/interface/inc/timer>

namespace Thor::LLD::TIMER
{
  /*---------------------------------------------------------------------------
  Public Methods
  ---------------------------------------------------------------------------*/
  void attach( Handle_rPtr timer, RegisterMap *const peripheral )
  {
    timer->registers   = peripheral;
    timer->type        = getHardwareType( reinterpret_cast<std::uintptr_t>( peripheral ) );
    timer->globalIndex = getGlobalResourceIndex( reinterpret_cast<std::uintptr_t>( peripheral ) );
    timer->typeIndex   = getTypeResourceIndex( reinterpret_cast<std::uintptr_t>( peripheral ) );
  }


  void reset( Handle_rPtr timer )
  {
    auto rcc = Thor::LLD::RCC::getPeriphClockCtrl();
    rcc->reset( Chimera::Peripheral::Type::PERIPH_TIMER, timer->globalIndex );
  }


  void clockEnable( Handle_rPtr timer )
  {
    auto rcc = Thor::LLD::RCC::getPeriphClockCtrl();
    rcc->enableClock( Chimera::Peripheral::Type::PERIPH_TIMER, timer->globalIndex );
  }


  void clockDisable( Handle_rPtr timer )
  {
    auto rcc = Thor::LLD::RCC::getPeriphClockCtrl();
    rcc->disableClock( Chimera::Peripheral::Type::PERIPH_TIMER, timer->globalIndex );
  }


  void open( Handle_rPtr timer )
  {
    /*-------------------------------------------------------------------------
    Power on the clock and reset the hardware to defaults
    -------------------------------------------------------------------------*/
    clockEnable( timer );
    reset( timer );

    /*-------------------------------------------------------------------------
    Ensure the counter is off
    -------------------------------------------------------------------------*/
    disableCounter( timer );
  }


  void close( Handle_rPtr timer )
  {
    clockDisable( timer );
  }

}    // namespace Thor::LLD::TIMER
