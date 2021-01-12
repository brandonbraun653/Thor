/********************************************************************************
 *  File Name:
 *    hld_exti_driver.cpp
 *
 *  Description:
 *    Implements the EXTI driver for STM32
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/exti>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/exti>
#include <Thor/lld/interface/exti/exti_intf.hpp>
#include <Thor/lld/interface/exti/exti_detail.hpp>

#if defined( THOR_HLD_EXTI )

namespace Thor::EXTI
{
  /*-------------------------------------------------------------------------------
  Aliases
  -------------------------------------------------------------------------------*/
  namespace LLD = ::Thor::LLD::EXTI;


  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t open()
  {
    return LLD::open();
  }


  Chimera::Status_t close()
  {
    return LLD::close();
  }


  Chimera::Status_t attach( const Chimera::EXTI::EventLine_t listener, const Chimera::EXTI::EdgeTrigger edge, Chimera::Function::vGeneric callback )
  {
    return LLD::attach( listener, edge, callback );
  }


  Chimera::Status_t detach( const Chimera::EXTI::EventLine_t listener )
  {
    return LLD::detach( listener );
  }


  Chimera::Status_t trigger( const Chimera::EXTI::EventLine_t listener )
  {
    return LLD::trigger( listener );
  }


  Chimera::Status_t disable( const Chimera::EXTI::EventLine_t listener )
  {
    return LLD::disable( listener );
  }


  Chimera::Status_t enable( const Chimera::EXTI::EventLine_t listener )
  {
    return LLD::enable( listener );
  }


  Chimera::EXTI::EventLine_t numInterruptLines()
  {
    return LLD::numInterruptLines();
  }

}    // namespace Thor::EXTI

#endif /* THOR_HLD_EXTI */
