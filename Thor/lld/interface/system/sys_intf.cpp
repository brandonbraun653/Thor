/********************************************************************************
 *  File Name:
 *    sys_intf.cpp
 *
 *  Description:
 *    LLD interface functions that are processor independent
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Thor Includes */
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/system/sys_detail.hpp>
#include <Thor/lld/interface/system/sys_prv_data.hpp>
#include <Thor/lld/interface/rcc/rcc_intf.hpp>


namespace Thor::LLD::SYS
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  bool isSupported()
  {
    return true;
  }


  RIndex_t getResourceIndex( const std::uintptr_t address )
  {
    /*-------------------------------------------------
    We support it! Figure out which peripheral it is
    -------------------------------------------------*/
    if ( address == reinterpret_cast<std::uintptr_t>( SYSCFG1_PERIPH ) )
    {
      return SYSCFG1_RESOURCE_INDEX;
    }
    else
    {
      return INVALID_RESOURCE_INDEX;
    }
  }


  void clockEnable()
  {
    auto rcc   = Thor::LLD::RCC::getPeripheralClock();
    auto index = getResourceIndex( reinterpret_cast<std::uintptr_t>( SYSCFG1_PERIPH ) );

    rcc->enableClock( Chimera::Peripheral::Type::PERIPH_SYSCFG, index );
  }


  void clockDisable()
  {
    auto rcc   = Thor::LLD::RCC::getPeripheralClock();
    auto index = getResourceIndex( reinterpret_cast<std::uintptr_t>( SYSCFG1_PERIPH ) );

    rcc->disableClock( Chimera::Peripheral::Type::PERIPH_SYSCFG, index );
  }

}  // namespace Thor::LLD:SYS
