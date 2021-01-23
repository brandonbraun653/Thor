/********************************************************************************
 *  File Name:
 *    sys_intf.cpp
 *
 *  Description:
 *    LLD interface functions that are processor independent
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/inc/sys>
#include <Thor/lld/interface/inc/rcc>

#if defined( CORTEX_M4 )
#include <Thor/lld/common/cortex-m4/utilities.hpp>
#endif


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


  RIndex_t getResourceIndex( const Chimera::Peripheral::Type periph )
  {
    for ( size_t x = 0; x < ARRAY_COUNT( AvailablePeriphs ); x++ )
    {
      if ( AvailablePeriphs[ x ] == periph )
      {
        return x;
      }
    }

    return INVALID_RESOURCE_INDEX;
  }


  void clockEnable()
  {
    auto rcc   = Thor::LLD::RCC::getPeriphClockCtrl();
    auto index = getResourceIndex( reinterpret_cast<std::uintptr_t>( SYSCFG1_PERIPH ) );

    rcc->enableClock( Chimera::Peripheral::Type::PERIPH_SYSCFG, index );
  }


  void clockDisable()
  {
    auto rcc   = Thor::LLD::RCC::getPeriphClockCtrl();
    auto index = getResourceIndex( reinterpret_cast<std::uintptr_t>( SYSCFG1_PERIPH ) );

    rcc->disableClock( Chimera::Peripheral::Type::PERIPH_SYSCFG, index );
  }


  bool inISR()
  {
    #if defined( CORTEX_M4 )
    return CortexM4::inISR();
    #else
    return false;
    #endif
  }


  void softwareReset()
  {
    #if defined( CORTEX_M4 )
    CortexM4::hardwareReset();
    #endif
  }

}  // namespace Thor::LLD:SYS
