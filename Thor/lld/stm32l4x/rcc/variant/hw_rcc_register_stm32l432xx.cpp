/********************************************************************************
 *  File Name:
 *    hw_rcc_register_stm32l432xx.cpp
 *
 *  Description:
 *    STM32L432 RCC registers and initialization
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_driver.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_mapping.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_prj.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_types.hpp>

#if defined( TARGET_STM32L4 ) && defined( THOR_LLD_RCC ) && defined( STM32L432xx )

namespace Thor::LLD::RCC
{
#if defined( EMBEDDED )
  RegisterMap * RCC1_PERIPH = reinterpret_cast<RegisterMap *>( RCC1_BASE_ADDR );
#elif defined( _SIM )
  RegisterMap * RCC1_PERIPH = nullptr;

  Thor::LLD::RIndexMap InstanceToResourceIndex;
#endif

  void initializeRegisters()
  {
    #if defined( _SIM )
    /*------------------------------------------------
    Allocate some memory to simulate the register blocks
    ------------------------------------------------*/
    RCC1_PERIPH = new RegisterMap;

    /*------------------------------------------------
    Update the memory listing
    ------------------------------------------------*/
    PeripheralList[ RCC1_RESOURCE_INDEX ] = RCC1_PERIPH;

    /*------------------------------------------------
    Update the resource indexer now that the registers actually exist
    ------------------------------------------------*/
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( RCC1_PERIPH ), RCC1_RESOURCE_INDEX );
    #endif
  }
}

#endif  /* TARGET_STM32L4 && THOR_LLD_RCC && STM32L432xx */
