/********************************************************************************
  *  File Name:
  *    hw_iwdg_register_stm32f446xx.cpp
  *
  *  Description:
  *    Explicit STM32F446xx IWDG data and routines
  *
  *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
  ********************************************************************************/

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/stm32f4x/iwdg/hw_iwdg_driver.hpp>
#include <Thor/lld/stm32f4x/iwdg/hw_iwdg_mapping.hpp>
#include <Thor/lld/stm32f4x/iwdg/variant/hw_iwdg_register_stm32f446xx.hpp>
#include <Thor/lld/stm32f4x/iwdg/hw_iwdg_types.hpp>
#include <Thor/lld/stm32f4x/rcc/hw_rcc_mapping.hpp>

#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_IWDG ) && defined( STM32F446xx )

namespace Thor::LLD::IWDG
{
#if defined( EMBEDDED )
  RegisterMap * IWDG1_PERIPH = reinterpret_cast<RegisterMap *>( IWDG1_BASE_ADDR );

  Chimera::Container::LightFlatMap<std::uintptr_t, size_t> InstanceToResourceIndex = {
    { reinterpret_cast<std::uintptr_t>( IWDG1_PERIPH ), IWDG1_RESOURCE_INDEX } 
  };

#elif defined( _SIM )
  RegisterMap *IWDG1_PERIPH = nullptr;

  Chimera::Container::LightFlatMap<std::uintptr_t, size_t> InstanceToResourceIndex;
#endif

  void initializeRegisters()
  {
#if defined( _SIM )
    /*------------------------------------------------
    Allocate some memory to simulate the register blocks
    ------------------------------------------------*/
    IWDG1_PERIPH = new RegisterMap;

    /*------------------------------------------------
    Update the memory listing
    ------------------------------------------------*/
    PeripheralList[ IWDG1_RESOURCE_INDEX ] = IWDG1_PERIPH;

    /*------------------------------------------------
    Update the resource indexer now that the registers actually exist
    ------------------------------------------------*/
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( IWDG1_PERIPH ), IWDG1_RESOURCE_INDEX );

#endif
  }
}    // namespace Thor::LLD::IWDG

#endif /* TARGET_STM32F4 && THOR_DRIVER_IWDG */