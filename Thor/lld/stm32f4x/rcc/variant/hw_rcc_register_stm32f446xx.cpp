/********************************************************************************
 *   File Name:
 *    hw_rcc_register_stm32f446xx.cpp
 *
 *   Description:
 *    Explicit STM32F446xx RCC data and routines
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_driver.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_mapping.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_register_stm32f446xx.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_RCC == 1 ) && defined( STM32F446xx )

namespace Thor::Driver::RCC
{
#if defined( _EMBEDDED )
  RegisterMap * RCC1_PERIPH = reinterpret_cast<RegisterMap *>( RCC1_BASE_ADDR );

  Chimera::Container::LightFlatMap<std::uintptr_t, size_t> InstanceToResourceIndex{
    { reinterpret_cast<std::uintptr_t>( RCC1_PERIPH ), RCC1_RESOURCE_INDEX }
  };

#elif defined( _SIM )
  RegisterMap * RCC1_PERIPH = nullptr;

  Chimera::Container::LightFlatMap<std::uintptr_t, size_t> InstanceToResourceIndex;
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

#endif /* TARGET_STM32F4 && THOR_DRIVER_RCC */
