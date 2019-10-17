/********************************************************************************
 *   File Name:
 *    hw_rcc_types.cpp
 *
 *   Description:
 *    Declarations of NVIC types
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/drivers/f4/rcc/hw_rcc_types.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_RCC == 1 )

namespace Thor::Driver::RCC
{
#if defined( _EMBEDDED )
  RegisterMap *const RCC_PERIPH = reinterpret_cast<RegisterMap *const>( RCC_BASE_ADDR );

#elif defined( _SIM )
  RegisterMap *const RCC_PERIPH = new RegisterMap;

#endif 

}    // namespace Thor::Driver::NVIC

#endif /* TARGET_STM32F4 && THOR_DRIVER_NVIC */
