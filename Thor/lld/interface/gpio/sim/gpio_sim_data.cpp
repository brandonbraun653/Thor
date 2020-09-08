/********************************************************************************
 *  File Name:
 *    gpio_sim_data.cpp
 *
 *  Description:
 *    Simulator data for the variant
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* STL Includes */
#include <cstddef>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/gpio/gpio_detail.hpp>

#if defined( TARGET_LLD_TEST ) && defined( THOR_LLD_GPIO )

namespace Thor::LLD::GPIO
{
#if defined( STM32_GPIOA_PERIPH_AVAILABLE )
  static RegisterMap A;
  RegisterMap *GPIOA_PERIPH = &A;
#endif
#if defined( STM32_GPIOB_PERIPH_AVAILABLE )
  static RegisterMap B;
  RegisterMap *GPIOB_PERIPH = &B;
#endif
#if defined( STM32_GPIOC_PERIPH_AVAILABLE )
  static RegisterMap C;
  RegisterMap *GPIOC_PERIPH = &C;
#endif
}    // namespace Thor::LLD::GPIO

#endif 
