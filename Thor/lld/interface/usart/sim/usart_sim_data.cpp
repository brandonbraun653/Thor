/********************************************************************************
 *  File Name:
 *    usart_sim_data.cpp
 *
 *  Description:
 *    Simulator variant data
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* STL Includes */
#include <cstddef>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/usart/usart_detail.hpp>

#if defined( TARGET_LLD_TEST ) && defined( THOR_LLD_USART )

namespace Thor::LLD::USART
{
  /*-------------------------------------------------------------------------------
  Peripheral Memory Maps
  -------------------------------------------------------------------------------*/
#if defined( STM32_USART1_PERIPH_AVAILABLE )
  static RegisterMap USART1;
  RegisterMap *USART1_PERIPH = &USART1;
#endif
#if defined( STM32_USART2_PERIPH_AVAILABLE )
  static RegisterMap USART2;
  RegisterMap *USART2_PERIPH = &USART2;
#endif
#if defined( STM32_USART3_PERIPH_AVAILABLE )
  static RegisterMap USART3;
  RegisterMap *USART3_PERIPH = &USART3;
#endif
}  // namespace Thor::LLD::USART

#endif
