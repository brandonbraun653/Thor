/********************************************************************************
 *  File Name:
 *    hw_exti_register_stm32l432kc.cpp
 *
 *  Description:
 *    EXTI register definitions for the STM32L432KC series chips
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

namespace Thor::LLD::RCC::LookupTables
{
  /*-------------------------------------------------------------------------------
  There are no clocks for the EXTI peripheral. Someone will come looking for them
  eventually, so I'm leaving this comment as this is where the clock configuration
  *should* be in keeping with the LLD driver scheme.

  I'm guessing the EXTI hardware is combinational logic only? Or perhaps it's just
  never disconnected from the APB bus cause the power consumption is so small?
  -------------------------------------------------------------------------------*/
}    // namespace Thor::LLD::RCC::LookupTables