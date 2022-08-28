/******************************************************************************
 *  File Name:
 *    i2c_cmn_isr.cpp
 *
 *  Description:
 *    ISR Handlers for I2C
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Thor/cfg>
#include <Thor/lld/interface/inc/i2c>

/*-----------------------------------------------------------------------------
IRQ Handlers
-----------------------------------------------------------------------------*/
#if defined( THOR_I2C )
using namespace Thor::LLD::I2C;

#if defined( STM32_I2C1_PERIPH_AVAILABLE )
void I2C1_EV_IRQHandler()
{
  getLLDriver( Chimera::I2C::Channel::I2C1 )->IRQEventHandler();
}


void I2C1_ER_IRQHandler()
{
  getLLDriver( Chimera::I2C::Channel::I2C1 )->IRQErrorHandler();
}
#endif /* STM32_I2C1_PERIPH_AVAILABLE */

#if defined( STM32_I2C2_PERIPH_AVAILABLE )
void I2C2_EV_IRQHandler()
{
  getLLDriver( Chimera::I2C::Channel::I2C2 )->IRQEventHandler();
}


void I2C2_ER_IRQHandler()
{
  getLLDriver( Chimera::I2C::Channel::I2C2 )->IRQErrorHandler();
}
#endif /* STM32_I2C2_PERIPH_AVAILABLE */


#if defined( STM32_I2C3_PERIPH_AVAILABLE )
void I2C3_EV_IRQHandler()
{
  getLLDriver( Chimera::I2C::Channel::I2C3 )->IRQEventHandler();
}


void I2C3_ER_IRQHandler()
{
  getLLDriver( Chimera::I2C::Channel::I2C3 )->IRQErrorHandler();
}
#endif /* STM32_I2C3_PERIPH_AVAILABLE */
#endif /* THOR_LLD_I2C */
