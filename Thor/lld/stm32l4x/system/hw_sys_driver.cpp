/********************************************************************************
 *  File Name:
 *    sys_config.cpp
 *
 *  Description:
 *    Insert Description
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Chimera Includes */
#include <Chimera/gpio>
#include <Chimera/utility>

/* Thor Includes */
#include <Thor/lld/interface/gpio/gpio_detail.hpp>
#include <Thor/lld/interface/system/sys_detail.hpp>
#include <Thor/lld/interface/system/sys_prv_data.hpp>

namespace Thor::LLD::SYS
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/

  void configureExtiSource( const Chimera::GPIO::Port port, const uint8_t pin )
  {
    /*-------------------------------------------------
    Input Protection
    -------------------------------------------------*/
    if( !( port < GPIO::PRJ_LAST_PORT ) || !( pin < GPIO::PRJ_MAX_PINS_PER_PORT ) )
    {
      return;
    }

    /*-------------------------------------------------
    Select the register to operate on. Convenietly,
    there are four pins per register, so use that to
    create an index.
    -------------------------------------------------*/
    const uint32_t PinsPerRegister = 4;
    const uint32_t cfgRegIdx = pin / PinsPerRegister;

    if( !( cfgRegIdx < ARRAY_COUNT( RegisterMap::EXTICR )))
    {
      return;
    }

    volatile uint32_t *cfgReg = &SYSCFG1_PERIPH->EXTICR[ cfgRegIdx ];

    /*-------------------------------------------------
    Select the value to write in the register. This
    assumes the Chimera::GPIO::Port enumerations are
    left in a linearly increasing order.
    -------------------------------------------------*/
    const uint32_t BaseRegVal = static_cast<uint32_t>( port );

    /*-------------------------------------------------
    Calculate the offset and mask
    -------------------------------------------------*/
    const uint32_t BaseMask = 0x7;
    const uint32_t BitShift = cfgRegIdx * PinsPerRegister;
    const uint32_t ShiftedMask = BaseMask << BitShift;

    /*-------------------------------------------------
    Assign the GPIO source
    -------------------------------------------------*/
    Reg32_t tmp = *cfgReg;
    tmp &= ~ShiftedMask;
    tmp |= ( ( BaseRegVal << BitShift ) & ShiftedMask );
    *cfgReg = tmp;
  }
}  // namespace Thor::LLD::SYS
