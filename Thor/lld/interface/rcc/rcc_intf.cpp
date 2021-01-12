/********************************************************************************
 *  File Name:
 *    rcc_intf.cpp
 *
 *  Description:
 *    Implementation of low level RCC functionality at the interface layer.
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Thor Includes */
#include <Thor/lld/interface/rcc/rcc_detail.hpp>
#include <Thor/lld/interface/rcc/rcc_types.hpp>
#include <Thor/lld/interface/rcc/rcc_prv_data.hpp>

namespace Thor::LLD::RCC
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  const PCC *const getPCCRegistry( const Chimera::Peripheral::Type periph )
  {
    /*-------------------------------------------------
    Input protection
    -------------------------------------------------*/
    if( periph >= Chimera::Peripheral::Type::NUM_OPTIONS )
    {
      return nullptr;
    }

    /*-------------------------------------------------
    Return the data
    -------------------------------------------------*/
    return PeripheralControlRegistry[ static_cast<size_t>( periph ) ];
  }

}  // namespace Thor::LLD::RCC
