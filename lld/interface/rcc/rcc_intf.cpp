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
  Static Data
  -------------------------------------------------------------------------------*/
  static ExternalOscillator s_ext_osc;

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


  void cacheExtOscFreq( const Chimera::Clock::Bus bus, const size_t freq )
  {
    switch( bus )
    {
      case Chimera::Clock::Bus::LSE:
        s_ext_osc.LSEFrequency = freq;
        break;

      case Chimera::Clock::Bus::HSE:
        s_ext_osc.HSEFrequency = freq;
        break;

      default:
        // Do nothing
        break;
    }
  }


  size_t getExtOscFreq( const Chimera::Clock::Bus bus )
  {
    switch( bus )
    {
      case Chimera::Clock::Bus::LSE:
        return s_ext_osc.LSEFrequency;
        break;

      case Chimera::Clock::Bus::HSE:
        return s_ext_osc.HSEFrequency;
        break;

      default:
        return INVALID_CLOCK;
        break;
    }
  }

}  // namespace Thor::LLD::RCC
