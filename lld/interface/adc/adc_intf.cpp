/******************************************************************************
 *  File Name:
 *    adc_intf.cpp
 *
 *  Description:
 *    LLD interface functions that are processor independent
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/thread>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/inc/adc>

#if defined( THOR_ADC )

namespace Thor::LLD::ADC
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  bool isSupported( const Chimera::ADC::Peripheral periph )
  {
    switch ( periph )
    {
#if defined( STM32_ADC1_PERIPH_AVAILABLE )
      case Chimera::ADC::Peripheral::ADC_0:
        return true;
        break;
#endif

      default:
        return false;
        break;
    };
  }


  RIndex_t getResourceIndex( const Chimera::ADC::Peripheral periph )
  {
    switch ( periph )
    {
#if defined( STM32_ADC1_PERIPH_AVAILABLE )
      case Chimera::ADC::Peripheral::ADC_0:
        return ADC1_RESOURCE_INDEX;
        break;
#endif

      default:
        return INVALID_RESOURCE_INDEX;
        break;
    };
  }


  RIndex_t getResourceIndex( const std::uintptr_t address )
  {
#if defined( STM32_ADC1_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( ADC1_PERIPH ) )
    {
      return ADC1_RESOURCE_INDEX;
    }
#endif

    return INVALID_RESOURCE_INDEX;
  }


  Chimera::ADC::Peripheral getChannel( const std::uintptr_t address )
  {
#if defined( STM32_ADC1_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( ADC1_PERIPH ) )
    {
      return Chimera::ADC::Peripheral::ADC_0;
    }
#endif

    return Chimera::ADC::Peripheral::UNKNOWN;
  }


  bool attachDriverInstances( Driver *const driverList, const size_t numDrivers )
  {
    /*-------------------------------------------------------------------------
    Reject bad inputs
    -------------------------------------------------------------------------*/
    if ( !driverList || !numDrivers || ( numDrivers != NUM_ADC_PERIPHS ) )
    {
      return false;
    }

    /*-------------------------------------------------------------------------
    Attach the drivers. The architecture of the LLD
    ensures the ordering and number is correct.
    -------------------------------------------------------------------------*/
    Chimera::Status_t result = Chimera::Status::OK;

#if defined( STM32_ADC1_PERIPH_AVAILABLE )
    result |= driverList[ ADC1_RESOURCE_INDEX ].attach( ADC1_PERIPH );
#endif

    return result == Chimera::Status::OK;
  }
}    // namespace Thor::LLD::ADC

#endif  /* THOR_LLD_ADC */
