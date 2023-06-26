/******************************************************************************
 *  File Name:
 *    adc_intf.cpp
 *
 *  Description:
 *    LLD interface functions that are processor independent
 *
 *  2020-2023 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <Chimera/thread>
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
#endif
#if defined( STM32_ADC2_PERIPH_AVAILABLE )
      case Chimera::ADC::Peripheral::ADC_1:
        return true;
#endif
#if defined( STM32_ADC3_PERIPH_AVAILABLE )
      case Chimera::ADC::Peripheral::ADC_2:
        return true;
#endif

      default:
        return false;
    }
  }


  RIndex_t getResourceIndex( const Chimera::ADC::Peripheral periph )
  {
    switch ( periph )
    {
#if defined( STM32_ADC1_PERIPH_AVAILABLE )
      case Chimera::ADC::Peripheral::ADC_0:
        return ADC1_RESOURCE_INDEX;
#endif
#if defined( STM32_ADC2_PERIPH_AVAILABLE )
      case Chimera::ADC::Peripheral::ADC_1:
        return ADC2_RESOURCE_INDEX;
#endif
#if defined( STM32_ADC1_PERIPH_AVAILABLE )
      case Chimera::ADC::Peripheral::ADC_2:
        return ADC3_RESOURCE_INDEX;
#endif

      default:
        return INVALID_RESOURCE_INDEX;
    }
  }


  RIndex_t getResourceIndex( const std::uintptr_t address )
  {
#if defined( STM32_ADC1_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( ADC1_PERIPH ) )
    {
      return ADC1_RESOURCE_INDEX;
    }
#endif
#if defined( STM32_ADC2_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( ADC2_PERIPH ) )
    {
      return ADC2_RESOURCE_INDEX;
    }
#endif
#if defined( STM32_ADC3_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( ADC3_PERIPH ) )
    {
      return ADC3_RESOURCE_INDEX;
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
#if defined( STM32_ADC2_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( ADC2_PERIPH ) )
    {
      return Chimera::ADC::Peripheral::ADC_1;
    }
#endif
#if defined( STM32_ADC3_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( ADC3_PERIPH ) )
    {
      return Chimera::ADC::Peripheral::ADC_2;
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
#if defined( STM32_ADC2_PERIPH_AVAILABLE )
    result |= driverList[ ADC2_RESOURCE_INDEX ].attach( ADC2_PERIPH );
#endif
#if defined( STM32_ADC3_PERIPH_AVAILABLE )
    result |= driverList[ ADC3_RESOURCE_INDEX ].attach( ADC3_PERIPH );
#endif

    return result == Chimera::Status::OK;
  }
}    // namespace Thor::LLD::ADC

#endif /* THOR_ADC */
