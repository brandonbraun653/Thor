/********************************************************************************
 *  File Name:
 *    adc_intf.cpp
 *
 *  Description:
 *    LLD interface functions that are processor independent
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/thread>

/* Thor Includes */
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/adc/adc_detail.hpp>
#include <Thor/lld/interface/adc/adc_prv_data.hpp>
#include <Thor/lld/interface/adc/adc_types.hpp>
#include <Thor/lld/interface/adc/adc_intf.hpp>

namespace Thor::LLD::ADC
{
  /*-------------------------------------------------------------------------------
  Shared Data
  -------------------------------------------------------------------------------*/
  Chimera::Threading::ThreadId ISRThreadId[ NUM_ADC_PERIPHS ];

  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  bool isSupported( const Chimera::ADC::Converter periph )
  {
    switch ( periph )
    {
#if defined( STM32_ADC1_PERIPH_AVAILABLE )
      case Chimera::ADC::Converter::ADC_0:
        return true;
        break;
#endif

      default:
        return false;
        break;
    };
  }


  RIndex_t getResourceIndex( const Chimera::ADC::Converter periph )
  {
    switch ( periph )
    {
#if defined( STM32_ADC1_PERIPH_AVAILABLE )
      case Chimera::ADC::Converter::ADC_0:
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


  Chimera::ADC::Converter getChannel( const std::uintptr_t address )
  {
#if defined( STM32_ADC1_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( ADC1_PERIPH ) )
    {
      return Chimera::ADC::Converter::ADC_0;
    }
#endif

    return Chimera::ADC::Converter::UNKNOWN;
  }


  bool attachDriverInstances( Driver *const driverList, const size_t numDrivers )
  {
    /*-------------------------------------------------
    Reject bad inputs
    -------------------------------------------------*/
    if ( !driverList || !numDrivers || ( numDrivers != NUM_ADC_PERIPHS ) )
    {
      return false;
    }

    /*-------------------------------------------------
    Attach the drivers. The architecture of the LLD
    ensures the ordering and number is correct.
    -------------------------------------------------*/
    Chimera::Status_t result = Chimera::Status::OK;

#if defined( STM32_ADC1_PERIPH_AVAILABLE )
    result |= driverList[ ADC1_RESOURCE_INDEX ].attach( ADC1_PERIPH );
#endif

    return result == Chimera::Status::OK;
  }
}    // namespace Thor::LLD::ADC
