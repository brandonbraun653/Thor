/********************************************************************************
 *  File Name:
 *    hw_adc_data.cpp
 *
 *  Description:
 *    Provides implementation details for private ADC driver data
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* STL Includes */
#include <limits>

/* Chimera Includes */
#include <Chimera/adc>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/inc/dma>
#include <Thor/lld/interface/inc/adc>

#if defined( TARGET_STM32F4 ) && defined( THOR_ADC )

namespace Thor::LLD::ADC
{
  /*-------------------------------------------------------------------------------
  Peripheral Memory Maps
  -------------------------------------------------------------------------------*/
  CommonRegisterMap *ADC_COMMON = reinterpret_cast<CommonRegisterMap *>( ADC_CMN_BASE_ADDR );

#if defined( STM32_ADC1_PERIPH_AVAILABLE )
  RegisterMap *ADC1_PERIPH = reinterpret_cast<RegisterMap *>( ADC1_BASE_ADDR );
#endif

  /*-------------------------------------------------------------------------------
  Configuration Maps
  -------------------------------------------------------------------------------*/
  namespace ConfigMap
  { /* clang-format off */
    /*-------------------------------------------------
    This must match exactly with Chimera::ADC::Sensor
    -------------------------------------------------*/
    LLD_CONST Chimera::ADC::Channel SensorToChannel[ static_cast<size_t>( Chimera::ADC::Sensor::NUM_OPTIONS ) ] = {
      Chimera::ADC::Channel::ADC_CH_18, // VBAT,
      Chimera::ADC::Channel::ADC_CH_17, // TEMP
      Chimera::ADC::Channel::ADC_CH_0,  // VREF_INT (RM 16.4.34)
    };
  } /* clang-format on */


  /*-------------------------------------------------------------------------------
  Peripheral Resources
  -------------------------------------------------------------------------------*/
  namespace Resource
  { /* clang-format off */

    LLD_CONST Thor::LLD::DMA::Source DMASignals[ NUM_ADC_PERIPHS ] = {
#if defined( STM32_ADC1_PERIPH_AVAILABLE )
      Thor::LLD::DMA::Source::ADC1,
#endif
    };

    LLD_CONST IRQn_Type IRQSignals[ NUM_ADC_PERIPHS ] = {
#if defined( STM32_ADC1_PERIPH_AVAILABLE )
      ADC_IRQn,
#endif
    };
  } /* clang-format on */
}    // namespace Thor::LLD::ADC

#endif /* TARGET_STM32L4 && THOR_LLD_ADC */
