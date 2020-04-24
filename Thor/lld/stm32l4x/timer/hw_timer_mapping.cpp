/********************************************************************************
 *  File Name:
 *    hw_timer_mapping.cpp
 *
 *  Description:
 *    Provides structures for conversion and mapping between data types for fast
 *    runtime performance of driver code.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* STL Includes */
#include <array>

/* Chimera Includes */
#include <Chimera/timer>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/stm32l4x/timer/hw_timer_mapping.hpp>
#include <Thor/lld/stm32l4x/timer/hw_timer_prj.hpp>
#include <Thor/lld/stm32l4x/timer/hw_timer_types.hpp>

#if defined( TARGET_STM32L4 ) && defined( THOR_LLD_TIMER )

namespace Thor::LLD::TIMER
{
  /*------------------------------------------------
  Chip Specific Resources
  ------------------------------------------------*/
  std::array<void *, NUM_TIMER_PERIPHS> LUT_PeripheralList;

  /*-------------------------------------------------------------------------------
  Chimera Option to Register Config Mappings
  -------------------------------------------------------------------------------*/
  /* clang-format off */
  // const std::array<uint32_t, static_cast<size_t>( Chimera::GPIO::Pull::NUM_OPTIONS )> PullMap = {};
  /* clang-format on */

  /*-------------------------------------------------------------------------------
  Timer Descriptor Mapping Data
  -------------------------------------------------------------------------------*/
  namespace
  {
    /* clang-format off */
    std::array<const DeviceDescription *, NUM_TIMER_PERIPHS> LUT_DeviceDescriptor;

    /*------------------------------------------------
    Timer 2 Description Resources
    ------------------------------------------------*/
    namespace T2
    {
      /*------------------------------------------------
      Supported Events
      ------------------------------------------------*/
      static std::array<Chimera::Timer::Event, 1> _EventList = {
        Chimera::Timer::Event::INVALID
      };

      static const Chimera::Algorithm::OptionsList Events = {
        .pData      = _EventList.data(),
        .totalSize  = sizeof( _EventList ),
        .objectSize = sizeof( Chimera::Timer::Event )
      };

      /*------------------------------------------------
      Supported Modes
      ------------------------------------------------*/
      static std::array<Chimera::Timer::Function, 4> _ModeList = {
        Chimera::Timer::Function::INPUT_CAPTURE,
        Chimera::Timer::Function::OUTPUT_COMPARE,
        Chimera::Timer::Function::PWM_OUTPUT,
        Chimera::Timer::Function::ONE_PULSE_OUTPUT
      };

      static const Chimera::Algorithm::OptionsList Modes = {
        .pData      = _ModeList.data(),
        .totalSize  = sizeof( _ModeList ),
        .objectSize = sizeof( Chimera::Timer::Function )
      };
    }

    static const DeviceDescription sTIMER2Description = {
      .counterWidth     = 32,
      .numChannels      = 4,
      .timerType        = Type::GENERAL_PURPOSE_TIMER,
      .registerMap      = TIMER2_PERIPH,
      .supportedEvents  = &T2::Events,
      .supportedModes   = &T2::Modes
    };

    /*------------------------------------------------
    As yet unsupported timers...
    ------------------------------------------------*/
    static const DeviceDescription sTIMER1Description = {};
    static const DeviceDescription sTIMER6Description = {};
    static const DeviceDescription sTIMER15Description = {};
    static const DeviceDescription sTIMER16Description = {};
    static const DeviceDescription sLPTIMER1Description = {};
    static const DeviceDescription sLPTIMER2Description = {};

    #if defined( STM32_TIMER3_PERIPH_AVAILABLE )
    static const DeviceDescription sTIMER3Description = {};
    #endif

    #if defined( STM32_TIMER7_PERIPH_AVAILABLE )
    static const DeviceDescription sTIMER7Description = {};
    #endif
    /* clang-format on */
  }

  /*-------------------------------------------------------------------------------
  Flat Map Data
  -------------------------------------------------------------------------------*/
  /* clang-format off */
  PTRIMapLLD PeripheralToLLDResourceIndex = {
    { Chimera::Timer::Peripheral::TIMER1,   TIMER1_RESOURCE_INDEX   },
    { Chimera::Timer::Peripheral::TIMER2,   TIMER2_RESOURCE_INDEX   },
    { Chimera::Timer::Peripheral::TIMER3,   TIMER3_RESOURCE_INDEX   },
    { Chimera::Timer::Peripheral::TIMER6,   TIMER6_RESOURCE_INDEX   },
    { Chimera::Timer::Peripheral::TIMER7,   TIMER7_RESOURCE_INDEX   },
    { Chimera::Timer::Peripheral::TIMER15,  TIMER15_RESOURCE_INDEX  },
    { Chimera::Timer::Peripheral::TIMER16,  TIMER16_RESOURCE_INDEX  },
    { Chimera::Timer::Peripheral::LPTIMER1, LPTIMER1_RESOURCE_INDEX },
    { Chimera::Timer::Peripheral::LPTIMER2, LPTIMER2_RESOURCE_INDEX }
  };

  PTRIMapHLD PeripheralToHLDResourceIndex = {
    { Chimera::Timer::Peripheral::TIMER1,   ADVANCED_TIMER1_RESOURCE_INDEX  },
    { Chimera::Timer::Peripheral::TIMER2,   GENERAL_TIMER1_RESOURCE_INDEX   },
    { Chimera::Timer::Peripheral::TIMER3,   GENERAL_TIMER2_RESOURCE_INDEX   },
    { Chimera::Timer::Peripheral::TIMER6,   BASIC_TIMER1_RESOURCE_INDEX     },
    { Chimera::Timer::Peripheral::TIMER7,   BASIC_TIMER2_RESOURCE_INDEX     },
    { Chimera::Timer::Peripheral::TIMER15,  GENERAL_TIMER3_RESOURCE_INDEX   },
    { Chimera::Timer::Peripheral::TIMER16,  GENERAL_TIMER4_RESOURCE_INDEX   },
    { Chimera::Timer::Peripheral::LPTIMER1, LOW_POWER_TIMER1_RESOURCE_INDEX },
    { Chimera::Timer::Peripheral::LPTIMER2, LOW_POWER_TIMER2_RESOURCE_INDEX }
  };
  /* clang-format on */

  /*-------------------------------------------------------------------------------
  Look Up Tables
  -------------------------------------------------------------------------------*/
  /* clang-format off */
  DirectionConverter LUT_Direction = {
    Configuration::Direction::COUNT_UP,
    Configuration::Direction::COUNT_DN
  };
  /* clang-format on */

  /*-------------------------------------------------------------------------------
  LLD Free Functions
  -------------------------------------------------------------------------------*/
  /*------------------------------------------------
  hw_timer_mapping.hpp
  ------------------------------------------------*/
  void initializeMapping()
  {
    /*------------------------------------------------
    Assign the peripheral list
    ------------------------------------------------*/
    LUT_PeripheralList[ TIMER1_RESOURCE_INDEX.value() ]   = TIMER1_PERIPH;
    LUT_PeripheralList[ TIMER2_RESOURCE_INDEX.value() ]   = TIMER2_PERIPH;
    LUT_PeripheralList[ TIMER6_RESOURCE_INDEX.value() ]   = TIMER6_PERIPH;
    LUT_PeripheralList[ TIMER15_RESOURCE_INDEX.value() ]  = TIMER15_PERIPH;
    LUT_PeripheralList[ TIMER16_RESOURCE_INDEX.value() ]  = TIMER16_PERIPH;
    LUT_PeripheralList[ LPTIMER1_RESOURCE_INDEX.value() ] = LPTIMER1_PERIPH;
    LUT_PeripheralList[ LPTIMER2_RESOURCE_INDEX.value() ] = LPTIMER2_PERIPH;

#if defined( STM32_TIMER3_PERIPH_AVAILABLE )
    LUT_PeripheralList[ TIMER3_RESOURCE_INDEX.value() ] = TIMER3_PERIPH;
#else
    LUT_PeripheralList[ TIMER3_RESOURCE_INDEX.value() ] = nullptr;
#endif

#if defined( STM32_TIMER7_PERIPH_AVAILABLE )
    LUT_PeripheralList[ TIMER7_RESOURCE_INDEX.value() ] = TIMER7_PERIPH;
#else
    LUT_PeripheralList[ TIMER7_RESOURCE_INDEX.value() ] = nullptr;
#endif

    /*------------------------------------------------
    Assign the device descriptors
    ------------------------------------------------*/
    LUT_DeviceDescriptor[ TIMER1_RESOURCE_INDEX.value() ]   = &sTIMER1Description;
    LUT_DeviceDescriptor[ TIMER2_RESOURCE_INDEX.value() ]   = &sTIMER2Description;
    LUT_DeviceDescriptor[ TIMER6_RESOURCE_INDEX.value() ]   = &sTIMER6Description;
    LUT_DeviceDescriptor[ TIMER15_RESOURCE_INDEX.value() ]  = &sTIMER15Description;
    LUT_DeviceDescriptor[ TIMER16_RESOURCE_INDEX.value() ]  = &sTIMER16Description;
    LUT_DeviceDescriptor[ LPTIMER1_RESOURCE_INDEX.value() ] = &sLPTIMER1Description;
    LUT_DeviceDescriptor[ LPTIMER2_RESOURCE_INDEX.value() ] = &sLPTIMER2Description;

#if defined( STM32_TIMER3_PERIPH_AVAILABLE )
    LUT_DeviceDescriptor[ TIMER3_RESOURCE_INDEX.value() ] = &sTIMER3Description;
#else
    LUT_DeviceDescriptor[ TIMER3_RESOURCE_INDEX.value() ] = nullptr;
#endif

#if defined( STM32_TIMER7_PERIPH_AVAILABLE )
    LUT_DeviceDescriptor[ TIMER7_RESOURCE_INDEX.value() ] = &sTIMER7Description;
#else
    LUT_DeviceDescriptor[ TIMER7_RESOURCE_INDEX.value() ] = nullptr;
#endif
  }



  /*------------------------------------------------
  timer_intf.hpp
  ------------------------------------------------*/

  const DeviceDescription *getPeripheralDescriptor( const Thor::LLD::RIndex resourceIndex )
  {
    if ( resourceIndex.value() < NUM_TIMER_PERIPHS )
    {
      return LUT_DeviceDescriptor[ resourceIndex.value() ];
    }
    else
    {
      return nullptr;
    }
  }

}    // namespace Thor::LLD::TIMER

#endif /* TARGET_STM32L4 && THOR_LLD_TIMER */
