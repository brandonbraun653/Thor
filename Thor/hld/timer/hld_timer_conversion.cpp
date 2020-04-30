/********************************************************************************
 *  File Name:
 *    hld_timer_conversion.cpp
 *
 *  Description:
 *    Implementation of conversions functions between STM32 timer driver types
 *    and drivers expected by Chimera.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Thor Includes */
#include <Thor/timer>
#include <Thor/hld/timer/hld_timer_prv_driver.hpp>
#include <Thor/lld/interface/timer/timer_intf.hpp>
#include <Thor/lld/interface/timer/timer_detail.hpp>

namespace Thor::TIMER
{
  namespace LLD = Thor::LLD::TIMER;


  bool timerHasFuntionality( const Chimera::Timer::Peripheral periph, const Chimera::Timer::Function func )
  {
    return true;
  }

  #if defined( VIRTUAL_FUNC )
  Chimera::Timer::ITimerBase_sPtr getTimerAsBase( const Chimera::Timer::Peripheral periph )
  {
    Chimera::Timer::ITimerBase_sPtr output_driver = nullptr;

    /*-------------------------------------------------
    Look up driver type associated with the peripheral
    -------------------------------------------------*/
    size_t x         = LLD::PeripheralToLLDResourceIndex.find( periph )->second;
    auto pDescriptor = LLD::getPeripheralDescriptor( x );
    auto type        = pDescriptor->timerType;

    /*-------------------------------------------------
    Reinterpret the driver to the proper type. If the driver hasn't been
    created yet, don't bother doing so. Force the user to properly initialize
    the driver using the correct channels.
    -------------------------------------------------*/

#define CAST_TO Chimera::Timer::ITimerBase
    switch ( type )
    {
      case LLD::Type::ADVANCED_TIMER:
      {
        AdvancedDriver_sPtr tmpDriver = getAdvancedDriver_sPtr( periph, false );
        output_driver                 = std::dynamic_pointer_cast<CAST_TO>( tmpDriver );
        return output_driver;
      }
      break;

      case LLD::Type::BASIC_TIMER:
      {
        BasicDriver_sPtr tmpDriver = getBasicDriver_sPtr( periph, false );
        output_driver              = std::dynamic_pointer_cast<CAST_TO>( tmpDriver );
        return output_driver;
      }
      break;

      case LLD::Type::GENERAL_PURPOSE_TIMER:
      {
        GeneralDriver_sPtr tmpDriver = getGeneralDriver_sPtr( periph, false );
        output_driver                = std::dynamic_pointer_cast<CAST_TO>( tmpDriver );
        return output_driver;
      }
      break;

      case LLD::Type::LOW_POWER_TIMER:
      {
        LowPowerDriver_sPtr tmpDriver = getLowPowerDriver_sPtr( periph, false );
        output_driver                 = std::dynamic_pointer_cast<CAST_TO>( tmpDriver );
        return output_driver;
      }
      break;

      default:
        // output_driver is default initialized to nullptr, which is the correct output
        break;
    }

#undef CAST_TO


    return output_driver;
  }

  Chimera::Timer::ITimerEncoder_sPtr getTimerAsENC( const Chimera::Timer::Peripheral periph )
  {
    Chimera::Timer::ITimerEncoder_sPtr output_driver = nullptr;

    /*-------------------------------------------------
    Ensure the peripheral will support the desired view
    -------------------------------------------------*/
    if ( timerHasFuntionality( periph, Chimera::Timer::Function::ENCODER ) )
    {
      /*-------------------------------------------------
      Look up driver type associated with the peripheral
      -------------------------------------------------*/
      size_t x         = LLD::PeripheralToLLDResourceIndex.find( periph )->second;
      auto pDescriptor = LLD::getPeripheralDescriptor( x );
      auto type        = pDescriptor->timerType;

      /*-------------------------------------------------
      Reinterpret the driver to the proper type. If the driver hasn't been
      created yet, don't bother doing so. Force the user to properly initialize
      the driver using the correct channels.
      -------------------------------------------------*/

#define CAST_TO Chimera::Timer::ITimerEncoder
      switch ( type )
      {
        case LLD::Type::ADVANCED_TIMER:
        {
          AdvancedDriver_sPtr tmpDriver = getAdvancedDriver_sPtr( periph, false );
          output_driver                 = std::dynamic_pointer_cast<CAST_TO>( tmpDriver );
          return output_driver;
        }
        break;

        case LLD::Type::BASIC_TIMER:
        {
          BasicDriver_sPtr tmpDriver = getBasicDriver_sPtr( periph, false );
          output_driver              = std::dynamic_pointer_cast<CAST_TO>( tmpDriver );
          return output_driver;
        }
        break;

        case LLD::Type::GENERAL_PURPOSE_TIMER:
        {
          GeneralDriver_sPtr tmpDriver = getGeneralDriver_sPtr( periph, false );
          output_driver                = std::dynamic_pointer_cast<CAST_TO>( tmpDriver );
          return output_driver;
        }
        break;

        case LLD::Type::LOW_POWER_TIMER:
        {
          LowPowerDriver_sPtr tmpDriver = getLowPowerDriver_sPtr( periph, false );
          output_driver                 = std::dynamic_pointer_cast<CAST_TO>( tmpDriver );
          return output_driver;
        }
        break;

        default:
          // output_driver is default initialized to nullptr, which is the correct output
          break;
      }

#undef CAST_TO
    }

    return output_driver;
  }

  Chimera::Timer::ITimerInputCapture_sPtr getTimerAsIC( const Chimera::Timer::Peripheral periph )
  {
    Chimera::Timer::ITimerInputCapture_sPtr output_driver = nullptr;

    /*-------------------------------------------------
    Ensure the peripheral will support the desired view
    -------------------------------------------------*/
    if ( timerHasFuntionality( periph, Chimera::Timer::Function::INPUT_CAPTURE ) )
    {
      /*-------------------------------------------------
      Look up driver type associated with the peripheral
      -------------------------------------------------*/
      size_t x         = LLD::PeripheralToLLDResourceIndex.find( periph )->second;
      auto pDescriptor = LLD::getPeripheralDescriptor( x );
      auto type        = pDescriptor->timerType;

      /*-------------------------------------------------
      Reinterpret the driver to the proper type. If the driver hasn't been
      created yet, don't bother doing so. Force the user to properly initialize
      the driver using the correct channels.
      -------------------------------------------------*/

#define CAST_TO Chimera::Timer::ITimerInputCapture
      switch ( type )
      {
        case LLD::Type::ADVANCED_TIMER:
        {
          AdvancedDriver_sPtr tmpDriver = getAdvancedDriver_sPtr( periph, false );
          output_driver                 = std::dynamic_pointer_cast<CAST_TO>( tmpDriver );
          return output_driver;
        }
        break;

        case LLD::Type::BASIC_TIMER:
        {
          BasicDriver_sPtr tmpDriver = getBasicDriver_sPtr( periph, false );
          output_driver              = std::dynamic_pointer_cast<CAST_TO>( tmpDriver );
          return output_driver;
        }
        break;

        case LLD::Type::GENERAL_PURPOSE_TIMER:
        {
          GeneralDriver_sPtr tmpDriver = getGeneralDriver_sPtr( periph, false );
          output_driver                = std::dynamic_pointer_cast<CAST_TO>( tmpDriver );
          return output_driver;
        }
        break;

        case LLD::Type::LOW_POWER_TIMER:
        {
          LowPowerDriver_sPtr tmpDriver = getLowPowerDriver_sPtr( periph, false );
          output_driver                 = std::dynamic_pointer_cast<CAST_TO>( tmpDriver );
          return output_driver;
        }
        break;

        default:
          // output_driver is default initialized to nullptr, which is the correct output
          break;
      }

#undef CAST_TO
    }

    return output_driver;
  }

  Chimera::Timer::ITimerOutputCompare_sPtr getTimerAsOC( const Chimera::Timer::Peripheral periph )
  {
    Chimera::Timer::ITimerOutputCompare_sPtr output_driver = nullptr;

    /*-------------------------------------------------
    Ensure the peripheral will support the desired view
    -------------------------------------------------*/
    if ( timerHasFuntionality( periph, Chimera::Timer::Function::OUTPUT_COMPARE ) )
    {
      /*-------------------------------------------------
      Look up driver type associated with the peripheral
      -------------------------------------------------*/
      size_t x         = LLD::PeripheralToLLDResourceIndex.find( periph )->second;
      auto pDescriptor = LLD::getPeripheralDescriptor( x );
      auto type        = pDescriptor->timerType;

      /*-------------------------------------------------
      Reinterpret the driver to the proper type. If the driver hasn't been
      created yet, don't bother doing so. Force the user to properly initialize
      the driver using the correct channels.
      -------------------------------------------------*/

#define CAST_TO Chimera::Timer::ITimerOutputCompare
      switch ( type )
      {
        case LLD::Type::ADVANCED_TIMER:
        {
          AdvancedDriver_sPtr tmpDriver = getAdvancedDriver_sPtr( periph, false );
          output_driver                 = std::dynamic_pointer_cast<CAST_TO>( tmpDriver );
          return output_driver;
        }
        break;

        case LLD::Type::BASIC_TIMER:
        {
          BasicDriver_sPtr tmpDriver = getBasicDriver_sPtr( periph, false );
          output_driver              = std::dynamic_pointer_cast<CAST_TO>( tmpDriver );
          return output_driver;
        }
        break;

        case LLD::Type::GENERAL_PURPOSE_TIMER:
        {
          GeneralDriver_sPtr tmpDriver = getGeneralDriver_sPtr( periph, false );
          output_driver                = std::dynamic_pointer_cast<CAST_TO>( tmpDriver );
          return output_driver;
        }
        break;

        case LLD::Type::LOW_POWER_TIMER:
        {
          LowPowerDriver_sPtr tmpDriver = getLowPowerDriver_sPtr( periph, false );
          output_driver                 = std::dynamic_pointer_cast<CAST_TO>( tmpDriver );
          return output_driver;
        }
        break;
      }

#undef CAST_TO
    }

    return output_driver;
  }

  Chimera::Timer::ITimerOnePulse_sPtr getTimerAsOP( const Chimera::Timer::Peripheral periph )
  {
    Chimera::Timer::ITimerOnePulse_sPtr output_driver = nullptr;

    /*-------------------------------------------------
    Ensure the peripheral will support the desired view
    -------------------------------------------------*/
    if ( timerHasFuntionality( periph, Chimera::Timer::Function::ONE_PULSE_OUTPUT ) )
    {
      /*-------------------------------------------------
      Look up driver type associated with the peripheral
      -------------------------------------------------*/
      size_t x         = LLD::PeripheralToLLDResourceIndex.find( periph )->second;
      auto pDescriptor = LLD::getPeripheralDescriptor( x );
      auto type        = pDescriptor->timerType;

      /*-------------------------------------------------
      Reinterpret the driver to the proper type. If the driver hasn't been
      created yet, don't bother doing so. Force the user to properly initialize
      the driver using the correct channels.
      -------------------------------------------------*/

#define CAST_TO Chimera::Timer::ITimerOnePulse
      switch ( type )
      {
        case LLD::Type::ADVANCED_TIMER:
        {
          AdvancedDriver_sPtr tmpDriver = getAdvancedDriver_sPtr( periph, false );
          output_driver                 = std::dynamic_pointer_cast<CAST_TO>( tmpDriver );
          return output_driver;
        }
        break;

        case LLD::Type::BASIC_TIMER:
        {
          BasicDriver_sPtr tmpDriver = getBasicDriver_sPtr( periph, false );
          output_driver              = std::dynamic_pointer_cast<CAST_TO>( tmpDriver );
          return output_driver;
        }
        break;

        case LLD::Type::GENERAL_PURPOSE_TIMER:
        {
          GeneralDriver_sPtr tmpDriver = getGeneralDriver_sPtr( periph, false );
          output_driver                = std::dynamic_pointer_cast<CAST_TO>( tmpDriver );
          return output_driver;
        }
        break;

        case LLD::Type::LOW_POWER_TIMER:
        {
          LowPowerDriver_sPtr tmpDriver = getLowPowerDriver_sPtr( periph, false );
          output_driver                 = std::dynamic_pointer_cast<CAST_TO>( tmpDriver );
          return output_driver;
        }
        break;

        default:
          // output_driver is default initialized to nullptr, which is the correct output
          break;
      }

#undef CAST_TO
    }

    return output_driver;
  }

  Chimera::Timer::ITimerPWM_sPtr getTimerAsPWM( const Chimera::Timer::Peripheral periph )
  {
    Chimera::Timer::ITimerPWM_sPtr output_driver = nullptr;

    /*-------------------------------------------------
    Ensure the peripheral will support the desired view
    -------------------------------------------------*/
    if ( timerHasFuntionality( periph, Chimera::Timer::Function::PWM_OUTPUT ) )
    {
      /*-------------------------------------------------
      Look up driver type associated with the peripheral
      -------------------------------------------------*/
      size_t x         = LLD::PeripheralToLLDResourceIndex.find( periph )->second;
      auto pDescriptor = LLD::getPeripheralDescriptor( x );
      auto type        = pDescriptor->timerType;

      /*-------------------------------------------------
      Reinterpret the driver to the proper type. If the driver hasn't been
      created yet, don't bother doing so. Force the user to properly initialize
      the driver using the correct channels.
      -------------------------------------------------*/

#define CAST_TO Chimera::Timer::ITimerPWM
      switch ( type )
      {
        case LLD::Type::ADVANCED_TIMER:
        {
          AdvancedDriver_sPtr tmpDriver = getAdvancedDriver_sPtr( periph, false );
          output_driver                 = std::dynamic_pointer_cast<CAST_TO>( tmpDriver );
          return output_driver;
        }
        break;

        case LLD::Type::BASIC_TIMER:
        {
          BasicDriver_sPtr tmpDriver = getBasicDriver_sPtr( periph, false );
          output_driver              = std::dynamic_pointer_cast<CAST_TO>( tmpDriver );
          return output_driver;
        }
        break;

        case LLD::Type::GENERAL_PURPOSE_TIMER:
        {
          GeneralDriver_sPtr tmpDriver = getGeneralDriver_sPtr( periph, false );
          output_driver                = std::dynamic_pointer_cast<CAST_TO>( tmpDriver );
          return output_driver;
        }
        break;

        case LLD::Type::LOW_POWER_TIMER:
        {
          LowPowerDriver_sPtr tmpDriver = getLowPowerDriver_sPtr( periph, false );
          output_driver                 = std::dynamic_pointer_cast<CAST_TO>( tmpDriver );
          return output_driver;
        }
        break;

        default:
          // output_driver is default initialized to nullptr, which is the correct output
          break;
      }

#undef CAST_TO
    }

    return output_driver;
  }

  #else

  static std::array<void *, static_cast<size_t>( Chimera::Timer::Peripheral::NUM_OPTIONS )> s_hld_timer_drivers;
  static std::array<size_t, static_cast<size_t>( Chimera::Timer::Peripheral::NUM_OPTIONS )>
      s_chimera_peripheral_to_resource_index;

  

  Chimera::Timer::ITimerBase_sPtr getTimerAsBase( const Chimera::Timer::Peripheral periph )
  {
    auto iLookup = static_cast<size_t>( periph );

    /*------------------------------------------------
    Initialize local HLD object memory cache for the given resource
    ------------------------------------------------*/
    if ( !s_hld_timer_drivers[ iLookup ] ) 
    {
      auto iResource = Thor::LLD::TIMER::PeripheralToHLDResourceIndex.find( periph )->second;
      auto hld = Thor::TIMER::getDriverAddress( periph, iResource );

      s_hld_timer_drivers[ iLookup ] = hld;
      s_chimera_peripheral_to_resource_index[ iLookup ] = iResource;
    }

    return reinterpret_cast<Chimera::Timer::TimerBaseImpl *>( s_hld_timer_drivers[ iLookup ] );
  }

  Chimera::Timer::ITimerEncoder_sPtr getTimerAsENC( const Chimera::Timer::Peripheral periph )
  {
    return nullptr;
  }

  Chimera::Timer::ITimerInputCapture_sPtr getTimerAsIC( const Chimera::Timer::Peripheral periph )
  {
    return nullptr;
  }

  Chimera::Timer::ITimerOutputCompare_sPtr getTimerAsOC( const Chimera::Timer::Peripheral periph )
  {
    return nullptr;
  }

  Chimera::Timer::ITimerOnePulse_sPtr getTimerAsOP( const Chimera::Timer::Peripheral periph )
  {
    return nullptr;
  }

  Chimera::Timer::ITimerPWM_sPtr getTimerAsPWM( const Chimera::Timer::Peripheral periph )
  {
    auto iLookup = static_cast<size_t>( periph );

    /*------------------------------------------------
    Initialize local HLD object memory cache for the given resource
    ------------------------------------------------*/
    if ( timerHasFuntionality( periph, Chimera::Timer::Function::PWM_OUTPUT ) && !s_hld_timer_drivers[ iLookup ] ) 
    {
      auto iResource = Thor::LLD::TIMER::PeripheralToHLDResourceIndex.find( periph )->second;
      auto hld = Thor::TIMER::getDriverAddress( periph, iResource );

      s_hld_timer_drivers[ iLookup ] = hld;
      s_chimera_peripheral_to_resource_index[ iLookup ] = iResource;
    }

    return reinterpret_cast<Chimera::Timer::TimerPWMImpl *>( s_hld_timer_drivers[ iLookup ] );
  }

  #endif 

}    // namespace Thor::TIMER
