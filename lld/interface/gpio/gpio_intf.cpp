/********************************************************************************
 *  File Name:
 *    gpio_intf.cpp
 *
 *  Description:
 *    LLD interface functions that are processor independent
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* STL Includes */
#include <type_traits>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/utility>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/inc/gpio>

#if defined( THOR_LLD_GPIO )
namespace Thor::LLD::GPIO
{
  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
#if defined( THOR_LLD_GPIO )
  /* clang-format off */
  static const uint8_t portIndex[ DRIVER_MAX_PORTS ] = {
    GPIOA_RESOURCE_INDEX,
    GPIOB_RESOURCE_INDEX,
    GPIOC_RESOURCE_INDEX,
    GPIOD_RESOURCE_INDEX,
    GPIOE_RESOURCE_INDEX,
    GPIOF_RESOURCE_INDEX,
    GPIOG_RESOURCE_INDEX,
    GPIOH_RESOURCE_INDEX,
    GPIOI_RESOURCE_INDEX,
    GPIOJ_RESOURCE_INDEX,
    GPIOK_RESOURCE_INDEX,
    GPIOL_RESOURCE_INDEX
  };

  static const uint8_t pinOffset[ DRIVER_MAX_PORTS ] = {
    GPIOA_PIN_RINDEX_OFFSET,
    GPIOB_PIN_RINDEX_OFFSET,
    GPIOC_PIN_RINDEX_OFFSET,
    GPIOD_PIN_RINDEX_OFFSET,
    GPIOE_PIN_RINDEX_OFFSET,
    GPIOF_PIN_RINDEX_OFFSET,
    GPIOG_PIN_RINDEX_OFFSET,
    GPIOH_PIN_RINDEX_OFFSET,
    GPIOI_PIN_RINDEX_OFFSET,
    GPIOJ_PIN_RINDEX_OFFSET,
    GPIOK_PIN_RINDEX_OFFSET,
    GPIOL_PIN_RINDEX_OFFSET
  };
  /* clang-format on */
#endif  // THOR_LLD_GPIO


  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  bool isSupported( const Chimera::GPIO::Port port, const Chimera::GPIO::Pin pin )
  {
    /*-------------------------------------------------
    Walk the configuration tree to find the desired
    alternate function configuration
    -------------------------------------------------*/
    bool pin_is_supported = false;

    for ( size_t portIdx = 0; portIdx < PRJ_MAX_PORTS; portIdx++ )
    {
      /*-------------------------------------------------
      Nothing yet...moving on.
      -------------------------------------------------*/
      if ( prjPortAttributes[ portIdx ].portID != port )
      {
        continue;
      }

      /*-------------------------------------------------
      Found the port! Check if it contains the desired pin
      -------------------------------------------------*/
      for ( size_t pinIdx = 0; pinIdx < prjPortAttributes[ portIdx ].pinListSize; pinIdx++ )
      {
        /*-------------------------------------------------
        *sigh...still nothing.
        -------------------------------------------------*/
        if ( prjPortAttributes[ portIdx ].pins[ pinIdx ].pinID != pin )
        {
          continue;
        }

        /*-------------------------------------------------
        Found the right pin, don't search further!
        -------------------------------------------------*/
        pin_is_supported = true;
        break;
      }

      /*-------------------------------------------------
      Found the right port, don't search further!
      -------------------------------------------------*/
      break;
    }

    return pin_is_supported;
  }


  RIndex_t getResourceIndex( const std::uintptr_t address )
  {
    /*-------------------------------------------------
    Look through all the registered port addresses and
    see if the given address parameter matches.
    -------------------------------------------------*/
    for ( size_t idx = 0; idx < ARRAY_COUNT( prjPortAddress ); idx++ )
    {
      if ( address != prjPortAddress[ idx ] )
      {
        continue;
      }

      /*-------------------------------------------------
      We support it! Figure out which peripheral it is
      -------------------------------------------------*/
      if ( address == reinterpret_cast<std::uintptr_t>( GPIOA_PERIPH ) )
      {
        return GPIOA_RESOURCE_INDEX;
      }
#if defined( STM32_GPIOB_PERIPH_AVAILABLE )
      else if ( address == reinterpret_cast<std::uintptr_t>( GPIOB_PERIPH ) )
      {
        return GPIOB_RESOURCE_INDEX;
      }
#endif
#if defined( STM32_GPIOC_PERIPH_AVAILABLE )
      else if ( address == reinterpret_cast<std::uintptr_t>( GPIOC_PERIPH ) )
      {
        return GPIOC_RESOURCE_INDEX;
      }
#endif
#if defined( STM32_GPIOD_PERIPH_AVAILABLE )
      else if ( address == reinterpret_cast<std::uintptr_t>( GPIOD_PERIPH ) )
      {
        return GPIOD_RESOURCE_INDEX;
      }
#endif
#if defined( STM32_GPIOE_PERIPH_AVAILABLE )
      else if ( address == reinterpret_cast<std::uintptr_t>( GPIOE_PERIPH ) )
      {
        return GPIOE_RESOURCE_INDEX;
      }
#endif
#if defined( STM32_GPIOF_PERIPH_AVAILABLE )
      else if ( address == reinterpret_cast<std::uintptr_t>( GPIOF_PERIPH ) )
      {
        return GPIOF_RESOURCE_INDEX;
      }
#endif
#if defined( STM32_GPIOG_PERIPH_AVAILABLE )
      else if ( address == reinterpret_cast<std::uintptr_t>( GPIOG_PERIPH ) )
      {
        return GPIOG_RESOURCE_INDEX;
      }
#endif
#if defined( STM32_GPIOH_PERIPH_AVAILABLE )
      else if ( address == reinterpret_cast<std::uintptr_t>( GPIOH_PERIPH ) )
      {
        return GPIOH_RESOURCE_INDEX;
      }
#endif
#if defined( STM32_GPIOI_PERIPH_AVAILABLE )
      else if ( address == reinterpret_cast<std::uintptr_t>( GPIOI_PERIPH ) )
      {
        return GPIOI_RESOURCE_INDEX;
      }
#endif
#if defined( STM32_GPIOJ_PERIPH_AVAILABLE )
      else if ( address == reinterpret_cast<std::uintptr_t>( GPIOJ_PERIPH ) )
      {
        return GPIOJ_RESOURCE_INDEX;
      }
#endif
#if defined( STM32_GPIOK_PERIPH_AVAILABLE )
      else if ( address == reinterpret_cast<std::uintptr_t>( GPIOK_PERIPH ) )
      {
        return GPIOK_RESOURCE_INDEX;
      }
#endif
      else
      {
        return INVALID_RESOURCE_INDEX;
      }
    }

    return INVALID_RESOURCE_INDEX;
  }


  RIndex_t getResourceIndex( const Chimera::GPIO::Port port )
  {
    switch ( port )
    {
#if defined( STM32_GPIOB_PERIPH_AVAILABLE )
      case Chimera::GPIO::Port::PORTA:
        return GPIOA_RESOURCE_INDEX;
        break;
#endif
#if defined( STM32_GPIOB_PERIPH_AVAILABLE )
      case Chimera::GPIO::Port::PORTB:
        return GPIOB_RESOURCE_INDEX;
        break;
#endif
#if defined( STM32_GPIOC_PERIPH_AVAILABLE )
      case Chimera::GPIO::Port::PORTC:
        return GPIOC_RESOURCE_INDEX;
        break;
#endif
#if defined( STM32_GPIOD_PERIPH_AVAILABLE )
      case Chimera::GPIO::Port::PORTD:
        return GPIOD_RESOURCE_INDEX;
        break;
#endif
#if defined( STM32_GPIOE_PERIPH_AVAILABLE )
      case Chimera::GPIO::Port::PORTE:
        return GPIOE_RESOURCE_INDEX;
        break;
#endif
#if defined( STM32_GPIOF_PERIPH_AVAILABLE )
      case Chimera::GPIO::Port::PORTF:
        return GPIOF_RESOURCE_INDEX;
        break;
#endif
#if defined( STM32_GPIOG_PERIPH_AVAILABLE )
      case Chimera::GPIO::Port::PORTG:
        return GPIOG_RESOURCE_INDEX;
        break;
#endif
#if defined( STM32_GPIOH_PERIPH_AVAILABLE )
      case Chimera::GPIO::Port::PORTH:
        return GPIOH_RESOURCE_INDEX;
        break;
#endif
#if defined( STM32_GPIOI_PERIPH_AVAILABLE )
      case Chimera::GPIO::Port::PORTI:
        return GPIOI_RESOURCE_INDEX;
        break;
#endif
#if defined( STM32_GPIOJ_PERIPH_AVAILABLE )
      case Chimera::GPIO::Port::PORTJ:
        return GPIOJ_RESOURCE_INDEX;
        break;
#endif
#if defined( STM32_GPIOK_PERIPH_AVAILABLE )
      case Chimera::GPIO::Port::PORTK:
        return GPIOK_RESOURCE_INDEX;
        break;
#endif
      default:
        return INVALID_RESOURCE_INDEX;
        break;
    };
  }


  RIndex_t getPinResourceIndex( const Chimera::GPIO::Port port, const Chimera::GPIO::Pin pin )
  {
    auto retVal = INVALID_RESOURCE_INDEX;

    /*-------------------------------------------------
    Boundary check against the project's description
    -------------------------------------------------*/
    if ( !isSupported( port, pin ) )
    {
      return retVal;
    }

    /*-------------------------------------------------
    Compute the resource index
    -------------------------------------------------*/
    // Base offset from a port perspective
    const size_t offset = pinOffset[ static_cast<uint8_t>( port ) ];

    // Get the port attributes that say how many pins are configured for the port
    const PortAttributes *availablePins = getPortAttributes( port );

    // Search through the port's registered pins
    for ( size_t pinIdx = 0; pinIdx < availablePins->pinListSize; pinIdx++ )
    {
      if ( availablePins->pins[ pinIdx ].pinID != pin )
      {
        continue;
      }

      // Not every pin may exist (might be holes..1,2,4,8) so use the positional index
      retVal = offset + pinIdx;
      break;
    }

    return retVal;
  }


  bool attachDriverInstances( Driver *const driverList, const size_t numDrivers )
  {
    /*-------------------------------------------------
    Algorithm vars
    -------------------------------------------------*/
    size_t initializedPorts         = 0;
    Chimera::GPIO::Port currentPort = Chimera::GPIO::Port::PORTA;
    Chimera::GPIO::Pin currentPin   = 0;
    RIndex_t resourceIndex          = 0;

    /*-------------------------------------------------
    Reject bad inputs
    -------------------------------------------------*/
    if ( !driverList || !numDrivers || ( numDrivers > NUM_GPIO_PERIPHS ) )
    {
      return false;
    }

    /*-------------------------------------------------
    Walk the port attribute configuration
    -------------------------------------------------*/
    for ( size_t portIdx = 0; portIdx < PRJ_MAX_PORTS; portIdx++ )
    {
      /*-------------------------------------------------
      Find the peripheral memory map of the current port
      -------------------------------------------------*/
      RegisterMap *periphInstance;
      currentPort = prjPortAttributes[ portIdx ].portID;

      switch ( currentPort )
      {
#if defined( STM32_GPIOA_PERIPH_AVAILABLE )
        case Chimera::GPIO::Port::PORTA:
          periphInstance = GPIOA_PERIPH;
          break;
#endif
#if defined( STM32_GPIOB_PERIPH_AVAILABLE )
        case Chimera::GPIO::Port::PORTB:
          periphInstance = GPIOB_PERIPH;
          break;
#endif
#if defined( STM32_GPIOC_PERIPH_AVAILABLE )
        case Chimera::GPIO::Port::PORTC:
          periphInstance = GPIOC_PERIPH;
          break;
#endif
#if defined( STM32_GPIOD_PERIPH_AVAILABLE )
        case Chimera::GPIO::Port::PORTD:
          periphInstance = GPIOD_PERIPH;
          break;
#endif
#if defined( STM32_GPIOE_PERIPH_AVAILABLE )
        case Chimera::GPIO::Port::PORTE:
          periphInstance = GPIOE_PERIPH;
          break;
#endif
#if defined( STM32_GPIOF_PERIPH_AVAILABLE )
        case Chimera::GPIO::Port::PORTF:
          periphInstance = GPIOF_PERIPH;
          break;
#endif
#if defined( STM32_GPIOG_PERIPH_AVAILABLE )
        case Chimera::GPIO::Port::PORTG:
          periphInstance = GPIOG_PERIPH;
          break;
#endif
#if defined( STM32_GPIOH_PERIPH_AVAILABLE )
        case Chimera::GPIO::Port::PORTH:
          periphInstance = GPIOH_PERIPH;
          break;
#endif
#if defined( STM32_GPIOI_PERIPH_AVAILABLE )
        case Chimera::GPIO::Port::PORTI:
          periphInstance = GPIOI_PERIPH;
          break;
#endif
#if defined( STM32_GPIOJ_PERIPH_AVAILABLE )
        case Chimera::GPIO::Port::PORTJ:
          periphInstance = GPIOJ_PERIPH;
          break;
#endif
#if defined( STM32_GPIOK_PERIPH_AVAILABLE )
        case Chimera::GPIO::Port::PORTK:
          periphInstance = GPIOK_PERIPH;
          break;
#endif
        default:
          /*-------------------------------------------------
          An unsupported port was attempted to be initialized
          -------------------------------------------------*/
          Chimera::insert_debug_breakpoint();
          return false;
          break;
      }

      /*-------------------------------------------------
      Attach the peripheral
      -------------------------------------------------*/
      resourceIndex = portIndex[ portIdx ];
      driverList[ resourceIndex ].attach( periphInstance );
      initializedPorts++;

      /*-------------------------------------------------
      Ensure we don't initialize too many pins
      -------------------------------------------------*/
      if ( initializedPorts < numDrivers )
      {
        continue;    // More things left to do
      }
      else if ( ( portIdx + 1 ) < PRJ_MAX_PORTS )
      {
        return false;    // Hit the limit, but the loop thinks we still have more to go.
      }
      else
      {
        break;
      }
    }

    /*-------------------------------------------------
    One last check: Were all the expected drivers initialized?
    -------------------------------------------------*/
    return initializedPorts == numDrivers;
  }


  Reg32_t findAlternateFunction( const Chimera::GPIO::Port port, const Chimera::GPIO::Pin pin,
                                 const Chimera::GPIO::Alternate alt )
  {
    /*-------------------------------------------------
    Design Note:
    Don't bother verifying the arguments because the
    function which performs that behavior also walks
    the tree in the same manner as below. There is no
    sense in doing that twice.
    -------------------------------------------------*/

    /*-------------------------------------------------
    Walk the configuration tree to find the desired
    alternate function configuration
    -------------------------------------------------*/
    Reg32_t altFunctionConfig = BAD_ALT_FUNC;

    for ( size_t portIdx = 0; portIdx < PRJ_MAX_PORTS; portIdx++ )
    {
      /*-------------------------------------------------
      Nothing yet...moving on.
      -------------------------------------------------*/
      if ( prjPortAttributes[ portIdx ].portID != port )
      {
        continue;
      }

      /*-------------------------------------------------
      Found the port! Check if it contains the desired pin
      -------------------------------------------------*/
      for ( size_t pinIdx = 0; pinIdx < prjPortAttributes[ portIdx ].pinListSize; pinIdx++ )
      {
        /*-------------------------------------------------
        *sigh...still nothing.
        -------------------------------------------------*/
        if ( prjPortAttributes[ portIdx ].pins[ pinIdx ].pinID != pin )
        {
          continue;
        }

        /*-------------------------------------------------
        Found the pin! Check if it supports the alternate function
        -------------------------------------------------*/
        for ( size_t altIdx = 0; altIdx < prjPortAttributes[ portIdx ].pins[ pinIdx ].afListSize; altIdx++ )
        {
          /*-------------------------------------------------
          Ugh are you kidding!? Still nothing?!
          -------------------------------------------------*/
          if ( prjPortAttributes[ portIdx ].pins[ pinIdx ].altFunc[ altIdx ].chimeraAltFunc != alt )
          {
            continue;
          }

          /*-------------------------------------------------
          FINALLY WE FOUND IT. PRAISE THE MCU GODS.
          -------------------------------------------------*/
          altFunctionConfig = prjPortAttributes[ portIdx ].pins[ pinIdx ].altFunc[ altIdx ].registerAltFunc;
          break;
        }

        /*-------------------------------------------------
        Found the right pin, don't search further!
        -------------------------------------------------*/
        break;
      }

      /*-------------------------------------------------
      Found the right port, don't search further!
      -------------------------------------------------*/
      break;
    }

    return altFunctionConfig;
  }


  const PortAttributes *getPortAttributes( const Chimera::GPIO::Port port )
  {
    for ( size_t portIdx = 0; portIdx < PRJ_MAX_PORTS; portIdx++ )
    {
      /*-------------------------------------------------
      Nothing yet...moving on.
      -------------------------------------------------*/
      if ( prjPortAttributes[ portIdx ].portID != port )
      {
        continue;
      }

      /*-------------------------------------------------
      Found it!
      -------------------------------------------------*/
      return &prjPortAttributes[ portIdx ];
    }

    return nullptr;
  }


  const PinAttributes *getPinAttributes( const Chimera::GPIO::Port port, const Chimera::GPIO::Pin pin )
  {
    for ( size_t portIdx = 0; portIdx < PRJ_MAX_PORTS; portIdx++ )
    {
      /*-------------------------------------------------
      Nothing yet...moving on.
      -------------------------------------------------*/
      if ( prjPortAttributes[ portIdx ].portID != port )
      {
        continue;
      }

      /*-------------------------------------------------
      Found the port! Check if it contains the desired pin
      -------------------------------------------------*/
      for ( size_t pinIdx = 0; pinIdx < prjPortAttributes[ portIdx ].pinListSize; pinIdx++ )
      {
        /*-------------------------------------------------
        *sigh...still nothing.
        -------------------------------------------------*/
        if ( prjPortAttributes[ portIdx ].pins[ pinIdx ].pinID != pin )
        {
          continue;
        }

        /*-------------------------------------------------
        Found the right pin, don't search further!
        -------------------------------------------------*/
        return &prjPortAttributes[ portIdx ].pins[ pinIdx ];
      }

      /*-------------------------------------------------
      Found the right port, don't search further!
      -------------------------------------------------*/
      break;
    }

    return nullptr;
  }


  Chimera::GPIO::Port getPort( const std::uintptr_t address )
  {
    /*-------------------------------------------------
    Look through all the registered port addresses and
    see if the given address parameter matches.
    -------------------------------------------------*/
    for ( size_t idx = 0; idx < ARRAY_COUNT( prjPortAddress ); idx++ )
    {
      if ( address != prjPortAddress[ idx ] )
      {
        continue;
      }

      /*-------------------------------------------------
      We support it! Figure out which peripheral it is
      -------------------------------------------------*/
      if ( address == reinterpret_cast<std::uintptr_t>( GPIOA_PERIPH ) )
      {
        return Chimera::GPIO::Port::PORTA;
      }
#if defined( STM32_GPIOB_PERIPH_AVAILABLE )
      else if ( address == reinterpret_cast<std::uintptr_t>( GPIOB_PERIPH ) )
      {
        return Chimera::GPIO::Port::PORTB;
      }
#endif
#if defined( STM32_GPIOC_PERIPH_AVAILABLE )
      else if ( address == reinterpret_cast<std::uintptr_t>( GPIOC_PERIPH ) )
      {
        return Chimera::GPIO::Port::PORTC;
      }
#endif
#if defined( STM32_GPIOD_PERIPH_AVAILABLE )
      else if ( address == reinterpret_cast<std::uintptr_t>( GPIOD_PERIPH ) )
      {
        return Chimera::GPIO::Port::PORTD;
      }
#endif
#if defined( STM32_GPIOE_PERIPH_AVAILABLE )
      else if ( address == reinterpret_cast<std::uintptr_t>( GPIOE_PERIPH ) )
      {
        return Chimera::GPIO::Port::PORTE;
      }
#endif
#if defined( STM32_GPIOF_PERIPH_AVAILABLE )
      else if ( address == reinterpret_cast<std::uintptr_t>( GPIOF_PERIPH ) )
      {
        return Chimera::GPIO::Port::PORTF;
      }
#endif
#if defined( STM32_GPIOG_PERIPH_AVAILABLE )
      else if ( address == reinterpret_cast<std::uintptr_t>( GPIOG_PERIPH ) )
      {
        return Chimera::GPIO::Port::PORTG;
      }
#endif
#if defined( STM32_GPIOH_PERIPH_AVAILABLE )
      else if ( address == reinterpret_cast<std::uintptr_t>( GPIOH_PERIPH ) )
      {
        return Chimera::GPIO::Port::PORTH;
      }
#endif
#if defined( STM32_GPIOI_PERIPH_AVAILABLE )
      else if ( address == reinterpret_cast<std::uintptr_t>( GPIOI_PERIPH ) )
      {
        return Chimera::GPIO::Port::PORTI;
      }
#endif
#if defined( STM32_GPIOJ_PERIPH_AVAILABLE )
      else if ( address == reinterpret_cast<std::uintptr_t>( GPIOJ_PERIPH ) )
      {
        return Chimera::GPIO::Port::PORTJ;
      }
#endif
#if defined( STM32_GPIOK_PERIPH_AVAILABLE )
      else if ( address == reinterpret_cast<std::uintptr_t>( GPIOK_PERIPH ) )
      {
        return Chimera::GPIO::Port::PORTK;
      }
#endif
      else
      {
        return Chimera::GPIO::Port::UNKNOWN_PORT;
      }
    }

    return Chimera::GPIO::Port::UNKNOWN_PORT;
  }


  Chimera::EXTI::EventLine_t findEventLine( const Chimera::GPIO::Port port, const Chimera::GPIO::Pin pin )
  {
    static_assert( sizeof( Chimera::EXTI::EventLine_t ) == sizeof( Chimera::GPIO::Pin ) );

    /*-------------------------------------------------
    Luckily ST seems to have made this simple. Each pin
    is directly mapped to a line of the same value.
    -------------------------------------------------*/
#if defined( STM32L432xx )
    return static_cast<Chimera::EXTI::EventLine_t>( pin );
#else
#pragma error( "Evaluate your processor for EXTI configuration" )
#endif
  }

}    // namespace Thor::LLD::GPIO

#endif  /* THOR_LLD_GPIO */
