/********************************************************************************
 *  File Name:
 *    gpio_intf.cpp
 *
 *  Description:
 *    LLD interface functions that are processor independent
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Chimera Includes */
#include <Chimera/common>

/* Thor Includes */
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/gpio/gpio_detail.hpp>
#include <Thor/lld/interface/gpio/gpio_prv_data.hpp>
#include <Thor/lld/interface/gpio/gpio_types.hpp>

namespace Thor::LLD::GPIO
{
  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  /* clang-format off */
  const uint8_t portIndex[ DRIVER_MAX_PORTS ] = {
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
  /* clang-format on */


  /*-------------------------------------------------------------------------------
  Private Functions
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
      if( portAttributes[ portIdx ].portID != port )
      {
        continue;
      }

      /*-------------------------------------------------
      Found the port! Check if it contains the desired pin
      -------------------------------------------------*/
      for ( size_t pinIdx = 0; pinIdx < portAttributes[ portIdx ].pinListSize; pinIdx++ )
      {
        /*-------------------------------------------------
        *sigh...still nothing.
        -------------------------------------------------*/
        if( portAttributes[ portIdx ].pins[ pinIdx ].pinID != pin )
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


  RIndex_t getResourceIndex( const Chimera::GPIO::Port port, const Chimera::GPIO::Pin pin )
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
    auto _port = static_cast<uint8_t>( port );
    auto _pin  = static_cast<uint8_t>( pin );
    retVal = portIndex[ _port ] + _pin;
    return retVal;
  }


  bool attachDriverInstances( Driver *const driverList, const size_t numDrivers )
  {
    /*-------------------------------------------------
    Algorithm vars
    -------------------------------------------------*/
    size_t initializedPins          = 0;
    Chimera::GPIO::Port currentPort = Chimera::GPIO::Port::PORTA;
    Chimera::GPIO::Pin currentPin   = 0;
    RIndex_t resourceIndex          = 0;

    /*-------------------------------------------------
    Walk the port attribute configuration
    -------------------------------------------------*/
    for ( size_t portIdx = 0; portIdx < PRJ_MAX_PORTS; portIdx++ )
    {
      /*-------------------------------------------------
      Find the peripheral memory map of the current port
      -------------------------------------------------*/
      RegisterMap *periphInstance;
      currentPort = portAttributes[ portIdx ].portID;

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
      Walk each pin and attach the found peripheral
      -------------------------------------------------*/
      for ( size_t pinIdx = 0; pinIdx < portAttributes[ portIdx ].pinListSize; pinIdx++ )
      {
        currentPin = portAttributes[ portIdx ].pins[ pinIdx ].pinID;
        resourceIndex = getResourceIndex( currentPort, currentPin );

        /*-------------------------------------------------
        Attach the peripheral
        -------------------------------------------------*/
        driverList[ resourceIndex ].attach( periphInstance );
        driverList[ resourceIndex ].mPort = currentPort;
        driverList[ resourceIndex ].mPin  = currentPin;

        /*-------------------------------------------------
        Ensure we don't initialize too many pins
        -------------------------------------------------*/
        if( initializedPins < numDrivers )
        {
          continue; // More things left to do
        }
        else if ( ( ( pinIdx + 1 ) < portAttributes[ portIdx ].pinListSize ) || ( ( portIdx + 1 ) < PRJ_MAX_PORTS ) )
        {
          return false; // Hit the limit, but the one of the loops think we still have more to go.
        }
        else
        {
          break;
        }
      }
    }

    /*-------------------------------------------------
    One last check: Were all the expected pins initialized?
    -------------------------------------------------*/
    return initializedPins == numDrivers;
  }


  Reg32_t findAlternateFunction( const Chimera::GPIO::Port port, const Chimera::GPIO::Pin pin, const Chimera::GPIO::Alternate alt )
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
      if( portAttributes[ portIdx ].portID != port )
      {
        continue;
      }

      /*-------------------------------------------------
      Found the port! Check if it contains the desired pin
      -------------------------------------------------*/
      for ( size_t pinIdx = 0; pinIdx < portAttributes[ portIdx ].pinListSize; pinIdx++ )
      {
        /*-------------------------------------------------
        *sigh...still nothing.
        -------------------------------------------------*/
        if( portAttributes[ portIdx ].pins[ pinIdx ].pinID != pin )
        {
          continue;
        }

        /*-------------------------------------------------
        Found the pin! Check if it supports the alternate function
        -------------------------------------------------*/
        for( size_t altIdx = 0; altIdx < portAttributes[ portIdx ].pins[ pinIdx ].afListSize; altIdx++ )
        {
          /*-------------------------------------------------
          Ugh are you kidding!? Still nothing?!
          -------------------------------------------------*/
          if( portAttributes[ portIdx ].pins[ pinIdx ].altFunc[ altIdx ].chimeraAltFunc != alt )
          {
            continue;
          }

          /*-------------------------------------------------
          FINALLY WE FOUND IT. PRAISE THE MCU GODS.
          -------------------------------------------------*/
          altFunctionConfig = portAttributes[ portIdx ].pins[ pinIdx ].altFunc[ altIdx ].registerAltFunc;
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

}  // namespace Thor::LLD::GPIO
