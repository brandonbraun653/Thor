/******************************************************************************
 *  File Name:
 *    usart_intf.cpp
 *
 *  Description:
 *    LLD interface functions that are processor independent
 *
 *  2020-2022 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <Thor/cfg>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/inc/serial>
#include <Thor/lld/interface/inc/usart>

#if defined( THOR_USART )
namespace Thor::LLD::USART
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  bool isSupported( const Chimera::Serial::Channel channel )
  {
    switch( channel )
    {
#if defined( STM32_USART1_PERIPH_AVAILABLE )
      case Chimera::Serial::Channel::SERIAL1:
        return true;
        break;
#endif
#if defined( STM32_USART2_PERIPH_AVAILABLE )
      case Chimera::Serial::Channel::SERIAL2:
        return true;
        break;
#endif
#if defined( STM32_USART3_PERIPH_AVAILABLE )
      case Chimera::Serial::Channel::SERIAL3:
        return true;
        break;
#endif
#if defined( STM32_USART6_PERIPH_AVAILABLE )
      case Chimera::Serial::Channel::SERIAL6:
        return true;
        break;
#endif

      default:
        return false;
        break;
    };
  }


  RIndex_t getResourceIndex( const Chimera::Serial::Channel channel )
  {
    switch( channel )
    {
#if defined( STM32_USART1_PERIPH_AVAILABLE )
      case Chimera::Serial::Channel::SERIAL1:
        return USART1_RESOURCE_INDEX;
        break;
#endif
#if defined( STM32_USART2_PERIPH_AVAILABLE )
      case Chimera::Serial::Channel::SERIAL2:
        return USART2_RESOURCE_INDEX;
        break;
#endif
#if defined( STM32_USART3_PERIPH_AVAILABLE )
      case Chimera::Serial::Channel::SERIAL3:
        return USART3_RESOURCE_INDEX;
        break;
#endif
#if defined( STM32_USART6_PERIPH_AVAILABLE )
      case Chimera::Serial::Channel::SERIAL6:
        return USART6_RESOURCE_INDEX;
        break;
#endif

      default:
        return INVALID_RESOURCE_INDEX;
        break;
    };
  }


  RIndex_t getResourceIndex( const std::uintptr_t address )
  {
#if defined( STM32_USART1_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( USART1_PERIPH ) )
    {
      return USART1_RESOURCE_INDEX;
    }
#endif
#if defined( STM32_USART2_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( USART2_PERIPH ) )
    {
      return USART2_RESOURCE_INDEX;
    }
#endif
#if defined( STM32_USART3_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( USART3_PERIPH ) )
    {
      return USART3_RESOURCE_INDEX;
    }
#endif
#if defined( STM32_USART6_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( USART6_PERIPH ) )
    {
      return USART6_RESOURCE_INDEX;
    }
#endif

    return INVALID_RESOURCE_INDEX;
  }


  Chimera::Serial::Channel getChannel( const std::uintptr_t address )
  {
#if defined( STM32_USART1_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( USART1_PERIPH ) )
    {
      return Chimera::Serial::Channel::SERIAL1;
    }
#endif
#if defined( STM32_USART2_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( USART2_PERIPH ) )
    {
      return Chimera::Serial::Channel::SERIAL2;
    }
#endif
#if defined( STM32_USART3_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( USART3_PERIPH ) )
    {
      return Chimera::Serial::Channel::SERIAL3;
    }
#endif
#if defined( STM32_USART6_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( USART6_PERIPH ) )
    {
      return Chimera::Serial::Channel::SERIAL6;
    }
#endif

    return Chimera::Serial::Channel::NOT_SUPPORTED;
  }


  bool attachDriverInstances( Driver *const driverList, const size_t numDrivers )
  {
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if ( !driverList || !numDrivers || ( numDrivers != NUM_USART_PERIPHS ) )
    {
      return false;
    }

    /*-------------------------------------------------------------------------
    Attach the drivers
    -------------------------------------------------------------------------*/
    Chimera::Status_t result = Chimera::Status::OK;

#if defined( STM32_USART1_PERIPH_AVAILABLE )
    result |= driverList[ USART1_RESOURCE_INDEX ].attach( USART1_PERIPH );

    Thor::LLD::Serial::registerInterface(
        getChannel( reinterpret_cast<std::uintptr_t>( USART1_PERIPH ) ),
        dynamic_cast<Thor::LLD::Serial::HwInterface*>( &driverList[ USART1_RESOURCE_INDEX ] ) );

#endif
#if defined( STM32_USART2_PERIPH_AVAILABLE )
    result |= driverList[ USART2_RESOURCE_INDEX ].attach( USART2_PERIPH );

    Thor::LLD::Serial::registerInterface(
        getChannel( reinterpret_cast<std::uintptr_t>( USART2_PERIPH ) ),
        dynamic_cast<Thor::LLD::Serial::HwInterface*>( &driverList[ USART2_RESOURCE_INDEX ] ) );
#endif
#if defined( STM32_USART3_PERIPH_AVAILABLE )
    result |= driverList[ USART3_RESOURCE_INDEX ].attach( USART3_PERIPH );

    Thor::LLD::Serial::registerInterface(
        getChannel( reinterpret_cast<std::uintptr_t>( USART3_PERIPH ) ),
        dynamic_cast<Thor::LLD::Serial::HwInterface*>( &driverList[ USART3_RESOURCE_INDEX ] ) );
#endif
#if defined( STM32_USART6_PERIPH_AVAILABLE )
    result |= driverList[ USART6_RESOURCE_INDEX ].attach( USART6_PERIPH );

    Thor::LLD::Serial::registerInterface(
        getChannel( reinterpret_cast<std::uintptr_t>( USART6_PERIPH ) ),
        dynamic_cast<Thor::LLD::Serial::HwInterface*>( &driverList[ USART6_RESOURCE_INDEX ] ) );
#endif

    return result == Chimera::Status::OK;
  }
}  // namespace Thor::LLD::USART

#endif  /* THOR_LLD_USART */
