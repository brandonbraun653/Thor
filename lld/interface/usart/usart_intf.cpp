/********************************************************************************
 *  File Name:
 *    usart_intf.cpp
 *
 *  Description:
 *    LLD interface functions that are processor independent
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Chimera Includes */
#include <Chimera/common>

/* Thor Includes */
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/inc/usart>


namespace Thor::LLD::USART
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
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

    return Chimera::Serial::Channel::NOT_SUPPORTED;
  }


  bool attachDriverInstances( Driver *const driverList, const size_t numDrivers )
  {
    /*-------------------------------------------------
    Reject bad inputs
    -------------------------------------------------*/
    if ( !driverList || !numDrivers || ( numDrivers != NUM_USART_PERIPHS ) )
    {
      return false;
    }

    /*-------------------------------------------------
    Attach the drivers. The architecture of the LLD
    ensures the ordering and number is correct.
    -------------------------------------------------*/
    Chimera::Status_t result = Chimera::Status::OK;

#if defined( STM32_USART1_PERIPH_AVAILABLE )
    result |= driverList[ USART1_RESOURCE_INDEX ].attach( USART1_PERIPH );
#endif
#if defined( STM32_USART2_PERIPH_AVAILABLE )
    result |= driverList[ USART2_RESOURCE_INDEX ].attach( USART2_PERIPH );
#endif
#if defined( STM32_USART3_PERIPH_AVAILABLE )
    result |= driverList[ USART3_RESOURCE_INDEX ].attach( USART3_PERIPH );
#endif

    return result == Chimera::Status::OK;
  }
}  // namespace Thor::LLD::USART
