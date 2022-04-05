/******************************************************************************
 *  File Name:
 *    i2c_cmn_intf.cpp
 *
 *  Description:
 *    LLD interface functions that are processor independent
 *
 *  2021-2022 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <Chimera/thread>
#include <Thor/cfg>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/inc/i2c>

#if defined( THOR_LLD_I2C )
namespace Thor::LLD::I2C
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static Chimera::DeviceManager<Driver, Chimera::I2C::Channel, NUM_I2C_PERIPHS> s_i2c_drivers;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  Chimera::Status_t __weak init_prj_driver()
  {
    return Chimera::Status::OK;
  }


  Chimera::Status_t initialize()
  {
    /*-------------------------------------------------------------------------
    Bind the memory mapped peripherals to the driver instances
    -------------------------------------------------------------------------*/
    Chimera::Status_t result = Chimera::Status::OK;

#if defined( STM32_I2C1_PERIPH_AVAILABLE )
    result |= getLLDriver( Chimera::I2C::Channel::I2C1 )->attach( I2C1_PERIPH );
#endif
#if defined( STM32_I2C2_PERIPH_AVAILABLE )
    result |= getLLDriver( Chimera::I2C::Channel::I2C2 )->attach( I2C2_PERIPH );
#endif
#if defined( STM32_I2C3_PERIPH_AVAILABLE )
    result |= getLLDriver( Chimera::I2C::Channel::I2C3 )->attach( I2C3_PERIPH );
#endif

    RT_HARD_ASSERT( result == Chimera::Status::OK );

    /*-------------------------------------------------------------------------
    Invoke the chip specific sequence if any
    -------------------------------------------------------------------------*/
    return init_prj_driver();
  }


  Driver_rPtr getLLDriver( const Chimera::I2C::Channel channel )
  {
    if( !isSupported( channel ) )
    {
      return nullptr;
    }

    return s_i2c_drivers.getOrCreate( channel );
  }


  bool isSupported( const Chimera::I2C::Channel periph )
  {
    switch ( periph )
    {
#if defined( STM32_I2C1_PERIPH_AVAILABLE )
      case Chimera::I2C::Channel::I2C1:
        return true;
        break;
#endif
#if defined( STM32_I2C2_PERIPH_AVAILABLE )
      case Chimera::I2C::Channel::I2C2:
        return true;
        break;
#endif
#if defined( STM32_I2C3_PERIPH_AVAILABLE )
      case Chimera::I2C::Channel::I2C3:
        return true;
        break;
#endif

      default:
        return false;
        break;
    };
  }


  RIndex_t getResourceIndex( const Chimera::I2C::Channel periph )
  {
    switch ( periph )
    {
#if defined( STM32_I2C1_PERIPH_AVAILABLE )
      case Chimera::I2C::Channel::I2C1:
        return I2C1_RESOURCE_INDEX;
        break;
#endif
#if defined( STM32_I2C2_PERIPH_AVAILABLE )
      case Chimera::I2C::Channel::I2C2:
        return I2C2_RESOURCE_INDEX;
        break;
#endif
#if defined( STM32_I2C3_PERIPH_AVAILABLE )
      case Chimera::I2C::Channel::I2C3:
        return I2C3_RESOURCE_INDEX;
        break;
#endif

      default:
        return INVALID_RESOURCE_INDEX;
        break;
    };
  }


  RIndex_t getResourceIndex( const std::uintptr_t address )
  {
#if defined( STM32_I2C1_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( I2C1_PERIPH ) )
    {
      return I2C1_RESOURCE_INDEX;
    }
#endif
#if defined( STM32_I2C2_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( I2C2_PERIPH ) )
    {
      return I2C2_RESOURCE_INDEX;
    }
#endif
#if defined( STM32_I2C3_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( I2C3_PERIPH ) )
    {
      return I2C3_RESOURCE_INDEX;
    }
#endif

    return INVALID_RESOURCE_INDEX;
  }


  Chimera::I2C::Channel getChannel( const std::uintptr_t address )
  {
#if defined( STM32_I2C1_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( I2C1_PERIPH ) )
    {
      return Chimera::I2C::Channel::I2C1;
    }
#endif
#if defined( STM32_I2C2_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( I2C2_PERIPH ) )
    {
      return Chimera::I2C::Channel::I2C2;
    }
#endif
#if defined( STM32_I2C3_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( I2C3_PERIPH ) )
    {
      return Chimera::I2C::Channel::I2C3;
    }
#endif

    return Chimera::I2C::Channel::NOT_SUPPORTED;
  }

}  // namespace Thor::LLD::I2C
#endif  /* THOR_LLD_I2C */
