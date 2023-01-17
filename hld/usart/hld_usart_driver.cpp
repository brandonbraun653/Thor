/******************************************************************************
 *  File Name:
 *    thor_custom_usart.cpp
 *
 *  Description:
 *    Implements the custom driver variant of the Thor USART interface.
 *
 *  2019-2022 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/constants>
#include <Aurora/utility>
#include <Chimera/assert>
#include <Chimera/event>
#include <Chimera/gpio>
#include <Chimera/interrupt>
#include <Chimera/serial>
#include <Chimera/thread>
#include <Chimera/usart>
#include <Thor/cfg>
#include <Thor/dma>
#include <Thor/event>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/inc/interrupt>
#include <Thor/lld/interface/inc/usart>
#include <array>
#include <cstdint>
#include <memory>


#if defined( THOR_USART )
namespace Chimera::USART
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  namespace LLD = ::Thor::LLD::USART;

  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t NUM_DRIVERS = LLD::NUM_USART_PERIPHS;

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct ThorImpl
  {
    LLD::Driver_rPtr            lldriver;
    Chimera::USART::Driver_rPtr hldriver;

    /*-------------------------------------------------------------------------
    Misc state variables
    -------------------------------------------------------------------------*/
    bool                     mEnabled;       /**< Has the peripheral been enabled */
    Chimera::Serial::Channel mChannel;       /**< Hardware channel associated with this driver */
    size_t                   mResourceIndex; /**< Lookup table index for USART resources */

    /*-------------------------------------------------------------------------
    Internal locks for protecting the data buffers
    -------------------------------------------------------------------------*/
    Chimera::Thread::RecursiveMutex  mRxLock;
    Chimera::Thread::BinarySemaphore mTxLock;

    /*-------------------------------------------------------------------------
    Selects the HW mode each data line operates in
    -------------------------------------------------------------------------*/
    Chimera::Hardware::PeripheralMode mTxMode;
    Chimera::Hardware::PeripheralMode mRxMode;

    ThorImpl() :
        mEnabled( false ), mChannel( Chimera::Serial::Channel::NOT_SUPPORTED ), mResourceIndex( 0 )
    {
      mTxLock.release();
    }

    Chimera::Status_t readBlocking( void *const buffer, const size_t length )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }


    Chimera::Status_t readInterrupt( void *const buffer, const size_t length )
    {
      Chimera::Status_t error = Chimera::Status::OK;

      // if ( !mRxBuffers.initialized() )
      // {
      //   return Chimera::Status::NOT_INITIALIZED;
      // }

      // if ( length <= mRxBuffers.linearSize() )
      // {
      //   memset( mRxBuffers.linearBuffer(), 0, mRxBuffers.linearSize() );
      //   error = lldriver->rxInterrupt( mRxBuffers.linearBuffer(), length );

      //   if ( error == Chimera::Status::OK )
      //   {
      //     error = Chimera::Serial::Status::RX_IN_PROGRESS;
      //   }
      // }
      // else
      // {
      //   error = Chimera::Status::MEMORY;
      // }

      return error;
    }


    Chimera::Status_t readDMA( void *const buffer, const size_t length )
    {
      Chimera::Status_t error = Chimera::Status::OK;

      // if ( !mRxBuffers.initialized() )
      // {
      //   return Chimera::Status::NOT_INITIALIZED;
      // }

      // if ( length <= mRxBuffers.linearSize() )
      // {
      //   memset( mRxBuffers.linearBuffer(), 0, mRxBuffers.linearSize() );
      //   error = lldriver->rxDMA( mRxBuffers.linearBuffer(), length );

      //   if ( error == Chimera::Status::OK )
      //   {
      //     error = Chimera::Serial::Status::RX_IN_PROGRESS;
      //   }
      // }
      // else
      // {
      //   error = Chimera::Status::MEMORY;
      // }

      return error;
    }


    Chimera::Status_t writeBlocking( const void *const buffer, const size_t length )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }


    Chimera::Status_t writeInterrupt( const void *const buffer, const size_t length )
    {
      Chimera::Status_t error = Chimera::Status::OK;

      // if ( !mTxBuffers.initialized() )
      // {
      //   return Chimera::Status::NOT_INITIALIZED;
      // }

      // /*------------------------------------------------
      // Hardware is free. Send the data directly. Otherwise
      // queue everything up to send later.
      // ------------------------------------------------*/
      // if ( mTxLock.try_acquire() )
      // {
      //   error = lldriver->txInterrupt( buffer, length );
      // }
      // else
      // {
      //   size_t pushed = 0;
      //   error         = Chimera::Status::BUSY;
      //   mTxBuffers.push( reinterpret_cast<const uint8_t *const>( buffer ), length, pushed );
      // }

      return error;
    }


    Chimera::Status_t writeDMA( const void *const buffer, const size_t length )
    {
      Chimera::Status_t error = Chimera::Status::OK;

      // if ( !mTxBuffers.initialized() )
      // {
      //   return Chimera::Status::NOT_INITIALIZED;
      // }

      // /*------------------------------------------------
      // Hardware is free. Send the data directly. Otherwise
      // queue everything up to send later.
      // ------------------------------------------------*/
      // if ( mTxLock.try_acquire() )
      // {
      //   error = lldriver->txDMA( buffer, length );
      // }
      // else
      // {
      //   size_t pushed = 0;
      //   error         = Chimera::Status::BUSY;
      //   mTxBuffers.push( reinterpret_cast<const uint8_t *const>( buffer ), length, pushed );
      // }

      return error;
    }
  };

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static DeviceManager<Driver, Chimera::Serial::Channel, NUM_DRIVERS>   s_raw_drivers;
  static DeviceManager<ThorImpl, Chimera::Serial::Channel, NUM_DRIVERS> s_impl_drivers;

  /*---------------------------------------------------------------------------
  Driver Implementation
  ---------------------------------------------------------------------------*/
  Driver::Driver()
  {
  }


  Driver::~Driver()
  {
  }

  Chimera::Status_t Driver::open( const Chimera::Serial::Config &config )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::close()
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  int Driver::write( const void *const buffer, const size_t length )
  {
    return 0;
  }


  int Driver::read( void *const buffer, const size_t length )
  {
    return 0;
  }

}    // namespace Chimera::USART


namespace Chimera::USART::Backend
{
  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  static Chimera::Status_t initialize()
  {
    return Chimera::Status::OK;
  }


  static Chimera::Status_t reset()
  {
    return Chimera::Status::OK;
  }

  static Driver_rPtr getDriver( const Chimera::Serial::Channel channel )
  {
    if ( auto idx = LLD::getResourceIndex( channel ); idx != ::Thor::LLD::INVALID_RESOURCE_INDEX )
    {
      return s_raw_drivers.getOrCreate( channel );
    }
    else
    {
      return nullptr;
    }
  }

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  Chimera::Status_t registerDriver( Chimera::USART::Backend::DriverConfig &registry )
  {
    registry.isSupported    = true;
    registry.getDriver      = ::Chimera::USART::Backend::getDriver;
    registry.initialize     = ::Chimera::USART::Backend::initialize;
    registry.reset          = ::Chimera::USART::Backend::reset;
    registry.isChannelUSART = LLD::isSupported;
    return Chimera::Status::OK;
  }
}    // namespace Chimera::USART::Backend
#endif /* THOR_USART */
