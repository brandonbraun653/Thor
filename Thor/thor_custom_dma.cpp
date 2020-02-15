/********************************************************************************
 * File Name:
 *   thor_custom_dma.cpp
 *
 * Description:
 *   Implements DMA for Thor using the custom low level drivers.
 *
 * 2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <array>
#include <cstring>

/* Aurora Includes */
#include <Aurora/constants/common.hpp>

/* Chimera Includes */
#include <Chimera/thread>

/* Thor Includes */
#include <Thor/thor.hpp>
#include <Thor/dma.hpp>
#include <Thor/drivers/dma.hpp>
#include <Thor/resources/dma_resources.hpp>

namespace DMADriver = Thor::Driver::DMA;

static std::shared_ptr<Thor::DMA::DMAClass> dmaSingleton = nullptr;

static DMADriver::Driver *currentDMAInstance = nullptr;
static DMADriver::StreamX *currentStream     = nullptr;


namespace Chimera::DMA
{
  Chimera::Status_t initialize()
  {
    return Thor::DMA::initialize();
  }
}

namespace Thor::DMA
{
  static DMADriver::Driver *const getDriver( const Chimera::DMA::Init &config );
  static DMADriver::StreamX *const getStream( const Chimera::DMA::Init &config );
  static DMADriver::StreamConfig convertConfig( const Chimera::DMA::Init &config );
  static DMADriver::TCB convertTCB( const Chimera::DMA::TCB &transfer );

  static bool compareFunction( DMADriver::StreamResources &a, DMADriver::StreamResources &b )
  {
    return a.requestID > b.requestID;
  }


  static size_t s_driver_initialized;

  Chimera::Status_t initialize()
  {
  s_driver_initialized = ~Chimera::DRIVER_INITIALIZED_KEY;

    /*------------------------------------------------
    Initialize the low level driver
    ------------------------------------------------*/
    Thor::Driver::DMA::initialize();

    /*------------------------------------------------
    Reset driver object memory
    ------------------------------------------------*/
    for ( size_t x = 0; x < DMADriver::dmaObjects.size(); x++ )
    {
      if ( DMADriver::dmaObjects[ x ] ) 
      {
        vPortFree( DMADriver::dmaObjects[ x ] );
      }

      DMADriver::dmaObjects[ x ] = nullptr;
    }

    s_driver_initialized = Chimera::DRIVER_INITIALIZED_KEY;
    return Chimera::CommonStatusCodes::OK;
  }


  DMAClass::DMAClass()
  {
    /*------------------------------------------------
    Lazy initialize all driver memory in case the user forgot
    ------------------------------------------------*/
    if ( s_driver_initialized != Chimera::DRIVER_INITIALIZED_KEY )
    {
      initialize();
    }

    lastLookup.clear();
  }

  DMAClass::~DMAClass()
  {
  }

  std::shared_ptr<DMAClass> DMAClass::get()
  {
    /* Gets around the private constructor issue with shared_ptr */
    struct A : public DMAClass
    {
    };

    if ( !dmaSingleton )
    {
      dmaSingleton = std::make_shared<A>();
    }

    return dmaSingleton;
  }

  Chimera::Status_t DMAClass::init()
  {
    /*------------------------------------------------
    Ensure an instance of DMA peripherals are created and initialized properly.
    ------------------------------------------------*/
    for ( uint8_t resourceIndex = 0; resourceIndex < DMADriver::dmaObjects.size(); resourceIndex++ )
    {
      /*------------------------------------------------
      Dynamically allocate peripheral drivers
      ------------------------------------------------*/
      if ( !DMADriver::dmaObjects[ resourceIndex ] )
      {
        DMADriver::dmaObjects[ resourceIndex ] = new DMADriver::Driver();
        DMADriver::dmaObjects[ resourceIndex ]->attach( DMADriver::periphInstanceList[ resourceIndex ] );
      }

      /*------------------------------------------------
      Initialize the drivers
      ------------------------------------------------*/
      if ( DMADriver::dmaObjects[ resourceIndex ]->init() != Chimera::CommonStatusCodes::OK )
      {
        return Chimera::CommonStatusCodes::FAIL;
      }
    }

    /*------------------------------------------------
    Sort the request resources such that lookup becomes O(log(N))
    ------------------------------------------------*/
    std::sort( RequestGenerators.begin(), RequestGenerators.end(), compareFunction );

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t DMAClass::reset()
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t DMAClass::start()
  {
    if ( currentDMAInstance && currentStream )
    {
      return currentDMAInstance->start( currentStream );
    }

    return Chimera::CommonStatusCodes::FAIL;
  }

  Chimera::Status_t DMAClass::configure( const Chimera::DMA::Init &config, const Chimera::DMA::TCB &transfer,
                                         const size_t timeout, Chimera::DMA::TransferHandle_t *const handle )
  {
    auto result = Chimera::CommonStatusCodes::FAIL;

    /*------------------------------------------------
    Convert from the generic Chimera representation of DMA settings into
    the actual register bit values needed for this device.
    ------------------------------------------------*/
    auto cfg = convertConfig( config );
    auto tcb = convertTCB( transfer );

    /*------------------------------------------------
    Lookup the associated DMA peripheral and Stream to be configured
    ------------------------------------------------*/
    currentDMAInstance = getDriver( config );
    currentStream      = getStream( config );

    /*------------------------------------------------
    Perform the configuration
    ------------------------------------------------*/
    if ( currentDMAInstance && currentStream )
    {
      result = currentDMAInstance->configure( currentStream, &cfg, &tcb );
    }

    return result;
  }

  Chimera::Status_t DMAClass::abort( Chimera::DMA::TransferHandle_t handle, const size_t timeout )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t DMAClass::status( Chimera::DMA::TransferHandle_t handle, const size_t timeout )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t DMAClass::registerListener( Driver::DMA::StreamX *const stream, Chimera::Event::Actionable &listener,
                                                const size_t timeout, size_t &registrationID )
  {
    auto result = Chimera::CommonStatusCodes::OK;

    if ( streamIsOnPeripheral( DMADriver::DMA1_PERIPH, stream ) && DMADriver::dmaObjects[ 0 ] )
    {
      result = DMADriver::dmaObjects[ 0 ]->registerListener( stream, listener, timeout, registrationID );
    }
    else if ( streamIsOnPeripheral( DMADriver::DMA2_PERIPH, stream ) && DMADriver::dmaObjects[ 1 ] )
    {
      result = DMADriver::dmaObjects[ 1 ]->registerListener( stream, listener, timeout, registrationID );
    }
    else
    {
      result = Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
    }

    return result;
  }

  Chimera::Status_t DMAClass::removeListener( Driver::DMA::StreamX *const stream, const size_t registrationID,
                                              const size_t timeout )
  {
    auto result = Chimera::CommonStatusCodes::OK;

    if ( streamIsOnPeripheral( DMADriver::DMA1_PERIPH, stream ) && DMADriver::dmaObjects[ 0 ] )
    {
      result = DMADriver::dmaObjects[ 0 ]->removeListener( stream, registrationID, timeout );
    }
    else if ( streamIsOnPeripheral( DMADriver::DMA2_PERIPH, stream ) && DMADriver::dmaObjects[ 1 ] )
    {
      result = DMADriver::dmaObjects[ 1 ]->removeListener( stream, registrationID, timeout );
    }
    else
    {
      result = Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
    }

    return result;
  }

  static DMADriver::Driver *const getDriver( const Chimera::DMA::Init &config )
  {
    DMADriver::Driver *driver = nullptr;

    /*------------------------------------------------
    Find the request meta info using binary search of sorted list
    ------------------------------------------------*/
    DMADriver::StreamResources c;
    c.clear();
    c.requestID = config.request;

    auto metaInfo = std::lower_bound(
        RequestGenerators.begin(), RequestGenerators.end(), c,
        []( const DMADriver::StreamResources &a, const DMADriver::StreamResources &b ) { return b.requestID < a.requestID; } );

    if ( metaInfo->requestID != config.request )
    {
      return driver;
    }

    /*------------------------------------------------
    Pull out the stream object
    ------------------------------------------------*/
    if ( metaInfo->cfgBitField & DMADriver::ConfigBitFields::DMA_ON_DMA1 )
    {
      driver = DMADriver::dmaObjects[ 0 ];
    }
    else if ( metaInfo->cfgBitField & DMADriver::ConfigBitFields::DMA_ON_DMA2 )
    {
      driver = DMADriver::dmaObjects[ 1 ];
    }

    return driver;
  }

  static DMADriver::StreamX *const getStream( const Chimera::DMA::Init &config )
  {
    DMADriver::StreamX *stream = nullptr;
    uint32_t streamNum         = 0;

    /*------------------------------------------------
    Find the request meta info using binary search of sorted list
    ------------------------------------------------*/
    DMADriver::StreamResources c;
    c.clear();
    c.requestID = config.request;

    auto metaInfo = std::lower_bound(
        RequestGenerators.begin(), RequestGenerators.end(), c,
        []( const DMADriver::StreamResources &a, const DMADriver::StreamResources &b ) { return b.requestID < a.requestID; } );

    if ( metaInfo->requestID != config.request )
    {
      return stream;
    }

    /*------------------------------------------------
    Pull out the stream object
    ------------------------------------------------*/
    streamNum =
        ( metaInfo->cfgBitField & DMADriver::ConfigBitFields::DMA_STREAM ) >> DMADriver::ConfigBitFields::DMA_STREAM_POS;
    if ( metaInfo->cfgBitField & DMADriver::ConfigBitFields::DMA_ON_DMA1 )
    {
      stream = DMADriver::getStreamRegisters( DMADriver::DMA1_PERIPH, streamNum );
    }
    else if ( metaInfo->cfgBitField & DMADriver::ConfigBitFields::DMA_ON_DMA2 )
    {
      stream = DMADriver::getStreamRegisters( DMADriver::DMA2_PERIPH, streamNum );
    }

    return stream;
  }

  static DMADriver::StreamConfig convertConfig( const Chimera::DMA::Init &config )
  {
    using namespace DMADriver;

    StreamConfig scfg;

    scfg.Channel             = Configuration::ChannelSelect::Channel4;
    scfg.Direction           = TransferMap[ static_cast<size_t>( config.direction ) ];
    scfg.MemBurst            = Configuration::MemoryBurst::Single;
    scfg.MemDataAlignment    = MemoryAlignmentMap[ static_cast<size_t>( config.mAlign ) ];
    scfg.MemInc              = MemoryIncrementMap[ static_cast<size_t>( config.mInc ) ];
    scfg.Mode                = ModeMap[ static_cast<size_t>( config.mode ) ];
    scfg.PeriphBurst         = Configuration::PeriphBurst::Single;
    scfg.PeriphInc           = PeripheralIncrementMap[ static_cast<size_t>( config.pInc ) ];
    scfg.PeriphDataAlignment = PeripheralAlignmentMap[ static_cast<size_t>( config.pAlign ) ];
    scfg.Priority            = PriorityMap[ static_cast<uint32_t>( config.priority ) ];

    // TODO: These two are a bit weird...how to abstract this higher?
    scfg.FIFOMode      = Configuration::FIFODirectMode::Enabled;
    scfg.FIFOThreshold = Configuration::FIFOThreshold::Threshold_4_4;

    return scfg;
  }

  static DMADriver::TCB convertTCB( const Chimera::DMA::TCB &transfer )
  {
    using namespace DMADriver;

    TCB tcb;
    tcb.srcAddress   = transfer.srcAddress;
    tcb.dstAddress   = transfer.dstAddress;
    tcb.transferSize = static_cast<uint32_t>( transfer.transferSize );

    return tcb;
  }
}    // namespace Thor::DMA
