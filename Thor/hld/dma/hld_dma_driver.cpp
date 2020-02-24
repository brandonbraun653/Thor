/********************************************************************************
 *  File Name:
 *   thor_custom_dma.cpp
 *
 *  Description:
 *   Implements DMA for Thor using the custom low level drivers.
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <array>
#include <cstring>

/* Aurora Includes */
#include <Aurora/constants/common.hpp>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/thread>

/* Thor Includes */
#include <Thor/lld/interface/dma/dma.hpp>


static std::shared_ptr<Thor::DMA::DMAClass> dmaSingleton = nullptr;

static Thor::LLD::DMA::Driver *currentDMAInstance = nullptr;
static Thor::LLD::DMA::StreamX *currentStream     = nullptr;


static std::array<Chimera::Threading::detail::native_thread_handle_type, Thor::LLD::DMA::NUM_DMA_STREAMS> postProcessorHandle;
static std::array<Chimera::Threading::BinarySemaphore, Thor::LLD::DMA::NUM_DMA_STREAMS> postProcessorSignal;
static std::array<Chimera::Function::void_func_void_ptr, Thor::LLD::DMA::NUM_DMA_STREAMS> postProcessorThread;

#ifdef STM32_DMA1_STREAM0_AVAILABLE
static void DMA1_Stream0_ISRPostProcessorThread( void *argument );
#endif

#ifdef STM32_DMA1_STREAM1_AVAILABLE
static void DMA1_Stream1_ISRPostProcessorThread( void *argument );
#endif

#ifdef STM32_DMA1_STREAM2_AVAILABLE
static void DMA1_Stream2_ISRPostProcessorThread( void *argument );
#endif

#ifdef STM32_DMA1_STREAM3_AVAILABLE
static void DMA1_Stream3_ISRPostProcessorThread( void *argument );
#endif

#ifdef STM32_DMA1_STREAM4_AVAILABLE
static void DMA1_Stream4_ISRPostProcessorThread( void *argument );
#endif

#ifdef STM32_DMA1_STREAM5_AVAILABLE
static void DMA1_Stream5_ISRPostProcessorThread( void *argument );
#endif

#ifdef STM32_DMA1_STREAM6_AVAILABLE
static void DMA1_Stream6_ISRPostProcessorThread( void *argument );
#endif

#ifdef STM32_DMA1_STREAM7_AVAILABLE
static void DMA1_Stream7_ISRPostProcessorThread( void *argument );
#endif

#ifdef STM32_DMA2_STREAM0_AVAILABLE
static void DMA2_Stream0_ISRPostProcessorThread( void *argument );
#endif

#ifdef STM32_DMA2_STREAM1_AVAILABLE
static void DMA2_Stream1_ISRPostProcessorThread( void *argument );
#endif

#ifdef STM32_DMA2_STREAM2_AVAILABLE
static void DMA2_Stream2_ISRPostProcessorThread( void *argument );
#endif

#ifdef STM32_DMA2_STREAM3_AVAILABLE
static void DMA2_Stream3_ISRPostProcessorThread( void *argument );
#endif

#ifdef STM32_DMA2_STREAM4_AVAILABLE
static void DMA2_Stream4_ISRPostProcessorThread( void *argument );
#endif

#ifdef STM32_DMA2_STREAM5_AVAILABLE
static void DMA2_Stream5_ISRPostProcessorThread( void *argument );
#endif

#ifdef STM32_DMA2_STREAM6_AVAILABLE
static void DMA2_Stream6_ISRPostProcessorThread( void *argument );
#endif

#ifdef STM32_DMA2_STREAM7_AVAILABLE
static void DMA2_Stream7_ISRPostProcessorThread( void *argument );
#endif


namespace Chimera::DMA
{
  Chimera::Status_t initialize()
  {
    return Thor::DMA::initialize();
  }
}

namespace Thor::DMA
{
  static Thor::LLD::DMA::Driver *const getDriver( const Chimera::DMA::Init &config );
  static Thor::LLD::DMA::StreamX *const getStream( const Chimera::DMA::Init &config );
  static Thor::LLD::DMA::StreamConfig convertConfig( const Chimera::DMA::Init &config );
  static Thor::LLD::DMA::TCB convertTCB( const Chimera::DMA::TCB &transfer );

  static bool compareFunction( Thor::LLD::DMA::StreamResources &a, Thor::LLD::DMA::StreamResources &b )
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
    Thor::LLD::DMA::initialize();

    /*------------------------------------------------
    Reset driver object memory
    ------------------------------------------------*/
    for ( size_t x = 0; x < Thor::LLD::DMA::dmaObjects.size(); x++ )
    {
      if ( Thor::LLD::DMA::dmaObjects[ x ] ) 
      {
        vPortFree( Thor::LLD::DMA::dmaObjects[ x ] );
      }

      Thor::LLD::DMA::dmaObjects[ x ] = nullptr;
    }

    /*-------------------------------------------------
    Register callback threads
    -------------------------------------------------*/
#ifdef STM32_DMA1_STREAM0_AVAILABLE
    postProcessorThread[ Thor::LLD::DMA::DMA1_STREAM0_RESOURCE_INDEX ] = DMA1_Stream0_ISRPostProcessorThread;
#endif

#ifdef STM32_DMA1_STREAM1_AVAILABLE
    postProcessorThread[ Thor::LLD::DMA::DMA1_STREAM1_RESOURCE_INDEX ] = DMA1_Stream1_ISRPostProcessorThread;
#endif

#ifdef STM32_DMA1_STREAM2_AVAILABLE
    postProcessorThread[ Thor::LLD::DMA::DMA1_STREAM2_RESOURCE_INDEX ] = DMA1_Stream2_ISRPostProcessorThread;
#endif

#ifdef STM32_DMA1_STREAM3_AVAILABLE
    postProcessorThread[ Thor::LLD::DMA::DMA1_STREAM3_RESOURCE_INDEX ] = DMA1_Stream3_ISRPostProcessorThread;
#endif

#ifdef STM32_DMA1_STREAM4_AVAILABLE
    postProcessorThread[ Thor::LLD::DMA::DMA1_STREAM4_RESOURCE_INDEX ] = DMA1_Stream4_ISRPostProcessorThread;
#endif

#ifdef STM32_DMA1_STREAM5_AVAILABLE
    postProcessorThread[ Thor::LLD::DMA::DMA1_STREAM5_RESOURCE_INDEX ] = DMA1_Stream5_ISRPostProcessorThread;
#endif

#ifdef STM32_DMA1_STREAM6_AVAILABLE
    postProcessorThread[ Thor::LLD::DMA::DMA1_STREAM6_RESOURCE_INDEX ] = DMA1_Stream6_ISRPostProcessorThread;
#endif

#ifdef STM32_DMA1_STREAM7_AVAILABLE
    postProcessorThread[ Thor::LLD::DMA::DMA1_STREAM7_RESOURCE_INDEX ] = DMA1_Stream7_ISRPostProcessorThread;
#endif

#ifdef STM32_DMA2_STREAM0_AVAILABLE
    postProcessorThread[ Thor::LLD::DMA::DMA2_STREAM0_RESOURCE_INDEX ] = DMA2_Stream0_ISRPostProcessorThread;
#endif

#ifdef STM32_DMA2_STREAM1_AVAILABLE
    postProcessorThread[ Thor::LLD::DMA::DMA2_STREAM1_RESOURCE_INDEX ] = DMA2_Stream1_ISRPostProcessorThread;
#endif

#ifdef STM32_DMA2_STREAM2_AVAILABLE
    postProcessorThread[ Thor::LLD::DMA::DMA2_STREAM2_RESOURCE_INDEX ] = DMA2_Stream2_ISRPostProcessorThread;
#endif

#ifdef STM32_DMA2_STREAM3_AVAILABLE
    postProcessorThread[ Thor::LLD::DMA::DMA2_STREAM3_RESOURCE_INDEX ] = DMA2_Stream3_ISRPostProcessorThread;
#endif

#ifdef STM32_DMA2_STREAM4_AVAILABLE
    postProcessorThread[ Thor::LLD::DMA::DMA2_STREAM4_RESOURCE_INDEX ] = DMA2_Stream4_ISRPostProcessorThread;
#endif

#ifdef STM32_DMA2_STREAM5_AVAILABLE
    postProcessorThread[ Thor::LLD::DMA::DMA2_STREAM5_RESOURCE_INDEX ] = DMA2_Stream5_ISRPostProcessorThread;
#endif

#ifdef STM32_DMA2_STREAM6_AVAILABLE
    postProcessorThread[ Thor::LLD::DMA::DMA2_STREAM6_RESOURCE_INDEX ] = DMA2_Stream6_ISRPostProcessorThread;
#endif

#ifdef STM32_DMA2_STREAM7_AVAILABLE
    postProcessorThread[ Thor::LLD::DMA::DMA2_STREAM7_RESOURCE_INDEX ] = DMA2_Stream7_ISRPostProcessorThread;
#endif

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
    for ( uint8_t resourceIndex = 0; resourceIndex < Thor::LLD::DMA::dmaObjects.size(); resourceIndex++ )
    {
      /*------------------------------------------------
      Dynamically allocate peripheral drivers
      ------------------------------------------------*/
      if ( !Thor::LLD::DMA::dmaObjects[ resourceIndex ] )
      {
        Thor::LLD::DMA::dmaObjects[ resourceIndex ] = new Thor::LLD::DMA::Driver();
        Thor::LLD::DMA::dmaObjects[ resourceIndex ]->attach( Thor::LLD::DMA::periphInstanceList[ resourceIndex ] );
      }

      /*------------------------------------------------
      Initialize the drivers
      ------------------------------------------------*/
      if ( Thor::LLD::DMA::dmaObjects[ resourceIndex ]->init() != Chimera::CommonStatusCodes::OK )
      {
        return Chimera::CommonStatusCodes::FAIL;
      }

      // TODO: This is going to fail big time
      force me to fix 
      /*------------------------------------------------
      Initialize ISR handler thread resources
      ------------------------------------------------*/
      if ( postProcessorThread[ streamResourceIndex ] )
      {
        Chimera::Threading::Thread thread;
        thread.initialize( postProcessorThread[ streamResourceIndex ], nullptr, Chimera::Threading::Priority::LEVEL_5, 500,
                           "" );
        thread.start();
        postProcessorHandle[ streamResourceIndex ] = thread.native_handle();
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

    if ( streamIsOnPeripheral( Thor::LLD::DMA::DMA1_PERIPH, stream ) && Thor::LLD::DMA::dmaObjects[ 0 ] )
    {
      result = Thor::LLD::DMA::dmaObjects[ 0 ]->registerListener( stream, listener, timeout, registrationID );
    }
    else if ( streamIsOnPeripheral( Thor::LLD::DMA::DMA2_PERIPH, stream ) && Thor::LLD::DMA::dmaObjects[ 1 ] )
    {
      result = Thor::LLD::DMA::dmaObjects[ 1 ]->registerListener( stream, listener, timeout, registrationID );
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

    if ( streamIsOnPeripheral( Thor::LLD::DMA::DMA1_PERIPH, stream ) && Thor::LLD::DMA::dmaObjects[ 0 ] )
    {
      result = Thor::LLD::DMA::dmaObjects[ 0 ]->removeListener( stream, registrationID, timeout );
    }
    else if ( streamIsOnPeripheral( Thor::LLD::DMA::DMA2_PERIPH, stream ) && Thor::LLD::DMA::dmaObjects[ 1 ] )
    {
      result = Thor::LLD::DMA::dmaObjects[ 1 ]->removeListener( stream, registrationID, timeout );
    }
    else
    {
      result = Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
    }

    return result;
  }

  static Thor::LLD::DMA::Driver *const getDriver( const Chimera::DMA::Init &config )
  {
    Thor::LLD::DMA::Driver *driver = nullptr;

    /*------------------------------------------------
    Find the request meta info using binary search of sorted list
    ------------------------------------------------*/
    Thor::LLD::DMA::StreamResources c;
    c.clear();
    c.requestID = config.request;

    auto metaInfo = std::lower_bound(
        RequestGenerators.begin(), RequestGenerators.end(), c,
        []( const Thor::LLD::DMA::StreamResources &a, const Thor::LLD::DMA::StreamResources &b ) { return b.requestID < a.requestID; } );

    if ( metaInfo->requestID != config.request )
    {
      return driver;
    }

    /*------------------------------------------------
    Pull out the stream object
    ------------------------------------------------*/
    if ( metaInfo->cfgBitField & Thor::LLD::DMA::ConfigBitFields::DMA_ON_DMA1 )
    {
      driver = Thor::LLD::DMA::dmaObjects[ 0 ];
    }
    else if ( metaInfo->cfgBitField & Thor::LLD::DMA::ConfigBitFields::DMA_ON_DMA2 )
    {
      driver = Thor::LLD::DMA::dmaObjects[ 1 ];
    }

    return driver;
  }

  static Thor::LLD::DMA::StreamX *const getStream( const Chimera::DMA::Init &config )
  {
    Thor::LLD::DMA::StreamX *stream = nullptr;
    uint32_t streamNum         = 0;

    /*------------------------------------------------
    Find the request meta info using binary search of sorted list
    ------------------------------------------------*/
    Thor::LLD::DMA::StreamResources c;
    c.clear();
    c.requestID = config.request;

    auto metaInfo = std::lower_bound(
        RequestGenerators.begin(), RequestGenerators.end(), c,
        []( const Thor::LLD::DMA::StreamResources &a, const Thor::LLD::DMA::StreamResources &b ) { return b.requestID < a.requestID; } );

    if ( metaInfo->requestID != config.request )
    {
      return stream;
    }

    /*------------------------------------------------
    Pull out the stream object
    ------------------------------------------------*/
    streamNum =
        ( metaInfo->cfgBitField & Thor::LLD::DMA::ConfigBitFields::DMA_STREAM ) >> Thor::LLD::DMA::ConfigBitFields::DMA_STREAM_POS;
    if ( metaInfo->cfgBitField & Thor::LLD::DMA::ConfigBitFields::DMA_ON_DMA1 )
    {
      stream = Thor::LLD::DMA::getStreamRegisters( Thor::LLD::DMA::DMA1_PERIPH, streamNum );
    }
    else if ( metaInfo->cfgBitField & Thor::LLD::DMA::ConfigBitFields::DMA_ON_DMA2 )
    {
      stream = Thor::LLD::DMA::getStreamRegisters( Thor::LLD::DMA::DMA2_PERIPH, streamNum );
    }

    return stream;
  }

  static Thor::LLD::DMA::StreamConfig convertConfig( const Chimera::DMA::Init &config )
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

  static Thor::LLD::DMA::TCB convertTCB( const Chimera::DMA::TCB &transfer )
  {
    using namespace DMADriver;

    TCB tcb;
    tcb.srcAddress   = transfer.srcAddress;
    tcb.dstAddress   = transfer.dstAddress;
    tcb.transferSize = static_cast<uint32_t>( transfer.transferSize );

    return tcb;
  }
}    // namespace Thor::DMA



static void DMA1_Stream0_ISRPostProcessorThread( void *argument )
{
  using namespace Thor::LLD::DMA;
  constexpr auto resourceIndex = DMA1_STREAM0_RESOURCE_INDEX;

  while ( 1 )
  {
    postProcessorSignal[ resourceIndex ].acquire();
    if ( auto stream = streamObjects[ resourceIndex ]; stream )
      {
        stream->postISRProcessing();
      }
  }
}

static void DMA1_Stream1_ISRPostProcessorThread( void *argument )
{
  using namespace Thor::LLD::DMA;
  constexpr auto resourceIndex = DMA1_STREAM1_RESOURCE_INDEX;
  
  while ( 1 )
  {
    postProcessorSignal[ resourceIndex ].acquire();
    if ( auto stream = streamObjects[ resourceIndex ]; stream )
      {
        stream->postISRProcessing();
      }
  }
}

static void DMA1_Stream2_ISRPostProcessorThread( void *argument )
{
  using namespace Thor::LLD::DMA;
  constexpr auto resourceIndex = DMA1_STREAM2_RESOURCE_INDEX;

  while ( 1 )
  {
    postProcessorSignal[ resourceIndex ].acquire();
    if ( auto stream = streamObjects[ resourceIndex ]; stream )
      {
        stream->postISRProcessing();
      }
  }
}

static void DMA1_Stream3_ISRPostProcessorThread( void *argument )
{
  using namespace Thor::LLD::DMA;
  constexpr auto resourceIndex = DMA1_STREAM3_RESOURCE_INDEX;

  while ( 1 )
  {
    postProcessorSignal[ resourceIndex ].acquire();
    if ( auto stream = streamObjects[ resourceIndex ]; stream )
      {
        stream->postISRProcessing();
      }
  }
}

static void DMA1_Stream4_ISRPostProcessorThread( void *argument )
{
  using namespace Thor::LLD::DMA;
  constexpr auto resourceIndex = DMA1_STREAM4_RESOURCE_INDEX;

  while ( 1 )
  {
    postProcessorSignal[ resourceIndex ].acquire();
    if ( auto stream = streamObjects[ resourceIndex ]; stream )
      {
        stream->postISRProcessing();
      }
  }
}

static void DMA1_Stream5_ISRPostProcessorThread( void *argument )
{
  using namespace Thor::LLD::DMA;
  constexpr auto resourceIndex = DMA1_STREAM5_RESOURCE_INDEX;

  while ( 1 )
  {
    postProcessorSignal[ resourceIndex ].acquire();
    if ( auto stream = streamObjects[ resourceIndex ]; stream )
      {
        stream->postISRProcessing();
      }
  }
}

static void DMA1_Stream6_ISRPostProcessorThread( void *argument )
{
  using namespace Thor::LLD::DMA;
  constexpr auto resourceIndex = DMA1_STREAM6_RESOURCE_INDEX;

  while ( 1 )
  {
    postProcessorSignal[ resourceIndex ].acquire();
    if ( auto stream = streamObjects[ resourceIndex ]; stream )
      {
        stream->postISRProcessing();
      }
  }
}

static void DMA1_Stream7_ISRPostProcessorThread( void *argument )
{
  using namespace Thor::LLD::DMA;
  constexpr auto resourceIndex = DMA1_STREAM7_RESOURCE_INDEX;

  while ( 1 )
  {
    postProcessorSignal[ resourceIndex ].acquire();
    if ( auto stream = streamObjects[ resourceIndex ]; stream )
      {
        stream->postISRProcessing();
      }
  }
}

static void DMA2_Stream0_ISRPostProcessorThread( void *argument )
{
  using namespace Thor::LLD::DMA;
  constexpr auto resourceIndex = DMA2_STREAM0_RESOURCE_INDEX;

  while ( 1 )
  {
    postProcessorSignal[ resourceIndex ].acquire();
    if ( auto stream = streamObjects[ resourceIndex ]; stream )
      {
        stream->postISRProcessing();
      }
  }
}

static void DMA2_Stream1_ISRPostProcessorThread( void *argument )
{
  using namespace Thor::LLD::DMA;
  constexpr auto resourceIndex = DMA2_STREAM1_RESOURCE_INDEX;

  while ( 1 )
  {
    postProcessorSignal[ resourceIndex ].acquire();
    if ( auto stream = streamObjects[ resourceIndex ]; stream )
      {
        stream->postISRProcessing();
      }
  }
}

static void DMA2_Stream2_ISRPostProcessorThread( void *argument )
{
  using namespace Thor::LLD::DMA;
  constexpr auto resourceIndex = DMA2_STREAM2_RESOURCE_INDEX;

  while ( 1 )
  {
    postProcessorSignal[ resourceIndex ].acquire();
    if ( auto stream = streamObjects[ resourceIndex ]; stream )
      {
        stream->postISRProcessing();
      }
  }
}

static void DMA2_Stream3_ISRPostProcessorThread( void *argument )
{
  using namespace Thor::LLD::DMA;
  constexpr auto resourceIndex = DMA2_STREAM3_RESOURCE_INDEX;

  while ( 1 )
  {
    postProcessorSignal[ resourceIndex ].acquire();
    if ( auto stream = streamObjects[ resourceIndex ]; stream )
      {
        stream->postISRProcessing();
      }
  }
}

static void DMA2_Stream4_ISRPostProcessorThread( void *argument )
{
  using namespace Thor::LLD::DMA;
  constexpr auto resourceIndex = DMA2_STREAM4_RESOURCE_INDEX;

  while ( 1 )
  {
    postProcessorSignal[ resourceIndex ].acquire();
    if ( auto stream = streamObjects[ resourceIndex ]; stream )
      {
        stream->postISRProcessing();
      }
  }
}

static void DMA2_Stream5_ISRPostProcessorThread( void *argument )
{
  using namespace Thor::LLD::DMA;
  constexpr auto resourceIndex = DMA2_STREAM5_RESOURCE_INDEX;

  while ( 1 )
  {
    postProcessorSignal[ resourceIndex ].acquire();
    if ( auto stream = streamObjects[ resourceIndex ]; stream )
      {
        stream->postISRProcessing();
      }
  }
}

static void DMA2_Stream6_ISRPostProcessorThread( void *argument )
{
  using namespace Thor::LLD::DMA;
  constexpr auto resourceIndex = DMA2_STREAM6_RESOURCE_INDEX;

  while ( 1 )
  {
    postProcessorSignal[ resourceIndex ].acquire();
    if ( auto stream = streamObjects[ resourceIndex ]; stream )
      {
        stream->postISRProcessing();
      }
  }
}

static void DMA2_Stream7_ISRPostProcessorThread( void *argument )
{
  using namespace Thor::LLD::DMA;
  constexpr auto resourceIndex = DMA2_STREAM7_RESOURCE_INDEX;

  while ( 1 )
  {
    postProcessorSignal[ resourceIndex ].acquire();
    if ( auto stream = streamObjects[ resourceIndex ]; stream )
      {
        stream->postISRProcessing();
      }
  }
}

