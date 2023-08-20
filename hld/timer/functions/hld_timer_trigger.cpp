/******************************************************************************
 *  File Name:
 *    hld_timer_trigger.cpp
 *
 *  Description:
 *    High level driver for triggered timers
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <memory>
#include <Chimera/common>
#include <Chimera/peripheral>
#include <Chimera/timer>
#include <Thor/lld/interface/inc/timer>
#include <Thor/lld/interface/inc/interrupt>

/*-----------------------------------------------------------------------------
Constants
-----------------------------------------------------------------------------*/
static constexpr size_t REQ_HW_TIMER_TYPES =
    ( Thor::LLD::TIMER::HardwareType::TIMER_HW_ADVANCED | Thor::LLD::TIMER::HardwareType::TIMER_HW_BASIC |
      Thor::LLD::TIMER::HardwareType::TIMER_HW_GENERAL );

/*-----------------------------------------------------------------------------
Structures
-----------------------------------------------------------------------------*/
/**
 * @brief Controller data
 */
struct TriggerControlBlock
{
  Thor::LLD::TIMER::Handle_rPtr timer;    /**< Handle to the timer */
  Chimera::Timer::Instance      instance; /**< The instance of the timer */
};

/*-----------------------------------------------------------------------------
Static Data
-----------------------------------------------------------------------------*/
static Chimera::DeviceManager<TriggerControlBlock, Chimera::Timer::Instance, EnumValue( Chimera::Timer::Instance::NUM_OPTIONS )>
    s_timer_data;

/*-----------------------------------------------------------------------------
Static Functions
-----------------------------------------------------------------------------*/
static inline TriggerControlBlock *getControlBlock( void * ptr )
{
  RT_DBG_ASSERT( ptr != nullptr );
  return reinterpret_cast<TriggerControlBlock *>( ptr );
}

/*-----------------------------------------------------------------------------
Trigger Class Implementation
-----------------------------------------------------------------------------*/
namespace Chimera::Timer::Trigger
{

  Master::Master() : mTimerImpl( nullptr )
  {
  }


  Master::~Master()
  {
  }


  Chimera::Status_t Master::init( const MasterConfig &cfg )
  {
    using namespace Thor::LLD::TIMER;
    using namespace Thor::LLD::INT;

    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if ( !( getHardwareType( cfg.coreConfig.instance ) & REQ_HW_TIMER_TYPES ) )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }

    /*-------------------------------------------------------------------------
    Grab the driver for this instance and register it with the class
    -------------------------------------------------------------------------*/
    auto                 result = Chimera::Status::OK;
    TriggerControlBlock *cb     = s_timer_data.getOrCreate( cfg.coreConfig.instance );
    RT_HARD_ASSERT( cb );

    if ( !mTimerImpl )
    {
      mTimerImpl = reinterpret_cast<void *>( cb );
    }

    /*-------------------------------------------------------------------------
    Initialize the control block data
    -------------------------------------------------------------------------*/
    RT_HARD_ASSERT( Chimera::Status::OK == allocate( cfg.coreConfig.instance ) );
    cb->timer    = Thor::LLD::TIMER::getHandle( cfg.coreConfig.instance );
    cb->instance = cfg.coreConfig.instance;

    RT_DBG_ASSERT( cb->timer );

    /*-------------------------------------------------------------------------
    Initialize the system timer interrupt priorities
    -------------------------------------------------------------------------*/
    const auto irqSignal = getHWISRIndex( cb->instance, ISRExtended::NONE );

    disableIRQ( irqSignal );
    clearPendingIRQ( irqSignal );
    setPriority( irqSignal, TIMER_IT_PREEMPT_PRIORITY, 0u );

    /*-------------------------------------------------------------------------
    Reset and configure the timer for desired operation
    -------------------------------------------------------------------------*/
    auto channel = Chimera::Timer::Channel::CHANNEL_1;
    auto output  = Chimera::Timer::Output::OUTPUT_1P;

    /* Base timer setup */
    result |= Thor::LLD::TIMER::Master::initCore( cb->timer, cfg.coreConfig );
    setAlignment( cb->timer, AlignMode::EDGE_ALIGNED );
    setCountDirection( cb->timer, CountDir::COUNT_UP );

    /* Trigger event rate set by overflow rate */
    result |= setEventRate( cb->timer, ( 1.0f / cfg.trigFreq ) * 1e9f );

    /* Set TRGO to fire on Output Compare 1 match */
    setMasterMode( cb->timer, MasterMode::COMPARE_OC1REF );

    /* Configure capture/compare behavior */
    disableCCOutput( cb->timer, output );
    setCCMode( cb->timer, channel, CCMode::CCM_OUTPUT );
    setCCOutputPolarity( cb->timer, output, CCPolarity::CCP_OUT_ACTIVE_HIGH );
    setOCMode( cb->timer, channel, OCMode::OC_MODE_TOGGLE_MATCH );
    setOCReference( cb->timer, channel, 0 );
    enableCCOutput( cb->timer, output );

    /*-------------------------------------------------------------------------
    Attach the interrupt handler
    -------------------------------------------------------------------------*/
    if ( cfg.isrCallback )
    {
      result |= this->attachISR( cfg.isrCallback );
    }

    return result;
  }


  Chimera::Status_t Master::enable()
  {
    Thor::LLD::TIMER::enableCounter( getControlBlock( mTimerImpl )->timer );
    return Chimera::Status::OK;
  }


  Chimera::Status_t Master::disable()
  {
    Thor::LLD::TIMER::disableCounter( getControlBlock( mTimerImpl )->timer );
    return Chimera::Status::OK;
  }


  Chimera::Status_t Master::attachISR( Chimera::Function::Opaque func )
  {
    using namespace Thor::LLD::TIMER;
    using namespace Thor::LLD::INT;

    /*-------------------------------------------------------------------------
    Enable the global timer interrupt
    -------------------------------------------------------------------------*/
    const auto irqSignal = getHWISRIndex( getControlBlock( mTimerImpl )->instance, ISRExtended::NONE );
    clearPendingIRQ( irqSignal );
    enableIRQ( irqSignal );

    /*-------------------------------------------------------------------------
    Turn on the specific capture compare interrupt from the init section
    -------------------------------------------------------------------------*/
    enableISR( getControlBlock( mTimerImpl )->timer, ISRSource::CC1 );

    /*-------------------------------------------------------------------------
    Register the interrupt handler
    -------------------------------------------------------------------------*/
    return Thor::LLD::TIMER::attachISR( getControlBlock( mTimerImpl )->instance, func, ISRExtended::NONE );
  }


  void Master::detachISR()
  {
    using namespace Thor::LLD::TIMER;
    using namespace Thor::LLD::INT;

    /*-------------------------------------------------------------------------
    Disable the interrupt from the timer
    -------------------------------------------------------------------------*/
    const auto irqSignal = getHWISRIndex( getControlBlock( mTimerImpl )->instance, ISRExtended::NONE );
    disableIRQ( irqSignal );
    clearPendingIRQ( irqSignal );

    /*-------------------------------------------------------------------------
    Detach the interrupt
    -------------------------------------------------------------------------*/
    Thor::LLD::TIMER::detachISR( getControlBlock( mTimerImpl )->instance, Thor::LLD::TIMER::ISRExtended::NONE );
  }


  void Master::ackISR()
  {
    using namespace Thor::LLD::TIMER;
    CC1IF::clear( getControlBlock( mTimerImpl )->timer->registers, SR_CC1IF );
  }

}    // namespace Chimera::Timer::Trigger
