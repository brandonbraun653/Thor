/******************************************************************************
 *  File Name:
 *    hld_timer_trigger.cpp
 *
 *  Description:
 *    High level driver for triggered timers
 *
 *  2022-2023 | Brandon Braun | brandonbraun653@protonmail.com
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

struct ControlBlock
{
  Thor::LLD::TIMER::Handle_rPtr         timer;     /**< Handle to the timer */
  Chimera::Timer::Instance              instance;  /**< The instance of the timer */
  Chimera::Timer::Trigger::MasterConfig masterCfg; /**< Configuration data */
  Chimera::Timer::Trigger::SlaveConfig  slaveCfg;  /**< Configuration data */
};

/*-----------------------------------------------------------------------------
Static Data
-----------------------------------------------------------------------------*/
static Chimera::DeviceManager<ControlBlock, Chimera::Timer::Instance, EnumValue( Chimera::Timer::Instance::NUM_OPTIONS )>
    s_timer_data;

/*-----------------------------------------------------------------------------
Static Functions
-----------------------------------------------------------------------------*/
static inline ControlBlock *getControlBlock( void *ptr )
{
  RT_DBG_ASSERT( ptr != nullptr );
  return reinterpret_cast<ControlBlock *>( ptr );
}


namespace Chimera::Timer::Trigger
{
  /*---------------------------------------------------------------------------
  Base Implementation
  ---------------------------------------------------------------------------*/
  ITimerTrigger::ITimerTrigger() : mTimerImpl( nullptr )
  {
  }


  ITimerTrigger::~ITimerTrigger()
  {
  }


  Chimera::Status_t ITimerTrigger::enable()
  {
    Thor::LLD::TIMER::enableCounter( getControlBlock( mTimerImpl )->timer );
    return Chimera::Status::OK;
  }


  Chimera::Status_t ITimerTrigger::disable()
  {
    Thor::LLD::TIMER::disableCounter( getControlBlock( mTimerImpl )->timer );
    return Chimera::Status::OK;
  }


  void ITimerTrigger::detachISR()
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


  void ITimerTrigger::ackISR()
  {
    using namespace Thor::LLD::TIMER;

    // TODO: Might want to ack everything here
    CC1IF::clear( getControlBlock( mTimerImpl )->timer->registers, SR_CC1IF );
  }


  /*---------------------------------------------------------------------------
  Master Implementation
  ---------------------------------------------------------------------------*/
  Master::Master()
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
    auto          result = Chimera::Status::OK;
    ControlBlock *cb     = s_timer_data.getOrCreate( cfg.coreConfig.instance );
    RT_HARD_ASSERT( cb );

    if ( !mTimerImpl )
    {
      mTimerImpl = reinterpret_cast<void *>( cb );
    }

    /*-------------------------------------------------------------------------
    Initialize the control block data
    -------------------------------------------------------------------------*/
    RT_HARD_ASSERT( Chimera::Status::OK == allocate( cfg.coreConfig.instance ) );

    cb->timer     = Thor::LLD::TIMER::getHandle( cfg.coreConfig.instance );
    cb->instance  = cfg.coreConfig.instance;
    cb->masterCfg = cfg;

    RT_DBG_ASSERT( cb->timer );

    /*-------------------------------------------------------------------------
    Initialize the system timer interrupt priorities
    -------------------------------------------------------------------------*/
    const auto irqSignal = getHWISRIndex( cb->instance, ISRExtended::NONE );

    disableIRQ( irqSignal );
    clearPendingIRQ( irqSignal );
    setPriority( irqSignal, TIMER_IT_PREEMPT_PRIORITY, 0u );

    /*-------------------------------------------------------------------------
    Base timer clocking setup
    -------------------------------------------------------------------------*/
    result |= Thor::LLD::TIMER::Master::initCore( cb->timer, cfg.coreConfig );
    setAlignment( cb->timer, AlignMode::EDGE_ALIGNED );
    setCountDirection( cb->timer, CountDir::COUNT_UP );

    /*-------------------------------------------------------------------------
    Set the overflow rate to the desired frequency
    -------------------------------------------------------------------------*/
    result |= setEventRate( cb->timer, ( 1.0f / cfg.trigFreq ) * 1e9f );

    /*-------------------------------------------------------------------------
    Set the output compare behavior to drive the TRGO signal
    -------------------------------------------------------------------------*/
    auto channel = Chimera::Timer::Channel::CHANNEL_1;
    auto output  = Chimera::Timer::Output::OUTPUT_1P;

    disableCCOutput( cb->timer, output );
    setCCMode( cb->timer, channel, CCMode::CCM_OUTPUT );
    setCCOutputPolarity( cb->timer, output, CCPolarity::CCP_OUT_ACTIVE_HIGH );
    setOCMode( cb->timer, channel, OCMode::OC_MODE_PWM_MODE_2 );
    setOCReference( cb->timer, channel, cb->timer->registers->ARR / 2 );
    enableCCOutput( cb->timer, output );
    setMasterMode( cb->timer, MasterMode::COMPARE_OC1REF );

    /*-------------------------------------------------------------------------
    Attach the interrupt handler
    -------------------------------------------------------------------------*/
    if ( cfg.isrCallback )
    {
      result |= this->attachISR( cfg.isrCallback );
    }

    return result;
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


  /*---------------------------------------------------------------------------
  Slave Implmentation
  ---------------------------------------------------------------------------*/
  Slave::Slave()
  {
  }


  Slave::~Slave()
  {
  }


  Chimera::Status_t Slave::init( const SlaveConfig &cfg )
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
    auto          result = Chimera::Status::OK;
    ControlBlock *cb     = s_timer_data.getOrCreate( cfg.coreConfig.instance );
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
    cb->slaveCfg = cfg;

    RT_DBG_ASSERT( cb->timer );

    /*-------------------------------------------------------------------------
    Initialize the system timer interrupt priorities
    -------------------------------------------------------------------------*/
    const auto irqSignal = getHWISRIndex( cb->instance, ISRExtended::NONE );

    disableIRQ( irqSignal );
    clearPendingIRQ( irqSignal );
    setPriority( irqSignal, TIMER_IT_PREEMPT_PRIORITY, 0u );

    /*-------------------------------------------------------------------------
    Base timer clocking setup
    -------------------------------------------------------------------------*/
    result |= Thor::LLD::TIMER::Master::initCore( cb->timer, cfg.coreConfig );
    setAlignment( cb->timer, AlignMode::EDGE_ALIGNED );
    setCountDirection( cb->timer, CountDir::COUNT_UP );

    /*-------------------------------------------------------------------------
    Set the overflow rate to the desired frequency
    -------------------------------------------------------------------------*/
    result |= setEventRate( cb->timer, ( 1.0f / cfg.frequency ) * 1e9f );

    /*-------------------------------------------------------------------------
    Set the output compare behavior to drive the TRGO signal
    -------------------------------------------------------------------------*/
    auto channel = Chimera::Timer::Channel::CHANNEL_1;
    auto output  = Chimera::Timer::Output::OUTPUT_1P;

    disableCCOutput( cb->timer, output );
    setCCMode( cb->timer, channel, CCMode::CCM_OUTPUT );
    setCCOutputPolarity( cb->timer, output, CCPolarity::CCP_OUT_ACTIVE_HIGH );
    setOCMode( cb->timer, channel, OCMode::OC_MODE_PWM_MODE_2 );
    setOCReference( cb->timer, channel, 0 );
    enableCCOutput( cb->timer, output );
    setMasterMode( cb->timer, MasterMode::COMPARE_PULSE );

    /*-------------------------------------------------------------------------
    Enable the master/slave synchronization, which might not be used.
    -------------------------------------------------------------------------*/
    // setMasterSlaveSync( cb->timer, MasterSlaveSync::ENABLED );

    /*-------------------------------------------------------------------------
    Map the synchronization action to a slave behavior mode
    -------------------------------------------------------------------------*/
    switch ( cfg.trigSyncAction )
    {
      case Chimera::Timer::Trigger::SyncAction::SYNC_RESET:
        setSlaveMode( cb->timer, SlaveMode::RESET );
        break;

      case Chimera::Timer::Trigger::SyncAction::SYNC_GATE:
        setSlaveMode( cb->timer, SlaveMode::GATED );
        break;

      case Chimera::Timer::Trigger::SyncAction::SYNC_TRIGGER:
        setSlaveMode( cb->timer, SlaveMode::TRIGGER );
        break;

      default:
        RT_HARD_ASSERT( false );
        break;
    }

    /*-------------------------------------------------------------------------
    Map the synchronization signal to a slave trigger source
    -------------------------------------------------------------------------*/
    switch ( cfg.trigSyncSignal )
    {
      case Chimera::Timer::Trigger::Signal::TRIG_SIG_0:
        setSlaveTriggerSource( cb->timer, Thor::LLD::TIMER::Trigger::INTERNAL_0 );
        break;

      case Chimera::Timer::Trigger::Signal::TRIG_SIG_1:
        setSlaveTriggerSource( cb->timer, Thor::LLD::TIMER::Trigger::INTERNAL_1 );
        break;

      case Chimera::Timer::Trigger::Signal::TRIG_SIG_2:
        setSlaveTriggerSource( cb->timer, Thor::LLD::TIMER::Trigger::INTERNAL_2 );
        break;

      case Chimera::Timer::Trigger::Signal::TRIG_SIG_3:
        setSlaveTriggerSource( cb->timer, Thor::LLD::TIMER::Trigger::INTERNAL_3 );
        break;

      case Chimera::Timer::Trigger::Signal::TRIG_SIG_4:
        setSlaveTriggerSource( cb->timer, Thor::LLD::TIMER::Trigger::FILTERED_TI1 );
        break;

      case Chimera::Timer::Trigger::Signal::TRIG_SIG_5:
        setSlaveTriggerSource( cb->timer, Thor::LLD::TIMER::Trigger::FILTERED_TI2 );
        break;

      case Chimera::Timer::Trigger::Signal::TRIG_SIG_6:
        setSlaveTriggerSource( cb->timer, Thor::LLD::TIMER::Trigger::EXTERNAL_TI1 );
        break;

      default:
        RT_HARD_ASSERT( false );
        break;
    }

    /*-------------------------------------------------------------------------
    Attach the interrupt handler
    -------------------------------------------------------------------------*/
    if ( cfg.isrCallback )
    {
      result |= this->attachISR( cfg.isrCallback );
    }

    return result;
  }


  uint32_t Slave::getTickPeriod() const
  {
    return getControlBlock( mTimerImpl )->timer->registers->ARR;
  }


  void Slave::setEventOffset( const uint32_t tickOffset )
  {
    using namespace Thor::LLD::TIMER;
    setOCReference( getControlBlock( mTimerImpl )->timer, Chimera::Timer::Channel::CHANNEL_1, tickOffset );
  }


  Chimera::Status_t Slave::attachISR( Chimera::Function::Opaque func )
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

}    // namespace Chimera::Timer::Trigger
