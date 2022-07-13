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
  Thor::LLD::TIMER::Handle_rPtr timer;   /**< Handle to the timer */
};

/*-----------------------------------------------------------------------------
Static Data
-----------------------------------------------------------------------------*/
static Chimera::DeviceManager<TriggerControlBlock, Chimera::Timer::Instance, EnumValue( Chimera::Timer::Instance::NUM_OPTIONS )>
    s_timer_data;

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
    TriggerControlBlock *cb = s_timer_data.getOrCreate( cfg.coreConfig.instance );
    RT_HARD_ASSERT( cb );

    if ( !mTimerImpl )
    {
      mTimerImpl  = std::make_shared<void *>();
      *mTimerImpl = reinterpret_cast<void *>( cb );
    }

    /*-------------------------------------------------------------------------
    Initialize the control block data
    -------------------------------------------------------------------------*/
    RT_HARD_ASSERT( Chimera::Status::OK == allocate( cfg.coreConfig.instance ) );
    cb->timer = Thor::LLD::TIMER::getHandle( cfg.coreConfig.instance );

    /*-------------------------------------------------------------------------
    Configure the timer for desired operation
    -------------------------------------------------------------------------*/
    auto channel = Chimera::Timer::Channel::CHANNEL_1;
    auto result = Chimera::Status::OK;

    /* Base timer setup */
    result |= Thor::LLD::TIMER::Master::initCore( cb->timer, cfg.coreConfig );

    /* Trigger event rate set by overflow rate */
    result |= setEventRate( cb->timer, ( 1.0f / cfg.trigFreq ) * 1e9f );

    /* Set TRGO to fire on Output Compare match */
    setMasterMode( cb->timer, MasterMode::COMPARE_OC1REF );

    /* Configure capture/compare behavior */
    disableCCChannel( cb->timer, channel );
    setCCMode( cb->timer, channel, CCMode::CCM_OUTPUT );
    setCCPolarity( cb->timer, channel, CCPolarity::CCP_OUT_ACTIVE_HIGH );
    setOCMode( cb->timer, channel, OCMode::OC_MODE_TOGGLE_MATCH );
    setOCReference( cb->timer, channel, 0 );
    enableCCChannel( cb->timer, channel );

    return result;
  }


  Chimera::Status_t Master::enable()
  {
    if( !mTimerImpl )
    {
      return Chimera::Status::NOT_INITIALIZED;
    }

    TriggerControlBlock *cb = reinterpret_cast<TriggerControlBlock *>( *mTimerImpl );
    Thor::LLD::TIMER::enableCounter( cb->timer );
    return Chimera::Status::OK;
  }


  Chimera::Status_t Master::disable()
  {
    if( !mTimerImpl )
    {
      return Chimera::Status::NOT_INITIALIZED;
    }

    TriggerControlBlock *cb = reinterpret_cast<TriggerControlBlock *>( *mTimerImpl );
    Thor::LLD::TIMER::disableCounter( cb->timer );
    return Chimera::Status::OK;
  }

}    // namespace Chimera::Timer::Trigger
