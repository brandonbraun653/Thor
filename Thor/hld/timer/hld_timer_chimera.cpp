/********************************************************************************
 *  File Name:
 *    hld_timer_chimera.cpp
 *
 *	 Description:
 *    Implementation of Chimera Timer driver hooks
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* STL Includes */
#include <memory>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/timer>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/timer>
#include <Thor/hld/timer/hld_timer_chimera.hpp>
#include <Thor/hld/timer/hld_timer_prv_driver.hpp>
#include <Thor/lld/interface/timer/timer_detail.hpp>



static Thor::TIMER::AdvancedDriver s_test_advanced_driver;
static Thor::TIMER::BasicDriver s_test_basic_driver;
static Thor::TIMER::GeneralDriver s_test_general_driver;
static Thor::TIMER::LowPowerDriver s_test_low_power_driver;

namespace Chimera::Timer
{
  namespace Backend
  {
    Chimera::Status_t initialize()
    {
      return Thor::TIMER::initialize();
    }

    Chimera::Status_t reset()
    {
      return Thor::TIMER::reset();
    }

    size_t millis()
    {
      return Thor::TIMER::millis();
    }

    void delayMilliseconds( const size_t val )
    {
      Thor::TIMER::delayMilliseconds( val );
    }

    void delayMicroseconds( const size_t val )
    {
      Thor::TIMER::delayMicroseconds( val );
    }


    // Function definitions to create 
//    ITimerEncoder_sPtr create_encoder_shared_ptr();
//    ITimerEncoder_uPtr create_encoder_unique_ptr();
//    ITimerInputCapture_sPtr create_input_capture_shared_ptr();
//    ITimerInputCapture_uPtr create_input_capture_unique_ptr();
//    ITimerOnePulse_sPtr create_one_pulse_shared_ptr();
//    ITimerOnePulse_uPtr create_one_pulse_unique_ptr();
//    ITimerOutputCompare_sPtr create_output_compare_shared_ptr();
//    ITimerOutputCompare_uPtr create_output_compare_unique_ptr();
//    ITimerPWM_sPtr create_pwm_shared_ptr();
//    ITimerPWM_uPtr create_pwm_unique_ptr();


    Chimera::Status_t registerDriver( Chimera::Timer::Backend::DriverRegistration &registry )
    {
#if defined( THOR_HLD_TIMER )
      registry.isSupported       = true;
      registry.initialize        = initialize;
      registry.reset             = reset;
      registry.delayMicroseconds = delayMicroseconds;
      registry.delayMilliseconds = delayMilliseconds;
      registry.millis            = millis;
      return Chimera::CommonStatusCodes::OK;
#else
      registry.isSupported       = false;
      registry.initialize        = nullptr;
      registry.reset             = nullptr;
      registry.create_shared_ptr = nullptr;
      registry.create_unique_ptr = nullptr;
      registry.delayMicroseconds = nullptr;
      registry.delayMilliseconds = nullptr;
      registry.millis            = nullptr;
      return Chimera::CommonStatusCodes::NOT_SUPPORTED;
#endif /* THOR_HLD_TIMER */
    }
  }    // namespace Backend


#if !defined( VIRTUAL_FUNC )


  Chimera::Timer::ITimerBase_sPtr getTimerAsBase( const Chimera::Timer::Peripheral periph )
  {
    return Thor::TIMER::getTimerAsBase( periph );
  }

  Chimera::Timer::ITimerEncoder_sPtr getTimerAsENC( const Chimera::Timer::Peripheral periph )
  {
    return Thor::TIMER::getTimerAsENC( periph );
  }

  Chimera::Timer::ITimerInputCapture_sPtr getTimerAsIC( const Chimera::Timer::Peripheral periph )
  {
    return Thor::TIMER::getTimerAsIC( periph );
  }

  Chimera::Timer::ITimerOutputCompare_sPtr getTimerAsOC( const Chimera::Timer::Peripheral periph )
  {
    return Thor::TIMER::getTimerAsOC( periph );
  }

  Chimera::Timer::ITimerOnePulse_sPtr getTimerAsOP( const Chimera::Timer::Peripheral periph )
  {
    return Thor::TIMER::getTimerAsOP( periph );
  }

  Chimera::Timer::ITimerPWM_sPtr getTimerAsPWM( const Chimera::Timer::Peripheral periph )
  {
    return Thor::TIMER::getTimerAsPWM( periph );
  }


  static std::array<size_t, static_cast<size_t>( Chimera::Timer::Peripheral::NUM_OPTIONS )>
      s_chimera_peripheral_to_resource_index;

  static std::array<void *, static_cast<size_t>( Chimera::Timer::Peripheral::NUM_OPTIONS )> s_hld_timer_drivers;


  TimerPWMImpl::TimerPWMImpl() : mCfg( {} )
  {
  }

  TimerPWMImpl::~TimerPWMImpl()
  {
  }

  Chimera::Status_t TimerPWMImpl::pwmInit( const Chimera::Timer::PWM::Config &cfg )
  {
    mCfg = cfg;

    /*------------------------------------------------
    Cache the resource index for later lookup
    ------------------------------------------------*/
    auto iLookup   = static_cast<size_t>( mCfg.peripheral );
    auto iResource = Thor::LLD::TIMER::PeripheralToHLDResourceIndex.find( mCfg.peripheral )->second;
    
    /*------------------------------------------------
    Initialize local HLD object memory cache for the given resource
    ------------------------------------------------*/
    auto hld = Thor::TIMER::getDriverAddress( mCfg.peripheral, iResource );

    if ( !s_hld_timer_drivers[ iLookup ] ) 
    {
      s_hld_timer_drivers[ iLookup ] = hld;
      s_chimera_peripheral_to_resource_index[ iLookup ] = iResource;
    }

    /*------------------------------------------------
    Call the driver's method
    ------------------------------------------------*/
    return reinterpret_cast<TimerPWMImpl *>( hld )->pwmInit( cfg );
  }




  TimerBaseImpl::TimerBaseImpl() : mCfg( {} )
  {
  }

  TimerBaseImpl::~TimerBaseImpl()
  {
  }

  Chimera::Status_t TimerBaseImpl::initPeripheral( const Chimera::Timer::DriverConfig &cfg )
  {
    mCfg = cfg;

    /*------------------------------------------------
    Cache the resource index for later lookup
    ------------------------------------------------*/
    auto iLookup   = static_cast<size_t>( mCfg.peripheral );
    auto iResource = Thor::LLD::TIMER::PeripheralToHLDResourceIndex.find( mCfg.peripheral )->second;

    /*------------------------------------------------
    Initialize local HLD object memory cache for the given resource
    ------------------------------------------------*/
    auto hld = Thor::TIMER::getDriverAddress( mCfg.peripheral, iResource );

    if ( !s_hld_timer_drivers[ iLookup ] ) 
    {
      s_hld_timer_drivers[ iLookup ] = hld;
      s_chimera_peripheral_to_resource_index[ iLookup ] = iResource;
    }

    /*------------------------------------------------
    Call the driver's method
    ------------------------------------------------*/
    return reinterpret_cast<TimerBaseImpl *>( hld )->initPeripheral( cfg );
  }

  bool TimerBaseImpl::hasFunction( const Chimera::Timer::Function func )
  {
    /*------------------------------------------------
    Retreive the driver instance
    ------------------------------------------------*/
    auto iResource = s_chimera_peripheral_to_resource_index[ static_cast<size_t>( mCfg.peripheral ) ];
    auto hld       = s_hld_timer_drivers[ iResource ];

    /*------------------------------------------------
    Call the driver's method
    ------------------------------------------------*/
    return reinterpret_cast<TimerBaseImpl *>( hld )->hasFunction( func );
  }

  Chimera::Status_t TimerBaseImpl::enable( const Chimera::Timer::Channel channel )
  {
    /*------------------------------------------------
    Retreive the driver instance
    ------------------------------------------------*/
    auto iResource = s_chimera_peripheral_to_resource_index[ static_cast<size_t>( mCfg.peripheral ) ];
    auto hld       = s_hld_timer_drivers[ iResource ];

    /*------------------------------------------------
    Call the driver's method
    ------------------------------------------------*/
    return reinterpret_cast<TimerBaseImpl *>( hld )->enable( channel );
  }

  Chimera::Status_t TimerBaseImpl::disable( const Chimera::Timer::Channel channel )
  {
    /*------------------------------------------------
    Retreive the driver instance
    ------------------------------------------------*/
    auto iResource = s_chimera_peripheral_to_resource_index[ static_cast<size_t>( mCfg.peripheral ) ];
    auto hld       = s_hld_timer_drivers[ iResource ];

    /*------------------------------------------------
    Call the driver's method
    ------------------------------------------------*/
    return reinterpret_cast<TimerBaseImpl *>( hld )->disable( channel );
  }

  Chimera::Status_t TimerBaseImpl::enableEvent( const Chimera::Timer::Channel channel, const Chimera::Timer::Event type )
  {
    /*------------------------------------------------
    Retreive the driver instance
    ------------------------------------------------*/
    auto iResource = s_chimera_peripheral_to_resource_index[ static_cast<size_t>( mCfg.peripheral ) ];
    auto hld       = s_hld_timer_drivers[ iResource ];

    /*------------------------------------------------
    Call the driver's method
    ------------------------------------------------*/
    return reinterpret_cast<TimerBaseImpl *>( hld )->enableEvent( channel, type );
  }

  Chimera::Status_t TimerBaseImpl::disableEvent( const Chimera::Timer::Channel channel, const Chimera::Timer::Event type )
  {
    /*------------------------------------------------
    Retreive the driver instance
    ------------------------------------------------*/
    auto iResource = s_chimera_peripheral_to_resource_index[ static_cast<size_t>( mCfg.peripheral ) ];
    auto hld       = s_hld_timer_drivers[ iResource ];

    /*------------------------------------------------
    Call the driver's method
    ------------------------------------------------*/
    return reinterpret_cast<TimerBaseImpl *>( hld )->disableEvent( channel, type );
  }

  bool TimerBaseImpl::configured()
  {
    /*------------------------------------------------
    Retreive the driver instance
    ------------------------------------------------*/
    auto iResource = s_chimera_peripheral_to_resource_index[ static_cast<size_t>( mCfg.peripheral ) ];
    auto hld       = s_hld_timer_drivers[ iResource ];

    /*------------------------------------------------
    Call the driver's method
    ------------------------------------------------*/
    return reinterpret_cast<TimerBaseImpl *>( hld )->configured();
  }

  size_t TimerBaseImpl::counterBitWidth()
  {
    /*------------------------------------------------
    Retreive the driver instance
    ------------------------------------------------*/
    auto iResource = s_chimera_peripheral_to_resource_index[ static_cast<size_t>( mCfg.peripheral ) ];
    auto hld       = s_hld_timer_drivers[ iResource ];

    /*------------------------------------------------
    Call the driver's method
    ------------------------------------------------*/
    return reinterpret_cast<TimerBaseImpl *>( hld )->counterBitWidth();
  }

  size_t TimerBaseImpl::tickRate( const Chimera::Units::Time units )
  {
    /*------------------------------------------------
    Retreive the driver instance
    ------------------------------------------------*/
    auto iResource = s_chimera_peripheral_to_resource_index[ static_cast<size_t>( mCfg.peripheral ) ];
    auto hld       = s_hld_timer_drivers[ iResource ];

    /*------------------------------------------------
    Call the driver's method
    ------------------------------------------------*/
    return reinterpret_cast<TimerBaseImpl *>( hld )->tickRate( units );
  }

  size_t TimerBaseImpl::maxPeriod( const Chimera::Units::Time units )
  {
    /*------------------------------------------------
    Retreive the driver instance
    ------------------------------------------------*/
    auto iResource = s_chimera_peripheral_to_resource_index[ static_cast<size_t>( mCfg.peripheral ) ];
    auto hld       = s_hld_timer_drivers[ iResource ];

    /*------------------------------------------------
    Call the driver's method
    ------------------------------------------------*/
    return reinterpret_cast<TimerBaseImpl *>( hld )->maxPeriod( units );
  }

  size_t TimerBaseImpl::minPeriod( const Chimera::Units::Time units )
  {
    /*------------------------------------------------
    Retreive the driver instance
    ------------------------------------------------*/
    auto iResource = s_chimera_peripheral_to_resource_index[ static_cast<size_t>( mCfg.peripheral ) ];
    auto hld       = s_hld_timer_drivers[ iResource ];

    /*------------------------------------------------
    Call the driver's method
    ------------------------------------------------*/
    return reinterpret_cast<TimerBaseImpl *>( hld )->minPeriod( units );
  }

#endif

}    // namespace Chimera::Timer::Backend
// namespace Chimera::Timer