/********************************************************************************
 *  File Name:
 *    hld_timer_driver.hpp
 *
 *  Description:
 *    Thor high level driver for Timer
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_HLD_TIMER_DRIVER_HPP
#define THOR_HLD_TIMER_DRIVER_HPP

/* STL Includes */
#include <cstdlib>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/event>
#include <Chimera/timer>
#include <Chimera/thread>

/* Thor Includes */
#include <Thor/hld/timer/hld_timer_types.hpp>

namespace Thor::TIMER
{
  /*-------------------------------------------------------------------------------
  Chimera Based Free Function Declarations (see Chimera::Timer for documentation)
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize();

  Chimera::Status_t reset();

  void incrementSystemTick();

  size_t millis();

  void delayMilliseconds( const size_t val );

  void delayMicroseconds( const size_t val );

  /*-------------------------------------------------------------------------------
  Thor Specific Free Function Declarations
  -------------------------------------------------------------------------------*/
  AdvancedDriver_sPtr getAdvancedDriver_sPtr( const Chimera::Timer::Peripheral periph, const bool create = false );
  BasicDriver_sPtr getBasicDriver_sPtr( const Chimera::Timer::Peripheral periph, const bool create = false );
  GeneralDriver_sPtr getGeneralDriver_sPtr( const Chimera::Timer::Peripheral periph, const bool create = false );
  LowPowerDriver_sPtr getLowPowerDriver_sPtr( const Chimera::Timer::Peripheral periph, const bool create = false );


  /*-------------------------------------------------------------------------------
  High Level Driver Declaration
  -------------------------------------------------------------------------------*/
  #if defined( VIRTUAL_FUNC )
  class AdvancedDriver : virtual public Chimera::Timer::ITimerBase,
                         virtual public Chimera::Timer::ITimerEncoder,
                         virtual public Chimera::Timer::ITimerInputCapture,
                         virtual public Chimera::Timer::ITimerOnePulse,
                         virtual public Chimera::Timer::ITimerOutputCompare,
                         virtual public Chimera::Timer::ITimerPWM,
                         public Chimera::Threading::Lockable
  {
  public:
    /*------------------------------------------------
    Advanced Driver Interface
    ------------------------------------------------*/
    AdvancedDriver();
    ~AdvancedDriver();

    /*------------------------------------------------
    Timer Base Interface
    ------------------------------------------------*/
    Chimera::Status_t initPeripheral( const Chimera::Timer::DriverConfig &cfg ) final override;
    bool configured() final override;
    size_t counterBitWidth() final override;
    size_t tickRate( const Chimera::Units::Time units ) final override;
    size_t maxPeriod( const Chimera::Units::Time units ) final override;
    size_t minPeriod( const Chimera::Units::Time units ) final override;
    bool hasFunction( const Chimera::Timer::Function func ) final override;

    /*------------------------------------------------
    Timer Channel Interface
    ------------------------------------------------*/
    Chimera::Status_t enable( const Chimera::Timer::Channel channel ) final override;
    Chimera::Status_t disable( const Chimera::Timer::Channel channel ) final override;
    Chimera::Status_t enableEvent( const Chimera::Timer::Channel channel, const Chimera::Timer::Event type ) final override;
    Chimera::Status_t disableEvent( const Chimera::Timer::Channel channel, const Chimera::Timer::Event type ) final override;

    /*------------------------------------------------
    Encoder Interface
    ------------------------------------------------*/
    Chimera::Status_t encInit( const Chimera::Timer::Encoder::Config &cfg ) final override;

    /*------------------------------------------------
    Input Capture Interface
    ------------------------------------------------*/
    Chimera::Status_t icInit( const Chimera::Timer::InputCapture::Config &cfg ) final override;

    /*------------------------------------------------
    One Pulse Interface
    ------------------------------------------------*/
    Chimera::Status_t opInit( const Chimera::Timer::OnePulse::Config &cfg ) final override;

    /*------------------------------------------------
    Output Compare Interface
    ------------------------------------------------*/
    Chimera::Status_t ocInit( const Chimera::Timer::OutputCompare::Config &cfg ) final override;

    /*------------------------------------------------
    PWM Interface
    ------------------------------------------------*/
    Chimera::Status_t pwmInit( const Chimera::Timer::PWM::Config &cfg ) final override;

  private:
    friend AdvancedDriver_sPtr getAdvancedDriver_sPtr( const Chimera::Timer::Peripheral, const bool );

    size_t mResourceIndex;
  };
  #else 
  class AdvancedDriver : public Chimera::Timer::ITimerBase<AdvancedDriver>,
                         public Chimera::Timer::ITimerEncoder<AdvancedDriver>,
                         public Chimera::Timer::ITimerInputCapture<AdvancedDriver>,
                         public Chimera::Timer::ITimerOnePulse<AdvancedDriver>,
                         public Chimera::Timer::ITimerOutputCompare<AdvancedDriver>,
                         public Chimera::Timer::ITimerPWM<AdvancedDriver>,
                         public Chimera::Threading::Lockable
  {
  public:
    /*------------------------------------------------
    Advanced Driver Interface
    ------------------------------------------------*/
    AdvancedDriver();
    ~AdvancedDriver();

    /*------------------------------------------------
    Timer Base Interface
    ------------------------------------------------*/
    Chimera::Status_t initPeripheral( const Chimera::Timer::DriverConfig &cfg );
    bool configured();
    size_t counterBitWidth();
    size_t tickRate( const Chimera::Units::Time units );
    size_t maxPeriod( const Chimera::Units::Time units );
    size_t minPeriod( const Chimera::Units::Time units );
    bool hasFunction( const Chimera::Timer::Function func );

    /*------------------------------------------------
    Timer Channel Interface
    ------------------------------------------------*/
    Chimera::Status_t enable( const Chimera::Timer::Channel channel );
    Chimera::Status_t disable( const Chimera::Timer::Channel channel );
    Chimera::Status_t enableEvent( const Chimera::Timer::Channel channel, const Chimera::Timer::Event type );
    Chimera::Status_t disableEvent( const Chimera::Timer::Channel channel, const Chimera::Timer::Event type );

    /*------------------------------------------------
    Encoder Interface
    ------------------------------------------------*/
    Chimera::Status_t encInit( const Chimera::Timer::Encoder::Config &cfg );

    /*------------------------------------------------
    Input Capture Interface
    ------------------------------------------------*/
    Chimera::Status_t icInit( const Chimera::Timer::InputCapture::Config &cfg );

    /*------------------------------------------------
    One Pulse Interface
    ------------------------------------------------*/
    Chimera::Status_t opInit( const Chimera::Timer::OnePulse::Config &cfg );

    /*------------------------------------------------
    Output Compare Interface
    ------------------------------------------------*/
    Chimera::Status_t ocInit( const Chimera::Timer::OutputCompare::Config &cfg );

    /*------------------------------------------------
    PWM Interface
    ------------------------------------------------*/
    Chimera::Status_t pwmInit( const Chimera::Timer::PWM::Config &cfg );

  private:
    friend AdvancedDriver_sPtr getAdvancedDriver_sPtr( const Chimera::Timer::Peripheral, const bool );

    size_t mResourceIndex;
  };
  #endif 

  #if defined( VIRTUAL_FUNC )
  class GeneralDriver : virtual public Chimera::Timer::ITimerBase,
                        virtual public Chimera::Timer::ITimerEncoder,
                        virtual public Chimera::Timer::ITimerInputCapture,
                        virtual public Chimera::Timer::ITimerOnePulse,
                        virtual public Chimera::Timer::ITimerOutputCompare,
                        virtual public Chimera::Timer::ITimerPWM,
                        public Chimera::Threading::Lockable
  {
  public:
    /*------------------------------------------------
    General Driver Interface
    ------------------------------------------------*/
    GeneralDriver();
    ~GeneralDriver();

    /*------------------------------------------------
    Timer Base Interface
    ------------------------------------------------*/
    Chimera::Status_t initPeripheral( const Chimera::Timer::DriverConfig &cfg ) final override;
    bool configured() final override;
    size_t counterBitWidth() final override;
    size_t tickRate( const Chimera::Units::Time units ) final override;
    size_t maxPeriod( const Chimera::Units::Time units ) final override;
    size_t minPeriod( const Chimera::Units::Time units ) final override;
    bool hasFunction( const Chimera::Timer::Function func ) final override;

    /*------------------------------------------------
    Timer Channel Interface
    ------------------------------------------------*/
    Chimera::Status_t enable( const Chimera::Timer::Channel channel ) final override;
    Chimera::Status_t disable( const Chimera::Timer::Channel channel ) final override;
    Chimera::Status_t enableEvent( const Chimera::Timer::Channel channel, const Chimera::Timer::Event type ) final override;
    Chimera::Status_t disableEvent( const Chimera::Timer::Channel channel, const Chimera::Timer::Event type ) final override;

    /*------------------------------------------------
    Encoder Interface
    ------------------------------------------------*/
    Chimera::Status_t encInit( const Chimera::Timer::Encoder::Config &cfg ) final override;

    /*------------------------------------------------
    Input Capture Interface
    ------------------------------------------------*/
    Chimera::Status_t icInit( const Chimera::Timer::InputCapture::Config &cfg ) final override;

    /*------------------------------------------------
    One Pulse Interface
    ------------------------------------------------*/
    Chimera::Status_t opInit( const Chimera::Timer::OnePulse::Config &cfg ) final override;

    /*------------------------------------------------
    Output Compare Interface
    ------------------------------------------------*/
    Chimera::Status_t ocInit( const Chimera::Timer::OutputCompare::Config &cfg ) final override;

    /*------------------------------------------------
    PWM Interface
    ------------------------------------------------*/
    Chimera::Status_t pwmInit( const Chimera::Timer::PWM::Config &cfg ) final override;

  private:
    friend GeneralDriver_sPtr getGeneralDriver_sPtr( const Chimera::Timer::Peripheral, const bool );

    size_t mResourceIndex;
  };

  #else
  class GeneralDriver : public Chimera::Timer::ITimerEncoder<GeneralDriver>,
                        public Chimera::Threading::Lockable
  {
  public:
    /*------------------------------------------------
    General Driver Interface
    ------------------------------------------------*/
    GeneralDriver();
    ~GeneralDriver();

    /*------------------------------------------------
    Timer Base Interface
    ------------------------------------------------*/
    Chimera::Status_t initPeripheral( const Chimera::Timer::DriverConfig &cfg );
    bool configured();
    size_t counterBitWidth();
    size_t tickRate( const Chimera::Units::Time units );
    size_t maxPeriod( const Chimera::Units::Time units );
    size_t minPeriod( const Chimera::Units::Time units );
    bool hasFunction( const Chimera::Timer::Function func );

    /*------------------------------------------------
    Timer Channel Interface
    ------------------------------------------------*/
    Chimera::Status_t enable( const Chimera::Timer::Channel channel );
    Chimera::Status_t disable( const Chimera::Timer::Channel channel );
    Chimera::Status_t enableEvent( const Chimera::Timer::Channel channel, const Chimera::Timer::Event type );
    Chimera::Status_t disableEvent( const Chimera::Timer::Channel channel, const Chimera::Timer::Event type );

    /*------------------------------------------------
    Encoder Interface
    ------------------------------------------------*/
    Chimera::Status_t encInit( const Chimera::Timer::Encoder::Config &cfg );

    /*------------------------------------------------
    Input Capture Interface
    ------------------------------------------------*/
    Chimera::Status_t icInit( const Chimera::Timer::InputCapture::Config &cfg );

    /*------------------------------------------------
    One Pulse Interface
    ------------------------------------------------*/
    Chimera::Status_t opInit( const Chimera::Timer::OnePulse::Config &cfg );

    /*------------------------------------------------
    Output Compare Interface
    ------------------------------------------------*/
    Chimera::Status_t ocInit( const Chimera::Timer::OutputCompare::Config &cfg );

    /*------------------------------------------------
    PWM Interface
    ------------------------------------------------*/
    Chimera::Status_t pwmInit( const Chimera::Timer::PWM::Config &cfg );

  private:
    friend GeneralDriver_sPtr getGeneralDriver_sPtr( const Chimera::Timer::Peripheral, const bool );

    size_t mResourceIndex;
  };
  #endif

  #if defined( VIRTUAL_FUNC )
  class BasicDriver : virtual public Chimera::Timer::ITimerBase,
                      virtual public Chimera::Timer::ITimerEncoder,
                      virtual public Chimera::Timer::ITimerInputCapture,
                      virtual public Chimera::Timer::ITimerOnePulse,
                      virtual public Chimera::Timer::ITimerOutputCompare,
                      virtual public Chimera::Timer::ITimerPWM,
                      public Chimera::Threading::Lockable
  {
  public:
    /*------------------------------------------------
    Basic Driver Interface
    ------------------------------------------------*/
    BasicDriver();
    ~BasicDriver();

    /*------------------------------------------------
    Timer Base Interface
    ------------------------------------------------*/
    Chimera::Status_t initPeripheral( const Chimera::Timer::DriverConfig &cfg ) final override;
    bool configured() final override;
    size_t counterBitWidth() final override;
    size_t tickRate( const Chimera::Units::Time units ) final override;
    size_t maxPeriod( const Chimera::Units::Time units ) final override;
    size_t minPeriod( const Chimera::Units::Time units ) final override;
    bool hasFunction( const Chimera::Timer::Function func ) final override;

    /*------------------------------------------------
    Timer Channel Interface
    ------------------------------------------------*/
    Chimera::Status_t enable( const Chimera::Timer::Channel channel ) final override;
    Chimera::Status_t disable( const Chimera::Timer::Channel channel ) final override;
    Chimera::Status_t enableEvent( const Chimera::Timer::Channel channel, const Chimera::Timer::Event type ) final override;
    Chimera::Status_t disableEvent( const Chimera::Timer::Channel channel, const Chimera::Timer::Event type ) final override;

    /*------------------------------------------------
    Encoder Interface
    ------------------------------------------------*/
    Chimera::Status_t encInit( const Chimera::Timer::Encoder::Config &cfg ) final override;

    /*------------------------------------------------
    Input Capture Interface
    ------------------------------------------------*/
    Chimera::Status_t icInit( const Chimera::Timer::InputCapture::Config &cfg ) final override;

    /*------------------------------------------------
    One Pulse Interface
    ------------------------------------------------*/
    Chimera::Status_t opInit( const Chimera::Timer::OnePulse::Config &cfg ) final override;

    /*------------------------------------------------
    Output Compare Interface
    ------------------------------------------------*/
    Chimera::Status_t ocInit( const Chimera::Timer::OutputCompare::Config &cfg ) final override;

    /*------------------------------------------------
    PWM Interface
    ------------------------------------------------*/
    Chimera::Status_t pwmInit( const Chimera::Timer::PWM::Config &cfg ) final override;

  private:
    friend BasicDriver_sPtr getBasicDriver_sPtr( const Chimera::Timer::Peripheral, const bool );

    size_t mResourceIndex;
  };
  #else 
  class BasicDriver : public Chimera::Timer::ITimerBase<BasicDriver>,
                      public Chimera::Timer::ITimerEncoder<BasicDriver>,
                      public Chimera::Timer::ITimerInputCapture<BasicDriver>,
                      public Chimera::Timer::ITimerOnePulse<BasicDriver>,
                      public Chimera::Timer::ITimerOutputCompare<BasicDriver>,
                      public Chimera::Timer::ITimerPWM<BasicDriver>,
                      public Chimera::Threading::Lockable
  {
  public:
    /*------------------------------------------------
    Basic Driver Interface
    ------------------------------------------------*/
    BasicDriver();
    ~BasicDriver();

    /*------------------------------------------------
    Timer Base Interface
    ------------------------------------------------*/
    Chimera::Status_t initPeripheral( const Chimera::Timer::DriverConfig &cfg );
    bool configured();
    size_t counterBitWidth();
    size_t tickRate( const Chimera::Units::Time units );
    size_t maxPeriod( const Chimera::Units::Time units );
    size_t minPeriod( const Chimera::Units::Time units );
    bool hasFunction( const Chimera::Timer::Function func );

    /*------------------------------------------------
    Timer Channel Interface
    ------------------------------------------------*/
    Chimera::Status_t enable( const Chimera::Timer::Channel channel );
    Chimera::Status_t disable( const Chimera::Timer::Channel channel );
    Chimera::Status_t enableEvent( const Chimera::Timer::Channel channel, const Chimera::Timer::Event type );
    Chimera::Status_t disableEvent( const Chimera::Timer::Channel channel, const Chimera::Timer::Event type );

    /*------------------------------------------------
    Encoder Interface
    ------------------------------------------------*/
    Chimera::Status_t encInit( const Chimera::Timer::Encoder::Config &cfg );

    /*------------------------------------------------
    Input Capture Interface
    ------------------------------------------------*/
    Chimera::Status_t icInit( const Chimera::Timer::InputCapture::Config &cfg );

    /*------------------------------------------------
    One Pulse Interface
    ------------------------------------------------*/
    Chimera::Status_t opInit( const Chimera::Timer::OnePulse::Config &cfg );

    /*------------------------------------------------
    Output Compare Interface
    ------------------------------------------------*/
    Chimera::Status_t ocInit( const Chimera::Timer::OutputCompare::Config &cfg );

    /*------------------------------------------------
    PWM Interface
    ------------------------------------------------*/
    Chimera::Status_t pwmInit( const Chimera::Timer::PWM::Config &cfg );

  private:
    friend BasicDriver_sPtr getBasicDriver_sPtr( const Chimera::Timer::Peripheral, const bool );

    size_t mResourceIndex;
  };
  #endif 

  #if defined( VIRTUAL_FUNC )
  class LowPowerDriver : virtual public Chimera::Timer::ITimerBase,
                         virtual public Chimera::Timer::ITimerEncoder,
                         virtual public Chimera::Timer::ITimerInputCapture,
                         virtual public Chimera::Timer::ITimerOnePulse,
                         virtual public Chimera::Timer::ITimerOutputCompare,
                         virtual public Chimera::Timer::ITimerPWM,
                         public Chimera::Threading::Lockable
  {
  public:
    /*------------------------------------------------
    Low Power Driver Interface
    ------------------------------------------------*/
    LowPowerDriver();
    ~LowPowerDriver();

    /*------------------------------------------------
    Timer Base Interface
    ------------------------------------------------*/
    Chimera::Status_t initPeripheral( const Chimera::Timer::DriverConfig &cfg ) final override;
    bool configured() final override;
    size_t counterBitWidth() final override;
    size_t tickRate( const Chimera::Units::Time units ) final override;
    size_t maxPeriod( const Chimera::Units::Time units ) final override;
    size_t minPeriod( const Chimera::Units::Time units ) final override;
    bool hasFunction( const Chimera::Timer::Function func ) final override;

    /*------------------------------------------------
    Timer Channel Interface
    ------------------------------------------------*/
    Chimera::Status_t enable( const Chimera::Timer::Channel channel ) final override;
    Chimera::Status_t disable( const Chimera::Timer::Channel channel ) final override;
    Chimera::Status_t enableEvent( const Chimera::Timer::Channel channel, const Chimera::Timer::Event type ) final override;
    Chimera::Status_t disableEvent( const Chimera::Timer::Channel channel, const Chimera::Timer::Event type ) final override;

    /*------------------------------------------------
    Encoder Interface
    ------------------------------------------------*/
    Chimera::Status_t encInit( const Chimera::Timer::Encoder::Config &cfg ) final override;

    /*------------------------------------------------
    Input Capture Interface
    ------------------------------------------------*/
    Chimera::Status_t icInit( const Chimera::Timer::InputCapture::Config &cfg ) final override;

    /*------------------------------------------------
    One Pulse Interface
    ------------------------------------------------*/
    Chimera::Status_t opInit( const Chimera::Timer::OnePulse::Config &cfg ) final override;

    /*------------------------------------------------
    Output Compare Interface
    ------------------------------------------------*/
    Chimera::Status_t ocInit( const Chimera::Timer::OutputCompare::Config &cfg ) final override;

    /*------------------------------------------------
    PWM Interface
    ------------------------------------------------*/
    Chimera::Status_t pwmInit( const Chimera::Timer::PWM::Config &cfg ) final override;

  private:
    friend LowPowerDriver_sPtr getLowPowerDriver_sPtr( const Chimera::Timer::Peripheral, const bool );

    size_t mResourceIndex;
  };
  #else
  class LowPowerDriver : public Chimera::Timer::ITimerBase<LowPowerDriver>,
                         public Chimera::Timer::ITimerEncoder<LowPowerDriver>,
                         public Chimera::Timer::ITimerInputCapture<LowPowerDriver>,
                         public Chimera::Timer::ITimerOnePulse<LowPowerDriver>,
                         public Chimera::Timer::ITimerOutputCompare<LowPowerDriver>,
                         public Chimera::Timer::ITimerPWM<LowPowerDriver>,
                         public Chimera::Threading::Lockable
  {
  public:
    /*------------------------------------------------
    Low Power Driver Interface
    ------------------------------------------------*/
    LowPowerDriver();
    ~LowPowerDriver();

    /*------------------------------------------------
    Timer Base Interface
    ------------------------------------------------*/
    Chimera::Status_t initPeripheral( const Chimera::Timer::DriverConfig &cfg );
    bool configured();
    size_t counterBitWidth();
    size_t tickRate( const Chimera::Units::Time units );
    size_t maxPeriod( const Chimera::Units::Time units );
    size_t minPeriod( const Chimera::Units::Time units );
    bool hasFunction( const Chimera::Timer::Function func );

    /*------------------------------------------------
    Timer Channel Interface
    ------------------------------------------------*/
    Chimera::Status_t enable( const Chimera::Timer::Channel channel );
    Chimera::Status_t disable( const Chimera::Timer::Channel channel );
    Chimera::Status_t enableEvent( const Chimera::Timer::Channel channel, const Chimera::Timer::Event type );
    Chimera::Status_t disableEvent( const Chimera::Timer::Channel channel, const Chimera::Timer::Event type );

    /*------------------------------------------------
    Encoder Interface
    ------------------------------------------------*/
    Chimera::Status_t encInit( const Chimera::Timer::Encoder::Config &cfg );

    /*------------------------------------------------
    Input Capture Interface
    ------------------------------------------------*/
    Chimera::Status_t icInit( const Chimera::Timer::InputCapture::Config &cfg );

    /*------------------------------------------------
    One Pulse Interface
    ------------------------------------------------*/
    Chimera::Status_t opInit( const Chimera::Timer::OnePulse::Config &cfg );

    /*------------------------------------------------
    Output Compare Interface
    ------------------------------------------------*/
    Chimera::Status_t ocInit( const Chimera::Timer::OutputCompare::Config &cfg );

    /*------------------------------------------------
    PWM Interface
    ------------------------------------------------*/
    Chimera::Status_t pwmInit( const Chimera::Timer::PWM::Config &cfg );

  private:
    friend LowPowerDriver_sPtr getLowPowerDriver_sPtr( const Chimera::Timer::Peripheral, const bool );

    size_t mResourceIndex;
  };
  #endif 
}    // namespace Thor::TIMER

#endif /* !THOR_HLD_TIMER_DRIVER_HPP */