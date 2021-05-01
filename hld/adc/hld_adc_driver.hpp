/********************************************************************************
 *  File Name:
 *    hld_adc_driver.hpp
 *
 *  Description:
 *    Thor ADC high level driver
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HLD_ADC_HPP
#define THOR_HLD_ADC_HPP

/* C++ Includes */
#include <cstdint>
#include <cstdlib>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/callback>
#include <Chimera/adc>
#include <Chimera/thread>

/* Thor Includes */
#include <Thor/hld/adc/hld_adc_types.hpp>

namespace Thor::ADC
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize();
  Chimera::Status_t reset();
  Driver_rPtr getDriver( const Chimera::ADC::Peripheral periph );
  bool featureSupported( const Chimera::ADC::Peripheral periph, const Chimera::ADC::Feature feature );


  /*-------------------------------------------------------------------------------
  Classes
  -------------------------------------------------------------------------------*/
  /**
   *  ADC Peripheral Driver
   *  Methods here at a minimum implement the interface specified in Chimera.
   *  Inheritance is avoided to minimize cost of virtual function lookup table.
   */
  class Driver : public Chimera::Thread::Lockable<Driver>
  {
  public:
    /*-------------------------------------------------
    Class Methods
    -------------------------------------------------*/
    Driver();
    ~Driver();

    /**
     *  User-space handler for processing ISR events
     *  @return void
     */
    void postISRProcessing();

    /*-------------------------------------------------
    Interface: Hardware
    -------------------------------------------------*/
    Chimera::Status_t open( const Chimera::ADC::DriverConfig &init );
    void close();
    Chimera::Status_t setSampleTime( const Chimera::ADC::Channel ch, const size_t cycles );
    Chimera::ADC::Sample sampleChannel( const Chimera::ADC::Channel ch );
    Chimera::Status_t configSequence( const Chimera::ADC::SequenceInit &cfg );
    void startSequence();
    void stopSequence();
    bool nextSample( const Chimera::ADC::Channel ch, Chimera::ADC::Sample &sample );
    void onInterrupt( const Chimera::ADC::Interrupt bmSignal, Chimera::ADC::ISRCallback cb );
    float toVoltage( const Chimera::ADC::Sample sample );

  private:
    friend Chimera::Thread::Lockable<Driver>;

    Chimera::ADC::Peripheral mPeriph;
    Chimera::ADC::DriverConfig mConfig;
    Chimera::ADC::CallbackArray mCallbacks;
  };
}    // namespace Thor::ADC

#endif /* THOR_HLD_ADC_HPP */
